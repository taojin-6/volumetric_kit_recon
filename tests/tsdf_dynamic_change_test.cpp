// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// Dynamic fusion evicts on change and averages conflict, through tsdf::Fuser.
// Two fixed cameras, A square to a wall 2 m away and B 0.4 m to its side
// turned toward the wall's centre, B's depth reading 8 mm long throughout --
// the cameras disagree by more than noise, as calibrated rigs do. Depth is
// ray-cast and noise-free, so every change below is real and nothing else
// is; each camera states the Femto Mega's noise, as its driver would. One
// sequence of 71 sets:
//
// - sets 0-34, a card 2.5 cm in front of the wall, inside the wall's band:
//   while the scene is still, no voxel's weight ever falls -- the cameras'
//   disagreement averages, it never evicts;
// - set 35, the card is gone: in that very set nothing of it meshes, and the
//   wall behind it does -- debris inside a surface's band is cleared on
//   change, which Dynamic's former single-sample clear never did;
// - set 40, camera B alone: a camera missing from a set leaves the other's
//   history its own, and nothing is reset;
// - set 50, a box 0.5 m in front of the wall arrives and stays to set 69: it
//   meshes in the set it arrives, its voxels' weights never fall while it
//   stays, and the wall it hides keeps every voxel's weight and its mesh;
// - set 70, the box leaves: in that set nothing of it meshes, no voxel of its
//   band reads as inside a surface, and the wall it hid meshes at once.
//
// Then a second scene, a fuser of its own, over no return: the wall only
// left of x = -0.3 m, nothing returning elsewhere, the same two cameras. The
// box is there over sets 40-49 and goes at 50. B, to the side, sees the wall
// past the box's left part, so that part goes in set 50, flagged Farther,
// and with it what lies within 6 pixels of it in B's image (to x = -0.04 m
// on the box), where a departure resets what a rim distance left beside it.
// Right of x = -0.05 m nothing returns behind it for either camera, so only
// its cameras' Vanished rays can clear it, and they flag on the second set
// with no return: in set 50 no weight falls right of x = 0, past that reach,
// and every triangle of it there stays, as after a single-frame dropout; in
// set 51 nothing of the box meshes and no voxel of its band reads as inside
// a surface. No weight falls in any other set.
//
// And the refusals: two frames of a set with one slot, a slot past the
// detector's, a noise model the detector refuses, an unknown one (all zero,
// a frame whose producer states none) among them -- Dynamic only, as Classic
// reads neither. Skips (exit 0) where no device is present.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <map>
#include <tuple>
#include <utility>
#include <vector>

#include "depth_inputs.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/testing/test_check.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/compute_util.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/grid/mesh/marching_cubes.hpp"
#include "volumetric_kit/grid/volume/voxel_block_grid.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"
#include "volumetric_kit/recon/core/math/vector_types.hpp"
#include "volumetric_kit/recon/tsdf/fuser.hpp"
#include "volumetric_kit/recon/tsdf/fusion_grid.hpp"
#include "volumetric_kit/recon/tsdf/tsdf_integrator.hpp"

#include "volumetric_kit/core/testing/gpu_test.hpp"
#include "volumetric_kit/grid/testing/grid_readback.hpp"

namespace vr = volumetric_kit::recon;
namespace sensor = volumetric_kit::sensor;
namespace vkc = volumetric_kit::core;
namespace vol = volumetric_kit::grid::volume;
namespace tsdf = volumetric_kit::recon::tsdf;
namespace mesh = volumetric_kit::grid::mesh;

namespace {

constexpr std::uint32_t kWidth = 160;
constexpr std::uint32_t kHeight = 120;
constexpr float kVoxel = 0.01f;
constexpr float kTrunc = 0.04f;
constexpr float kWall = 2.0f;
constexpr float kCard = 1.975f;  // inside the wall's band
constexpr float kBox = 1.5f;
constexpr float kOffsetB = 0.008f;  // camera B reads 8 mm long
constexpr float kMaxWeight = 5.0f;

// The card and the box: a rectangle facing the cameras.
constexpr float kHalfX = 0.25f;
constexpr float kHalfY = 0.2f;

enum class Thing { None, Card, Box };

Thing thing_at(int set) {
  if (set < 35) return Thing::Card;
  if (set >= 50 && set < 70) return Thing::Box;
  return Thing::None;
}

sensor::DepthCameraParams camera(const vr::Mat4f& pose) {
  sensor::DepthCameraParams cam{};
  cam.fx = cam.fy = 140.0f;
  cam.cx = 79.5f;
  cam.cy = 59.5f;
  cam.min_depth = 0.2f;
  cam.max_depth = 5.0f;
  cam.width = kWidth;
  cam.height = kHeight;
  cam.cam_to_world = pose;
  return cam;
}

// A at the origin looking down +z; B at x = 0.4 turned toward (0, 0, 2).
sensor::DepthCameraParams camera_a() { return camera(vr::Mat4f(1.0f)); }
sensor::DepthCameraParams camera_b() {
  const float yaw = std::atan2(-0.4f, kWall);
  const float c = std::cos(yaw);
  const float s = std::sin(yaw);
  return camera(vr::Mat4f(
      vr::Vec4f(c, 0.0f, -s, 0.0f), vr::Vec4f(0.0f, 1.0f, 0.0f, 0.0f),
      vr::Vec4f(s, 0.0f, c, 0.0f), vr::Vec4f(0.4f, 0.0f, 0.0f, 1.0f)));
}

// The camera-space depth of each pixel: the wall at z = 2, or the thing in
// front of it where the ray meets it first; plus `offset` along the ray.
// With `wall_left_only`, the wall returns only left of x = -0.3 m, and 0 (no
// return) elsewhere.
std::vector<float> render(const sensor::DepthCameraParams& cam, Thing thing,
                          float offset, bool wall_left_only = false) {
  std::vector<float> depth(std::size_t(kWidth) * kHeight);
  const vr::Mat4f& m = cam.cam_to_world;
  const vr::Vec3f origin(m[3]);
  for (std::uint32_t v = 0; v < kHeight; ++v) {
    for (std::uint32_t u = 0; u < kWidth; ++u) {
      // A ray whose camera-space z is 1, so its parameter is the depth.
      const vr::Vec3f ray_cam((float(u) - cam.cx) / cam.fx,
                              (float(v) - cam.cy) / cam.fy, 1.0f);
      const vr::Vec3f ray(m * vr::Vec4f(ray_cam, 0.0f));
      float t = (kWall - origin.z) / ray.z;
      float d = t + offset;
      if (wall_left_only && (origin + t * ray).x > -0.3f) d = 0.0f;
      if (thing != Thing::None) {
        const float z = thing == Thing::Card ? kCard : kBox;
        const float tt = (z - origin.z) / ray.z;
        const vr::Vec3f p = origin + tt * ray;
        if (std::fabs(p.x) <= kHalfX && std::fabs(p.y) <= kHalfY) {
          d = tt + offset;
        }
      }
      depth[std::size_t(v) * kWidth + u] = d;
    }
  }
  return depth;
}

// Every voxel of the grid's active blocks: its world position, and its
// element in the attributes.
struct Voxel {
  vr::Vec3f world;
  std::size_t index;
};

vkc::Result<std::vector<Voxel>> voxels_of(vol::VoxelBlockGrid& grid) {
  VKC_ASSIGN(const auto blocks, vgrid_test::blocks_of(grid));
  std::vector<Voxel> out;
  for (const auto& [coord, ptr] : blocks) {
    const int b[3] = {std::get<0>(coord), std::get<1>(coord),
                      std::get<2>(coord)};
    for (int l = 0; l < 512; ++l) {
      const int v[3] = {b[0] * 8 + l % 8, b[1] * 8 + (l / 8) % 8,
                        b[2] * 8 + l / 64};
      out.push_back({vr::Vec3f(float(v[0]), float(v[1]), float(v[2])) * kVoxel,
                     std::size_t(ptr) + std::size_t(l)});
    }
  }
  return out;
}

// Within the thing's rectangle, 4 cm in from its rim, so neither camera's
// view of an edge counts.
bool inside(const vr::Vec3f& p) {
  return std::fabs(p.x) < kHalfX - 0.04f && std::fabs(p.y) < kHalfY - 0.04f;
}

// Triangles every corner of which lies inside the rectangle within 1 cm of
// z = `z`, right of x = `x_min`.
vkc::Result<std::size_t> triangles_at(mesh::MarchingCubes& mc,
                                      vol::VoxelBlockGrid& grid, float z,
                                      float x_min = -1.0f) {
  VKC_ASSIGN(const mesh::Mesh m, mc.extract_host(grid));
  std::size_t n = 0;
  for (std::size_t t = 0; t < m.triangle_count(); ++t) {
    bool on = true;
    for (std::size_t c = 0; c < 3; ++c) {
      const vr::Vec3f& p = m.vertices[m.indices[3 * t + c]].position;
      on = on && inside(p) && std::fabs(p.z - z) < kVoxel && p.x > x_min;
    }
    n += on ? 1 : 0;
  }
  return n;
}

int refusals(vkc::test::GpuContext& gpu, tsdf::Fuser& fuser,
             vol::VoxelBlockGrid& grid) {
  const sensor::DepthCameraParams a = camera_a();
  const std::vector<float> depth = render(a, Thing::None, 0.0f);
  const vkc::StorageInput in(depth.data());
  const auto refused = [&](const std::vector<tsdf::FrameInput>& frames) {
    auto r =
        fuser.fuse(grid, frames, kMaxWeight, tsdf::IntegrationMode::Dynamic);
    return !r.ok() && r.status().domain() == vkc::Status::Code::InvalidArgument;
  };
  const sensor::DepthNoise& tof = vr_test::kFemtoNoise;
  CHECK(refused({{in, a, nullptr, 3, tof}, {in, a, nullptr, 3, tof}}));
  CHECK(refused({{in, a, nullptr, tsdf::ChangeDetector::kMaxCameras, tof}}));
  // A frame that states no noise, alone or beside one that does.
  CHECK(refused({{in, a, nullptr, 0}}));
  CHECK(refused({{in, a, nullptr, 0, tof}, {in, a, nullptr, 1}}));
  CHECK(refused({{in, a, nullptr, 0, sensor::DepthNoise{0.0f, -1e-3f, 0.0f}}}));
  // Classic reads neither the slots nor the noise.
  CHECK(fuser
            .fuse(grid, {{in, a, nullptr, 3}, {in, a, nullptr, 3}}, kMaxWeight,
                  tsdf::IntegrationMode::Classic)
            .ok());
  (void)gpu;
  return 0;
}

// The second scene: a box over no return, gone at set 50, cleared at 51.
int no_return_case(vkc::test::GpuContext& gpu) {
  // Right of x = -0.05, no camera has anything behind the box: B's rays past
  // its band there meet z = 2 right of x = -0.22, where the wall is gone.
  // B's departure, where it sees the wall past the box's left part, reaches
  // 6 pixels further in its image, to x = -0.04 at the band's front, and
  // resets there what a rim distance left beside it (DECISIONS.md,
  // 2026-10-29). Right of this, set 50 changes nothing.
  constexpr float kStillX = 0.0f;
  vkc::Device& dev = gpu.device;
  vkc::Allocator& alloc = gpu.allocator;
  auto fuser = tsdf::Fuser::create(dev, alloc);
  CHECK(fuser.ok());
  auto mc = mesh::MarchingCubes::create(dev, alloc);
  CHECK(mc.ok());
  auto made = tsdf::create_fusion_grid(dev, alloc, kVoxel, kTrunc, 1024);
  CHECK(made.ok());
  vol::VoxelBlockGrid& grid = made.value();
  const sensor::DepthCameraParams a = camera_a();
  const sensor::DepthCameraParams b = camera_b();
  std::vector<float> weights_before;
  std::size_t stays_49 = 0;  // the box's triangles right of kStillX
  for (int set = 0; set <= 51; ++set) {
    const Thing thing = set >= 40 && set < 50 ? Thing::Box : Thing::None;
    const std::vector<float> da = render(a, thing, 0.0f, true);
    const std::vector<float> db = render(b, thing, kOffsetB, true);
    auto fused = fuser->fuse(
        grid,
        {{vkc::StorageInput(da.data()), a, nullptr, 0, vr_test::kFemtoNoise},
         {vkc::StorageInput(db.data()), b, nullptr, 1, vr_test::kFemtoNoise}},
        kMaxWeight, tsdf::IntegrationMode::Dynamic);
    CHECK(fused.ok());
    auto w = vgrid_test::read_attribute<float>(dev, alloc, grid,
                                               vol::kWeightAttribute);
    CHECK(w.ok());
    const std::vector<float>& weights = w.value();
    if (!weights_before.empty() && set != 51) {
      // In set 50, only where a camera sees the wall past the box, or within
      // the reach of that departure.
      std::vector<bool> may_fall(weights_before.size(), false);
      if (set == 50) {
        auto vox = voxels_of(grid);
        CHECK(vox.ok());
        for (const Voxel& x : vox.value()) {
          if (x.index < may_fall.size()) {
            may_fall[x.index] = x.world.x <= kStillX;
          }
        }
      }
      std::size_t fell = 0;
      for (std::size_t i = 0; i < weights_before.size(); ++i) {
        fell += weights[i] < weights_before[i] && !may_fall[i] ? 1 : 0;
      }
      if (fell != 0) {
        std::fprintf(stderr, "over no return, set %d: %zu weights fell\n", set,
                     fell);
        return 1;
      }
    }
    if (set >= 49) {
      auto box = triangles_at(mc.value(), grid, kBox);
      auto stays = triangles_at(mc.value(), grid, kBox, kStillX);
      CHECK(box.ok() && stays.ok());
      std::printf(
          "over no return, set %d: %zu triangles on the box, %zu right of "
          "x = %.2f m\n",
          set, box.value(), stays.value(), double(kStillX));
      if (set == 49) {
        stays_49 = stays.value();
        CHECK(stays_49 > 500);
      }
      if (set == 50) CHECK(stays.value() == stays_49);
      if (set == 51) CHECK(box.value() == 0);
    }
    if (set == 51) {
      auto vox = voxels_of(grid);
      auto t = vgrid_test::read_attribute<float>(dev, alloc, grid,
                                                 vol::kTsdfAttribute);
      CHECK(vox.ok() && t.ok());
      std::size_t inside_surface = 0;
      for (const Voxel& x : vox.value()) {
        if (inside(x.world) && std::fabs(x.world.z - kBox) <= kTrunc &&
            weights[x.index] > 0.0f && t.value()[x.index] < 0.0f) {
          ++inside_surface;
        }
      }
      CHECK(inside_surface == 0);
    }
    weights_before = weights;
  }
  return 0;
}

}  // namespace

int gpu_main(vkc::test::GpuContext& gpu) {
  vkc::Device& dev = gpu.device;
  vkc::Allocator& alloc = gpu.allocator;
  auto fuser = tsdf::Fuser::create(dev, alloc);
  CHECK(fuser.ok());
  auto mc = mesh::MarchingCubes::create(dev, alloc);
  CHECK(mc.ok());
  auto made = tsdf::create_fusion_grid(dev, alloc, kVoxel, kTrunc, 1024);
  CHECK(made.ok());
  vol::VoxelBlockGrid& grid = made.value();

  const sensor::DepthCameraParams a = camera_a();
  const sensor::DepthCameraParams b = camera_b();
  std::vector<float> weights_before;  // the last set's weights
  std::vector<float> hidden_before;   // the wall's, before the box came
  std::size_t evicted = 0;            // weights fallen while still
  for (int set = 0; set <= 70; ++set) {
    const Thing thing = thing_at(set);
    const std::vector<float> da = render(a, thing, 0.0f);
    const std::vector<float> db = render(b, thing, kOffsetB);
    std::vector<tsdf::FrameInput> frames;
    if (set != 40) {
      frames.push_back(
          {vkc::StorageInput(da.data()), a, nullptr, 0, vr_test::kFemtoNoise});
    }
    frames.push_back(
        {vkc::StorageInput(db.data()), b, nullptr, 1, vr_test::kFemtoNoise});
    auto fused =
        fuser->fuse(grid, frames, kMaxWeight, tsdf::IntegrationMode::Dynamic);
    CHECK(fused.ok());

    auto w = vgrid_test::read_attribute<float>(dev, alloc, grid,
                                               vol::kWeightAttribute);
    CHECK(w.ok());
    const std::vector<float>& weights = w.value();
    // A set with no change resets nothing: no weight falls. That is every
    // set but the three that change something.
    const bool changes = set == 35 || set == 50 || set == 70;
    if (!weights_before.empty() && !changes) {
      for (std::size_t i = 0; i < weights_before.size(); ++i) {
        evicted += weights[i] < weights_before[i] ? 1 : 0;
      }
      if (evicted != 0) {
        std::fprintf(stderr, "set %d: %zu weights fell\n", set, evicted);
        return 1;
      }
    }

    if (set == 34 || set == 35) {
      // The card meshes until it leaves, and in the set it leaves nothing of
      // it does while the wall it covered does.
      auto card = triangles_at(mc.value(), grid, kCard);
      auto wall = triangles_at(mc.value(), grid, kWall + 0.5f * kOffsetB);
      CHECK(card.ok() && wall.ok());
      std::printf("set %d: %zu triangles on the card, %zu on the wall\n", set,
                  card.value(), wall.value());
      if (set == 34) CHECK(card.value() > 500);
      if (set == 35) CHECK(card.value() == 0 && wall.value() > 500);
    }
    if (set == 49 || set == 69) {
      // The wall the box hides, just before it comes and just before it
      // goes: every voxel within 2 cm of the wall that the box hides from
      // both cameras -- B, to the side, sees past the box's rim to x = 0.2 --
      // keeps its weight, bit for bit.
      auto vox = voxels_of(grid);
      CHECK(vox.ok());
      std::vector<float> hidden;
      for (const Voxel& x : vox.value()) {
        if (inside(x.world) && x.world.x < 0.15f &&
            std::fabs(x.world.z - kWall) <= 0.02f) {
          hidden.push_back(weights[x.index]);
        }
      }
      if (set == 49) {
        hidden_before = hidden;
        std::size_t weighted = 0;
        for (const float h : hidden) weighted += h > 0.0f ? 1 : 0;
        std::printf("set 49: %zu of %zu wall voxels hold weight\n", weighted,
                    hidden.size());
        CHECK(weighted > 500);
      } else {
        CHECK(hidden == hidden_before);
      }
    }
    if (set == 50 || set == 69 || set == 70) {
      auto box = triangles_at(mc.value(), grid, kBox);
      auto wall = triangles_at(mc.value(), grid, kWall + 0.5f * kOffsetB);
      CHECK(box.ok() && wall.ok());
      std::printf("set %d: %zu triangles on the box, %zu on the wall\n", set,
                  box.value(), wall.value());
      // The wall it hides stays, behind it, as its history does.
      if (set < 70) CHECK(box.value() > 500 && wall.value() > 500);
      if (set == 70) CHECK(box.value() == 0 && wall.value() > 500);
    }
    if (set == 70) {
      // No voxel of the box's band says "inside a surface" any more.
      auto vox = voxels_of(grid);
      auto t = vgrid_test::read_attribute<float>(dev, alloc, grid,
                                                 vol::kTsdfAttribute);
      CHECK(vox.ok() && t.ok());
      std::size_t inside_surface = 0;
      for (const Voxel& x : vox.value()) {
        if (inside(x.world) && std::fabs(x.world.z - kBox) <= kTrunc &&
            weights[x.index] > 0.0f && t.value()[x.index] < 0.0f) {
          ++inside_surface;
        }
      }
      CHECK(inside_surface == 0);
    }
    weights_before = weights;
  }

  if (refusals(gpu, fuser.value(), grid) != 0) return 1;
  if (no_return_case(gpu) != 0) return 1;
  std::puts("tsdf_dynamic_change: OK");
  return 0;
}

int main() {
  return vkc::test::run_on_gpu(vr::device_requirements(), gpu_main);
}
