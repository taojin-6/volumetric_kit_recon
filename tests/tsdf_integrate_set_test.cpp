// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The several-camera overloads of VoxelHashMap::allocate_from_depth and
// TsdfIntegrator::integrate against the one-frame ones. Three cameras round a
// sphere, fused twice into one grid one frame at a time and into another as a
// set, must give the same blocks and the same tsdf, weight and colour bit for
// bit, classic and dynamic: each frame is a dispatch of its own in both, in the
// same order. The frames mix host arrays and storage buffers, colour and none,
// and the set carries an empty frame, which both calls skip. The set grows the
// map's and the integrator's sets past an earlier call's, and stamps changed
// the blocks one frame at a time changes, so the stamps are bound on every
// set. And a set with one bad frame is refused before anything is fused.
// Exits 0 (skip) where no device is present.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <map>
#include <tuple>
#include <vector>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/buffer.hpp"
#include "volumetric_kit/core/vulkan/compute_util.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/core/vulkan/instance.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"
#include "volumetric_kit/recon/tsdf/tsdf_integrator.hpp"
#include "volumetric_kit/recon/volume/hash_types.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"
#include "volumetric_kit/recon/volume/voxel_hash_map.hpp"

#include "grid_readback.hpp"
#include "no_device.hpp"

namespace vr = volumetric_kit::recon;
namespace vkc = volumetric_kit::core;
namespace vol = volumetric_kit::recon::volume;
namespace tsdf = volumetric_kit::recon::tsdf;

#define CHECK(cond)                                                        \
  do {                                                                     \
    if (!(cond)) {                                                         \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      return 1;                                                            \
    }                                                                      \
  } while (0)

namespace {

constexpr std::uint32_t kWidth = 160;
constexpr std::uint32_t kHeight = 120;
constexpr int kCameras = 3;
constexpr float kRadius = 0.25f;  // the sphere, at the world origin

using Coord = std::tuple<int, int, int>;

vol::VoxelGridParams grid_params() {
  vol::VoxelGridParams grid{};
  grid.voxel_size = 0.01f;
  grid.block_size = 8;
  grid.voxels_per_block = 512;
  grid.trunc_dist = 0.04f;
  grid.bucket_size = 8;
  grid.num_buckets = 1024;
  grid.num_blocks = 1024 * 8;
  grid.max_chain = 128;
  return grid;
}

vkc::Result<vol::VoxelBlockGrid> make_grid(vkc::Device& device,
                                           vkc::Allocator& allocator) {
  const vol::AttributeSpec attrs[] = {{"tsdf", sizeof(float)},
                                      {"weight", sizeof(float)},
                                      {"color", sizeof(std::uint32_t)}};
  return vol::VoxelBlockGrid::create(device, allocator, grid_params(), attrs,
                                     3);
}

// One camera on a circle round the sphere, looking at it, and what it sees.
struct View {
  vr::DepthCameraParams cam{};
  std::vector<float> depth;
  std::vector<std::uint32_t> color;
};

View make_view(int index) {
  const float angle = 2.1f * static_cast<float>(index);
  const vr::Vec3f eye(0.9f * std::sin(angle), 0.1f * static_cast<float>(index),
                      -0.9f * std::cos(angle));
  // OpenCV axes: z forward, y down, x = y cross z.
  const vr::Vec3f z = glm::normalize(-eye);
  const vr::Vec3f x = glm::normalize(glm::cross(vr::Vec3f(0, 1, 0), z));
  const vr::Vec3f y = glm::cross(z, x);
  View v;
  v.cam.fx = 140.0f;
  v.cam.fy = 140.0f;
  v.cam.cx = 79.5f;
  v.cam.cy = 59.5f;
  v.cam.min_depth = 0.1f;
  v.cam.max_depth = 5.0f;
  v.cam.width = kWidth;
  v.cam.height = kHeight;
  v.cam.cam_to_world = vr::Mat4f(vr::Vec4f(x, 0), vr::Vec4f(y, 0),
                                 vr::Vec4f(z, 0), vr::Vec4f(eye, 1));
  v.depth.assign(kWidth * kHeight, 0.0f);
  v.color.assign(kWidth * kHeight, 0u);
  for (std::uint32_t row = 0; row < kHeight; ++row) {
    for (std::uint32_t col = 0; col < kWidth; ++col) {
      const vr::Vec3f ray = glm::normalize(
          (static_cast<float>(col) + 0.5f - v.cam.cx) / v.cam.fx * x +
          (static_cast<float>(row) + 0.5f - v.cam.cy) / v.cam.fy * y + z);
      // |eye + t ray| = r, the nearer root.
      const float b = glm::dot(eye, ray);
      const float disc = b * b - glm::dot(eye, eye) + kRadius * kRadius;
      const std::size_t i = row * kWidth + col;
      v.color[i] = (col * 255 / kWidth) | ((row * 255 / kHeight) << 8) |
                   (static_cast<std::uint32_t>(60 * index) << 16) | 0xFF000000u;
      if (disc <= 0.0f) continue;  // misses: no return
      const float t = -b - std::sqrt(disc);
      v.depth[i] = t * glm::dot(ray, z);
    }
  }
  return v;
}

// Every active block's coordinate and first voxel (`ptr` is a voxel offset).
vkc::Result<std::map<Coord, std::int32_t>> blocks_of(vol::VoxelBlockGrid& g) {
  VKC_ASSIGN(std::vector<vol::BlockIndex> active,
             g.map().compact_active_blocks());
  std::map<Coord, std::int32_t> out;
  for (const vol::BlockIndex& b : active) {
    out[Coord{b.coord.x, b.coord.y, b.coord.z}] = b.ptr;
  }
  return out;
}

// The same blocks, and in each the same bits of weight, tsdf and colour in
// every voxel. A slot can differ: allocation order is the GPU's.
// The blocks of `g` stamped changed after `since`.
vkc::Result<std::uint32_t> changed_after(const vol::VoxelBlockGrid& g,
                                         std::uint32_t since) {
  VKC_ASSIGN(const std::vector<vol::BlockStamp> stamps,
             g.map().read_block_stamps());
  std::uint32_t n = 0;
  for (const vol::BlockStamp& st : stamps) {
    n += vol::tick_after(st.changed, since) ? 1u : 0u;
  }
  return n;
}

int check_same(const vr_test::Gpu& ctx, vol::VoxelBlockGrid& a,
               vol::VoxelBlockGrid& b) {
  auto ba = blocks_of(a);
  auto bb = blocks_of(b);
  CHECK(ba.ok() && bb.ok());
  CHECK(!ba.value().empty());
  CHECK(ba.value().size() == bb.value().size());
  const auto view = [&](vol::VoxelBlockGrid& g, const char* name) {
    return vr_test::read_attribute<std::uint32_t>(ctx.device, ctx.allocator, g,
                                                  name)
        .value();
  };
  const std::vector<std::uint32_t> wa = view(a, "weight");
  const std::vector<std::uint32_t> wb = view(b, "weight");
  const std::vector<std::uint32_t> ta = view(a, "tsdf");
  const std::vector<std::uint32_t> tb = view(b, "tsdf");
  const std::vector<std::uint32_t> ca = view(a, "color");
  const std::vector<std::uint32_t> cb = view(b, "color");
  const std::size_t voxels = grid_params().voxels_per_block;
  std::size_t observed = 0;
  for (const auto& [coord, ptr] : ba.value()) {
    const auto other = bb.value().find(coord);
    CHECK(other != bb.value().end());
    for (std::size_t k = 0; k < voxels; ++k) {
      const std::size_t ia = static_cast<std::size_t>(ptr) + k;
      const std::size_t ib = static_cast<std::size_t>(other->second) + k;
      CHECK(wa[ia] == wb[ib]);
      CHECK(ta[ia] == tb[ib]);
      CHECK(ca[ia] == cb[ib]);
      observed += wa[ia] != 0 ? 1 : 0;
    }
  }
  std::printf("  %zu blocks, %zu voxels observed\n", ba.value().size(),
              observed);
  CHECK(observed > 10000);
  return 0;
}

// Allocate, retrying rounds that only lost bucket-lock races (as
// examples/common/fuse_frame.hpp does).
template <typename Allocate>
int settle(Allocate&& allocate) {
  for (int round = 0; round < 5; ++round) {
    vol::AllocFailures why;
    auto failed = allocate(&why);
    CHECK(failed.ok() && !why.capacity_limited());
    if (failed.value() == 0) return 0;
  }
  std::fprintf(stderr, "FAIL: allocation kept losing lock races\n");
  return 1;
}

}  // namespace

int main() {
  vkc::Result<vkc::Instance> instance = vkc::Instance::create({});
  if (!instance) {
    return vr_test::no_device("no Vulkan instance",
                              instance.status().message());
  }
  vkc::Result<vkc::PhysicalDeviceInfo> gpu =
      instance.value().select_physical_device(vr::device_requirements());
  if (!gpu) {
    return vr_test::no_device("no compute-capable device",
                              gpu.status().message());
  }
  vkc::Result<vkc::Device> device = vkc::Device::create(
      instance.value(), gpu.value(), vr::device_requirements());
  CHECK(device.ok());
  vkc::Result<vkc::Allocator> allocator =
      vkc::Allocator::create(instance.value().handle(), device.value());
  CHECK(allocator.ok());
  vkc::Device& dev = device.value();
  vkc::Allocator& alloc = allocator.value();
  const vr_test::Gpu ctx{dev, alloc};

  std::vector<View> views;
  for (int c = 0; c < kCameras; ++c) views.push_back(make_view(c));
  // Camera 0 marks its coverage: a band of its colour says it has none.
  for (std::uint32_t row = 0; row < kHeight; ++row) {
    for (std::uint32_t col = 60; col < 80; ++col) {
      views[0].color[row * kWidth + col] = 0u;
    }
  }
  // Cameras 0 and 2 are on the device, 1 on the host; 2 has no colour.
  std::vector<vkc::Buffer> depth_bufs;
  for (int c = 0; c < kCameras; ++c) {
    auto buf =
        vr_test::upload_device_buffer(dev, alloc, views[c].depth.data(),
                                      views[c].depth.size() * sizeof(float));
    CHECK(buf.ok());
    CHECK(buf.value().is_device_local() && buf.value().mapped() == nullptr);
    depth_bufs.push_back(std::move(buf).value());
  }
  auto color0 = vr_test::upload_device_buffer(
      dev, alloc, views[0].color.data(),
      views[0].color.size() * sizeof(std::uint32_t));
  CHECK(color0.ok());
  const auto color_cam = [&](const View& v) {
    return vr::ColorCameraParams{v.cam.fx,          v.cam.fy,    v.cam.cx,
                                 v.cam.cy,          v.cam.width, v.cam.height,
                                 v.cam.cam_to_world};
  };
  tsdf::ColorFrame c0{};
  c0.buffer = &color0.value();
  c0.cam = color_cam(views[0]);
  c0.coverage_in_alpha = true;
  tsdf::ColorFrame c1{views[1].color.data(), color_cam(views[1]), {}};

  const std::vector<tsdf::FrameInput> frames{
      {{vkc::StorageInput(depth_bufs[0]), views[0].cam}, &c0},
      {{vkc::StorageInput(views[1].depth.data()), views[1].cam}, &c1},
      {{vkc::StorageInput(depth_bufs[2]), views[2].cam}, nullptr}};
  // The set, with an empty frame second, and the list it allocates from.
  vr::DepthCameraParams empty_cam = views[1].cam;
  empty_cam.width = 0;
  std::vector<tsdf::FrameInput> set_frames = frames;
  set_frames.insert(
      set_frames.begin() + 1,
      tsdf::FrameInput{{vkc::StorageInput(views[1].depth.data()), empty_cam},
                       &c1});
  const std::vector<vol::DepthInput> depths(set_frames.begin(),
                                            set_frames.end());

  auto integrator = tsdf::TsdfIntegrator::create(dev, alloc);
  CHECK(integrator.ok());
  for (const tsdf::IntegrationMode mode :
       {tsdf::IntegrationMode::Classic, tsdf::IntegrationMode::Dynamic}) {
    auto one = make_grid(dev, alloc);
    auto set = make_grid(dev, alloc);
    CHECK(one.ok() && set.ok());
    // Twice, so the running average and the colour blend run too. Each pass
    // allocates every frame's band before fusing any, as the set does, and
    // counts the blocks each grid stamped changed after its tick going in:
    // four ticks a pass for one frame at a time, one for the set.
    for (int pass = 0; pass < 2; ++pass) {
      const std::uint32_t one_since = one->map().tick();
      const std::uint32_t set_since = set->map().tick();
      for (int c = 0; c < kCameras; ++c) {
        CHECK(settle([&](vol::AllocFailures* why) {
                return c == 1 ? one->map().allocate_from_depth(
                                    views[1].depth.data(), views[1].cam, why)
                              : one->map().allocate_from_depth(
                                    depth_bufs[c], views[c].cam, why);
              }) == 0);
      }
      for (int c = 0; c < kCameras; ++c) {
        const tsdf::FrameInput& f = frames[c];
        const vkc::Status fused =
            c == 1 ? integrator->integrate(one.value(), views[1].depth.data(),
                                           f.camera, 5.0f, mode, f.color)
                   : integrator->integrate(one.value(), depth_bufs[c], f.camera,
                                           5.0f, mode, f.color);
        CHECK(fused.ok());
      }
      CHECK(one->map().tick() == one_since + std::uint32_t{kCameras});
      auto dirty_one = changed_after(one.value(), one_since);
      CHECK(dirty_one.ok() && dirty_one.value() > 0);
      if (pass == 0) {
        // One frame first, so the full set grows the map's sets.
        CHECK(settle([&](vol::AllocFailures* why) {
                return set->map().allocate_from_depth(
                    std::vector<vol::DepthInput>{depths[0]}, why);
              }) == 0);
      }
      CHECK(settle([&](vol::AllocFailures* why) {
              return set->map().allocate_from_depth(depths, why);
            }) == 0);
      CHECK(integrator->integrate(set.value(), set_frames, 5.0f, mode).ok());
      CHECK(set->map().tick() == set_since + 1);
      auto dirty_set = changed_after(set.value(), set_since);
      CHECK(dirty_set.ok());
      CHECK(dirty_set.value() == dirty_one.value());
    }
    std::printf("%s:\n",
                mode == tsdf::IntegrationMode::Classic ? "classic" : "dynamic");
    if (check_same(ctx, one.value(), set.value()) != 0) return 1;
  }

  // A set with one bad frame -- here the last, its depth smaller than its
  // image -- is refused before anything is allocated or fused.
  {
    auto grid = make_grid(dev, alloc);
    CHECK(grid.ok());
    auto small = vr_test::upload_device_buffer(
        dev, alloc, views[2].depth.data(), sizeof(float) * kWidth);
    CHECK(small.ok());
    std::vector<vol::DepthInput> bad_depths(frames.begin(), frames.end());
    bad_depths[2] = {vkc::StorageInput(small.value()), views[2].cam};
    CHECK(grid->map().allocate_from_depth(bad_depths).status().domain() ==
          vkc::Status::Code::InvalidArgument);
    auto none = blocks_of(grid.value());
    CHECK(none.ok() && none.value().empty());

    CHECK(settle([&](vol::AllocFailures* why) {
            return grid->map().allocate_from_depth(depths, why);
          }) == 0);
    std::vector<tsdf::FrameInput> bad = frames;
    bad[2] = {{vkc::StorageInput(small.value()), views[2].cam}, nullptr};
    CHECK(integrator->integrate(grid.value(), bad).domain() ==
          vkc::Status::Code::InvalidArgument);
    auto weight =
        vr_test::read_attribute<float>(dev, alloc, grid.value(), "weight");
    CHECK(weight.ok());
    for (const float w : weight.value()) CHECK(w == 0.0f);

    // And an empty set does nothing, without refusing.
    CHECK(grid->map().allocate_from_depth(std::vector<vol::DepthInput>{}).ok());
    CHECK(integrator->integrate(grid.value(), std::vector<tsdf::FrameInput>{})
              .ok());
  }

  std::puts("tsdf_integrate_set: OK");
  return 0;
}
