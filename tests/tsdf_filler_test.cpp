// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// A filler (tsdf::FrameRole::Filler) through tsdf::Fuser, on small
// ray-cast scenes (fill_scenes.hpp):
//
// - I5, tiers in every order: an Unanchored frame's band starts unanchored
//   fill, an Anchored one's anchored fill; an Anchored band sample restarts
//   unanchored fill as anchored, an Unanchored frame never writes anchored
//   fill, neither writes a fixed camera's voxel, and a filler's free space
//   goes only into fill of its tier or below; its weight stops at the cap
//   and is never lowered; returns past 3 m fill nothing;
// - each veto: a fixed band leaves fill as the fixed frame alone leaves it;
//   a fixed camera seeing past a filler's face keeps only its free side; a
//   fixed camera's holes and an occluded fixed camera veto nothing;
// - I1 with a filler: a rig with a box that comes and goes, and a filler
//   that sees the same room 1 cm off and an object only it sees: every voxel
//   no filler wrote is, as the box comes and goes and at the end, the voxel
//   of the same rig run without the filler, bit for bit, its byte too;
// - the refusals.
//
// Skips (exit 0) where no device is present.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <vector>

#include "fill_scenes.hpp"
#include "volumetric_kit/core/testing/gpu_test.hpp"
#include "volumetric_kit/core/testing/test_check.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"
#include "volumetric_kit/recon/tsdf/fuser.hpp"
#include "volumetric_kit/recon/tsdf/fusion_grid.hpp"
#include "volumetric_kit/recon/tsdf/source.hpp"

using namespace fill_test;

namespace {

constexpr std::int32_t kBuckets = 64;
constexpr float kFace = 1.5f;
constexpr float kWall = 2.0f;
constexpr auto kAnchored = tsdf::FillTier::Anchored;
constexpr auto kUnanchored = tsdf::FillTier::Unanchored;
const std::uint8_t kA = tsdf::mobile_source(0, kAnchored);
const std::uint8_t kU = tsdf::mobile_source(0, kUnanchored);
constexpr tsdf::IntegrationMode kDynamic = tsdf::IntegrationMode::Dynamic;

struct Rig {
  vol::VoxelBlockGrid grid;
  tsdf::Fuser fuser;
};

vkc::Result<Rig> make_rig(vkc::test::GpuContext& gpu) {
  tsdf::FuserConfig config;
  config.change.warm_up = 2;
  VKC_ASSIGN(vol::VoxelBlockGrid grid,
             tsdf::create_fusion_grid(gpu.device, gpu.allocator, kVoxel, kTrunc,
                                      kBuckets));
  VKC_ASSIGN(tsdf::Fuser fuser,
             tsdf::Fuser::create(gpu.device, gpu.allocator, config));
  return Rig{std::move(grid), std::move(fuser)};
}

vkc::Status fuse(Rig& rig, const std::vector<tsdf::FrameInput>& frames,
                 tsdf::IntegrationMode mode = kDynamic) {
  return rig.fuser.fuse(rig.grid, frames, kMaxWeight, mode).status();
}

// The interior of a 0.6 x 0.5 m rectangle at the face, away from outlines,
// inside the band's front 3 cm and back 2 cm.
bool interior(const vr::Vec3f& p, float z = kFace) {
  return std::fabs(p.x) < 0.2f && std::fabs(p.y) < 0.15f && p.z > z - 0.03f &&
         p.z < z + 0.02f;
}

// The voxels of `v` in the face's interior: how many, how many hold `byte`,
// and the least and most weight among them.
struct Census {
  std::size_t n = 0, with_byte = 0, empty = 0;
  float least = std::numeric_limits<float>::infinity(), most = 0.0f;
};
Census census(const Voxels& v, std::uint8_t byte, float z = kFace) {
  Census c;
  for (std::size_t k = 0; k < v.index.size(); ++k) {
    if (!interior(v.world[k], z)) continue;
    const std::size_t i = v.index[k];
    ++c.n;
    c.with_byte += v.source[i] == byte ? 1 : 0;
    c.empty += v.weight[i] > 0.0f ? 0 : 1;
    if (v.weight[i] > 0.0f) {
      c.least = std::min(c.least, v.weight[i]);
      c.most = std::max(c.most, v.weight[i]);
    }
  }
  return c;
}

// The filler's frames of a face at `z` in front of nothing, from the origin.
std::vector<float> face_depth(float z = kFace) {
  return render(camera(), {{z, 0.3f, 0.25f}});
}

// A tier order: the frames' tiers, one set each, no fixed camera.
vkc::Result<Voxels> after(vkc::test::GpuContext& gpu,
                          const std::vector<tsdf::FillTier>& tiers,
                          Voxels* before_last = nullptr) {
  VKC_ASSIGN(Rig rig, make_rig(gpu));
  const std::vector<float> depth = face_depth();
  for (std::size_t s = 0; s < tiers.size(); ++s) {
    if (before_last != nullptr && s + 1 == tiers.size()) {
      VKC_ASSIGN(*before_last, read_voxels(gpu, rig.grid));
    }
    VKC_TRY(fuse(rig, {filler(depth, camera(), tiers[s])}));
  }
  return read_voxels(gpu, rig.grid);
}

int tier_case(vkc::test::GpuContext& gpu) {
  // A sample's most weight in the interior, 1 / z^2 at its nearest voxel,
  // and the cap, 0.5 x 5.
  const float w = 1.0f / ((kFace - 0.03f) * (kFace - 0.03f));
  const float cap = 0.5f * kMaxWeight;
  {
    auto v = after(gpu, {kUnanchored});
    CHECK(v.ok() && byte_weight_mismatches(v.value()) == 0);
    const Census c = census(v.value(), kU);
    CHECK(c.n > 500 && c.with_byte == c.n);
  }
  {
    // Unanchored, then Anchored: restarted as anchored, from one sample.
    Voxels before;
    auto v = after(gpu, {kUnanchored, kUnanchored, kAnchored}, &before);
    CHECK(v.ok() && byte_weight_mismatches(v.value()) == 0);
    const Census c = census(v.value(), kA);
    std::size_t lower = 0;
    for (std::size_t k = 0; k < v->index.size(); ++k) {
      const std::size_t i = v->index[k];
      if (interior(v->world[k]) && v->weight[i] < at(before.weight, i)) {
        ++lower;
      }
    }
    std::printf(
        "tiers U, U, A: %zu of %zu restarted as anchored, %zu "
        "lower\n",
        c.with_byte, c.n, lower);
    CHECK(c.n > 500 && c.with_byte == c.n && lower == c.n);
    CHECK(c.most <= w * 1.0001f);
  }
  {
    // Anchored, then Unanchored: the unanchored frames write nothing.
    Voxels before;
    auto v = after(gpu, {kAnchored, kUnanchored}, &before);
    CHECK(v.ok());
    for (const std::size_t i : v->index) {
      CHECK(v->tsdf[i] == at(before.tsdf, i) &&
            v->weight[i] == at(before.weight, i) &&
            v->source[i] == at(before.source, i));
    }
    const Census c = census(v.value(), kA);
    CHECK(c.n > 500 && c.with_byte == c.n);
  }
  {
    // Same tier twice: they average, each byte kept.
    for (const tsdf::FillTier t : {kAnchored, kUnanchored}) {
      auto v = after(gpu, {t, t});
      CHECK(v.ok());
      const Census c = census(v.value(), t == kAnchored ? kA : kU);
      CHECK(c.n > 500 && c.with_byte == c.n);
      CHECK(c.most > w && c.most <= 2.0f * w * 1.0001f);
    }
  }
  {
    // Twenty sets: the cap.
    auto v = after(gpu, std::vector<tsdf::FillTier>(20, kAnchored));
    CHECK(v.ok());
    const Census c = census(v.value(), kA);
    CHECK(c.most == cap);
  }
  {
    // Fill above the cap (fixed samples raised it) is never lowered.
    auto rig = make_rig(gpu);
    CHECK(rig.ok());
    auto seeded = seed(gpu, rig->grid, {-0.25f, -0.2f, kFace - kTrunc},
                       {0.25f, 0.2f, kFace + kTrunc}, face(kFace), kA, 4.0f);
    CHECK(seeded.ok() && seeded->size() > 500);
    CHECK(fuse(rig.value(), {filler(face_depth(), camera(), kAnchored)}).ok());
    auto v = read_voxels(gpu, rig->grid);
    CHECK(v.ok());
    for (const std::size_t i : seeded.value()) CHECK(v->weight[i] == 4.0f);
  }
  {
    // A fixed camera's voxels: a filler writes none of them.
    auto rig = make_rig(gpu);
    CHECK(rig.ok());
    const std::vector<float> depth = face_depth();
    CHECK(fuse(rig.value(), {fixed(depth, camera(), 0)}).ok());
    auto before = read_voxels(gpu, rig->grid);
    for (const tsdf::FillTier t : {kAnchored, kUnanchored}) {
      CHECK(fuse(rig.value(), {filler(depth, camera(), t)}).ok());
    }
    auto v = read_voxels(gpu, rig->grid);
    CHECK(before.ok() && v.ok());
    std::size_t rig_voxels = 0;
    for (const std::size_t i : before->index) {
      rig_voxels += before->source[i] == tsdf::fixed_source(0) ? 1 : 0;
      CHECK(v->tsdf[i] == before->tsdf[i] &&
            v->weight[i] == before->weight[i] &&
            v->source[i] == before->source[i]);
    }
    CHECK(rig_voxels > 500);
  }
  {
    // Free space: an Anchored frame's averages into unanchored fill and
    // leaves its byte; an Unanchored frame's leaves anchored fill alone;
    // neither writes an empty voxel.
    for (const tsdf::FillTier first : {kUnanchored, kAnchored}) {
      auto rig = make_rig(gpu);
      CHECK(rig.ok());
      CHECK(fuse(rig.value(), {filler(face_depth(), camera(), first)}).ok());
      auto before = read_voxels(gpu, rig->grid);
      const tsdf::FillTier then =
          first == kUnanchored ? kAnchored : kUnanchored;
      CHECK(
          fuse(rig.value(), {filler(face_depth(kWall), camera(), then)}).ok());
      auto v = read_voxels(gpu, rig->grid);
      CHECK(before.ok() && v.ok() && byte_weight_mismatches(v.value()) == 0);
      std::size_t moved = 0, wall = 0, n = 0;
      for (std::size_t k = 0; k < v->index.size(); ++k) {
        const std::size_t i = v->index[k];
        if (interior(v->world[k])) {
          ++n;
          CHECK(v->source[i] == at(before->source, i));
          moved += v->tsdf[i] != at(before->tsdf, i) ? 1 : 0;
        } else if (interior(v->world[k], kWall)) {
          wall += v->weight[i] > 0.0f ? 1 : 0;
        }
      }
      std::printf(
          "free space of %s over %s fill: %zu of %zu moved; %zu "
          "written at the wall\n",
          then == kAnchored ? "Anchored" : "Unanchored",
          first == kAnchored ? "anchored" : "unanchored", moved, n, wall);
      CHECK(n > 500 && wall > 500);  // the wall itself is the filler's band
      CHECK(first == kUnanchored ? moved == n : moved == 0);
    }
  }
  {
    // Returns past 3 m fill nothing; nearer ones do.
    for (const float z : {3.5f, 2.9f}) {
      auto rig = make_rig(gpu);
      CHECK(rig.ok());
      CHECK(
          fuse(rig.value(), {filler(face_depth(z), camera(), kAnchored)}).ok());
      auto v = read_voxels(gpu, rig->grid);
      CHECK(v.ok());
      const Census c = census(v.value(), kA, z);
      CHECK(z > 3.0f ? c.with_byte == 0 : (c.n > 100 && c.with_byte == c.n));
    }
  }
  return 0;
}

// Each veto, fixed camera 0 at the origin beside the filler.
int veto_case(vkc::test::GpuContext& gpu) {
  const sensor::DepthCameraParams f = camera();
  {
    // A fixed band: the set leaves fill as the fixed frame alone does.
    auto x = make_rig(gpu);
    auto y = make_rig(gpu);
    CHECK(x.ok() && y.ok());
    const std::vector<float> depth = face_depth();
    for (Rig* r : {&x.value(), &y.value()}) {
      CHECK(fuse(*r, {filler(depth, camera(), kAnchored)}).ok());
    }
    for (int set = 0; set < 4; ++set) {
      CHECK(fuse(x.value(),
                 {fixed(depth, f, 0), filler(depth, camera(), kAnchored)})
                .ok());
      CHECK(fuse(y.value(), {fixed(depth, f, 0)}).ok());
      CHECK(vgrid_test::same_grids(gpu, x->grid, y->grid));
      auto vx = read_voxels(gpu, x->grid);
      auto vy = read_voxels(gpu, y->grid);
      CHECK(vx.ok() && vy.ok() && same_sources(vx.value(), vy.value()));
      CHECK(census(vx.value(), kA).with_byte > 500);
    }
  }
  {
    // Seen past: the filler's face at 1.5 m before a fixed camera's wall at
    // 2 m keeps only its free side; the fixed camera's holes over the face
    // veto nothing.
    for (const bool holes : {false, true}) {
      auto rig = make_rig(gpu);
      CHECK(rig.ok());
      std::vector<Rect> wall{{kWall}};
      if (holes) wall.insert(wall.begin(), {kWall, 0.35f, 0.3f, 0, 0, true});
      const std::vector<float> fd = render(f, wall);
      for (int set = 0; set < 2; ++set) {
        CHECK(fuse(rig.value(),
                   {fixed(fd, f, 0), filler(face_depth(), camera(), kAnchored)})
                  .ok());
      }
      auto v = read_voxels(gpu, rig->grid);
      CHECK(v.ok() && byte_weight_mismatches(v.value()) == 0);
      std::size_t front = 0, front_fill = 0, back = 0, back_fill = 0;
      for (std::size_t k = 0; k < v->index.size(); ++k) {
        if (!interior(v->world[k])) continue;
        const std::size_t i = v->index[k];
        const bool fill = v->source[i] == kA;
        // In front of the face: its sample there is positive.
        if (v->world[k].z < kFace) {
          ++front;
          front_fill += fill ? 1 : 0;
        } else {
          ++back;
          back_fill += fill ? 1 : 0;
        }
      }
      std::printf(
          "veto, fixed camera %s: front %zu of %zu fill, back %zu of "
          "%zu\n",
          holes ? "over holes" : "seeing past", front_fill, front, back_fill,
          back);
      CHECK(front > 200 && front_fill == front);
      CHECK(back > 200 && back_fill == (holes ? back : 0));
    }
  }
  {
    // Occluded: behind a 0.2 m occluder at 1 m, which the fixed camera sees,
    // the fixed camera vetoes nothing; the filler, 0.4 m to the side, sees
    // the face there past the occluder and fills its whole band.
    auto rig = make_rig(gpu);
    CHECK(rig.ok());
    const Rect occluder{1.0f, 0.1f, 0.1f};
    const std::vector<float> fd = render(f, {occluder, {kWall}});
    const sensor::DepthCameraParams p = camera({0.4f, 0.0f, 0.0f});
    const std::vector<float> pd =
        render(p, {occluder, {kFace, 0.3f, 0.25f}, {kWall}});
    for (int set = 0; set < 2; ++set) {
      CHECK(
          fuse(rig.value(), {fixed(fd, f, 0), filler(pd, p, kAnchored)}).ok());
    }
    auto v = read_voxels(gpu, rig->grid);
    CHECK(v.ok());
    std::size_t n = 0, fill = 0;
    for (std::size_t k = 0; k < v->index.size(); ++k) {
      const vr::Vec3f& q = v->world[k];
      if (!(q.x > 0.0f && q.x < 0.08f && std::fabs(q.y) < 0.05f &&
            interior(q))) {
        continue;
      }
      ++n;
      fill += v->source[v->index[k]] == kA ? 1 : 0;
    }
    std::printf("veto, fixed camera occluded: %zu of %zu fill\n", fill, n);
    CHECK(n > 100 && fill == n);
  }
  return 0;
}

// I1: every voxel no filler wrote, bit for bit the rig's run without one.
int invariant_case(vkc::test::GpuContext& gpu) {
  auto x = make_rig(gpu);
  auto y = make_rig(gpu);
  CHECK(x.ok() && y.ok());
  const sensor::DepthCameraParams a = camera();
  const sensor::DepthCameraParams b = camera({0.3f, 0.05f, 0.0f});
  const sensor::DepthCameraParams p = camera({-0.1f, -0.05f, 0.4f});
  std::size_t fill_most = 0, compared = 0, extra_rig = 0;
  for (int set = 0; set < 18; ++set) {
    std::vector<Rect> scene{{kWall}};
    if (set >= 5 && set < 12) {
      scene.insert(scene.begin(), {kFace, 0.2f, 0.15f, 0.05f, 0.0f});
    }
    const std::vector<float> da = render(a, scene);
    const std::vector<float> db = render(b, scene);
    // The filler: the room 1 cm off, a box only it sees, Anchored and
    // Unanchored by turns.
    std::vector<Rect> seen = scene;
    for (Rect& r : seen) r.z += 0.01f;
    seen.insert(seen.begin(), {1.2f, 0.06f, 0.05f, -0.15f, 0.1f});
    const std::vector<float> dp = render(p, seen);
    const tsdf::FillTier tier = set % 2 == 0 ? kAnchored : kUnanchored;
    CHECK(
        fuse(x.value(), {fixed(da, a, 0), fixed(db, b, 1), filler(dp, p, tier)})
            .ok());
    CHECK(fuse(y.value(), {fixed(da, a, 0), fixed(db, b, 1)}).ok());
    // As the box comes and the set after, as it goes and the set after, and
    // at the end: what differs once stays different.
    if (set != 5 && set != 6 && set != 12 && set != 13 && set != 17) continue;
    auto vx = read_voxels(gpu, x->grid);
    auto vy = read_voxels(gpu, y->grid);
    CHECK(vx.ok() && vy.ok());
    CHECK(byte_weight_mismatches(vx.value()) == 0);
    // Y's voxel at each of X's; X's block set holds Y's.
    const std::vector<std::size_t> in_x = match(vy.value(), vx.value());
    CHECK(std::find(in_x.begin(), in_x.end(), kNone) == in_x.end());
    const std::vector<std::size_t> in_y = match(vx.value(), vy.value());
    std::size_t fill = 0, diff = 0;
    for (std::size_t k = 0; k < vx->index.size(); ++k) {
      const std::size_t i = vx->index[k];
      const bool is_fill = (vx->source[i] & tsdf::kMobileBit) != 0;
      fill += is_fill ? 1 : 0;
      if (is_fill) continue;
      const std::size_t j = in_y[k];
      if (j == kNone) {
        // A block only the filler's band allocated: a fixed camera may
        // write there what its own allocation left out (an outline's
        // rim distance), which Y has no block for.
        extra_rig += vx->weight[i] > 0.0f ? 1 : 0;
        continue;
      }
      ++compared;
      diff += vx->tsdf[i] != vy->tsdf[j] || vx->weight[i] != vy->weight[j] ||
                      vx->color[i] != vy->color[j] ||
                      vx->source[i] != vy->source[j]
                  ? 1
                  : 0;
    }
    if (diff != 0) {
      std::fprintf(stderr, "I1, set %d: %zu voxels no filler wrote differ\n",
                   set, diff);
      return 1;
    }
    fill_most = std::max(fill_most, fill);
  }
  std::printf(
      "I1: %zu voxel comparisons at 5 of 18 sets, none differs; up to "
      "%zu fill voxels; %zu fixed samples in blocks only the filler "
      "allocated\n",
      compared, fill_most, extra_rig);
  CHECK(fill_most > 1000);
  return 0;
}

int refusal_case(vkc::test::GpuContext& gpu) {
  auto rig = make_rig(gpu);
  CHECK(rig.ok());
  const std::vector<float> depth = face_depth();
  const sensor::DepthCameraParams c = camera();
  const auto refused = [&](vol::VoxelBlockGrid& grid,
                           const std::vector<tsdf::FrameInput>& frames,
                           tsdf::IntegrationMode mode) {
    auto r = rig->fuser.fuse(grid, frames, kMaxWeight, mode);
    return !r.ok() && r.status().domain() == vkc::Status::Code::InvalidArgument;
  };
  for (const tsdf::IntegrationMode mode :
       {kDynamic, tsdf::IntegrationMode::Classic}) {
    tsdf::FrameInput f = filler(depth, c, kAnchored);
    f.slot = tsdf::kMaxSourcesPerClass;
    CHECK(refused(rig->grid, {f}, mode));
    CHECK(
        refused(rig->grid, {fixed(depth, c, tsdf::kMaxSourcesPerClass)}, mode));
    for (const float cap : {0.0f, 1.5f, std::nanf("")}) {
      f = filler(depth, c, kAnchored);
      f.fill.cap_fraction = cap;
      CHECK(refused(rig->grid, {f}, mode));
    }
    for (const float far : {0.0f, std::numeric_limits<float>::infinity()}) {
      f = filler(depth, c, kAnchored);
      f.fill.max_depth_m = far;
      CHECK(refused(rig->grid, {f}, mode));
    }
    f = fixed(depth, c, 0);
    f.role = static_cast<tsdf::FrameRole>(7);
    CHECK(refused(rig->grid, {f}, mode));
  }
  // A filler needs the byte.
  const vol::AttributeSpec three[] = {{vol::kTsdfAttribute, sizeof(float)},
                                      {vol::kWeightAttribute, sizeof(float)},
                                      {vol::kColorAttribute, 4}};
  auto bare = vol::VoxelBlockGrid::create(
      gpu.device, gpu.allocator, vol::block_grid_params(kVoxel, kTrunc, 64),
      three, 3);
  CHECK(bare.ok());
  CHECK(refused(bare.value(), {filler(depth, c, kAnchored)}, kDynamic));
  // A filler needs Dynamic: Classic's free space would start voxels in the
  // blocks its band allocates.
  CHECK(refused(rig->grid, {filler(depth, c, kAnchored)},
                tsdf::IntegrationMode::Classic));
  // A filler shares index 0 with a fixed frame and states no noise: it
  // takes no change test.
  CHECK(fuse(rig.value(), {fixed(depth, c, 0), filler(depth, c, kAnchored, 0)})
            .ok());
  return 0;
}

}  // namespace

int gpu_main(vkc::test::GpuContext& gpu) {
  if (tier_case(gpu) != 0) return 1;
  if (veto_case(gpu) != 0) return 1;
  if (invariant_case(gpu) != 0) return 1;
  if (refusal_case(gpu) != 0) return 1;
  std::puts("tsdf_filler: OK");
  return 0;
}

int main() {
  return vkc::test::run_on_gpu(vr::device_requirements(), gpu_main);
}
