// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The source byte (tsdf/source.hpp) and how fixed cameras treat fill, through
// tsdf::Fuser, with fill seeded straight into the grid (fill_scenes.hpp):
//
// - the value table and the two cast filters, value by value;
// - the byte costs the rig nothing: a rig of two cameras with a box that
//   comes and goes, fused into a grid with the byte and one without, is the
//   same grid bit for bit at every set, in either mode, and every weighted
//   voxel holds the byte of the camera that started it, every other 0;
// - I2, no standing, no change: one fixed camera leaves fill bit-identical
//   over its holes and an outline beside them, beside a departure (rule 2),
//   on a Vanished ray and behind an occluder that moves; the same voxels
//   seeded as the rig's change (but behind the occluder), so each case is
//   live;
// - I3, removal needs change and contradiction: fill seen past once its
//   occluder leaves loses exactly its tsdf <= 0 side in that set; an arrival
//   on a fill face starts the face as the rig's in that set; a surface 2 cm
//   behind the face that appears (Farther, inside the band) removes nothing;
//   a quiet camera seeing past fill never lowers its weight;
// - I4, noise under a band: a camera 2 cm behind a fill face, noisy, with
//   injected alarms of each kind, keeps the face extractable;
// - a fixed sample writes only its own byte, the word's other bytes kept.
//
// Skips (exit 0) where no device is present.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>

#include "fill_scenes.hpp"
#include "volumetric_kit/core/testing/gpu_test.hpp"
#include "volumetric_kit/core/testing/test_check.hpp"
#include "volumetric_kit/grid/mesh/marching_cubes.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"
#include "volumetric_kit/recon/tsdf/fuser.hpp"
#include "volumetric_kit/recon/tsdf/fusion_grid.hpp"
#include "volumetric_kit/recon/tsdf/source.hpp"

namespace mesh = volumetric_kit::grid::mesh;
using namespace fill_test;

namespace {

constexpr std::int32_t kBuckets = 64;
// The fill's face and its rectangle, in front of a wall at 2 m.
constexpr float kFace = 1.5f;
constexpr float kWall = 2.0f;
constexpr float kHalfX = 0.2f;
constexpr float kHalfY = 0.15f;
const std::uint8_t kFill = tsdf::mobile_source(0, tsdf::FillTier::Anchored);
const std::uint8_t kRigByte = tsdf::fixed_source(0);

tsdf::FuserConfig fuser_config() {
  tsdf::FuserConfig config;
  config.change.warm_up = 2;  // short sequences
  return config;
}

bool allowed(const vol::CastFilter& f, std::uint32_t v) {
  return ((f.allow[v >> 5] >> (v & 31u)) & 1u) != 0u;
}

int table_case() {
  CHECK(tsdf::fixed_source(0) == 1 && tsdf::fixed_source(62) == 63);
  CHECK(tsdf::mobile_source(0, tsdf::FillTier::Anchored) == 0x81);
  CHECK(tsdf::mobile_source(62, tsdf::FillTier::Anchored) == 0xBF);
  CHECK(tsdf::mobile_source(0, tsdf::FillTier::Unanchored) == 0xC1);
  CHECK(tsdf::mobile_source(62, tsdf::FillTier::Unanchored) == 0xFF);
  const vol::CastFilter rig = tsdf::rig_cast();
  const vol::CastFilter track = tsdf::tracking_cast(1.5f);
  CHECK(rig.attribute == tsdf::kSourceAttribute && rig.min_weight == 2.0f);
  CHECK(track.attribute == tsdf::kSourceAttribute && track.min_weight == 1.5f);
  for (std::uint32_t v = 0; v < 256; ++v) {
    const bool fixed = v >= 1 && v <= 63;
    const bool anchored = v >= 129 && v <= 191;
    CHECK(allowed(rig, v) == fixed);
    CHECK(allowed(track, v) == (fixed || anchored));
  }
  return 0;
}

// A rig of two cameras, a box at 1.5 m that comes at set 6 and goes at 14,
// fused into a grid with the byte and one without: the same grid every set.
int rig_case(vkc::test::GpuContext& gpu, tsdf::IntegrationMode mode) {
  auto with = tsdf::create_fusion_grid(gpu.device, gpu.allocator, kVoxel,
                                       kTrunc, kBuckets);
  const vol::AttributeSpec three[] = {{vol::kTsdfAttribute, sizeof(float)},
                                      {vol::kWeightAttribute, sizeof(float)},
                                      {vol::kColorAttribute, 4}};
  auto without = vol::VoxelBlockGrid::create(
      gpu.device, gpu.allocator,
      vol::block_grid_params(
          kVoxel, kTrunc,
          tsdf::fusion_buckets(
              kVoxel, vkc::max_storage_buffer_range(gpu.device), kBuckets)),
      three, 3);
  auto fa = tsdf::Fuser::create(gpu.device, gpu.allocator, fuser_config());
  auto fb = tsdf::Fuser::create(gpu.device, gpu.allocator, fuser_config());
  CHECK(with.ok() && without.ok() && fa.ok() && fb.ok());
  const sensor::DepthCameraParams a = camera();
  const sensor::DepthCameraParams b = camera({0.3f, 0.05f, 0.0f});
  std::size_t reset = 0, other = 0;
  std::vector<float> before;
  for (int set = 0; set < 12; ++set) {
    std::vector<Rect> scene{{kWall}};
    if (set >= 3 && set < 8)
      scene.insert(scene.begin(), {kFace, kHalfX, kHalfY});
    const std::vector<float> da = render(a, scene);
    const std::vector<float> db = render(b, scene);
    const std::vector<tsdf::FrameInput> frames{fixed(da, a, 0),
                                               fixed(db, b, 1)};
    CHECK(fa->fuse(with.value(), frames, kMaxWeight, mode).ok());
    CHECK(fb->fuse(without.value(), frames, kMaxWeight, mode).ok());
    // Before and as the box comes, before and as it goes, and at the end.
    if (set != 2 && set != 3 && set != 7 && set != 8 && set != 11) continue;
    std::size_t observed = 0;
    CHECK(
        vgrid_test::same_grids(gpu, with.value(), without.value(), &observed));
    CHECK(observed > 1000);
    auto v = read_voxels(gpu, with.value());
    CHECK(v.ok());
    CHECK(byte_weight_mismatches(v.value()) == 0);
    for (const std::size_t i : v->index) {
      other += v->source[i] > 2 ? 1 : 0;
      if (i < before.size() && before[i] > 0.0f && !(v->weight[i] > 0.0f)) {
        ++reset;
      }
    }
    before = v->weight;
  }
  std::printf(
      "rig, %s: the same grid with and without the byte at 5 of 12 sets; "
      "%zu voxels reset to byte 0\n",
      mode == tsdf::IntegrationMode::Dynamic ? "Dynamic" : "Classic", reset);
  CHECK(other == 0);
  if (mode == tsdf::IntegrationMode::Dynamic) CHECK(reset > 100);
  return 0;
}

// One fixed camera's sequence over seeded fill, in a grid of its own: the
// scene at each set, and the set after whose fuse the fill is seeded.
struct Sequence {
  int sets = 0;
  int seed_after = -1;  // -1: before the first set
  std::vector<Rect> (*scene)(int set) = nullptr;
};

// Where `cam`'s nearest pixel to `p` is a hole of `depth`.
bool over_hole(const sensor::DepthCameraParams& cam,
               const std::vector<float>& depth, const vr::Vec3f& p) {
  const float u = cam.fx * p.x / p.z + cam.cx;
  const float v = cam.fy * p.y / p.z + cam.cy;
  const int x =
      std::min(std::max(int(std::floor(u + 0.5f)), 0), int(kWidth) - 1);
  const int y =
      std::min(std::max(int(std::floor(v + 0.5f)), 0), int(kHeight) - 1);
  return depth[std::size_t(y) * kWidth + std::size_t(x)] == 0.0f;
}

struct Outcome {
  std::size_t seeded = 0;
  std::size_t changed = 0;  // seeded voxels whose tsdf, weight or byte moved
  std::size_t reset = 0;    // seeded voxels at weight 0
  std::size_t bad = 0;      // byte/weight mismatches, all sets
  std::size_t mixed = 0;    // words holding fill beside a rig byte, at the end
};

// `seq` with fill (or the rig's byte) seeded as `face` over the rectangle
// in `depth_z`, only over the camera's holes when `holes_only`.
Outcome run(vkc::test::GpuContext& gpu, const Sequence& seq, float depth_z,
            std::uint8_t byte, bool holes_only) {
  Outcome out;
  auto grid = tsdf::create_fusion_grid(gpu.device, gpu.allocator, kVoxel,
                                       kTrunc, kBuckets);
  auto fuser = tsdf::Fuser::create(gpu.device, gpu.allocator, fuser_config());
  if (!grid.ok() || !fuser.ok()) return {0, 0, 0, 1};
  const sensor::DepthCameraParams c = camera();
  std::vector<std::size_t> seeded;
  Voxels at_seed;
  const auto do_seed = [&](int set) -> bool {
    const std::vector<float> depth = render(c, seq.scene(set < 0 ? 0 : set));
    auto s = seed(
        gpu, grid.value(), {-kHalfX, -kHalfY, depth_z - kTrunc},
        {kHalfX, kHalfY, depth_z + kTrunc},
        [&](const vr::Vec3f& p) -> std::optional<float> {
          if (holes_only && !over_hole(c, depth, p)) return std::nullopt;
          return face(depth_z)(p);
        },
        byte);
    auto v = read_voxels(gpu, grid.value());
    if (!s.ok() || !v.ok()) return false;
    seeded = s.value();
    at_seed = v.value();
    return true;
  };
  if (seq.seed_after < 0 && !do_seed(-1)) return {0, 0, 0, 1};
  for (int set = 0; set < seq.sets; ++set) {
    const std::vector<float> depth = render(c, seq.scene(set));
    if (!fuser
             ->fuse(grid.value(), {fixed(depth, c, 0)}, kMaxWeight,
                    tsdf::IntegrationMode::Dynamic)
             .ok()) {
      return {0, 0, 0, 1};
    }
    if (set == seq.seed_after && !do_seed(set)) return {0, 0, 0, 1};
    // Changed fill would stay changed: the last set says it.
    if (set != seq.sets - 1) continue;
    auto v = read_voxels(gpu, grid.value());
    if (!v.ok()) return {0, 0, 0, 1};
    out.bad += byte_weight_mismatches(v.value());
    out.mixed = mixed_words(v.value());
    for (const std::size_t i : seeded) {
      out.changed += v->tsdf[i] != at_seed.tsdf[i] ||
                             v->weight[i] != at_seed.weight[i] ||
                             v->source[i] != at_seed.source[i]
                         ? 1
                         : 0;
      out.reset += v->weight[i] > 0.0f ? 0 : 1;
    }
  }
  out.seeded = seeded.size();
  return out;
}

// I2's scenes: the camera at the origin; the wall at 2 m where named.
std::vector<Rect> holes_scene(int) {
  // A dark patch over the fill inside a ring of surface at its depth: the
  // fill's edge voxels see the ring's returns in their footprint, a rim
  // distance's case.
  return {{kFace, kHalfX, kHalfY, 0, 0, true}, {kFace, 0.3f, 0.25f}, {kWall}};
}
std::vector<Rect> departure_scene(int set) {
  // A box beside the dark patch leaves at set 8: Farther, the wall behind.
  std::vector<Rect> s{{kFace, kHalfX, kHalfY, 0, 0, true}};
  if (set < 8) s.push_back({kFace, 0.05f, kHalfY, kHalfX + 0.05f, 0.0f});
  s.push_back({kWall});
  return s;
}
std::vector<Rect> vanished_scene(int set) {
  // A box at 1.5 m over a patch that returns nothing, there from set 3 (after
  // the camera's warm-up, so its pixels' history is the box's) and gone from
  // set 8: Vanished at 9.
  const bool box = set >= 3 && set < 8;
  return {{kFace, kHalfX, kHalfY, 0, 0, !box}, {kWall}};
}
std::vector<Rect> occluder_scene(int set) {
  // An occluder at 1 m that steps to 0.9 m at set 8 (Nearer).
  return {{set < 8 ? 1.0f : 0.9f, 0.3f, 0.25f}, {kWall}};
}

int no_standing_case(vkc::test::GpuContext& gpu) {
  struct Case {
    const char* name;
    Sequence seq;
    float z;
    bool holes_only;
    bool live;  // the rig's twin changes
  };
  const Case cases[] = {
      {"holes and an outline", {10, -1, holes_scene}, kFace, true, true},
      {"beside a departure", {12, -1, departure_scene}, kFace, true, true},
      {"Vanished", {12, 8, vanished_scene}, 1.3f, true, true},
      {"occluded", {12, -1, occluder_scene}, kFace, false, false},
  };
  for (const Case& c : cases) {
    const Outcome fill = run(gpu, c.seq, c.z, kFill, c.holes_only);
    const Outcome rig = run(gpu, c.seq, c.z, kRigByte, c.holes_only);
    std::printf(
        "I2 %s: %zu fill voxels, %zu changed, %zu words beside a rig "
        "byte; the rig's twin: %zu changed, %zu reset\n",
        c.name, fill.seeded, fill.changed, fill.mixed, rig.changed, rig.reset);
    CHECK(fill.bad == 0 && rig.bad == 0);
    CHECK(fill.seeded > 100 && fill.seeded == rig.seeded);
    CHECK(fill.changed == 0);
    if (c.live) CHECK(rig.changed > 0);
    // A fixed sample's byte write keeps the word's other bytes: the holes'
    // fill, untouched, shares words with the ring's rig bytes beside it.
    if (&c == &cases[0]) CHECK(fill.mixed > 10);
  }
  return 0;
}

// I3: the occluder leaves at set 8 and the camera sees past the fill.
std::vector<Rect> seen_past_scene(int set) {
  std::vector<Rect> s;
  if (set < 8) s.push_back({1.0f, 0.3f, 0.25f});
  s.push_back({kWall});
  return s;
}
// I3: a box arrives on the face at set 8.
std::vector<Rect> arrival_scene(int set) {
  std::vector<Rect> s;
  if (set >= 8) s.push_back({kFace, 0.3f, 0.25f});
  s.push_back({kWall});
  return s;
}
// I3: the occluder leaves at set 8 to show a surface 2 cm behind the face.
std::vector<Rect> farther_band_scene(int set) {
  std::vector<Rect> s;
  if (set < 8) s.push_back({1.0f, 0.3f, 0.25f});
  s.push_back({kFace + 0.02f, 0.3f, 0.25f});
  s.push_back({kWall});
  return s;
}

// The interior of the fill's rectangle, away from every outline.
bool interior(const vr::Vec3f& p) {
  return std::fabs(p.x) < kHalfX - 0.05f && std::fabs(p.y) < kHalfY - 0.05f;
}

int contradiction_case(vkc::test::GpuContext& gpu) {
  const sensor::DepthCameraParams c = camera();
  // Seen past, and arrival: the grid at sets 7 and 8 against the seed.
  for (int which = 0; which < 3; ++which) {
    auto grid = tsdf::create_fusion_grid(gpu.device, gpu.allocator, kVoxel,
                                         kTrunc, kBuckets);
    auto fuser = tsdf::Fuser::create(gpu.device, gpu.allocator, fuser_config());
    CHECK(grid.ok() && fuser.ok());
    auto seeded = seed(gpu, grid.value(), {-kHalfX, -kHalfY, kFace - kTrunc},
                       {kHalfX, kHalfY, kFace + kTrunc}, face(kFace), kFill);
    auto at_seed = read_voxels(gpu, grid.value());
    CHECK(seeded.ok() && at_seed.ok());
    std::vector<Rect> (*scene)(int) = which == 0   ? seen_past_scene
                                      : which == 1 ? arrival_scene
                                                   : farther_band_scene;
    std::vector<float> last = at_seed->weight;
    for (int set = 0; set <= 8; ++set) {
      const std::vector<float> depth = render(c, scene(set));
      CHECK(fuser
                ->fuse(grid.value(), {fixed(depth, c, 0)}, kMaxWeight,
                       tsdf::IntegrationMode::Dynamic)
                .ok());
      auto v = read_voxels(gpu, grid.value());
      CHECK(v.ok());
      CHECK(byte_weight_mismatches(v.value()) == 0);
      std::size_t fallen = 0, wiped = 0, kept = 0, rig = 0, inside = 0;
      for (std::size_t k = 0; k < v->index.size(); ++k) {
        const std::size_t i = v->index[k];
        if (!interior(v->world[k]) || at(at_seed->source, i) != kFill) {
          continue;
        }
        ++inside;
        fallen += v->weight[i] < at(last, i) ? 1 : 0;
        const bool behind = !(at_seed->tsdf[i] > 0.0f);
        if (behind && !(v->weight[i] > 0.0f)) ++wiped;
        if (v->source[i] == kFill) ++kept;
        if (v->source[i] == kRigByte) ++rig;
      }
      last = v->weight;
      // A quiet camera never lowers fill's weight; nothing changes before 8.
      if (set < 8) {
        CHECK(fallen == 0 && kept == inside);
        continue;
      }
      std::printf(
          "I3 %s, set 8: of %zu interior fill voxels %zu wiped, %zu "
          "kept, %zu the rig's\n",
          which == 0   ? "seen past"
          : which == 1 ? "an arrival"
                       : "Farther in the band",
          inside, wiped, kept, rig);
      if (which == 0) {
        // Exactly the tsdf <= 0 side goes; the free side stays fill.
        std::size_t behind = 0;
        for (std::size_t k = 0; k < v->index.size(); ++k) {
          const std::size_t i = v->index[k];
          if (interior(v->world[k]) && at(at_seed->source, i) == kFill &&
              !(at_seed->tsdf[i] > 0.0f)) {
            ++behind;
          }
        }
        CHECK(behind > 100 && wiped == behind && kept == inside - behind);
      } else if (which == 1) {
        // The arrival's band starts as the rig's in the set it arrives.
        std::size_t band = 0, band_rig = 0;
        for (std::size_t k = 0; k < v->index.size(); ++k) {
          const std::size_t i = v->index[k];
          if (!interior(v->world[k]) || at(at_seed->source, i) != kFill) {
            continue;
          }
          if (std::fabs(v->world[k].z - kFace) > 0.02f) continue;
          ++band;
          band_rig += v->source[i] == kRigByte && v->weight[i] > 0.0f &&
                              (v->tsdf[i] > 0.0f) == (v->world[k].z < kFace)
                          ? 1
                          : 0;
        }
        CHECK(band > 100 && band_rig == band);
      } else {
        // Farther, inside the band: averages, removes nothing.
        CHECK(kept == inside && fallen == 0);
      }
    }
  }
  return 0;
}

// I4: a fill face, then a camera that sees a surface 2 cm behind it, noisy,
// with an alarm of each kind injected; the face stays extractable.
int noise_case(vkc::test::GpuContext& gpu) {
  auto grid = tsdf::create_fusion_grid(gpu.device, gpu.allocator, kVoxel,
                                       kTrunc, kBuckets);
  auto fuser = tsdf::Fuser::create(gpu.device, gpu.allocator, fuser_config());
  auto mc = mesh::MarchingCubes::create(gpu.device, gpu.allocator);
  CHECK(grid.ok() && fuser.ok() && mc.ok());
  CHECK(seed(gpu, grid.value(), {-kHalfX, -kHalfY, kFace - kTrunc},
             {kHalfX, kHalfY, kFace + kTrunc}, face(kFace), kFill)
            .ok());
  const sensor::DepthCameraParams c = camera();
  std::mt19937 rng(7);
  std::normal_distribution<float> noise(0.0f, 0.0015f);
  std::size_t least = SIZE_MAX;
  for (int set = 0; set < 60; ++set) {
    std::vector<float> depth =
        render(c, {{kFace + 0.02f, 0.3f, 0.25f}, {kWall}});
    for (float& d : depth) d += d > 0.0f ? noise(rng) : 0.0f;
    // A 4 x 4 patch over the face: 10 cm nearer for one set (Nearer, then
    // Farther as it returns), and gone for two (Vanished).
    for (std::uint32_t y = 28; y < 32; ++y) {
      for (std::uint32_t x = 38; x < 42; ++x) {
        float& d = depth[std::size_t(y) * kWidth + x];
        if (set == 15 || set == 30) d -= 0.1f;
        if (set == 40 || set == 41) d = 0.0f;
      }
    }
    CHECK(fuser
              ->fuse(grid.value(), {fixed(depth, c, 0)}, kMaxWeight,
                     tsdf::IntegrationMode::Dynamic)
              .ok());
    if (set % 5 != 4) continue;
    auto m = mc->extract_host(grid.value());
    CHECK(m.ok());
    std::size_t on_face = 0;
    for (std::size_t t = 0; t < m->triangle_count(); ++t) {
      bool on = true;
      for (std::size_t k = 0; k < 3; ++k) {
        const vr::Vec3f& p = m->vertices[m->indices[3 * t + k]].position;
        on = on && interior(p) && p.z > kFace - 0.01f && p.z < kFace + 0.03f;
      }
      on_face += on ? 1 : 0;
    }
    least = std::min(least, on_face);
  }
  std::printf("I4: at least %zu triangles on the face at every fifth set\n",
              least);
  CHECK(least > 200);
  return 0;
}

}  // namespace

int gpu_main(vkc::test::GpuContext& gpu) {
  if (table_case() != 0) return 1;
  if (rig_case(gpu, tsdf::IntegrationMode::Dynamic) != 0) return 1;
  if (rig_case(gpu, tsdf::IntegrationMode::Classic) != 0) return 1;
  if (no_standing_case(gpu) != 0) return 1;
  if (contradiction_case(gpu) != 0) return 1;
  if (noise_case(gpu) != 0) return 1;
  std::puts("tsdf_source: OK");
  return 0;
}

int main() {
  return vkc::test::run_on_gpu(vr::device_requirements(), gpu_main);
}
