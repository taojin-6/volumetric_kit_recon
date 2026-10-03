// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// Host geometry only: hierarchical ownership must partition space independently
// of the uniform grid's nearest-sample convention, with checked integer bounds.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>

#include "volumetric_kit/recon/volume/hierarchy_layout.hpp"
#include "volumetric_kit/recon/volume/voxel_coords.hpp"

namespace vr = volumetric_kit::recon;
namespace vol = volumetric_kit::recon::volume;

#define CHECK(cond)                                                        \
  do {                                                                     \
    if (!(cond)) {                                                         \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      return 1;                                                            \
    }                                                                      \
  } while (0)

namespace {

bool same_key(const vol::LevelBlockKey& a, const vol::LevelBlockKey& b) {
  return a.coord == b.coord && a.level == b.level;
}

}  // namespace

int main() {
  const auto defaults = vol::VoxelGridParams::defaults();
  auto made = vol::HierarchyLayout::create(defaults, 4);
  CHECK(made.ok());
  const auto layout = made.value();
  CHECK(layout.level_count() == 4);

  // Level zero preserves every uniform parameter byte. Coarser levels change
  // spacing alone: physical truncation and capacity do not grow with it.
  auto first = layout.level_grid(0);
  CHECK(first.ok());
  CHECK(std::memcmp(&first.value(), &defaults, sizeof(defaults)) == 0);
  for (std::uint32_t level = 0; level < layout.level_count(); ++level) {
    auto actual = layout.level_grid(level);
    CHECK(actual.ok());
    CHECK(actual->voxel_size ==
          defaults.voxel_size * static_cast<float>(1u << level));
    actual->voxel_size = defaults.voxel_size;
    CHECK(std::memcmp(&actual.value(), &defaults, sizeof(defaults)) == 0);
  }
  CHECK(!layout.level_grid(4));
  CHECK(!layout.level_grid(std::numeric_limits<std::uint32_t>::max()));

  // One finest block is exactly one metre here, isolating boundary ownership
  // from decimal-to-float rounding in the production 5 mm spacing.
  auto grid = defaults;
  grid.voxel_size = 0.125f;
  auto exact_result = vol::HierarchyLayout::create(grid, 4);
  CHECK(exact_result.ok());
  const auto exact = exact_result.value();
  const float inf = std::numeric_limits<float>::infinity();
  const float below_one = std::nextafter(1.0f, 0.0f);
  const float below_minus_one = std::nextafter(-1.0f, -inf);
  auto owner = exact.owning_block(vr::Vec3f(below_one, 1.0f, -1.0f), 0);
  CHECK(owner.ok());
  CHECK(same_key(owner.value(), {vr::Vec3i(0, 1, -1), 0}));
  owner = exact.owning_block(
      vr::Vec3f(below_minus_one, -std::numeric_limits<float>::denorm_min(),
                -0.0f),
      0);
  CHECK(owner.ok());
  CHECK(same_key(owner.value(), {vr::Vec3i(-2, -1, 0), 0}));

  // Near the positive face, nearest-sample lookup crosses into block 1 before
  // the actual region ends. Hierarchical ownership must stay in block 0.
  const vr::Vec3f near_face(0.99f, 0.0f, 0.0f);
  CHECK(vol::world_to_block(near_face, grid).x == 1);
  owner = exact.owning_block(near_face, 0);
  CHECK(owner.ok());
  CHECK(owner->coord.x == 0);

  // The owner of any fine region's world point is its parent at the next
  // level, including exact boundaries and mixed-sign axes.
  for (int x = -32; x <= 32; ++x) {
    const vr::Vec3f world(static_cast<float>(x) * 0.25f,
                          static_cast<float>(7 - x) * 0.125f,
                          static_cast<float>(x - 3) * 0.5f);
    for (std::uint32_t level = 0; level + 1 < exact.level_count(); ++level) {
      const auto fine = exact.owning_block(world, level);
      const auto coarse = exact.owning_block(world, level + 1);
      CHECK(fine.ok() && coarse.ok());
      const auto parent = exact.parent(fine.value());
      CHECK(parent.ok());
      CHECK(same_key(parent.value(), coarse.value()));
    }
  }

  // Eight distinct octants cover the parent's region and all map back to the
  // same parent. Exercise negative odd/even and positive parent coordinates.
  for (int coordinate = -3; coordinate <= 3; ++coordinate) {
    const vol::LevelBlockKey parent{vr::Vec3i(coordinate, -coordinate, -1), 2};
    for (std::uint32_t octant = 0; octant < 8; ++octant) {
      const auto child = exact.child(parent, octant);
      CHECK(child.ok());
      CHECK(child->level == 1);
      CHECK(child->coord.x == coordinate * 2 + static_cast<int>(octant & 1u));
      CHECK(child->coord.y ==
            -coordinate * 2 + static_cast<int>((octant >> 1) & 1u));
      CHECK(child->coord.z == -2 + static_cast<int>((octant >> 2) & 1u));
      const auto recovered = exact.parent(child.value());
      CHECK(recovered.ok());
      CHECK(same_key(recovered.value(), parent));
      const vr::Vec3f child_centre =
          (vr::Vec3f(child->coord) + vr::Vec3f(0.5f)) * 2.0f;
      const auto fine_owner = exact.owning_block(child_centre, 1);
      const auto coarse_owner = exact.owning_block(child_centre, 2);
      CHECK(fine_owner.ok() && coarse_owner.ok());
      CHECK(same_key(fine_owner.value(), child.value()));
      CHECK(same_key(coarse_owner.value(), parent));
    }
  }

  const auto int_min = std::numeric_limits<std::int32_t>::min();
  const auto int_max = std::numeric_limits<std::int32_t>::max();
  const vol::LevelBlockKey extremes{vr::Vec3i(int_min, int_max, -1), 0};
  const auto parent = exact.parent(extremes);
  CHECK(parent.ok());
  CHECK(parent->coord == vr::Vec3i(int_min / 2, int_max / 2, -1));
  const auto restored = exact.child(parent.value(), 6);
  CHECK(restored.ok());
  CHECK(same_key(restored.value(), extremes));
  CHECK(!exact.child({vr::Vec3i(int_max / 2 + 1, 0, 0), 1}, 0));
  CHECK(!exact.child({vr::Vec3i(0, int_min / 2 - 1, 0), 1}, 7));
  CHECK(!exact.child({vr::Vec3i(0, 0, int_max), 1}, 0));
  CHECK(!exact.child({vr::Vec3i(0), 0}, 0));
  CHECK(!exact.child({vr::Vec3i(0), 4}, 0));
  CHECK(!exact.child({vr::Vec3i(0), 1}, 8));
  CHECK(!exact.parent({vr::Vec3i(0), 3}));
  CHECK(
      !exact.parent({vr::Vec3i(0), std::numeric_limits<std::uint32_t>::max()}));

  // Bounds checks precede casts. INT32_MIN is representable as a float; the
  // positive INT32_MAX rounds up to 2^31 and must therefore be refused.
  owner =
      exact.owning_block(vr::Vec3f(static_cast<float>(int_min), 0.0f, 0.0f), 0);
  CHECK(owner.ok());
  CHECK(owner->coord.x == int_min);
  CHECK(!exact.owning_block(vr::Vec3f(static_cast<float>(int_max), 0.0f, 0.0f),
                            0));
  CHECK(!exact.owning_block(
      vr::Vec3f(std::nextafter(static_cast<float>(int_min), -inf)), 0));
  CHECK(!exact.owning_block(vr::Vec3f(inf, 0.0f, 0.0f), 0));
  CHECK(!exact.owning_block(vr::Vec3f(0.0f, -inf, 0.0f), 0));
  CHECK(!exact.owning_block(
      vr::Vec3f(0.0f, 0.0f, std::numeric_limits<float>::quiet_NaN()), 0));
  CHECK(!exact.owning_block(vr::Vec3f(0.0f), 4));

  CHECK(vol::HierarchyLayout::create(defaults, 1));
  CHECK(
      vol::HierarchyLayout::create(defaults, vol::HierarchyLayout::kMaxLevels));
  CHECK(!vol::HierarchyLayout::create(defaults, 0));
  CHECK(!vol::HierarchyLayout::create(defaults,
                                      vol::HierarchyLayout::kMaxLevels + 1));
  auto bad = defaults;
  bad.voxel_size = inf;
  CHECK(!vol::HierarchyLayout::create(bad, 4));
  bad = defaults;
  bad.trunc_dist = inf;
  CHECK(!vol::HierarchyLayout::create(bad, 4));
  bad = defaults;
  bad.voxel_size = std::numeric_limits<float>::max() / 8.0f;
  CHECK(vol::HierarchyLayout::create(bad, 1));
  CHECK(!vol::HierarchyLayout::create(bad, 2));
  bad = defaults;
  bad.voxel_size = 0.0f;
  CHECK(!vol::HierarchyLayout::create(bad, 4));
  bad = defaults;
  bad.trunc_dist = -1.0f;
  CHECK(!vol::HierarchyLayout::create(bad, 4));
  bad = defaults;
  bad.block_size = 4;
  bad.voxels_per_block = 64;
  CHECK(!vol::HierarchyLayout::create(bad, 4));
  bad = defaults;
  bad.num_blocks = 1;
  CHECK(!vol::HierarchyLayout::create(bad, 4));

  std::printf("recon volume hierarchy layout test passed\n");
  return 0;
}
