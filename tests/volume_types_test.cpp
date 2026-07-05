// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// Validates the ported voxel-hash POD layouts. The static_assert size/offset
// guards in hash_types.hpp do the structural checking at compile time (a build
// failure there is the real test); this binary adds a few runtime sanity checks
// and proves the header compiles and links through the recon_volume tier.

#include <cstdio>

#include "volumetric_kit/recon/core/math/vector_types.hpp"
#include "volumetric_kit/recon/volume/hash_types.hpp"

namespace vol = volumetric_kit::recon::volume;
namespace vr = volumetric_kit::recon;

int main() {
  int failures = 0;
  auto check = [&](bool cond, const char* what) {
    if (!cond) {
      std::fprintf(stderr, "FAIL: %s\n", what);
      ++failures;
    }
  };

  // Voxel accessor round-trip.
  vol::Voxel v{};
  v.set_sdf(0.25f);
  check(v.get_sdf() == 0.25f, "Voxel set/get sdf");

  // HashEntry aggregate holds its block coordinate.
  const vol::HashEntry e{7, vr::Vec3i{1, 2, 3}, -1};
  check(e.ptr == 7 && e.pos == (vr::Vec3i{1, 2, 3}) && e.offset == -1,
        "HashEntry aggregate fields");

  // VoxelData is a faithful view onto the table's storage; color off by
  // default.
  vol::Voxel blocks[1];
  vol::HashTable table{};
  table.sdf_blocks = blocks;
  const vol::VoxelData data = table.voxel_data();
  check(data.sdf_blocks == blocks, "VoxelData view aliases sdf_blocks");
  check(data.color_blocks == nullptr, "color disabled by default");

  if (failures == 0) {
    std::puts("recon_volume types test passed");
    return 0;
  }
  std::fprintf(stderr, "%d checks failed\n", failures);
  return 1;
}
