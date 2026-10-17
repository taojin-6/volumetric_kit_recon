// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// recon_package_headers compiles this against the installed recon and never
// runs it. The hash table's internals stay out of the install: volume/hash.hpp
// is not installed; every installed volume header, and every installed header
// that includes one, compiles without it; and each definition below is
// ill-formed while an installed header still declares its name.

#if __has_include("volumetric_kit/recon/volume/hash.hpp")
#error "volume/hash.hpp is installed, but the hash table's layout is internal"
#endif

#include "volumetric_kit/recon/codec/decoder.hpp"
#include "volumetric_kit/recon/codec/encoder.hpp"
#include "volumetric_kit/recon/mesh/marching_cubes.hpp"
#include "volumetric_kit/recon/tsdf/fuser.hpp"
#include "volumetric_kit/recon/tsdf/mesh_integrator.hpp"
#include "volumetric_kit/recon/tsdf/tsdf_integrator.hpp"
#include "volumetric_kit/recon/volume/frustum.hpp"
#include "volumetric_kit/recon/volume/grid_growth.hpp"
#include "volumetric_kit/recon/volume/hash_types.hpp"
#include "volumetric_kit/recon/volume/triangle_candidates.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"
#include "volumetric_kit/recon/volume/voxel_coords.hpp"
#include "volumetric_kit/recon/volume/voxel_grid.hpp"
#include "volumetric_kit/recon/volume/voxel_hash_map.hpp"

// HashEntry is a class, since a variable may share a class's name, and
// hash_bucket a variable, since a function would only overload one.
namespace volumetric_kit::recon::volume {
struct HashEntry {};
inline constexpr int hash_bucket = 0;
inline constexpr int kFreeEntry = 0;
inline constexpr int kLockEntry = 0;
inline constexpr int kNoOffset = 0;
inline constexpr int kHashPrimeX = 0;
inline constexpr int kHashPrimeY = 0;
inline constexpr int kHashPrimeZ = 0;
inline constexpr int kFailTotal = 0;
inline constexpr int kFailLock = 0;
inline constexpr int kFailChain = 0;
inline constexpr int kFailHeap = 0;
inline constexpr int kFailTerminal = 0;
inline constexpr int kFailTable = 0;
inline constexpr int kFailSlots = 0;
}  // namespace volumetric_kit::recon::volume
