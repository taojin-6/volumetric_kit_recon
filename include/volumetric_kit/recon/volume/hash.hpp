// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file volume/hash.hpp
/// @brief The spatial hash function and the hash-table slot sentinels.

#include <cstdint>

#include "volumetric_kit/recon/core/device_macros.hpp"
#include "volumetric_kit/recon/core/math/vector_types.hpp"

namespace volumetric_kit::recon::volume {

/// @name Hash-table slot sentinels
/// Shared by the host allocator and the compute shaders. `kFreeEntry` and
/// `kLockEntry` occupy a slot's `ptr` field when it holds no live voxel-block
/// pointer; `kNoOffset` terminates a collision chain in the `offset` field.
/// @{
inline constexpr std::int32_t kFreeEntry = -1;  ///< Empty slot.
inline constexpr std::int32_t kLockEntry = -2;  ///< Slot locked mid-insert.
inline constexpr std::int32_t kNoOffset = 0;    ///< Collision-chain terminator.
/// @}

/// @name Fail-count slots
/// The layout of the per-dispatch failure tally the allocate and delete kernels
/// share, mirrored from `volume/shaders/hash_common.glsl`.
///
/// `[kFailTotal .. kFailHeap]` are **retryable**: the host re-dispatches while
/// the total keeps dropping, because the element that failed is still
/// unprocessed. @ref kFailTerminal is not, and the split is what keeps the
/// reported count honest -- a delete whose heap append fails has already
/// cleared its table entry, so the next round finds the coord absent and counts
/// nothing. Accumulated across rounds rather than read from the last one, or a
/// permanently leaked block index would be reported as a clean delete.
/// @{
inline constexpr std::uint32_t kFailTotal = 0;     ///< Retryable total.
inline constexpr std::uint32_t kFailLock = 1;      ///< Lock contention.
inline constexpr std::uint32_t kFailChain = 2;     ///< Collision chain full.
inline constexpr std::uint32_t kFailHeap = 3;      ///< Block heap empty.
inline constexpr std::uint32_t kFailTerminal = 4;  ///< Non-retryable.
/// No free non-anchor slot within the overflow probe window (see
/// `kMaxOverflowProbes` in `volume/shaders/hash_common.glsl`): "nothing free
/// near this bucket", not "the table is full". The probe is bounded because
/// scanning the whole table cost a contended atomic per slot and hung the GPU
/// at high occupancy; the trade is that this is now evidence of pressure rather
/// than proof of exhaustion. Either way the caller's response is the same --
/// grow.
inline constexpr std::uint32_t kFailTable = 5;
inline constexpr std::uint32_t kFailSlots = 6;  ///< Slots in the tally.
/// @}

/// @name Spatial-hash primes
/// The large primes of the Teschner et al. spatial hash. Unsigned so the
/// coordinate multiply wraps with defined behaviour.
/// @{
inline constexpr std::uint32_t kHashPrimeX = 73856093u;
inline constexpr std::uint32_t kHashPrimeY = 19349669u;
inline constexpr std::uint32_t kHashPrimeZ = 83492791u;
/// @}

/// @brief Map a voxel-block coordinate to a bucket index in `[0, num_buckets)`.
///
/// The XOR-of-prime-products spatial hash. Deliberately all-unsigned: the block
/// coordinate can be negative, and signed integer overflow is undefined, so
/// each axis is cast to `uint32_t` before the multiply (a straight
/// reinterpretation of the bits, matching the GLSL mirror).
/// @param block        The voxel-block coordinate.
/// @param num_buckets  The hash-table bucket count. @pre `num_buckets > 0`
///                     (enforced by @ref VoxelGridParams::validate); a value of
///                     0 is undefined -- integer division by zero.
/// @return The bucket index the coordinate hashes to.
VR_DEVICE_HOST inline std::uint32_t hash_bucket(Vec3i block,
                                                std::int32_t num_buckets) {
  const std::uint32_t hash =
      (static_cast<std::uint32_t>(block.x) * kHashPrimeX) ^
      (static_cast<std::uint32_t>(block.y) * kHashPrimeY) ^
      (static_cast<std::uint32_t>(block.z) * kHashPrimeZ);
  return hash % static_cast<std::uint32_t>(num_buckets);
}

}  // namespace volumetric_kit::recon::volume
