// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file tsdf/source.hpp
/// @brief Which sensor wrote a voxel: one byte a voxel, the `"source"`
///        attribute every fusion grid carries (@ref create_fusion_grid), and
///        the cast filters that read it.
///
/// Four voxels share a `uint32` word of the attribute. A kernel writes a
/// voxel's byte from the voxel's one thread: `atomicAnd` to clear it, then
/// `atomicOr` to set it, so the word's other three bytes keep what their own
/// threads write. Every reset writes @ref kNoSource, so a nonzero byte implies
/// a weight above 0. Marching cubes and the codec never read it.
///
/// The value table:
///
/// | Byte | Meaning |
/// |---|---|
/// | 0 | No sensor: the voxel holds no weight |
/// | 1-63 | Fixed camera `byte - 1` started the voxel's history (@ref
/// fixed_source) | | 128 + (1-63) | Mobile sensor `(byte & 63) - 1` wrote it,
/// posed by a rig-only fit: anchored fill | | 192 + (1-63) | The same from a
/// frame no rig-only fit anchored: unanchored fill |
///
/// A fixed camera writes its byte only into a voxel at weight 0, so later
/// samples, its own and other cameras', average in and leave the byte as it
/// is; a fixed sample never turns fill into rig. A voxel another writer
/// weighted (a voxelizer, a decoder) keeps byte 0: a filler leaves it alone,
/// as a fixed camera's, and the casts below skip it.

#include <cstdint>
#include <string_view>

#include "volumetric_kit/grid/volume/ray_caster.hpp"
#include "volumetric_kit/recon/tsdf/export.hpp"

namespace volumetric_kit::recon::tsdf {

/// The attribute's name, as a `vgrid::volume::AttributeSpec` names it: one
/// byte a voxel.
inline constexpr std::string_view kSourceAttribute = "source";

/// No sensor: the byte of a voxel at weight 0, which every reset writes.
inline constexpr std::uint8_t kNoSource = 0;
/// Bits 0-5: the sensor's index in its class, plus 1.
inline constexpr std::uint8_t kSourceIndexMask = 0x3F;
/// Bit 6: with @ref kMobileBit, some write came from a frame no rig-only fit
/// anchored (@ref FillTier::Unanchored). Casts that track exclude it.
inline constexpr std::uint8_t kUnanchoredBit = 0x40;
/// Bit 7: a mobile sensor (a filler, `FrameRole::Filler`) wrote the voxel:
/// it is fill.
inline constexpr std::uint8_t kMobileBit = 0x80;
/// The sensors a class names: indices 0 to 62.
inline constexpr std::uint32_t kMaxSourcesPerClass = 63;

/// @brief How a filler's frame was posed.
enum class FillTier : std::uint8_t {
  /// By a fit to the rig alone: its fill may be tracked against.
  Anchored = 0,
  /// Any other accepted pose: its fill is shown, never tracked.
  Unanchored = 1,
};

/// @return The byte fixed camera @p index writes: `index + 1`.
/// @pre @p index is below @ref kMaxSourcesPerClass.
constexpr std::uint8_t fixed_source(std::uint32_t index) noexcept {
  return static_cast<std::uint8_t>(index + 1u);
}

/// @return The byte mobile sensor @p index writes from a frame of @p tier:
///         @ref kMobileBit, @ref kUnanchoredBit when Unanchored, and
///         `index + 1`.
/// @pre @p index is below @ref kMaxSourcesPerClass.
constexpr std::uint8_t mobile_source(std::uint32_t index,
                                     FillTier tier) noexcept {
  return static_cast<std::uint8_t>(
      kMobileBit | (tier == FillTier::Unanchored ? kUnanchoredBit : 0u) |
      (index + 1u));
}

/// @brief The rig alone: voxels a fixed camera started, at weight
///        @p min_weight or more. The cast that decides whether a fit
///        anchored on the rig.
/// @param min_weight  The least weight that answers; 2 by default, below
///                    which low-weight fill would hide the rig.
/// @return The filter, on @ref kSourceAttribute: bytes 1 to 63 answer.
VR_TSDF_API volumetric_kit::grid::volume::CastFilter rig_cast(
    float min_weight = 2.0f);

/// @brief The rig plus anchored fill, at weight @p min_weight or more: what a
///        filler's frames may be tracked against.
/// @param min_weight  As @ref rig_cast.
/// @return The filter, on @ref kSourceAttribute: bytes 1 to 63 and 129 to
///         191 answer.
VR_TSDF_API volumetric_kit::grid::volume::CastFilter tracking_cast(
    float min_weight = 2.0f);

}  // namespace volumetric_kit::recon::tsdf
