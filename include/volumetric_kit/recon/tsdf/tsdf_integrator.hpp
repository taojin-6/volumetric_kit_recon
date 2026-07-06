// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file tsdf/tsdf_integrator.hpp
/// @brief Projective TSDF integration (classic or dynamic) of a posed depth
///        frame into a @ref VoxelBlockGrid's per-voxel `tsdf` + `weight`
///        attributes.

#include <cstdint>

#include "volumetric_kit/recon/core/buffer.hpp"
#include "volumetric_kit/recon/core/compute_kernel.hpp"
#include "volumetric_kit/recon/core/descriptor.hpp"
#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/tsdf/export.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"
#include "volumetric_kit/recon/volume/voxel_hash_map.hpp"  // DepthCameraParams

namespace volumetric_kit::recon {
class Device;
class Allocator;
}  // namespace volumetric_kit::recon

namespace volumetric_kit::recon::tsdf {

/// @brief How @ref TsdfIntegrator::integrate treats a voxel that projects into
///        free space ahead of the surface (its projective SDF exceeds
///        `trunc_dist`).
enum class IntegrationMode : std::uint32_t {
  Classic = 0,  ///< Keep it: clamp to +`trunc_dist` and fuse a smooth field
                ///< ahead of surfaces.
  Dynamic = 1,  ///< Clear it: reset stale geometry there, so a receded surface
                ///< leaves no ghost (moving scenes).
};

/// @brief An optional color frame to fuse alongside depth: packed-RGB pixels
///        plus the (separate) color camera they were captured with.
struct ColorFrame {
  /// Row-major color image, `cam.width * cam.height` pixels, RGB packed in each
  /// `uint`'s low three bytes (alpha ignored) -- the mesh tier's `color`
  /// layout.
  const std::uint32_t* pixels = nullptr;
  /// The color camera: intrinsics + camera->world pose + dimensions (its
  /// depth-range fields are unused). May differ from the depth camera
  /// (unregistered RGB-D); pass the depth camera for registered capture.
  volume::DepthCameraParams cam{};
};

/// @brief Fuses posed depth frames into a @ref VoxelBlockGrid's `tsdf` +
///        `weight` attributes by projective TSDF integration (classic or
///        dynamic).
///
/// One GLSL dispatch runs a thread per voxel of every active block: it projects
/// the voxel centre into the depth camera, computes the truncated projective
/// signed distance (`sdf = depth - Zc`, positive in front of the surface), and
/// fuses it into `tsdf`/`weight` by a weighted running average (inverse-square
/// observation weight with a behind-surface dropoff, capped at `max_weight`).
/// Node-centred voxels (`voxel * voxel_size`), matching @ref voxel_to_world and
/// the prior engine's numerics. Each voxel is owned by exactly one thread (a
/// unique `BlockIndex::ptr + local`), so the fusion needs no atomics.
///
/// @ref IntegrationMode::Dynamic instead clears stale geometry ahead of a
/// receded surface (classic keeps a smooth field there). Depth is sampled
/// bilinearly, falling back to nearest-neighbour at image edges and across
/// depth discontinuities.
///
/// @warning The @ref Device and @ref Allocator passed to @ref create must
///          outlive this object; it stores references to them.
class VR_TSDF_API TsdfIntegrator {
 public:
  /// @brief Build the integrate pipeline + descriptors on @p device.
  /// @param device     The compute device (must outlive this object).
  /// @param allocator  The allocator its transient buffers come from (must
  ///                   outlive this).
  /// @return The integrator, or a non-OK @ref Status if a pipeline or
  ///         descriptor object fails to build.
  static Result<TsdfIntegrator> create(Device& device, Allocator& allocator);

  // Rule of zero: every owned pipeline / layout / pool self-frees and self-
  // resets on move; device_ / allocator_ are borrowed, so the defaulted moves
  // leave a moved-from integrator empty (valid() == false).
  ~TsdfIntegrator() = default;
  TsdfIntegrator(TsdfIntegrator&&) noexcept = default;
  TsdfIntegrator& operator=(TsdfIntegrator&&) noexcept = default;
  TsdfIntegrator(const TsdfIntegrator&) = delete;
  TsdfIntegrator& operator=(const TsdfIntegrator&) = delete;

  /// @brief Integrate one posed depth frame into @p grid's active blocks.
  /// @param grid        The block grid; must carry `float` `tsdf` + `weight`
  ///                    attributes (see @ref VoxelBlockGrid::create). Its
  ///                    active set (@ref VoxelHashMap::compact_active_blocks)
  ///                    is fused.
  /// @param depth       Row-major depth image in **metres**, length
  ///                    `cam.width * cam.height` (the host applies any raw
  ///                    sensor depth-scale first, as @ref
  ///                    VoxelHashMap::allocate_from_depth does).
  /// @param cam         Intrinsics + camera->world pose + depth range; the
  ///                    integrator inverts the pose to project world -> camera.
  /// @param max_weight  The running-average weight cap (the ported default is
  ///                    5.0).
  /// @param mode        Classic keeps free space ahead of the surface; dynamic
  ///                    clears stale geometry there (see @ref IntegrationMode).
  /// @param color       Optional @ref ColorFrame fused into the grid's `color`
  ///                    attribute (a `uint32` packed-RGB attribute the grid
  ///                    must then carry); `nullptr` integrates depth only. A
  ///                    voxel's first color observation assigns the sampled
  ///                    RGB; later ones running-average it with the SDF
  ///                    weights.
  /// @note  Integrate a given grid with one consistent mode across a sequence:
  ///        a dynamic frame clears every weighted free-space voxel past the
  ///        band, including one a prior classic frame fused there -- not only
  ///        genuinely receded geometry. Dynamic also clears the `color` of a
  ///        receded voxel whenever the grid carries the attribute, including on
  ///        a depth-only (`color == nullptr`) frame.
  /// @note  A **separate/unregistered** color camera (a @p color with its own
  ///        pose or intrinsics) carries the usual projective-color limits the
  ///        registered case avoids: a voxel occluded in the color view but
  ///        near-surface for depth takes the occluder's color, and a voxel is
  ///        colored only on frames where its depth pixel is valid (color fusion
  ///        follows the depth projection). Color also shares the SDF weight
  ///        cap, so a changed color converges over several frames once the
  ///        weight saturates.
  /// @return OK on success, or a non-OK @ref Status:
  ///         @ref Status::Code::InvalidArgument if the integrator is
  ///         moved-from, @p depth is null, @p grid lacks a `float`
  ///         `tsdf`/`weight` attribute, @p color is set but empty or @p grid
  ///         lacks a `uint32` `color` attribute, or the active set is too large
  ///         for a single 1-D dispatch (its voxel count exceeds the device's
  ///         `maxComputeWorkGroupCount[0]`, or 2^32 threads); otherwise a
  ///         buffer or dispatch failure.
  Status integrate(volume::VoxelBlockGrid& grid, const float* depth,
                   const volume::DepthCameraParams& cam,
                   float max_weight = 5.0f,
                   IntegrationMode mode = IntegrationMode::Classic,
                   const ColorFrame* color = nullptr);

  /// @return `true` if this owns a live pipeline (`false` when moved-from).
  bool valid() const noexcept { return kernel_.valid(); }

 private:
  TsdfIntegrator() = default;

  // Borrowed (must outlive this).
  Device* device_ = nullptr;
  Allocator* allocator_ = nullptr;

  // Cached maxComputeWorkGroupCount[0] -- the device cap on a 1-D dispatch's
  // groupCountX; integrate() rejects an active set that would exceed it.
  std::uint32_t max_workgroup_count_x_ = 0;

  // The integrate kernel's bundled layout + pipeline + descriptor set, its set
  // allocated from pool_ (which must outlive it) by KernelSetBuilder at
  // create().
  ComputeKernel kernel_;
  DescriptorPool pool_;
  // Fixed-size camera-params SSBO (volume::DepthCameraParams): bound once at
  // create() and rewritten each integrate(), not reallocated per frame (mirrors
  // the volume tier's persistent camera params).
  Buffer cam_buf_;
  // Color path: the persistent (separate) color-camera SSBO, and a 1-element
  // dummy bound to the color-image + color-attribute slots when no color frame
  // is fused (so every declared descriptor stays bound).
  Buffer color_cam_buf_;
  Buffer color_dummy_;
};

}  // namespace volumetric_kit::recon::tsdf
