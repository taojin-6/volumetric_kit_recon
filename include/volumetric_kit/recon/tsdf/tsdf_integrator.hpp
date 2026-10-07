// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file tsdf/tsdf_integrator.hpp
/// @brief Projective TSDF integration (classic or dynamic) of a posed depth
///        frame into a @ref VoxelBlockGrid's per-voxel `tsdf` + `weight`
///        attributes.

#include <cstddef>
#include <cstdint>
#include <vector>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/base/stage_metrics.hpp"
#include "volumetric_kit/core/vulkan/buffer.hpp"
#include "volumetric_kit/core/vulkan/compute_kernel.hpp"
#include "volumetric_kit/core/vulkan/descriptor.hpp"
#include "volumetric_kit/core/vulkan/gpu_timer.hpp"
#include "volumetric_kit/recon/core/camera_params.hpp"
#include "volumetric_kit/recon/core/color_space.hpp"
#include "volumetric_kit/recon/core/fwd.hpp"
#include "volumetric_kit/recon/tsdf/export.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"
#include "volumetric_kit/recon/volume/voxel_hash_map.hpp"

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
  /// layout. Null when @ref buffer holds the image instead.
  const std::uint32_t* pixels = nullptr;
  /// The color camera (@ref ColorCameraParams): intrinsics + camera->world pose
  /// + dimensions. May differ from the depth camera (unregistered RGB-D); pass
  /// the depth camera's matching intrinsics + pose for registered capture.
  ColorCameraParams cam{};

  /// What @ref pixels are encoded as. Defaults to the canonical form -- the
  /// working space (linear BT.709/D65) through the exact piecewise sRGB
  /// transfer -- which is what the fusion kernel decodes with, so a caller
  /// already producing canonical bytes says nothing.
  ///
  /// It rides here rather than inside @ref cam because that struct is uploaded
  /// verbatim to the kernel under scalar block layout, pinned at 88 bytes with
  /// GLSL mirrors in two tiers; the encoding is host-side policy the kernel
  /// never reads. @ref TsdfIntegrator::integrate **rejects** a non-canonical
  /// declaration rather than fusing it: converting is the sensor boundary's
  /// job, once, via `sensor::to_canonical` -- which is what "convert once at
  /// the sensor boundary" means operationally.
  ColorEncoding encoding{};

  /// The same image already on the device, in the same layout: a storage
  /// buffer of at least `cam.width * cam.height` words, read in place with no
  /// upload (a GPU pre-processing pass's output). Its memory metadata must
  /// establish device locality, as required by `StorageInput`. Set this or
  /// @ref pixels, not both. Borrowed for the call. After @ref encoding, so
  /// `{pixels, cam, encoding}` still initializes a host frame.
  const core::Buffer* buffer = nullptr;

  /// The image marks its own coverage in each word's high byte: 0 is a pixel
  /// with no colour, such as one a lens maps outside the captured picture,
  /// and it fuses nothing, as a pixel outside the image does; any other value
  /// is colour. Off, the high byte is ignored, as a host image's is.
  /// `sensor::GpuFramePrep` marks its output this way.
  bool coverage_in_alpha = false;
};

/// @brief One camera's frame, for @ref TsdfIntegrator::integrate over several
///        cameras at once: the depth frame its band was allocated from, plus
///        its colour.
///
/// A host depth array is staged in the call's batch, a storage buffer bound
/// in place. A list of these slices into the list
/// @ref volume::VoxelHashMap::allocate_from_depth takes, so one list feeds
/// both.
struct FrameInput : volume::DepthInput {
  /// Optional colour, as @ref TsdfIntegrator::integrate takes it; borrowed
  /// for the call.
  const ColorFrame* color = nullptr;
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
/// unique `BlockIndex::ptr + local`), so the fusion needs no atomics but the
/// one shared write per block: its `changed` stamp (see @ref integrate).
///
/// @ref IntegrationMode::Dynamic instead clears stale geometry ahead of a
/// receded surface (classic keeps a smooth field there). Depth is sampled
/// bilinearly, falling back to nearest-neighbour at image edges and across
/// depth discontinuities or taps that are non-positive or non-finite.
///
/// @warning The `Device` and `Allocator` passed to @ref create must
///          outlive this object; it stores references to them.
class VR_TSDF_API TsdfIntegrator {
 public:
  /// @brief Build the integrate pipeline + descriptors on @p device.
  /// @param device     The compute device (must outlive this object).
  /// @param allocator  The allocator its transient buffers come from (must
  ///                   outlive this).
  /// @return The integrator, or a non-OK `Status` if a pipeline or
  ///         descriptor object fails to build.
  static core::Result<TsdfIntegrator> create(core::Device& device,
                                             core::Allocator& allocator);

  // Rule of zero: every owned pipeline / layout / pool self-frees and self-
  // resets on move; device_ / allocator_ are borrowed, so the defaulted moves
  // leave a moved-from integrator empty (valid() == false).
  ~TsdfIntegrator() = default;
  TsdfIntegrator(TsdfIntegrator&&) noexcept = default;
  TsdfIntegrator& operator=(TsdfIntegrator&&) noexcept = default;
  TsdfIntegrator(const TsdfIntegrator&) = delete;
  TsdfIntegrator& operator=(const TsdfIntegrator&) = delete;

  /// @brief Integrate one posed depth frame into @p grid's active blocks.
  ///
  /// Advances the map's tick (@ref volume::VoxelHashMap::tick) and stamps
  /// `changed` with it on every block whose `tsdf`, `weight` or `color` the
  /// call changed (@ref volume::BlockStamp). Only a store that leaves a
  /// **different** value counts, so a scan revisiting converged surface at
  /// `max_weight` stamps nothing, and Dynamic's clear of a weighted voxel
  /// counts as a change. A consumer keeps the tick it last read at and asks
  /// which blocks are newer.
  /// @param grid        The block grid; must carry `float` `tsdf` + `weight`
  ///                    attributes (see @ref VoxelBlockGrid::create). Its
  ///                    active set (@ref
  ///                    VoxelHashMap::compact_active_blocks_on_device) is
  ///                    fused.
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
  /// @param metrics  Optional `StageMetrics` collecting this call's timing:
  ///                  an `"integrate"` host row around the whole call, and its
  ///                  device half from a timestamp span around the fusion
  ///                  dispatch. `nullptr` measures nothing -- no timer runs, no
  ///                  query is written, and the dispatch takes the untimed
  ///                  submit path (the `ExtractTimings` shape, and the
  ///                  2026-08-01 bar: no global sink, no state retained between
  ///                  calls).
  ///
  ///                  The two halves answer different questions and the gap
  ///                  between them is the point: the host row is wall clock
  ///                  around a fence-blocked submit, so it carries the command
  ///                  buffer, the submit and the stall; the device row is the
  ///                  kernel. A tier whose host row dwarfs its device row is
  ///                  not a slow kernel.
  ///
  ///                  A fuse runs *two* dispatches, so the second -- the active
  ///                  set's compaction (@ref
  ///                  volume::VoxelHashMap::compact_active_blocks_on_device) --
  ///                  reports itself as a `"  ..active set"` breakdown row
  ///                  beneath this one. Without it that kernel's device time
  ///                  would fall into the gap above and read as submit
  ///                  overhead.
  /// @return OK on success, or a non-OK `Status`:
  ///         `Status::Code::InvalidArgument` if the integrator is
  ///         moved-from, @p depth is null, @p grid lacks a `float`
  ///         `tsdf`/`weight` attribute, @p color is set but empty or @p grid
  ///         lacks a `uint32` `color` attribute, or the active set is too large
  ///         for a single 1-D dispatch (its voxel count exceeds the device's
  ///         `maxComputeWorkGroupCount[0]`, or 2^32 threads); otherwise a
  ///         buffer or dispatch failure.
  core::Status integrate(volume::VoxelBlockGrid& grid, const float* depth,
                         const DepthCameraParams& cam, float max_weight = 5.0f,
                         IntegrationMode mode = IntegrationMode::Classic,
                         const ColorFrame* color = nullptr,
                         core::StageMetrics* metrics = nullptr);

  /// @brief @ref integrate from a depth image already on the device.
  ///
  /// The same fusion, but @p depth is bound where it lives -- a GPU
  /// pre-processing pass's output -- rather than uploaded from the host. A
  /// @p color may likewise carry its image as @ref ColorFrame::buffer.
  /// @param depth  A storage buffer holding the image in metres, at least
  ///               `cam.width * cam.height` floats, row-major. Borrowed for the
  ///               call; the writer's dispatch must have finished, which a
  ///               `dispatch` on this device guarantees.
  /// @return As the host overload; `Status::Code::InvalidArgument` also for
  ///         a @p depth that is empty, not a storage buffer, has unknown or
  ///         non-device-local memory, or is smaller than
  ///         the image.
  core::Status integrate(volume::VoxelBlockGrid& grid,
                         const core::Buffer& depth,
                         const DepthCameraParams& cam, float max_weight = 5.0f,
                         IntegrationMode mode = IntegrationMode::Classic,
                         const ColorFrame* color = nullptr,
                         core::StageMetrics* metrics = nullptr);

  /// @brief @ref integrate several cameras' frames at once: one compaction and
  ///        one submit for them all, rather than two submits a frame.
  ///
  /// Each frame is a dispatch of its own over the one active set, in order, so
  /// every voxel takes the frames in turn as integrating them one after another
  /// does: the same arithmetic in the same order, Dynamic's clearing included.
  /// The set is compacted once, so allocate every frame's band first (@ref
  /// volume::VoxelHashMap::allocate_from_depth takes them together too); a
  /// block first allocated for a later frame is then fused from the earlier
  /// ones as well. The call is one tick, so a rig's set ages as one fuse.
  /// @param grid        As @ref integrate.
  /// @param frames      The frames, each checked as @ref integrate checks one
  ///                    before any work. One with no pixels fuses nothing, as
  ///                    @ref volume::VoxelHashMap::allocate_from_depth
  ///                    allocates nothing for it, and so does an empty list.
  /// @param max_weight  As @ref integrate, for every frame.
  /// @param mode        As @ref integrate, for every frame.
  /// @param metrics     As @ref integrate: one `"integrate"` row, a device span
  ///                    per frame, over one `"  ..active set"` sub-row.
  /// @return As @ref integrate.
  core::Status integrate(volume::VoxelBlockGrid& grid,
                         const std::vector<FrameInput>& frames,
                         float max_weight = 5.0f,
                         IntegrationMode mode = IntegrationMode::Classic,
                         core::StageMetrics* metrics = nullptr);

  /// @return `true` if this owns a live pipeline (`false` when moved-from).
  bool valid() const noexcept { return kernel_.valid(); }

 private:
  TsdfIntegrator() = default;

  // Borrowed (must outlive this).
  core::Device* device_ = nullptr;
  core::Allocator* allocator_ = nullptr;

  // Cached maxComputeWorkGroupCount[0] -- the device cap on a 1-D dispatch's
  // groupCountX; integrate() rejects an active set that would exceed it.
  std::uint32_t max_workgroup_count_x_ = 0;
  // The ceiling on one storage-buffer binding's range, read once at create().
  // The depth and colour frames are staged and bound whole each integrate().
  VkDeviceSize max_storage_buffer_range_ = 0;

  // The integrate kernel's bundled layout + pipeline + descriptor set, its set
  // allocated from pool_ (which must outlive it) by KernelSetBuilder at
  // create().
  core::ComputeKernel kernel_;
  core::DescriptorPool pool_;
  // The kernel's sets, one a frame of a call, so one batch fuses several
  // cameras; kernel_.set goes unused. Grown to the most frames a call has
  // had; every set is written whole each call.
  core::KernelSets frame_sets_;
  // The device-span collector, created once rather than per call: a query pool
  // of a few timestamps is negligible, and a lazily-created one would need a
  // mutable member and a failure path on a diagnostic. Idle -- no query
  // written, no span recorded -- until a caller passes a StageMetrics.
  core::GpuTimer gpu_timer_;
  // Fixed-size camera-params SSBO (DepthCameraParams), rewritten inline ahead
  // of each frame's dispatch rather than reallocated per frame (mirrors the
  // volume tier's persistent camera params).
  core::Buffer cam_buf_;
  // Color path: the persistent (separate) color-camera SSBO, and a 1-element
  // dummy bound to the color-image + color-attribute slots when no color frame
  // is fused (so every declared descriptor stays bound).
  core::Buffer color_cam_buf_;
  core::Buffer color_dummy_;
};

}  // namespace volumetric_kit::recon::tsdf
