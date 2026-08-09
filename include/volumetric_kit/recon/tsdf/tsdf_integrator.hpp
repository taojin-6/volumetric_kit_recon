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

#include "volumetric_kit/recon/core/buffer.hpp"
#include "volumetric_kit/recon/core/camera_params.hpp"
#include "volumetric_kit/recon/core/color_space.hpp"
#include "volumetric_kit/recon/core/compute_kernel.hpp"
#include "volumetric_kit/recon/core/descriptor.hpp"
#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/tsdf/export.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"

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
};

/// @brief What optional machinery a @ref TsdfIntegrator carries, chosen at
///        @ref TsdfIntegrator::create.
struct TsdfIntegratorConfig {
  /// Track which blocks each fuse actually changed (@ref
  /// TsdfIntegrator::dirty_block_count and friends).
  ///
  /// Off by default, and the default costs nothing: no `num_blocks * 4`
  /// host-visible allocation (6 MB at `VoxelGridParams::defaults()`, and it
  /// doubles with every map grow), and not one store in the fusion kernel,
  /// which binds a 1-element dummy to the flag slot instead. That is the bar a
  /// tier-level measurement has to clear here -- nothing measured for a caller
  /// who did not ask -- and a fuse-only consumer (a viewer, an offline
  /// exporter, a scanner that re-meshes the whole volume) should leave it off.
  ///
  /// Turning it on is what an **incremental** re-mesh needs: the set of blocks
  /// whose extracted geometry the last few frames invalidated.
  bool track_dirty_blocks = false;
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
/// unique `BlockIndex::ptr + local`), so the fusion needs no atomics (the
/// opt-in per-block dirty flag is the one shared write; see @ref
/// TsdfIntegratorConfig::track_dirty_blocks).
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
  /// @param config     Optional machinery to carry; see @ref
  ///                   TsdfIntegratorConfig. The default carries none.
  /// @return The integrator, or a non-OK @ref Status if a pipeline or
  ///         descriptor object fails to build.
  static Result<TsdfIntegrator> create(Device& device, Allocator& allocator,
                                       const TsdfIntegratorConfig& config = {});

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
                   const DepthCameraParams& cam, float max_weight = 5.0f,
                   IntegrationMode mode = IntegrationMode::Classic,
                   const ColorFrame* color = nullptr);

  /// @brief How many blocks this integrator has CHANGED since the last @ref
  ///        reset_dirty (requires @ref
  ///        TsdfIntegratorConfig::track_dirty_blocks).
  ///
  /// Not "how many were dispatched", not "how many were in view", and not "how
  /// many were stored to". The dispatch covers every active block and returns
  /// early for most; a frustum test counts the whole depth cone; and classic
  /// mode fuses the free-space cone ahead of the surface too, so counting
  /// stores would report roughly the view. Only a store that leaves
  /// `tsdf`/`weight`/`color` holding a **different** value marks a block, so a
  /// scan revisiting converged surface at `max_weight` marks nothing -- which
  /// is the steady state, and the whole reason this is narrower than the active
  /// set.
  ///
  /// Accumulates across calls, because a consumer may fuse several frames per
  /// remesh; @ref reset_dirty clears it, and the natural place to call that is
  /// immediately after an extract has consumed the set. The flags survive @ref
  /// VoxelBlockGrid::resize, which preserves every block's index.
  ///
  /// Counted on the host over a host-visible buffer, so it is O(num_blocks) and
  /// meant for diagnostics and for driving a re-mesh, not for a per-voxel path.
  ///
  /// @warning Not synchronized, and `const` only in the C++ sense: it reads a
  ///          mapping that a concurrent @ref integrate on another thread can
  ///          free outright (the flag array is reallocated when the map grows,
  ///          and `Buffer` frees synchronously). Serializing this against
  ///          @ref integrate and @ref reset_dirty is the caller's job, exactly
  ///          as it is for `mesh::MarchingCubes::release_through`.
  /// @return The count; 0 before any integrate has run, and 0 when tracking is
  ///         off.
  std::uint32_t dirty_block_count() const;

  /// @brief Clear every dirty flag, and re-arm the integrator after a topology
  ///        change (see @ref dirty_remesh_blocks).
  ///
  /// @warning Not synchronized; see @ref dirty_block_count.
  void reset_dirty();

  /// @brief The blocks an incremental re-mesh would actually have to redo: the
  ///        changed blocks dilated into the `-x/-y/-z` octant.
  ///
  /// Dirty is not the re-mesh set. Marching cubes reads a cell's eight corners
  /// as `base + {0,1}^3`, so a block's cells reach one block in `+x/+y/+z` and
  /// no further -- which inverts to: a block whose voxels changed invalidates
  /// the mesh of every block in its `{0,-1}^3` octant, itself included. Skip
  /// that and the surface goes stale exactly at block seams, under
  /// `Status::ok`.
  ///
  /// One block deep, and **not** a function of `trunc_dist` -- that governs
  /// which voxels are written (already reflected in the flags) and how far
  /// `allocate_from_depth` dilates the band, neither of which widens the
  /// meshing stencil. So the multiplier is bounded at 8x, and far below it in
  /// practice because a dirty set is a contiguous surface patch rather than
  /// scattered blocks: dilating a connected region adds roughly its perimeter.
  ///
  /// Returns the block **coordinates**, not a count: a count cannot drive the
  /// incremental extract this exists for, and the coordinates are what the
  /// walk already computes. Take `.size()` for the count.
  ///
  /// Takes the active set rather than compacting one, because every caller has
  /// just compacted (the flags carry slots, not coordinates -- the active set
  /// is what resolves them) and a second compaction inside here is a dispatch,
  /// a fence wait and a full read-back that measured 0.15-0.26 ms at the
  /// examples' defaults, on a call already O(active blocks).
  ///
  /// @param grid          The grid these flags were accumulated against -- the
  ///                      one most recently passed to @ref integrate.
  /// @param active        Its active set (@ref
  ///                      VoxelHashMap::compact_active_blocks).
  /// @param active_count  How many.
  /// @return The coordinates to re-mesh, or @ref Status::Code::InvalidArgument
  ///         if tracking is off, @p active is null with a non-zero count, @p
  ///         grid is not the grid the flags were accumulated against, or blocks
  ///         have been removed from it since (@ref
  ///         VoxelBlockGrid::topology_epoch moved). That last one is a refusal
  ///         rather than a stale answer on purpose: a removed block's geometry
  ///         is stale and its flag cannot say so -- the slot went back to a
  ///         LIFO heap and now means whichever block was allocated next -- so
  ///         the honest answer is "re-mesh everything", which only the caller
  ///         can do. @ref reset_dirty re-arms it.
  /// @warning Not synchronized; see @ref dirty_block_count.
  Result<std::vector<Vec3i>> dirty_remesh_blocks(
      const volume::VoxelBlockGrid& grid, const volume::BlockIndex* active,
      std::size_t active_count) const;

  /// @return `true` if this owns a live pipeline (`false` when moved-from).
  bool valid() const noexcept { return kernel_.valid(); }

 private:
  TsdfIntegrator() = default;

  /// @brief Re-anchor the dirty flags on @p grid and size them to it.
  ///
  /// Called from @ref integrate only when tracking is on. Split out because
  /// what it does is a contract (which grid do these flags describe, and is
  /// that still true) rather than another line of buffer bookkeeping.
  Status prepare_dirty_flags(const volume::VoxelBlockGrid& grid);

  // Borrowed (must outlive this).
  Device* device_ = nullptr;
  Allocator* allocator_ = nullptr;

  // Cached maxComputeWorkGroupCount[0] -- the device cap on a 1-D dispatch's
  // groupCountX; integrate() rejects an active set that would exceed it.
  std::uint32_t max_workgroup_count_x_ = 0;
  // The ceiling on one storage-buffer binding's range, read once at create().
  // The depth and colour frames are uploaded and bound whole each integrate().
  VkDeviceSize max_storage_buffer_range_ = 0;

  // The integrate kernel's bundled layout + pipeline + descriptor set, its set
  // allocated from pool_ (which must outlive it) by KernelSetBuilder at
  // create().
  ComputeKernel kernel_;
  DescriptorPool pool_;
  // Fixed-size camera-params SSBO (DepthCameraParams): bound once at
  // create() and rewritten each integrate(), not reallocated per frame (mirrors
  // the volume tier's persistent camera params).
  Buffer cam_buf_;
  // Color path: the persistent (separate) color-camera SSBO, and a 1-element
  // dummy bound to the color-image + color-attribute slots when no color frame
  // is fused (so every declared descriptor stays bound).
  Buffer color_cam_buf_;
  Buffer color_dummy_;
  TsdfIntegratorConfig config_{};
  // One flag per block slot, set by the kernel when it actually CHANGES a
  // voxel. Sized to the grid's num_blocks and grown (contents carried forward,
  // since a slot means the same block across a resize) when that grows; see
  // dirty_block_count(). Never allocated when tracking is off.
  Buffer dirty_blocks_;
  std::uint32_t dirty_capacity_ = 0;
  // What the flags describe: the grid most recently integrated, and its
  // topology_epoch() at that moment. A flag is keyed by block SLOT, which only
  // means anything relative to one grid and only while the block living there
  // stays live -- so both halves are recorded and checked rather than left to
  // the caller, who has no way to compare them. Borrowed for identity only:
  // never dereferenced, so a dangling grid is compared, not read.
  const volume::VoxelBlockGrid* dirty_grid_ = nullptr;
  std::uint64_t dirty_epoch_ = 0;
  // Latched when blocks were removed from dirty_grid_ while flags were live:
  // the removed geometry is stale and no flag can say so. Cleared by
  // reset_dirty().
  bool dirty_topology_stale_ = false;
};

}  // namespace volumetric_kit::recon::tsdf
