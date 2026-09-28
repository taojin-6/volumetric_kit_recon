// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file tsdf/mesh_integrator.hpp
/// @brief Write a triangle mesh's truncated distance field into a
///        @ref VoxelBlockGrid's `tsdf` + `weight` attributes, signed or as a
///        shell.

#include <cstdint>

#include "volumetric_kit/recon/core/buffer.hpp"
#include "volumetric_kit/recon/core/compute_kernel.hpp"
#include "volumetric_kit/recon/core/descriptor.hpp"
#include "volumetric_kit/recon/core/gpu_timer.hpp"
#include "volumetric_kit/recon/core/math/vector_types.hpp"
#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/core/stage_metrics.hpp"
#include "volumetric_kit/recon/tsdf/export.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"

namespace volumetric_kit::recon {
class Device;
class Allocator;
}  // namespace volumetric_kit::recon

namespace volumetric_kit::recon::tsdf {

/// @brief How @ref MeshIntegrator::integrate turns distance to the mesh into a
///        signed field.
enum class MeshSdfMode : std::uint32_t {
  /// Signed distance to the mesh: negative inside, positive outside, with the
  /// sign read off the closest triangle's face normal -- the rule the research
  /// codec shipped. For a closed mesh whose faces wind counter-clockwise seen
  /// from outside (`cross(v1 - v0, v2 - v0)` points out). The surface extracts
  /// where the mesh is.
  ///
  /// Nothing about the mesh is checked, and no topology is built: the field is
  /// the codec's input, not a reconstruction of the mesh to be measured
  /// against it (the 2026-09-27 decision). What that gives up: an open mesh
  /// grows a skirt up to `trunc_dist` past its rim, a mesh wound inside out
  /// comes back inside out, and past an edge or corner whose faces meet at
  /// under 90 degrees part of its region can take the wrong side, since the
  /// faces tied for nearest there disagree and the lowest-indexed one decides.
  /// A 90-degree edge off the grid's axes does too: past a corner, on the line
  /// that continues an edge, a tied face's normal is perpendicular to the
  /// offset and rounding picks the side.
  /// Use @ref Shell for a mesh that is not closed.
  Signed = 0,
  /// Unsigned distance minus a shell half-thickness: negative within
  /// @ref MeshSdfParams::shell_voxels of any triangle, positive beyond. Needs
  /// nothing from the mesh -- open, non-manifold, self-intersecting and
  /// inconsistently wound meshes are all fine -- and extracts as a thin solid
  /// around every triangle: two walls around an open sheet, and around a closed
  /// solid an outer surface plus an inner one the same distance inside.
  ///
  /// This is the unsigned offset field of Xu & Barbic, "Signed Distance Fields
  /// for Polygon Soup Meshes" (GI 2014), before their interior-component
  /// removal.
  Shell = 1,
};

/// @brief The per-call parameters of @ref MeshIntegrator::integrate.
struct MeshSdfParams {
  /// How the field is signed.
  MeshSdfMode mode = MeshSdfMode::Signed;
  /// @ref MeshSdfMode::Shell only: the shell half-thickness, in voxels. The
  /// prior engine's 1.5 by default.
  ///
  /// Refused below `sqrt(3) / 2` (0.866): a point can be that far from every
  /// voxel, so a thinner shell can fall between voxels and extract with holes.
  /// Refused at or above `trunc_dist` too, where no observed voxel would ever
  /// read as outside.
  float shell_voxels = 1.5f;
};

/// @brief What one @ref MeshIntegrator::integrate wrote.
struct MeshIntegrateStats {
  /// Triangles that took part. A zero-area or non-finite triangle is skipped.
  std::uint32_t triangles = 0;
  /// Blocks written: every block the mesh's truncation band reaches.
  std::uint32_t blocks = 0;
  /// Triangle-in-block incidences, which is what the kernel's cost scales
  /// with: each voxel measures every triangle binned into its block.
  std::uint32_t bin_entries = 0;
  /// The dispatches the write was split into, so that none measures more than
  /// @ref MeshIntegrator::kMaxDispatchBinEntries bin entries.
  std::uint32_t dispatches = 0;
};

/// @brief Writes a triangle mesh's truncated distance field into a
///        @ref VoxelBlockGrid's `tsdf` + `weight` attributes.
///
/// Every voxel of every block the mesh's truncation band reaches is
/// **overwritten**, not blended: a voxel within `trunc_dist` of the mesh gets
/// its distance, clamped to `+-trunc_dist`, and `weight = 1`; every other voxel
/// of those blocks gets `tsdf = 0, weight = 0`, which is what the mesher and
/// the codec read as unobserved. That makes the result look exactly like a
/// frame the codec decoded, and it is what a mesh sequence -- one mesh per
/// frame -- converts through. Blocks the band does not reach keep whatever
/// they held, so to convert one mesh on its own, clear the grid first. The
/// grid's `color` attribute, if it has one, is not touched.
///
/// The blocks must already be allocated:
/// @ref VoxelHashMap::allocate_from_triangles with the same mesh allocates
/// exactly them, because both calls enumerate the same candidates through
/// `volume::triangle_candidate_offsets` and keep them with the same band test.
/// A band block that is missing is refused, with the count, before anything is
/// written.
///
/// Two dispatches bin every triangle into each block its band reaches (a
/// count, then a fill), each over one work item per (triangle, candidate block)
/// pair, so a large triangle costs more items and never a longer one. Then one
/// thread per voxel measures its block's bin, in as many dispatches as it takes
/// to keep each under @ref kMaxDispatchBinEntries bin entries. The minimum over
/// a bin is taken with the triangle index as tie-break, so the result does not
/// depend on the order the atomics filled the bin in -- the same mesh writes
/// the same bytes.
///
/// A voxel's cost is its bin, which a mesh near the grid's resolution keeps in
/// the hundreds. A mesh far finer than the voxels is where it grows, so a bin
/// past @ref kMaxBinTriangles is refused rather than measured: decimate the
/// mesh toward the voxel size first.
///
/// @warning The @ref Device and @ref Allocator passed to @ref create must
///          outlive this object; it stores references to them. The grid must be
///          quiescent across the call: the binning kernel probes its hash table
///          without a lock, as the mesh tier does.
///
/// @note The blocks this writes are not reported as dirty. Dirty flags belong
///       to the @ref TsdfIntegrator that fuses into a grid, so a grid written
///       here is meshed with a full extract, not
///       `mesh::MarchingCubes::extract_device_incremental` against another
///       integrator's flags.
class VR_TSDF_API MeshIntegrator {
 public:
  /// The most triangles one block's bin may hold. Each voxel of the block
  /// measures all of them in one thread, so this bounds how long a thread
  /// runs. About 230 times the average bin of the 2026-09-27 entry's sphere,
  /// whose triangles are 1.6 voxels on a side; a bin reaches it with
  /// triangles near a tenth of a voxel across, detail the grid cannot hold.
  static constexpr std::uint32_t kMaxBinTriangles = 1u << 16;
  /// The most bin entries one integrate dispatch measures: each is one
  /// closest-point test per voxel of its block. A write with more is split
  /// across dispatches, so no one submission grows with the mesh -- the shape
  /// the 2026-08-08 entry records hanging an M5 iPad.
  static constexpr std::uint32_t kMaxDispatchBinEntries = 1u << 20;

  /// @brief Build the binning and integrate pipelines on @p device.
  /// @param device     The compute device (must outlive this object).
  /// @param allocator  The allocator its transient buffers come from (must
  ///                   outlive this).
  /// @return The integrator, or a non-OK @ref Status if a pipeline or
  ///         descriptor object fails to build.
  static Result<MeshIntegrator> create(Device& device, Allocator& allocator);

  // Rule of zero: every owned pipeline / pool / buffer self-frees and
  // self-resets on move; device_ / allocator_ are borrowed, so the defaulted
  // moves leave a moved-from integrator empty (valid() == false).
  ~MeshIntegrator() = default;
  MeshIntegrator(MeshIntegrator&&) noexcept = default;
  MeshIntegrator& operator=(MeshIntegrator&&) noexcept = default;
  MeshIntegrator(const MeshIntegrator&) = delete;
  MeshIntegrator& operator=(const MeshIntegrator&) = delete;

  /// @brief Write the mesh's truncated distance field into @p grid.
  /// @param grid            A grid carrying float `tsdf` and `weight`
  ///                        attributes, with the mesh's band allocated (see
  ///                        the class notes).
  /// @param vertices        World-space vertex positions, metres.
  /// @param vertex_count    How many @p vertices.
  /// @param indices         `3 * triangle_count` indices into @p vertices.
  /// @param triangle_count  How many triangles. 0 writes nothing.
  /// @param params          The mode, and the shell's thickness.
  /// @param metrics         Optional: receives a `"mesh integrate"` row with
  ///                        both halves.
  /// @return What was written, or a non-OK @ref Status:
  ///         @ref Status::Code::InvalidArgument for a moved-from integrator; a
  ///         grid without float `tsdf` / `weight`; an unknown mode or a shell
  ///         thickness outside `[sqrt(3)/2, trunc_dist)`; a null @p vertices /
  ///         @p indices with triangles to read, an index at or past
  ///         @p vertex_count, or a mesh too large for the grid (see
  ///         `volume::triangle_candidate_offsets`); a band block that is not
  ///         allocated; a bin past
  ///         @ref kMaxBinTriangles; or a buffer past what one storage-buffer
  ///         binding can cover, or a binning pass past what one 1-D dispatch
  ///         can launch. Every refusal comes before the grid is written.
  ///         Otherwise whatever a buffer or a dispatch returns; a dispatch
  ///         that fails after the first has written leaves the blocks before
  ///         it written.
  Result<MeshIntegrateStats> integrate(volume::VoxelBlockGrid& grid,
                                       const Vec3f* vertices,
                                       std::uint32_t vertex_count,
                                       const std::uint32_t* indices,
                                       std::uint32_t triangle_count,
                                       const MeshSdfParams& params = {},
                                       StageMetrics* metrics = nullptr);

  /// @return `true` if this owns live pipelines (`false` when moved-from).
  bool valid() const noexcept { return bin_.valid() && integrate_.valid(); }

 private:
  MeshIntegrator() = default;

  // Borrowed (must outlive this).
  Device* device_ = nullptr;
  Allocator* allocator_ = nullptr;

  // Device limits read once at create(): the cap on a 1-D dispatch's
  // groupCountX, and on one storage-buffer binding's range. Every input here is
  // per-call and bound whole, so each is checked against the second.
  std::uint32_t max_workgroup_count_x_ = 0;
  VkDeviceSize max_storage_buffer_range_ = 0;

  // The binning kernel (dispatched twice: count, then fill) and the integrate
  // kernel, their sets allocated from pool_ (which must outlive them).
  ComputeKernel bin_;
  ComputeKernel integrate_;
  DescriptorPool pool_;
  // The device-span collector, idle until a caller passes a StageMetrics.
  GpuTimer gpu_timer_;
  // The count pass's one output the host must read before anything is
  // written: how many band blocks the hash table could not find.
  Buffer missing_;
  // A 1-element stand-in for the fill pass's output during the count pass, so
  // every declared descriptor stays bound.
  Buffer dummy_;
};

}  // namespace volumetric_kit::recon::tsdf
