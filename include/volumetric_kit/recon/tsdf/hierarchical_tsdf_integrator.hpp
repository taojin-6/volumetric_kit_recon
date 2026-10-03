// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file tsdf/hierarchical_tsdf_integrator.hpp
/// @brief Online projective fusion and depth-driven refinement of dyadic
/// leaves.

#include <memory>
#include <vector>

#include "volumetric_kit/recon/tsdf/tsdf_integrator.hpp"
#include "volumetric_kit/recon/volume/hierarchical_field_view.hpp"

namespace volumetric_kit::recon::tsdf {

/// @brief Metric controls for an incoming-depth refinement estimate.
///
/// The classifier compares a local depth patch with its tangent plane and
/// scales the residual to each candidate spacing. Candidate cell normal spans
/// must also fit the projective truncation band at the observed view angle, so
/// a grazing planar surface can require fine samples. This is a refinement
/// heuristic, not a bound on reconstruction error. Depth discontinuities
/// and valid foreground beside missing depth request the finest supported
/// level. Image boundaries and unseen nodes receive no request.
struct HierarchicalRefinementParams {
  float surface_error = 0.002f;    ///< Allowed estimated plane error, metres.
  float noise_floor = 0.0005f;     ///< Residual ignored as measurement noise.
  std::uint32_t pixel_stride = 4;  ///< Sample one pixel in each stride square.
  std::uint32_t patch_radius = 4;  ///< Tangent patch radius in depth pixels.
  /// Enable an additional GPU pass for conservative merge evidence. It requires
  /// a direct incoming surface vote in the sibling group, valid depth at every
  /// projected sample, and a smooth center/corner footprint. Existing weighted
  /// samples must be visible within the physical truncation band. Entirely
  /// unknown children may receive evidence behind the observed surface because
  /// they contain no historical samples to lose. Fine surface votes remain a
  /// veto; this pass never refines band or free-space leaves. Off by default
  /// because scanning each refined leaf adds work when merges are not used.
  bool support_coarsening = false;
};

/// @brief Fuses cell-centered hierarchical leaves without changing uniform
/// fusion.
///
/// Uses the same projective distance, depth sampling, color convention, and
/// integration weights as TsdfIntegrator. Only sample placement and addressing
/// differ. Frames are dispatched in order in one CommandBatch. All field and
/// refinement buffers stay on the device; no voxel readback is performed.
/// Calls must be serialized with the grid and its other consumers. Device and
/// allocator passed to create must outlive this object.
class VR_TSDF_API HierarchicalTsdfIntegrator {
 public:
  /// @brief Build fusion and refinement pipelines.
  /// @param device Compute device that outlives this integrator.
  /// @param allocator Allocator that outlives this integrator.
  /// @return A live integrator, invalid_argument for empty device/allocator,
  ///         or a backend/allocation error.
  static Result<HierarchicalTsdfIntegrator> create(Device& device,
                                                   Allocator& allocator);

  HierarchicalTsdfIntegrator() noexcept;
  ~HierarchicalTsdfIntegrator();
  HierarchicalTsdfIntegrator(HierarchicalTsdfIntegrator&& other) noexcept;
  HierarchicalTsdfIntegrator& operator=(
      HierarchicalTsdfIntegrator&& other) noexcept;
  HierarchicalTsdfIntegrator(const HierarchicalTsdfIntegrator&) = delete;
  HierarchicalTsdfIntegrator& operator=(const HierarchicalTsdfIntegrator&) =
      delete;

  /// @return Whether this owns live pipelines; false after a move.
  bool valid() const noexcept;

  /// @brief Integrate a camera set into the current leaves.
  /// @param field Current field view; topology stays fixed through this call.
  /// @param frames Posed depth and optional canonical color frames.
  /// @param max_weight Finite positive cap on accumulated observation weight.
  /// @param mode Classic retains free space; Dynamic clears receded surfaces.
  /// @param metrics Optional host/device timing rows.
  /// @return OK, or invalid input/storage or a backend error. Validation takes
  ///         place before any field values are changed.
  Status integrate(const volume::HierarchicalFieldView& field,
                   const std::vector<FrameInput>& frames,
                   float max_weight = 5.0f,
                   IntegrationMode mode = IntegrationMode::Classic,
                   StageMetrics* metrics = nullptr);

  /// @brief Estimate desired leaf levels directly from incoming depth.
  /// @param field Current field; this pass changes no TSDF values or topology.
  /// @param frames Depth frames; color is not used by this pass. Empty frame
  ///               sets clear every request to UINT32_MAX.
  /// @param params Refinement controls, validated before dispatch.
  /// @param metrics Optional host/device timing rows.
  /// @return A borrowed device buffer of node_capacity uint32 values: desired
  ///         level per current leaf, UINT32_MAX for no evidence. Valid until
  ///         the next classify call, integrator move, or destruction. Consume
  ///         it with HierarchicalGrid::split or merge before changing topology.
  ///         With support_coarsening enabled, unsupported coarse surface votes
  ///         are suppressed; any fine surface request is preserved.
  Result<const Buffer*> classify(
      const volume::HierarchicalFieldView& field,
      const std::vector<FrameInput>& frames,
      const HierarchicalRefinementParams& params = {},
      StageMetrics* metrics = nullptr);

 private:
  struct Impl;
  explicit HierarchicalTsdfIntegrator(std::unique_ptr<Impl> impl) noexcept;
  std::unique_ptr<Impl> impl_;
};

}  // namespace volumetric_kit::recon::tsdf
