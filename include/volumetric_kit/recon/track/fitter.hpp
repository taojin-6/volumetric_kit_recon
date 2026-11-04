// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file track/fitter.hpp
/// @brief A fitter is a local refiner. From a prior correction it fits 1..K
///        frames of one pose chain to a model on the device, and returns one
///        world correction and its conditioning. It is the tier's one backend
///        interface. It records into the caller's `core::CommandBatch` and
///        never submits or waits.
///
/// A method that proposes a pose from no prior is not a fitter: a global
/// localizer, learned registration, a many-seed re-lock, RGB 2D-3D. It hands a
/// chain a new prior, and the chain's fitter then refines it.

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/base/stage_metrics.hpp"
#include "volumetric_kit/core/vulkan/fwd.hpp"
#include "volumetric_kit/grid/volume/ray_caster.hpp"
#include "volumetric_kit/grid/volume/voxel_block_grid.hpp"
#include "volumetric_kit/recon/core/math/vector_types.hpp"
#include "volumetric_kit/recon/track/export.hpp"
#include "volumetric_kit/sensor/frame/device_frame.hpp"

namespace volumetric_kit::recon::track {

/// @brief The most frames one fit holds; they are cast as one stacked view.
inline constexpr std::size_t kMaxFitFrames = 32;

/// @brief Where a fit starts, and how firmly it is held there.
struct PosePrior {
  /// World from the frames' registered world: frame i sits at
  /// `correction * frames[i].depth_camera.cam_to_world`. The seed and the
  /// prior's mean. Rigid.
  Mat4f correction{1.0f};
  /// The prior term, `diag(1/sigma_m^2 x3, 1/sigma_rad^2 x3)` on the twist
  /// about the frames' mean camera centre, against the data rescaled to
  /// @ref IcpConfig::prior_n_eff points for each frame with hits. 0 holds
  /// nothing on that block; both 0: no prior.
  float sigma_m = 0.0f;
  float sigma_rad = 0.0f;  ///< As @ref sigma_m, on the rotation.
};

/// @brief What one fit found.
///
/// Twists are (v, w) about @ref origin, in world axes, with v divided by
/// @ref scale_m. The data's H at the finest level's last evaluation is
/// carried as the device's own eigensystem:
/// `H = lambda_max * D diag(relative_eigenvalues) D^T`. A fit that solved
/// nothing (no data, an empty model, a step that diverged) has
/// @ref lambda_max 0, every relative eigenvalue 0, D the identity, every
/// direction held and the prior's correction. A kind whose solve is not
/// Gauss-Newton returns the point-to-plane system at its prior, so one judge
/// reads every kind.
struct TrackFit {
  /// The prior's, moved along its kept directions only; rigid, so it can be
  /// the next fit's prior.
  Mat4f correction{1.0f};
  float lambda_max = 0.0f;
  /// lambda / lambda_max, ascending.
  std::array<float, 6> relative_eigenvalues{};
  /// D: unit twists, column-major (direction i is elements 6i..6i+5), in
  /// the order of @ref relative_eigenvalues.
  std::array<float, 36> directions{};
  /// Bit i: direction i kept the prior (below
  /// @ref IcpConfig::min_relative_eigenvalue).
  std::uint8_t held = 0x3f;
  std::uint8_t passes = 0;   ///< Casts run.
  std::uint32_t frames = 0;  ///< Frames fitted.
  /// Valid pixels at the finest level, over the frames.
  std::uint32_t attempts = 0;
  std::uint32_t hits = 0;     ///< Of them, those the model answered.
  std::uint32_t inliers = 0;  ///< Of the hits, those inside every gate.
  /// Of the hits outside a gate, those more than @ref see_through_m in front
  /// of the model's surface along the cast ray (an occluder: a person); 0
  /// with the distance 0. Never an inlier: `inliers + in_front + behind`
  /// is at most @ref hits.
  std::uint32_t in_front = 0;
  /// As @ref in_front, behind the surface (the view sees through the model:
  /// something left).
  std::uint32_t behind = 0;
  /// The distance @ref in_front and @ref behind counted past (metres): the
  /// fitter's `FitterConfig::see_through_m`; 0 counted none.
  float see_through_m = 0.0f;
  float rms_m = 0.0f;  ///< The inliers' RMS point-to-plane residual.
  /// The twists' centre: the frames' mean camera centre at the prior (world).
  Vec3f origin{0.0f};
  /// The twists' translation scale: the finest points' median distance from
  /// @ref origin at the prior (metres).
  float scale_m = 1.0f;

  /// @return Whether the fit solved anything.
  bool solved() const noexcept { return lambda_max > 0.0f; }
};

/// @brief A recorded fit, for @ref IFitter::finish or @ref IFitter::abandon.
struct PendingFit {
  std::uint64_t serial = 0;
};

/// @brief The rows a fit adds to the caller's metrics. `core::StageMetrics`
///        stores names by pointer and matches rows by text, so they outlive
///        every read, and two fitters in one metrics set need different names.
struct FitRows {
  const char* total = "track";
  const char* pyramid = "  ..pyramid";
  const char* cast = "  ..ray cast";
  const char* icp = "  ..icp";
};

/// @brief A fitting backend. One thread, one record outstanding.
///        Implementations are `final`; the library's come from
///        @ref create_fitter.
/// @warning The `Device` and `Allocator` outlive it.
class VR_TRACK_API IFitter {
 public:
  virtual ~IFitter() = default;
  IFitter(const IFitter&) = delete;
  IFitter& operator=(const IFitter&) = delete;

  /// @brief Record one fit of @p frames to @p model from @p prior into
  ///        @p batch, after what it holds (the newest frame's prep).
  ///
  /// The fitter keeps each frame's pyramid by timestamp, and reuses it only
  /// for the same depth buffer; otherwise it rebuilds it. What a record
  /// builds counts only once its @ref finish returns. An @ref abandon, or a
  /// failed record, drops it.
  /// @param batch    Records the fit; the caller submits it.
  /// @param model    Read only; not written, grown or freed until @p batch
  ///                 resolves.
  /// @param frames   1..@ref kMaxFitFrames of one camera and one size,
  ///                 timestamps ascending, one pose chain (no map step
  ///                 between them), each with depth and a known
  ///                 `DepthNoise`. They and their buffers stay alive until
  ///                 @p batch resolves.
  /// @param prior    Where the fit starts.
  /// @param metrics  Optional: @p rows' four rows, host halves now, device
  ///                 halves from `core::PendingBatch::device_rows`.
  /// @param rows     The rows' names; they outlive every read of @p metrics.
  /// @return The pending fit. With nothing recorded:
  ///         `Status::Code::InvalidArgument` for a record outstanding, no
  ///         frames or too many, mixed sizes, timestamps not ascending, a
  ///         prior not rigid or with a negative or non-finite sigma, or a
  ///         frame or model this kind cannot read (saying which). After the
  ///         first commands, so the caller drops @p batch: a buffer failure.
  ///         A failed record leaves nothing outstanding.
  virtual core::Result<PendingFit> record(
      core::CommandBatch& batch,
      const volumetric_kit::grid::volume::VoxelBlockGrid& model,
      const std::vector<sensor::DeviceFrame>& frames, const PosePrior& prior,
      core::StageMetrics* metrics = nullptr, const FitRows& rows = {}) = 0;
  /// @pre The batch's wait returned OK.
  /// @return The fit; `Status::Code::InvalidArgument` for a pending fit not
  ///         the last record's, or a readback not this record's (its batch
  ///         never ran).
  virtual core::Result<TrackFit> finish(const PendingFit& pending) = 0;
  /// @brief Drop a record whose batch failed or never ran. One this fitter
  ///        no longer holds is ignored.
  /// @pre The batch is no longer in flight (its `PendingBatch` is gone, or
  ///      `in_flight()` is false): the fitter may reuse what it bound.
  virtual void abandon(const PendingFit& pending) noexcept = 0;

 protected:
  IFitter() = default;
};

/// @brief The residuals. Closed: a kind is added in one PR and deleted when
///        its question closes.
enum class FitKind : std::uint8_t {
  /// Projective point-to-plane (KinectFusion) against one ray-cast view of
  /// each frame at the estimate: 3 levels, Huber, the eigen guard, the prior
  /// term.
  PointToPlane = 1,
};
/// Every kind, in value order. The conformance test and the bench parser
/// iterate it.
inline constexpr std::array<FitKind, 1> kFitKinds{FitKind::PointToPlane};
/// @return The kind's stable name ("point_to_plane"): logs, run files, the
///         bench's syntax; "unknown" for a value outside the enum.
VR_TRACK_API const char* to_string(FitKind kind) noexcept;

/// @brief The Gauss-Newton kinds' settings.
struct IcpConfig {
  /// Coarsest level first [M, 0a]; each 0..100. All 0: one evaluation at the
  /// prior and no step, a score. The finest level at 0 ends the fit with one
  /// evaluation there.
  std::array<int, 3> iterations{4, 5, 10};
  /// Casts a fit, 1..8; each after the first recasts at the moved estimate.
  /// A pass that moves under 1 cm / 0.3 deg (at the frames' mean camera
  /// centre) skips the remaining passes' solves, but not their casts.
  int passes = 1;
  /// The two coarser levels' distance gates, coarsest first (metres).
  std::array<float, 2> coarse_gates_m{0.10f, 0.05f};
  /// The finest gate: max(gate_sigmas x sigma, finest_gate_floor_m).
  float gate_sigmas = 3.0f;
  float finest_gate_floor_m = 0.015f;
  float normal_cos = 0.866f;  ///< 30 degrees.
  float huber_sigmas = 2.0f;
  /// The model's own noise, in quadrature with the frame's sigma(z).
  float model_sigma_m = 0.005f;
  /// A frame point whose normal is farther than this from its ray is
  /// dropped (degrees, in (0, 90]).
  float grazing_deg = 75.0f;
  /// The band dropped beside a depth edge or a hole at level 0 (pixels,
  /// 0..4); 1 at the coarser levels.
  int edge_px = 1;
  /// tau: directions of the scaled H below this share of its largest
  /// eigenvalue keep the prior [M, 0a]; in (0, 1).
  float min_relative_eigenvalue = 1e-3f;
  /// Points each frame with hits counts for against the prior.
  float prior_n_eff = 500.0f;
};

/// @brief What @ref create_fitter builds.
struct FitterConfig {
  FitKind kind = FitKind::PointToPlane;
  /// What the model may answer with, for every read of it. Copied; its
  /// attribute name outlives the fitter, as an `inline constexpr` name's
  /// does.
  volumetric_kit::grid::volume::CastFilter model_filter{};
  IcpConfig icp{};
  /// The distance @ref TrackFit::in_front and @ref TrackFit::behind count
  /// past (metres), at the finest level; 0 counts none. `create_fitters`
  /// takes the method's judge's, and each fit carries it
  /// (@ref TrackFit::see_through_m), so a `Tracker` refuses a fit counted at
  /// another distance than its judge's.
  float see_through_m = 0.0f;
};

/// @return The fitter; `Status::Code::InvalidArgument` for a config out of
///         range or a kind this version does not know; or what
///         `check_device_requirements` refuses.
VR_TRACK_API core::Result<std::unique_ptr<IFitter>> create_fitter(
    core::Device& device, core::Allocator& allocator,
    const FitterConfig& config);

}  // namespace volumetric_kit::recon::track
