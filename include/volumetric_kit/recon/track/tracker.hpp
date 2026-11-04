// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file track/tracker.hpp
/// @brief Tracking methods side by side on one camera's frames against one
///        model. The driver places frames for fill; shadows are measured
///        beside it.
///
/// A pose is a world correction of the frame's own: the camera sits at
/// `correction * frame.depth_camera.cam_to_world`. The frame's pose is the
/// device's (ARKit's), registered once by the tag, with the device's map
/// steps removed.
///
/// Chains, in every output's order: the driver once for each entry of
/// @ref TrackerConfig::drivers, named by it; then the shadows, by their
/// names, on `drivers[0]`'s filter. Each chain has its own fitter and judge,
/// so a fit that saw fill never moves a chain whose filter excludes fill
/// (one generation). Chains share the frames and nothing else.
///
/// **The rules.**
/// - *Windows and hops.* A chain's fit at a frame of time h holds the held
///   frames in (h - window_ns, h], at most @ref kMaxFitFrames, the newest
///   kept. A fit is due at the first frame of each hop: the frame whose
///   `timestamp_ns / hop_ns` passes the last fit's (every frame for a hop
///   of 0). So every driver chain fits at the same step, on the same frames,
///   against the same model. A hop whose step was abandoned stays due.
/// - *Decisions.* A chain decides a frame once none of its fits can hold it
///   any more (the newest frame is window_ns - 1 past it): Device at once,
///   for a method with no fit kind; Anchored by the newest Anchored fit
///   that held it; Interpolated between the chain's nearest Anchored fits
///   that did not hold it, one ending before it and one starting after it,
///   each within hold_ns, once both exist; Coasting at the chain's
///   correction once the newest frame is hold_ns past it. Partial fits do
///   not anchor.
/// - *Release* reads the driver chains only. Once they can no longer fit a
///   frame: the first in priority order that anchored it (Anchored);
///   otherwise, at the first step where one interpolates it, the first that
///   does (Interpolated); otherwise, once hold_ns has run out, the frame is
///   dropped. A shadow never delays, changes or fails a release.
/// - *The ring.* A frame is held until the driver has released or dropped
///   it and every chain has decided it: up to the longer of hold_ns and the
///   methods' windows of frames. It holds each frame as recorded, its colour
///   too; a caller that does not need a released frame's colour records
///   frames without it.
///
/// All timing is by frame timestamps, never wall time, so replays repeat
/// exactly. One thread, one step outstanding. The caller owns the queue: it
/// records after the model's last write and finishes before its next.

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/base/stage_metrics.hpp"
#include "volumetric_kit/core/vulkan/fwd.hpp"
#include "volumetric_kit/grid/volume/ray_caster.hpp"
#include "volumetric_kit/grid/volume/voxel_block_grid.hpp"
#include "volumetric_kit/recon/core/math/vector_types.hpp"
#include "volumetric_kit/recon/track/correction.hpp"
#include "volumetric_kit/recon/track/export.hpp"
#include "volumetric_kit/recon/track/fitter.hpp"
#include "volumetric_kit/sensor/frame/device_frame.hpp"

namespace volumetric_kit::recon::track {

/// @brief A tracking method: a fit kind, or none, and its policy. A value.
struct Method {
  std::string name;            ///< [a-z0-9_@-]{1,32}.
  std::optional<FitKind> fit;  ///< None: the device's pose alone, no GPU.
  /// A fit holds the frames within this before its newest, at most
  /// @ref kMaxFitFrames (the newest kept); 0: the newest only.
  std::uint64_t window_ns = 0;
  /// A fit is due at the first frame at or past each multiple of this on
  /// the frames' clock; 0: at every frame.
  std::uint64_t hop_ns = 0;
  IcpConfig icp{};
  CorrectionFilterConfig judge{};
  /// The correction before any fit: the identity for the run's
  /// registration, or a second seed (another tag fix). Rigid.
  Mat4f initial_correction{1.0f};
  /// The registration that set @ref initial_correction, as a log names it;
  /// empty: the run's. A chain is never scored at a tag hold of its own
  /// seed.
  std::string seed;
};

/// @return "arkit": no fit kind, the frames' own poses.
VR_TRACK_API Method device_method();
/// @return "frame": `PointToPlane` on each frame, single-frame gates
///         (coverage 0.4, 100 inliers, ratio 0.25 over hits).
VR_TRACK_API Method frame_method();
/// @return "window": `PointToPlane` over 2 s windows every 0.5 s, 0b's gates.
VR_TRACK_API Method window_method();

/// @brief One chain of the driver: its name and what the model may answer
///        with.
struct DriverChain {
  std::string name;  ///< [a-z0-9_@-]{1,32}.
  volumetric_kit::grid::volume::CastFilter model_filter{};
};

/// @brief The methods a @ref Tracker runs.
struct TrackerConfig {
  Method driver = window_method();  ///< Needs a fit kind.
  /// At least one, in placement priority. A lane with fill passes the rig
  /// alone first, then the rig with anchored fill; a dataset with one
  /// filter passes one.
  std::vector<DriverChain> drivers;
  /// Fitted on `drivers[0]`'s filter.
  std::vector<Method> shadows{device_method()};
  /// How long a frame no fit anchored waits for an Anchored fit after it.
  std::uint64_t hold_ns = 5'000'000'000;
};

/// @return One fitter per chain, in chain order: @ref create_fitter for a
///         chain whose method has a fit kind (its filter, its method's icp,
///         its judge's see-through distance), null for one without; or
///         @ref create_fitter's refusal.
VR_TRACK_API core::Result<std::vector<std::unique_ptr<IFitter>>> create_fitters(
    core::Device& device, core::Allocator& allocator,
    const TrackerConfig& config);

/// @brief How a chain posed a frame, once none of its fits can change it.
enum class PoseFrom : std::uint8_t {
  Device = 0,    ///< No fitter: the chain's initial correction.
  Anchored = 1,  ///< The newest Anchored fit that held the frame.
  /// Anchored fits of this chain within hold_ns on both sides: rotation
  /// slerped and translation lerped by time, between the earlier fit's
  /// newest frame and the later fit's oldest.
  Interpolated = 2,
  Coasting = 3,  ///< Neither: the chain's correction when the hold ran out.
};

/// @brief A chain's decision about one frame.
struct FramePose {
  Mat4f correction{1.0f};
  PoseFrom from = PoseFrom::Coasting;
  /// The @ref ChainFit serial that set it (an interpolation's later fit); 0
  /// for none.
  std::uint64_t fit = 0;
  std::uint64_t decided_pts_ns = 0;  ///< The newest frame's time when decided.
};

/// @brief One chain's decision about one frame.
struct ChainPose {
  std::size_t chain = 0;
  sensor::DeviceFrame frame;  ///< At its device pose.
  FramePose pose;
};

/// @brief How the driver placed a frame for fill.
enum class Placement : std::uint8_t {
  /// A driver chain anchored it: the first in priority order that did.
  Anchored = 0,
  /// None did; the first that interpolates it, at the first step where one
  /// does.
  Interpolated = 1,
};

/// @brief A frame the driver placed.
struct Released {
  /// At its device pose: the caller places it with @ref corrected.
  sensor::DeviceFrame frame;
  Placement placement = Placement::Interpolated;
  /// Index into @ref TrackerConfig::drivers, which is also its chain index.
  std::size_t driver = 0;
  Mat4f correction{1.0f};
};

/// @brief One fit, and how its chain judged it.
struct ChainFit {
  std::size_t chain = 0;
  std::uint64_t serial = 0;        ///< 1, 2, ... in this Tracker.
  std::uint64_t first_pts_ns = 0;  ///< The fit's oldest frame.
  std::uint64_t last_pts_ns = 0;   ///< Its newest.
  PosePrior prior;
  TrackFit fit;
  /// The verdict, and the chain's correction after it.
  Correction judged;
  /// A shadow's fitter refused the record: why. Nothing ran and the chain
  /// is unchanged.
  std::string refused;
};

/// @brief What one step decided.
struct TrackStep {
  /// The fits this step ran or refused, in recording order.
  std::vector<ChainFit> fits;
  /// Decisions made this step, oldest frame first, each frame once per
  /// chain, in chain order.
  std::vector<ChainPose> poses;
  /// Frames the driver placed, oldest first.
  std::vector<Released> released;
  /// Frames the driver could not place: never fused.
  std::vector<std::uint64_t> dropped_ns;
};

/// @brief A recorded step, for @ref Tracker::finish or @ref Tracker::abandon.
struct PendingStep {
  std::uint64_t serial = 0;
};

/// @brief N methods on one camera's frames. An aggregate: neither copied nor
///        moved.
class VR_TRACK_API Tracker {
 public:
  /// @param config   The methods.
  /// @param fitters  One per chain, in chain order, as @ref create_fitters
  ///                 makes them (a test passes scripted ones).
  /// @return The tracker; `Status::Code::InvalidArgument` for a driver
  ///         without a fit kind, no driver chain, a chain name duplicated
  ///         or malformed, a hold_ns of 0, a method's judge out of range or
  ///         initial correction not rigid, or fitters that do not match the
  ///         chains (null exactly where a method has no fit kind).
  static core::Result<std::unique_ptr<Tracker>> create(
      TrackerConfig config, std::vector<std::unique_ptr<IFitter>> fitters);
  ~Tracker();
  Tracker(const Tracker&) = delete;
  Tracker& operator=(const Tracker&) = delete;
  Tracker(Tracker&&) = delete;
  Tracker& operator=(Tracker&&) = delete;

  /// @return The chains' names, in chain order.
  const std::vector<std::string>& chains() const noexcept;
  /// @return The configuration this was created with.
  const TrackerConfig& config() const noexcept;

  /// @brief Hold @p frame, and record into @p batch, after its prep, every
  ///        fit due at it: the driver's chains first, in priority order,
  ///        then the shadows.
  ///
  /// A shadow whose fitter refuses (`Status::Code::InvalidArgument`) gets
  /// a @ref ChainFit with `refused` set, and the step goes on.
  /// @param batch    Records the fits; the caller submits it.
  /// @param model    As `IFitter::record`.
  /// @param frame    The camera's next frame.
  /// @param metrics  Rows `"track: <chain>"`, with `"  ..<chain> pyramid"`,
  ///                 `"  ..<chain> ray cast"` and `"  ..<chain> icp"`. This
  ///                 Tracker owns the names: read the metrics before it is
  ///                 destroyed.
  /// @return The pending step. Any error leaves no frame held and no fitter
  ///         outstanding. `Status::Code::InvalidArgument` for a step
  ///         outstanding; a frame any fitter would refuse (no depth, or one
  ///         smaller than its camera or not copyable; no known
  ///         `DepthNoise`; not rigid; invalid intrinsics or range), checked
  ///         at its own record whether or not a fit is due; a frame of
  ///         another size than the first, or not newer than the last; or a
  ///         driver chain's refusal. A buffer failure poisons @p batch
  ///         (core), so the caller drops it.
  core::Result<PendingStep> record(
      core::CommandBatch& batch,
      const volumetric_kit::grid::volume::VoxelBlockGrid& model,
      sensor::DeviceFrame frame, core::StageMetrics* metrics = nullptr);
  /// @brief Judge each fit with its chain's judge, then decide frames and
  ///        release the driver's placements.
  /// @pre The batch's wait returned OK.
  /// @return The step; `Status::Code::InvalidArgument` for a pending step
  ///         not the last record's, or, dropping the step as @ref abandon
  ///         does (no fitter left holding a record), a fitter's refusal of
  ///         its finish or a fit counted at another see-through distance
  ///         than its chain's judge's (`TrackFit::see_through_m`).
  core::Result<TrackStep> finish(const PendingStep& pending);
  /// @brief The batch failed or never ran: its frame and fits are dropped,
  ///        no judge moves, and a hop due at it stays due at the next frame.
  ///        One this tracker no longer holds is ignored.
  /// @pre The batch is no longer in flight (it may read the frames this
  ///      frees).
  void abandon(const PendingStep& pending) noexcept;
  /// @brief No more frames: every held frame is decided and released, or
  ///        dropped, with the fits there are.
  /// @pre No step is outstanding; with one, nothing changes and the step is
  ///      empty.
  TrackStep flush();

 private:
  struct State;
  explicit Tracker(std::unique_ptr<State> state) noexcept;
  std::unique_ptr<State> state_;
};

/// @return @p frame with both its cameras moved by @p correction (world
///         from world).
VR_TRACK_API sensor::DeviceFrame corrected(sensor::DeviceFrame frame,
                                           const Mat4f& correction);

}  // namespace volumetric_kit::recon::track
