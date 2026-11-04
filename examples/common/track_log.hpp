// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/track_log.hpp
/// @brief One tracking run (`track::Tracker`) on disk, the same for the phone
///        lane (`live_mobile.hpp`) and a bench, and its one summary.
///
/// A run is its chains, every decision each chain made about each frame,
/// every fit and its verdict, every placement of the driver, and each step's
/// stage rows. A log keeps it in memory (@ref TrackLog::run) and, given a
/// directory, writes it there as it goes; @ref read_track_log reads the files
/// back into the same run, and @ref summarize computes every metric of a run,
/// once.
///
/// The files, one header line each, numbers in a form that reads back to the
/// same value (a matrix is 16 numbers, column-major, a pose camera to world):
/// - `run.csv`: `key,value` lines: `format` (`volumetric_kit.track_log`),
///   `version` (1), `session`, `registration`, `device`, `recon` (its
///   version), `hold_ns` and `tracking_to_world` (16 numbers).
/// - `chains.csv`: per chain, in chain order: `name`, `role` (`driver` or
///   `shadow`), `fit` (`point_to_plane`, or `none`), `window_ns`, `hop_ns`,
///   `passes`, `initial_correction` and `seed` (the rest of the line).
/// - `steps.csv`: the device's map steps that its pose chain had removed:
///   `pts_ns` and `raw_from_sent`; from `pts_ns` on, the device's own pose is
///   `raw_from_sent * sent` in its tracking world.
/// - `frames.csv`: a chain's decision about a frame: `pts_ns`, `chain`,
///   `from` (`device`, `anchored`, `interpolated`, `coasting`), `fit` (the
///   serial that set it; 0 for none), `decided_pts_ns`, `correction` and
///   `device_pose` (the frame's depth camera as the device posed it).
/// - `fits.csv`: a fit: `chain`, `serial`, `first_pts_ns`, `last_pts_ns`,
///   `frames`, `passes`, `state` (`anchored`, `partial`, `coasting`,
///   `rejected`), `prior_sigma_m`, `prior_sigma_rad`, `prior`, `fit` (the
///   fitter's correction), `judged` (the chain's after it), `lambda_max`,
///   `rel_eig_min`, `held`, `attempts`, `hits`, `inliers`, `in_front`,
///   `behind`, `rms_m`, `origin_x`..`origin_z`, `scale_m` and `refused` (the
///   rest of the line; empty unless a shadow's fitter refused).
/// - `fill.csv`: the driver's placement of a frame: `pts_ns`, `placement`
///   (`anchored`, `interpolated` or `dropped`) and `driver` (its chain).
/// - `rows.csv`: a step's stage rows: `step`, `name`, `cpu_ms`, `gpu_ms`
///   (-1 where the device half was not measured).
/// - `poses_<chain>.tum`: `t tx ty tz qx qy qz qw`, each frame's decided depth
///   camera pose in the world (`correction * device_pose`), `t` its timestamp
///   in seconds; written at @ref TrackLog::close, in time order, so evo and the
///   TUM tools read them.
/// - `tags.csv`: each hold of the origin tag (live_tag.hpp's `TagHold`):
///   `first_pts_ns`, `last_pts_ns`, `stills`, `use` (`registered`, `passed`,
///   `refused`), `miss`, `rms_px`, `worst_px`, `range_m`, `off_normal_deg`,
///   `tracking_to_world` and `reason` (the rest of the line). Written as it
///   goes, for analysis; not part of @ref TrackRun, so not read back.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "live_tag.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/base/stage_metrics.hpp"
#include "volumetric_kit/recon/core/math/vector_types.hpp"
#include "volumetric_kit/recon/track/tracker.hpp"

namespace vr_example {

namespace track = volumetric_kit::recon::track;
namespace vkc = volumetric_kit::core;

/// @brief A map step the device took out of its pose chain: from @ref pts_ns
///        on, its own pose is `raw_from_sent * sent` (cumulative).
struct DeviceStep {
  std::uint64_t pts_ns = 0;
  volumetric_kit::recon::Mat4f raw_from_sent{1.0f};
};

/// @brief Where a run's frames came from.
struct RunSource {
  std::string session;       ///< The phone's session: a file, or tcp://.
  std::string registration;  ///< Its registration file.
  std::string device;        ///< The GPU.
  /// The registration's tracking world in the world: with
  /// @ref device_steps, what places the device's own poses (`arkit_raw`).
  volumetric_kit::recon::Mat4f tracking_to_world{1.0f};
  /// The device's removed map steps, oldest first; empty: none known, and
  /// the summary has no `arkit_raw` row.
  std::vector<DeviceStep> device_steps;
};

/// @brief A chain, as the run's files name it.
struct ChainInfo {
  std::string name;
  bool driver = false;
  std::optional<track::FitKind> fit;
  std::uint64_t window_ns = 0;
  std::uint64_t hop_ns = 0;
  int passes = 0;
  volumetric_kit::recon::Mat4f initial_correction{1.0f};
  std::string seed;
};

/// @brief A chain's decision about a frame.
struct FrameRecord {
  std::uint64_t pts_ns = 0;
  std::size_t chain = 0;
  track::FramePose pose;
  volumetric_kit::recon::Mat4f device_pose{1.0f};  ///< Its depth camera.
};

/// @brief A fit and its verdict (`track::ChainFit`).
struct FitRecord {
  std::size_t chain = 0;
  std::uint64_t serial = 0;
  std::uint64_t first_pts_ns = 0;
  std::uint64_t last_pts_ns = 0;
  track::PosePrior prior;
  track::TrackFit fit;  ///< Its eigenvectors are not kept.
  track::Correction judged;
  std::string refused;
};

/// @brief The driver's placement of a frame.
struct PlacementRecord {
  std::uint64_t pts_ns = 0;
  bool dropped = false;
  track::Placement placement = track::Placement::Anchored;  ///< Placed only.
  std::size_t driver = 0;                                   ///< Placed only.
};

/// @brief One stage row of one step.
struct RowSample {
  std::uint64_t step = 0;
  std::string name;
  double cpu_ms = 0.0;
  double gpu_ms = -1.0;  ///< -1: the device half was not measured.
};

/// @brief A run, in memory: what the files hold and the summary reads.
struct TrackRun {
  RunSource source;
  std::uint64_t hold_ns = 0;
  std::vector<ChainInfo> chains;
  std::vector<FrameRecord> frames;
  std::vector<FitRecord> fits;
  std::vector<PlacementRecord> placements;
  std::vector<RowSample> rows;
};

/// @brief A run's log: kept in memory, and written to a directory as it goes.
///        An aggregate: neither copied nor moved.
///
/// @code
/// VKC_ASSIGN(std::unique_ptr<TrackLog> log, TrackLog::open(dir, *tracker,
/// source));
/// // each step:
/// VKC_ASSIGN(sensor::DeviceFrame f, prep.record(batch, frame, &log->rows()));
/// VKC_ASSIGN(track::PendingStep p, tracker->record(batch, model, f,
/// &log->rows()));
/// // ... submit, wait, log->rows().merge_device(pending.device_rows()) ...
/// VKC_ASSIGN(track::TrackStep step, tracker->finish(p));
/// VKC_TRY(log->write(step));
/// // at the end:
/// VKC_TRY(log->write(tracker->flush()));
/// VKC_TRY(log->close());
/// VKC_ASSIGN(const std::string table, summarize(log->run(), dir));
/// @endcode
class TrackLog {
 public:
  /// @param directory  Made if missing; its files are replaced. Empty: no
  ///                   files, the run kept in memory alone.
  /// @param tracker    The run's tracker: its chains and hold.
  /// @param source     Where the frames came from.
  /// @return The log; `Status::Code::IoError` for a directory or file that
  ///         cannot be made.
  static vkc::Result<std::unique_ptr<TrackLog>> open(
      const std::string& directory, const track::Tracker& tracker,
      RunSource source);
  ~TrackLog();
  TrackLog(const TrackLog&) = delete;
  TrackLog& operator=(const TrackLog&) = delete;
  TrackLog(TrackLog&&) = delete;
  TrackLog& operator=(TrackLog&&) = delete;

  /// @return The next step's rows: pass them to the step's prep and record,
  ///         and merge its batch's device rows in; @ref write takes them.
  vkc::StageMetrics& rows() noexcept;

  /// @brief Take one step: its decisions, fits and placements, and the rows
  ///        gathered since the last write, which it clears (their names are
  ///        copied, so the tracker that named them may go after).
  /// @return OK; `Status::Code::IoError` for a write that fails, or once
  ///         closed (`Status::Code::InvalidArgument`).
  vkc::Status write(const track::TrackStep& step);

  /// @brief One hold of the origin tag: a `tags.csv` row.
  /// @return OK; `Status::Code::IoError` for a write that fails, or once
  ///         closed (`Status::Code::InvalidArgument`).
  vkc::Status write(const TagHold& hold);

  /// @brief Place the run once the tag registers it: `run.csv`'s
  ///        `registration` and `tracking_to_world` rewritten, and @ref run's.
  /// @param registration       What placed it: its registration file, say.
  /// @param tracking_to_world  The registration's transform.
  /// @return OK; `Status::Code::InvalidArgument` after a step with frames, a
  ///         second time, or once closed; `Status::Code::IoError` for a write
  ///         that fails.
  vkc::Status registered(std::string registration,
                         const volumetric_kit::recon::Mat4f& tracking_to_world);

  /// @return The run so far.
  const TrackRun& run() const noexcept;

  /// @return The directory it writes into; empty for none.
  const std::string& directory() const noexcept;

  /// @brief Write the pose files and flush and close every file. Idempotent.
  /// @return OK, or the first write's failure.
  vkc::Status close();

 private:
  struct State;
  explicit TrackLog(std::unique_ptr<State> state) noexcept;
  std::unique_ptr<State> state_;
};

/// @brief Read the run a log wrote into @p directory.
/// @return The run, equal to the log's own; `Status::Code::NotFound` for a
///         missing file, `Status::Code::InvalidArgument` for another format,
///         version, or a line that does not parse (naming its file and
///         line).
vkc::Result<TrackRun> read_track_log(const std::string& directory);

/// @brief Every metric of @p run, computed here and nowhere else: one row per
///        chain in chain order, then `arkit_raw` when the run knows the
///        device's map steps (the device's poses as it delivered them, from
///        `arkit` and the steps), then the driver's fill.
///
/// Per chain: frames decided, by how (`PoseFrom`), the lock share (Anchored
/// of decided), the decision's latency (decided frame time minus the frame's)
/// p50 / p95, the distance from `arkit` at the same frames (the camera
/// centres' and the rotations', p50 / p95 / max, mm and deg; a chain is
/// compared only if `arkit` is a chain), the jitter (the RMS change of the
/// correction between consecutive Anchored frames, mm at the camera and deg),
/// its fits by verdict, and its `track: <chain>` row's device time p50 / p95.
/// `arkit_raw` has the distance alone. The fill line counts the driver's
/// placements by kind and chain, and the drops.
/// @param directory  Given, `summary.csv` (the table's numbers) and
///                   `poses_arkit_raw.tum` (with the steps) are written
///                   there too.
/// @return The table, or a file's write error.
vkc::Result<std::string> summarize(const TrackRun& run,
                                   const std::string& directory = {});

/// @return How far pose @p a is from pose @p b: their camera centres (mm) and
///         their rotations (deg), as every distance of @ref summarize is
///         measured.
std::pair<double, double> pose_gap(const volumetric_kit::recon::Mat4f& a,
                                   const volumetric_kit::recon::Mat4f& b);

}  // namespace vr_example
