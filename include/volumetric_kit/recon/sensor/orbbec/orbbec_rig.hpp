// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file sensor/orbbec/orbbec_rig.hpp
/// @brief A hardware-synchronised rig of Orbbec cameras, polled for one set
///        of frames per sync trigger.

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "volumetric_kit/recon/core/camera_params.hpp"
#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/sensor/camera_capture.hpp"
#include "volumetric_kit/recon/sensor/orbbec/export.hpp"
#include "volumetric_kit/recon/sensor/orbbec/orbbec_capture.hpp"
#include "volumetric_kit/recon/sensor/orbbec/orbbec_sync_config.hpp"
#include "volumetric_kit/recon/sensor/raw_frame.hpp"
#include "volumetric_kit/recon/sensor/rig_calibration.hpp"

namespace volumetric_kit::recon::sensor {

/// @brief The frames one sync trigger produced across the rig: processed
///        (@ref OrbbecRigFrameSet) or as the cameras captured them
///        (@ref OrbbecRigRawSet, for `sensor/utils`'s GPU pass).
///
/// Borrows the rig's storage: valid until the rig's next poll or
/// @ref OrbbecRig::stop.
template <typename Frame>
struct OrbbecRigSet {
  /// The trigger's time on the rig's clock (ns): the primary's frame's.
  std::uint64_t timestamp_ns = 0;
  /// One entry per camera, in @ref OrbbecRig::Options::cameras order; empty
  /// where that camera's frame for this trigger never arrived, or where the
  /// SDK failed on it -- the only way the primary's is empty. Each frame is
  /// posed by its camera's calibration.
  std::vector<std::optional<Frame>> frames;

  /// @return How many cameras this set holds a frame from.
  std::size_t count() const noexcept {
    std::size_t n = 0;
    for (const std::optional<Frame>& f : frames) n += f.has_value() ? 1 : 0;
    return n;
  }
  /// @return `true` when every camera's frame is here.
  bool complete() const noexcept { return count() == frames.size(); }
};

using OrbbecRigFrameSet = OrbbecRigSet<CapturedFrame>;
using OrbbecRigRawSet = OrbbecRigSet<RawFrame>;

/// @brief Counters a caller reads to see how the rig is keeping up.
struct OrbbecRigStats {
  std::uint64_t sets = 0;  ///< Sets handed out since @ref OrbbecRig::start.
  std::uint64_t incomplete = 0;  ///< Of those, sets missing a camera.
  /// Per camera, in @ref OrbbecRig::Options::cameras order. A frame whose
  /// trigger was overtaken before a poll took it counts as dropped.
  std::vector<OrbbecCaptureStats> cameras;
};

/// @brief Several Orbbec cameras wired for hardware sync, read as one rig.
///
/// @ref open checks each camera's stored sync settings against the rig's sync
/// configuration, and writes the configuration to the cameras that differ
/// only when @ref Options::apply_sync_config asks. @ref start starts every
/// secondary before the primary -- the primary's first trigger is what they
/// wait for -- and has the SDK keep the cameras' clocks on the host's. @ref
/// poll_set groups frames by that clock, within
/// @ref Options::sync_tolerance_us of a primary frame, and only then processes
/// the set's frames as @ref OrbbecCapture processes one. A trigger with a
/// secondary's frame missing is still handed out, with that camera's slot
/// empty; one the primary's frame is missing from is not, since its frames
/// name the triggers. The measurements behind the defaults are in the
/// 2026-09-27 decision.
///
/// Also an @ref ICameraCapture: @ref poll hands out the current set's frames
/// one at a time, each posed by its own camera, so a fusion loop written for
/// one camera fuses the rig unchanged. A rig opened with
/// @ref OrbbecStreamOptions::raw reads the same two ways raw, through
/// @ref poll_raw_set and @ref poll_raw, and @ref raw_frames says so. Read
/// one way between one @ref start and the next; the others return
/// @ref Status::Code::InvalidArgument.
///
/// @warning Not thread-safe: open, start, poll and stop from one thread.
class VR_SENSOR_ORBBEC_API OrbbecRig final : public ICameraCapture {
 public:
  /// @brief The cameras, their streams (the same for all), and the grouping.
  struct Options : OrbbecStreamOptions {
    /// The rig's cameras, in the order the rig reports them, and each one's
    /// sync settings -- what @ref read_orbbec_sync_config returns. At least
    /// two: one primary, the rest its secondaries.
    OrbbecRigSyncConfig sync;
    /// Each camera's pose, by serial -- what @ref read_rig_calibration
    /// returns; it may list other cameras too. Empty places every camera at
    /// the world origin.
    std::vector<RigCameraCalibration> calibration;
    /// Write @ref sync to each camera whose stored settings differ, where it
    /// persists, rather than refusing to open.
    bool apply_sync_config = false;
    /// How long @ref open waits for every camera to answer discovery.
    std::uint32_t discovery_timeout_ms = 8000;
    /// A secondary's frame within this of a primary frame on the rig clock
    /// belongs to its trigger. Under half a frame period, or neighbouring
    /// triggers would share frames.
    std::uint32_t sync_tolerance_us = 5000;
    /// How often the SDK re-syncs the cameras' clocks to the host's.
    std::uint32_t clock_sync_interval_ms = 60000;
    /// As @ref OrbbecCapture::Options::configure_sdk_logging.
    bool configure_sdk_logging = true;
  };

  /// @brief Find every camera, reconcile its sync settings, check its role and
  ///        streams, and read its calibration. Does not start streaming.
  /// @return The rig; or @ref Status::Code::InvalidArgument for options that
  ///         cannot describe one (fewer than two cameras, a repeated serial,
  ///         a calibration that is invalid or misses a camera, a sync
  ///         tolerance of zero or of half a frame period or more, a stream
  ///         @ref OrbbecCapture::open would refuse);
  ///         @ref Status::Code::NotFound naming the cameras that did not
  ///         answer; @ref Status::Code::Unsupported for cameras whose sync
  ///         settings differ from @ref Options::sync (named, field by field)
  ///         without @ref Options::apply_sync_config, for a rig that is not
  ///         one primary and its secondaries, or a camera
  ///         @ref OrbbecCapture::open would refuse; @ref Status::Code::IoError
  ///         for another SDK failure.
  static Result<OrbbecRig> open(const Options& options);

  OrbbecRig(OrbbecRig&& other) noexcept;
  OrbbecRig& operator=(OrbbecRig&& other) noexcept;
  /// Stops the rig if it is streaming.
  ~OrbbecRig() override;

  /// @return How many cameras the rig has; 0 on a moved-from rig.
  std::size_t camera_count() const noexcept;
  /// @return Camera @p i's report, in @ref Options::cameras order. Empty for
  ///         an @p i past @ref camera_count, as on a moved-from rig.
  const OrbbecDeviceInfo& device_info(std::size_t i) const noexcept;
  /// @return Camera @p i's colour camera, posed as the options say. Zeroed
  ///         for an @p i past @ref camera_count.
  const ColorCameraParams& color_camera(std::size_t i) const noexcept;
  /// @return The index of the sync primary.
  std::size_t primary() const noexcept;
  /// @return The counters since the last @ref start.
  OrbbecRigStats stats() const;

  /// @brief Sync the cameras' clocks and start every camera, secondaries
  ///        first. Idempotent.
  /// @return OK once all stream; @ref Status::Code::InvalidArgument on a
  ///         moved-from rig; @ref Status::Code::IoError if a camera refuses
  ///         or has disconnected; for H.265 colour or raw MJPEG, what
  ///         @ref OrbbecCapture::start returns when a camera's decoder does
  ///         not start. On any failure, none is left streaming.
  Status start() override;

  /// @brief Stop every camera and drop the frames held. Idempotent. The
  ///        cameras stay open, and held.
  void stop() noexcept override;

  /// @brief Take the newest trigger ready to hand out, its frames processed.
  ///
  /// A trigger is ready when every camera's frame for it has arrived, or when
  /// each missing secondary has moved past it or stayed silent for ~1.5 frame
  /// periods. An older ready trigger is dropped for a newer one, and a
  /// secondary's frame near no primary frame is dropped -- so a camera whose
  /// clock is off by more than the tolerance costs its own frames, not the
  /// rig's sets.
  ///
  /// @return The set; an empty optional when none is ready or the rig is not
  ///         started; @ref Status::Code::IoError if a camera disconnected or
  ///         its frames stopped processing (see @ref OrbbecCapture::poll);
  ///         @ref Status::Code::InvalidArgument on a moved-from rig, a rig
  ///         opened raw, or after another reader since the last
  ///         @ref start.
  Result<std::optional<OrbbecRigFrameSet>> poll_set();

  /// @brief @ref poll_set for a rig opened with @ref OrbbecStreamOptions::raw:
  ///        the trigger's frames as the cameras captured them, nothing done
  ///        to them on the host. The cameras are independent, so
  ///        `sensor::prepare_set` prepares them on the GPU at once, one
  ///        thread per camera.
  /// @return As @ref poll_set; @ref Status::Code::InvalidArgument also on a
  ///         rig not opened raw.
  Result<std::optional<OrbbecRigRawSet>> poll_raw_set();

  /// @brief The next frame of the current set, taking a new set when this one
  ///        is spent. Frames of one set come in camera order; a missing
  ///        camera is skipped.
  /// @return As @ref poll_set, one frame at a time.
  Result<std::optional<CapturedFrame>> poll() override;

  /// @brief @ref poll for a rig opened raw: the next frame of the current
  ///        @ref poll_raw_set set.
  /// @return As @ref poll_raw_set, one frame at a time.
  Result<std::optional<RawFrame>> poll_raw() override;

  /// @return `true` if the rig was opened with @ref OrbbecStreamOptions::raw,
  ///         so its frames come through @ref poll_raw and @ref poll_raw_set;
  ///         `false` otherwise, and on a moved-from rig.
  bool raw_frames() const noexcept override;

  /// @return `true` on a moved-from rig and once any camera has disconnected.
  bool exhausted() const noexcept override;

 private:
  struct Impl;
  explicit OrbbecRig(std::unique_ptr<Impl> impl) noexcept;
  std::unique_ptr<Impl> impl_;
};

}  // namespace volumetric_kit::recon::sensor
