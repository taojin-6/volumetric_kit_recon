// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file sensor/orbbec/orbbec_capture.hpp
/// @brief An Orbbec RGB-D camera (the Femto Mega rig) as an
///        @ref volumetric_kit::recon::sensor::ICameraCapture.
///
/// No Orbbec SDK type appears here -- the SDK is held behind a pointer to an
/// implementation -- so a consumer includes this header without the SDK's
/// headers. Linking still needs the SDK's library, which the target carries.
/// The measurements behind the frame path are in the 2026-09-26 decision.

#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "volumetric_kit/recon/core/camera_params.hpp"
#include "volumetric_kit/recon/core/math/vector_types.hpp"
#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/sensor/camera_capture.hpp"
#include "volumetric_kit/recon/sensor/orbbec/export.hpp"

namespace volumetric_kit::recon::sensor {

/// @brief A camera's role in a hardware-synchronised rig, as the camera itself
///        reports it.
///
/// Read at @ref OrbbecCapture::open and never written: the role persists on the
/// camera, and changing it for a one-camera run would break the rig for the
/// next. See @ref waits_for_primary for what it means to a caller.
enum class OrbbecSyncMode {
  FreeRun,     ///< Not synchronised; streams on its own clock.
  Standalone,  ///< Not synchronised; streams on its own clock.
  Primary,     ///< Streams on its own and drives the sync line.
  Secondary,   ///< Captures only on the primary's sync signal.
  /// The rig's secondaries. The SDK says this mode captures on its own and
  /// re-times to a signal; measured, it delivers nothing without one, so this
  /// driver treats it as @ref Secondary.
  SecondarySynced,
  /// Captures only on a host trigger, which this driver never sends;
  /// @ref OrbbecCapture::open refuses it.
  SoftwareTriggering,
  /// Captures only on a trigger another camera sends down the sync line.
  HardwareTriggering,
  Other,  ///< A mode this driver does not name.
};

/// @param mode  A camera's reported sync mode.
/// @return `true` when the camera produces frames only on another camera's
///         signal. Started on its own, such a camera delivers nothing --
///         @ref OrbbecCapture::poll keeps returning no frame, as if it were
///         merely slow. `false` for @ref OrbbecSyncMode::SoftwareTriggering,
///         which waits for the host and which @ref OrbbecCapture::open
///         refuses.
VR_SENSOR_ORBBEC_API bool waits_for_primary(OrbbecSyncMode mode) noexcept;

/// @return A stable lowercase name for @p mode (`"primary"`,
///         `"secondary-synced"`, ...), for logs.
VR_SENSOR_ORBBEC_API const char* to_string(OrbbecSyncMode mode) noexcept;

/// @brief What the camera said about itself when it was opened.
struct OrbbecDeviceInfo {
  std::string name;              ///< Model, e.g. "Orbbec Femto Mega".
  std::string serial;            ///< Serial number, e.g. "CL2A141000N".
  std::string firmware_version;  ///< Firmware version string.
  std::string connection_type;   ///< "Ethernet", "USB3.1", ...
  std::string ip_address;        ///< Empty for a USB camera.
  OrbbecSyncMode sync_mode = OrbbecSyncMode::Other;  ///< Its rig role.
};

/// @brief Counters a caller reads to see whether it keeps up with the camera.
struct OrbbecCaptureStats {
  /// Synchronised depth + colour pairs the camera delivered since @ref
  /// OrbbecCapture::start.
  std::uint64_t received = 0;
  /// Pairs handed out by @ref OrbbecCapture::poll.
  std::uint64_t delivered = 0;
  /// Pairs replaced by a newer one before any poll took them -- the contract's
  /// "dropped, not queued", counted.
  std::uint64_t dropped = 0;
  /// Pairs a poll took but could not hand out, skipped or refused. Each pair
  /// is counted once, so `delivered + dropped + failed <= received`; the
  /// difference is a pair still pending or discarded by
  /// @ref OrbbecCapture::stop.
  std::uint64_t failed = 0;
};

/// @brief One Orbbec RGB-D camera, polled for posed frames with depth
///        registered to colour.
///
/// Each @ref poll takes the newest synchronised pair and, on the polling
/// thread:
/// 1. **undistorts** the colour image to the pinhole model this repo projects
///    with;
/// 2. **registers depth** to that colour camera with the SDK's alignment, which
///    corrects the depth lens but ignores the colour one -- hence step 1: after
///    both, the two images share one pinhole camera;
/// 3. converts depth to metres and colour to the contract's `R | G<<8 | B<<16`
///    words, in storage this object reuses on the next @ref poll.
///
/// Both of the frame's cameras are therefore the colour camera, posed by
/// @ref Options::cam_to_world; the depth one is derived with
/// @ref depth_from_registered_color so the two cannot drift apart.
///
/// A camera wired as a sync secondary starts cleanly and then delivers nothing
/// unless its primary streams; @ref device_info names its role. A camera the
/// SDK reports removed never streams again through this object: @ref poll and
/// @ref start return @ref Status::Code::IoError and @ref exhausted is `true`.
///
/// @warning Not thread-safe: open, start, poll and stop from one thread. The
///          SDK delivers frames on a thread of its own, which this class
///          hands over internally.
class VR_SENSOR_ORBBEC_API OrbbecCapture final : public ICameraCapture {
 public:
  /// @brief Which camera, which streams, and where the camera sits.
  struct Options {
    /// Serial number of the camera to open. Empty opens the only camera that
    /// answers -- after waiting out all of @ref discovery_timeout_ms, since
    /// cameras answer seconds apart -- and is refused when more than one does.
    /// Naming the camera skips the wait.
    std::string serial;
    /// How long @ref open re-queries the network for the camera. An Ethernet
    /// camera can take seconds to answer from cold, so one query is not proof
    /// of absence.
    std::uint32_t discovery_timeout_ms = 8000;
    /// Depth stream mode. The default is the Femto Mega's narrow-field
    /// unbinned mode; 320x288, 512x512 and 1024x1024 (15 fps) are the others.
    std::uint32_t depth_width = 640;
    std::uint32_t depth_height = 576;  ///< See @ref depth_width.
    /// Colour stream size. Registered depth is produced at this size, so it
    /// also sets the frame's depth resolution.
    std::uint32_t color_width = 1280;
    std::uint32_t color_height = 720;  ///< See @ref color_width.
    /// Frame rate of both streams; the camera pairs them only at one rate.
    std::uint32_t fps = 30;
    /// Reject depth nearer than this (metres); stamped on every frame's depth
    /// camera as the fusion gate.
    float min_depth = 0.25f;
    /// Reject depth farther than this (metres).
    float max_depth = 5.0f;
    /// Colour camera -> world, in this repo's convention (+Z forward, +Y
    /// down; column-major) -- the pose of the whole frame, since depth is
    /// registered to colour. Identity places the world at the camera.
    Mat4f cam_to_world = Mat4f(1.0f);
    /// Switch off the SDK's log file (it writes `./Log/` at DEBUG by default)
    /// and route its console sink at WARN. Process-wide: the SDK has one
    /// logger, so an application configuring it itself turns this off.
    bool configure_sdk_logging = true;
  };

  /// @brief Find the camera, check it can stream the requested modes, and
  ///        read its calibration. Does not start streaming.
  ///
  /// @param options  The camera and its streams.
  /// @return The capture, not yet started; or:
  ///         - @ref Status::Code::InvalidArgument for options that cannot
  ///           describe a stream -- a zero size or rate, a depth range that is
  ///           negative, non-finite or empty, a pose that is not finite --
  ///           checked before the SDK is touched; or an empty
  ///           @ref Options::serial with more than one camera answering
  ///           within the discovery window;
  ///         - @ref Status::Code::NotFound if no camera (or not the named one)
  ///           answered within @ref Options::discovery_timeout_ms, naming the
  ///           cameras that did;
  ///         - @ref Status::Code::Unsupported if the camera has no depth or
  ///           colour mode matching the options (the modes it offers are
  ///           listed), reports its image mirrored, flipped or rotated, or
  ///           is in software-triggering mode;
  ///         - @ref Status::Code::IoError for any other SDK failure, with the
  ///           SDK's message.
  static Result<OrbbecCapture> open(const Options& options);

  OrbbecCapture(OrbbecCapture&& other) noexcept;
  OrbbecCapture& operator=(OrbbecCapture&& other) noexcept;
  /// Stops the camera if it is streaming.
  ~OrbbecCapture() override;

  /// @return What the camera reported at @ref open. Empty on a moved-from
  ///         capture.
  const OrbbecDeviceInfo& device_info() const noexcept;

  /// @return The colour camera every frame is stamped with: the pinhole
  ///         intrinsics of the undistorted colour image, its size, and
  ///         @ref Options::cam_to_world. Known from @ref open, before a frame
  ///         exists. Zeroed on a moved-from capture.
  const ColorCameraParams& color_camera() const noexcept;

  /// @return The counters since the last @ref start (zero before one).
  OrbbecCaptureStats stats() const noexcept;

  /// @brief Start both streams. Idempotent: starting a running capture is OK.
  /// @return OK once streaming; @ref Status::Code::InvalidArgument on a
  ///         moved-from capture; @ref Status::Code::IoError if the SDK refuses
  ///         or the camera has disconnected.
  Status start() override;

  /// @brief Stop both streams and drop the frame the last @ref poll handed
  ///        out. Idempotent, and safe on a capture that never started.
  ///
  /// The camera stays open -- and, the cameras being exclusive, held -- so a
  /// later @ref start resumes without rediscovering it. Destroying the
  /// capture is what releases it to another process.
  void stop() noexcept override;

  /// @brief Take the newest synchronised pair not yet handed out, processed as
  ///        the class description says.
  ///
  /// The frame borrows storage this object reuses: it is valid until the next
  /// @ref poll or @ref stop, including a poll that returns no frame. A pair
  /// the SDK fails to process is skipped and counted in
  /// @ref OrbbecCaptureStats::failed; only a run of them is an error.
  ///
  /// @return The frame; an empty optional when no new pair has arrived, the
  ///         capture is not started, or the pair was skipped; or
  ///         @ref Status::Code::IoError if the camera disconnected, a pair
  ///         does not match the stream @ref open negotiated, or ~a second's
  ///         worth of pairs in a row could not be processed; and
  ///         @ref Status::Code::InvalidArgument on a moved-from capture.
  Result<std::optional<CapturedFrame>> poll() override;

  /// @return `true` on a moved-from capture and once the camera has
  ///         disconnected, neither of which can produce another frame. A
  ///         connected camera is never exhausted, only stopped.
  bool exhausted() const noexcept override;

 private:
  struct Impl;
  explicit OrbbecCapture(std::unique_ptr<Impl> impl) noexcept;
  // Behind a pointer so no SDK type reaches this header, and so the frame a
  // poll handed out, which borrows the Impl's storage, survives a move.
  std::unique_ptr<Impl> impl_;
};

}  // namespace volumetric_kit::recon::sensor
