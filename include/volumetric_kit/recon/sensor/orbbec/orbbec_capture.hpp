// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file sensor/orbbec/orbbec_capture.hpp
/// @brief An Orbbec RGB-D camera (the Femto Mega rig) as an
///        @ref volumetric_kit::recon::sensor::ICameraCapture.
///
/// The one driver this repo hosts, and it qualifies under the 2026-08-02 rule
/// because this repo can build *and* test it: the Orbbec SDK is a
/// cross-platform C++ library, and CI builds against it on the legs that set
/// `VR_WITH_ORBBEC` (the 2026-09-24 decision). It is therefore its own opt-in
/// target, `volumetric_kit::recon_sensor_orbbec`, beside the contract rather
/// than inside it: `recon_sensor` stays the SDK-free, Vulkan-free surface an
/// out-of-tree driver compiles against, and a build without the SDK never sees
/// this header installed.
///
/// No Orbbec SDK type appears here -- the SDK is held behind a pointer to an
/// implementation -- so a consumer includes this header without the SDK's
/// headers on its include path. Linking still needs the SDK's library, which
/// the target carries.

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
/// Read at @ref OrbbecCapture::open and never written: the rig's sync wiring
/// is configured once on the cameras and persists there, and a driver that
/// "fixed" it for a single-camera run would break the rig for the next one.
/// What it changes for a caller is whether frames arrive on their own; see
/// @ref waits_for_primary.
enum class OrbbecSyncMode {
  FreeRun,     ///< Not synchronised; streams on its own clock.
  Standalone,  ///< Not synchronised; streams on its own clock.
  Primary,     ///< Streams on its own and drives the sync line.
  Secondary,   ///< Captures only on the primary's sync signal.
  /// The Femto Mega rig's secondaries. The SDK documents this mode as
  /// capturing on its own and merely re-timing to a signal when one arrives;
  /// the rig's secondaries do not -- opened alone, one produced a single frame
  /// set in ~6 s (the 2026-09-26 measurement) -- so this driver treats it as
  /// @ref Secondary.
  SecondarySynced,
  /// Captures only when the host sends a trigger. This driver sends none, so
  /// @ref OrbbecCapture::open refuses a camera in this mode.
  SoftwareTriggering,
  /// Captures only on a trigger another camera sends down the sync line.
  HardwareTriggering,
  Other,  ///< A mode this driver does not name.
};

/// @param mode  A camera's reported sync mode.
/// @return `true` when the camera produces frames only on another camera's
///         signal -- a streaming primary, or the camera that triggers it. Such
///         a camera, started on its own, delivers nothing, and from the
///         consumer's side that is indistinguishable from a camera that is
///         merely slow: @ref OrbbecCapture::poll keeps returning no frame.
///         `false` for @ref OrbbecSyncMode::SoftwareTriggering, which waits
///         for the host rather than another camera, and which
///         @ref OrbbecCapture::open refuses.
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
  /// Pairs a poll took and could not hand out: skipped because the SDK could
  /// not process them, or refused with an error. Every pair is counted once,
  /// so `delivered + dropped + failed <= received`, the difference being a
  /// pair still waiting (or discarded by @ref OrbbecCapture::stop).
  std::uint64_t failed = 0;
};

/// @brief One Orbbec RGB-D camera, polled for posed frames with depth
///        registered to colour.
///
/// **What a frame is.** Each @ref poll hands out the newest synchronised pair,
/// processed on the polling thread in three steps whose order is the point:
/// 1. The colour image is **undistorted** to the pinhole model this repo
///    projects with. The camera's lens moves pixels by ~5 px on average and
///    ~10 px at the corners at 1280x720 (measured on the rig's primary);
///    leaving it would put every colour sample that far from the depth it is
///    fused with.
/// 2. Depth is **registered to that colour camera** -- the SDK's
///    depth-to-colour alignment, which undistorts the depth lens (a far larger
///    correction: ~31 px on average, ~220 px at the edge of the 640x576 ToF
///    image), transforms each sample into the colour camera and re-projects
///    it with the pinhole intrinsics. The SDK ignores the colour lens's
///    distortion here, which is exactly why step 1 is needed: after both, the
///    two images share one pinhole camera, the registered case the fusion and
///    texture tiers are written for.
/// 3. Depth is converted to metres and colour packed into the contract's
///    `R | G<<8 | B<<16` words, in storage this object owns and recycles on
///    the next @ref poll.
///
/// Both of the frame's cameras therefore carry the colour intrinsics, the
/// colour image size, and the one pose from @ref Options::cam_to_world; the
/// depth camera is derived from the colour one with
/// @ref depth_from_registered_color so the two cannot drift apart.
///
/// **Rig roles.** A camera wired as a sync secondary captures only while its
/// primary streams (see @ref OrbbecSyncMode). Opened on its own it starts
/// cleanly and then delivers nothing; @ref device_info says which role the
/// camera has, so a caller can say why before it waits.
///
/// **Disconnects.** A camera the SDK reports removed never streams again
/// through this object -- its device handle is gone, and a camera that comes
/// back is a new device -- so from then on @ref poll and @ref start return
/// @ref Status::Code::IoError and @ref exhausted is `true`. Open a new capture
/// to use it again.
///
/// @warning Not thread-safe: open, start, poll and stop from one thread. The
///          SDK delivers frames on a thread of its own, which this class
///          hands over internally.
class VR_SENSOR_ORBBEC_API OrbbecCapture final : public ICameraCapture {
 public:
  /// @brief Which camera, which streams, and where the camera sits.
  struct Options {
    /// Serial number of the camera to open. Empty opens the only camera found
    /// -- and is refused when discovery finds more than one, since which of
    /// several a first query happens to list is not a choice worth making for
    /// the caller. Cameras answer discovery seconds apart, so a query listing
    /// one is no proof it is alone: empty waits out the whole
    /// @ref discovery_timeout_ms before it opens anything, and naming the
    /// camera is what skips that wait.
    std::string serial;
    /// How long @ref open waits for the camera to appear. An Ethernet camera
    /// can take seconds to answer discovery from cold (the 2026-09-24
    /// decision measured a full ~4 s window with no answer), so one query is
    /// not proof of absence; @ref open re-queries until this runs out, or
    /// until the named camera answers.
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
    /// down; column-major). The colour camera is the one depth is registered
    /// to, so this is the pose of the whole frame -- and the camera a
    /// calibration of the colour stream reports. Identity places the world at
    /// the camera.
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
  ///           listed), reports its image mirrored, flipped or rotated -- a
  ///           mirrored frame with un-mirrored intrinsics reconstructs a
  ///           mirror image without a word -- or is in software-triggering
  ///           mode, which waits for a trigger this driver never sends;
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
  /// @ref poll or @ref stop, including a poll that returns no frame.
  ///
  /// A pair the SDK fails to process -- a filter handing back nothing, a throw
  /// on one frame -- is skipped and counted in @ref OrbbecCaptureStats::failed
  /// rather than reported: one bad pair on a network link must not end a
  /// capture. A run of them is an error, since a camera whose every pair
  /// fails would otherwise look like one that is merely slow.
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
  // poll handed out, which borrows the Impl's storage, survives a move. The
  // SDK's callbacks hold only a shared part of it (see the .cpp).
  std::unique_ptr<Impl> impl_;
};

}  // namespace volumetric_kit::recon::sensor
