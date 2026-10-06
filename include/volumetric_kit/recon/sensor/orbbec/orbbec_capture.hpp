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

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/recon/core/camera_params.hpp"
#include "volumetric_kit/recon/core/fwd.hpp"
#include "volumetric_kit/recon/core/math/vector_types.hpp"
#include "volumetric_kit/recon/sensor/camera_capture.hpp"
#include "volumetric_kit/recon/sensor/orbbec/export.hpp"
#include "volumetric_kit/recon/sensor/rgbd_frame.hpp"

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

/// @brief How the colour stream crosses the wire.
enum class OrbbecColorCodec {
  /// Motion JPEG, decoded by the SDK -- or, for raw frames, by
  /// sensor/video's `JpegDecoder` on a thread per camera, onto the GPU where
  /// nvJPEG or VideoToolbox takes it. About 185 Mbit/s at 4K.
  Mjpeg,
  /// H.265, about 21 Mbit/s at 720p and at 4K, decoded by
  /// sensor/video's `HevcDecoder` on a thread per camera. Needs a build with
  /// VR_WITH_FFMPEG; without one, open refuses it.
  Hevc,
};

/// @return A stable lowercase name for @p codec (`"mjpeg"`, `"hevc"`).
VR_SENSOR_ORBBEC_API const char* to_string(OrbbecColorCodec codec) noexcept;

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
  /// Frame sets the camera delivered since @ref OrbbecCapture::start:
  /// synchronised depth + colour pairs, and for H.265 every colour frame,
  /// whether or not its depth came (see @ref lost).
  std::uint64_t received = 0;
  /// Pairs handed out by @ref OrbbecCapture::poll.
  std::uint64_t delivered = 0;
  /// Pairs replaced by a newer one before any poll took them -- the contract's
  /// "dropped, not queued", counted.
  std::uint64_t dropped = 0;
  /// Pairs a poll took but could not hand out, skipped or refused.
  std::uint64_t failed = 0;
  /// H.265 frame sets that will not be handed out: a colour frame that came
  /// without its depth (decoded, since the frames after it are predicted
  /// from it, and dropped), and the colour that did not decode -- after a
  /// gap in the stream (a frame lost on the network, or empty), a decode
  /// error or a decoder falling two seconds behind, until the next key
  /// frame, and before the first. For raw MJPEG, a JPEG that did not
  /// decode, a pair missing a frame, and a pair a newer one replaced while
  /// the decoder was busy; zero for host MJPEG. Each is counted once, so
  /// `delivered + dropped + failed + lost <= received`; the difference is a
  /// pair still pending or discarded by @ref OrbbecCapture::stop.
  std::uint64_t lost = 0;
  /// Of @ref delivered, the raw frames handed out with their colour on the host
  /// although the stream was opened onto a device
  /// (`OrbbecStreamOptions::device`): the decoder could not keep the picture
  /// there, or its device path failed and every later picture followed. Each
  /// costs a 4K frame's 12 MB across the bus on a discrete GPU, so a run that
  /// should stay on the device reads 0 here.
  std::uint64_t host_pictures = 0;
};

/// @brief The streams a camera is opened with -- the same for every camera of
///        an @ref OrbbecRig.
struct OrbbecStreamOptions {
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
  /// How colour crosses the wire. Either way a frame's colour is the camera's
  /// sRGB, undistorted.
  OrbbecColorCodec color_codec = OrbbecColorCodec::Mjpeg;
  /// Reject depth nearer than this (metres); stamped on every frame's depth
  /// camera as the fusion gate.
  float min_depth = 0.25f;
  /// Reject depth farther than this (metres).
  float max_depth = 5.0f;
  /// Hand frames out as the cameras captured them, through
  /// @ref OrbbecCapture::poll_raw, or a rig's @ref OrbbecRig::poll_raw_set and
  /// @ref OrbbecRig::poll_raw, for `sensor/utils`'s GPU pass to undistort
  /// and convert: the host undistorts, registers and converts nothing, and
  /// depth and colour keep their own cameras, lenses and poses, read from the
  /// camera's factory calibration. Either codec is decoded for the pass, so
  /// raw frames need a build with VR_WITH_FFMPEG.
  bool raw = false;
  /// With @ref raw, the device the GPU pass prepares the frames on. A
  /// picture the hardware decoder leaves there -- NVDEC's or nvJPEG's
  /// (VR_WITH_CUDA), or VideoToolbox's -- stays there, and a raw frame's
  /// colour is that picture
  /// (`YuvImage::device` or `YuvImage::image`) rather than host planes, so it
  /// never crosses to the host. Null, or a decode elsewhere, gives host
  /// planes. Borrowed: it must outlive the capture and every frame on it.
  const core::Device* device = nullptr;
  /// With @ref device, the allocator NVDEC's and nvJPEG's pictures are made
  /// through (exported device-only memory); without it, their pictures come
  /// to the host. VideoToolbox's need none. Borrowed: it must outlive the
  /// capture and every frame on it.
  core::Allocator* allocator = nullptr;
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
/// @ref start return `Status::Code::IoError` and @ref exhausted is `true`.
///
/// @warning Not thread-safe: open, start, poll and stop from one thread. The
///          SDK delivers frames on a thread of its own, which this class
///          hands over internally.
class VR_SENSOR_ORBBEC_API OrbbecCapture final : public ICameraCapture {
 public:
  /// @brief Which camera, which streams (@ref OrbbecStreamOptions), and where
  ///        the camera sits.
  struct Options : OrbbecStreamOptions {
    /// Serial number of the camera to open. Empty opens the only camera that
    /// answers -- after waiting out all of @ref discovery_timeout_ms, since
    /// cameras answer seconds apart -- and is refused when more than one does.
    /// Naming the camera skips the wait.
    std::string serial;
    /// How long @ref open re-queries the network for the camera. An Ethernet
    /// camera can take seconds to answer from cold, so one query is not proof
    /// of absence.
    std::uint32_t discovery_timeout_ms = 8000;
    /// Colour camera -> world, in this repo's convention (+Z forward, +Y
    /// down; column-major) -- the pose of the whole frame, since depth is
    /// registered to colour. Identity places the world at the camera.
    Mat4f cam_to_world = Mat4f(1.0f);
    /// Switch off the SDK's log file (it writes `./Log/` at DEBUG by default)
    /// and route its console sink at WARN, at @ref open; for H.265 colour or
    /// raw frames, also set FFmpeg's log level to ERROR, at the first
    /// @ref start.
    /// Process-wide: the SDK and FFmpeg have one logger each, so an
    /// application configuring either itself turns this off.
    bool configure_sdk_logging = true;
  };

  /// @brief Find the camera, check it can stream the requested modes, and
  ///        read its calibration. Does not start streaming.
  ///
  /// @param options  The camera and its streams.
  /// @return The capture, not yet started; or:
  ///         - `Status::Code::InvalidArgument` for options that cannot
  ///           describe a stream -- a zero size or rate, a depth range that is
  ///           negative, non-finite or empty, a pose that is not finite --
  ///           checked before the SDK is touched; or an empty
  ///           @ref Options::serial with more than one camera answering
  ///           within the discovery window;
  ///         - `Status::Code::NotFound` if no camera (or not the named one)
  ///           answered within @ref Options::discovery_timeout_ms, naming the
  ///           cameras that did;
  ///         - `Status::Code::Unsupported` if the camera has no depth or
  ///           colour mode matching the options (the modes it offers are
  ///           listed), reports its image mirrored, flipped or rotated, or
  ///           is in software-triggering mode; for H.265 colour or raw
  ///           frames, if the colour mode on the wire (H.265, or MJPG for
  ///           raw MJPEG) reports a calibration other than its RGB mode's,
  ///           or -- before the SDK is touched -- if the build has no video
  ///           decoders (VR_WITH_FFMPEG);
  ///         - `Status::Code::IoError` for any other SDK failure, with the
  ///           SDK's message.
  static core::Result<OrbbecCapture> open(const Options& options);

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
  /// @return OK once streaming; `Status::Code::InvalidArgument` on a
  ///         moved-from capture; `Status::Code::IoError` if the SDK refuses
  ///         or the camera has disconnected. For H.265 colour, whose decoder
  ///         opens here: `Status::Code::Unsupported` if FFmpeg has no HEVC
  ///         decoder, and `Status::Code::IoError` if it will not open one
  ///         or its thread will not start. For raw MJPEG, likewise
  ///         `Status::Code::IoError` if the JPEG decoder will not open or
  ///         its thread will not start.
  core::Status start() override;

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
  ///         `Status::Code::IoError` if the camera disconnected, a pair
  ///         does not match the stream @ref open negotiated, or ~a second's
  ///         worth of pairs in a row could not be processed; and
  ///         `Status::Code::InvalidArgument` on a moved-from capture.
  core::Result<std::optional<CapturedFrame>> poll() override;

  /// @brief Take the newest synchronised pair not yet handed out, as the
  ///        cameras captured it: raw depth, the decoded Y'CbCr planes with
  ///        the matrix and range the stream codes them in, each camera's
  ///        factory model, and the poses (@ref OrbbecStreamOptions::raw).
  ///
  /// `RgbdFrame::color_to_world` is @ref Options::cam_to_world, and
  /// `RgbdFrame::depth_to_color` the camera's factory extrinsic. The frame
  /// holds the pair it was read from (`RgbdFrame::pixels`), so it outlives
  /// the next poll; the SDK has that pair's buffers back once every copy of
  /// the frame is gone.
  /// @return As @ref poll; `Status::Code::InvalidArgument` also when the
  ///         capture was not opened with @ref OrbbecStreamOptions::raw, and
  ///         @ref poll returns it when it was; `Status::Code::Unsupported`
  ///         for a stream whose transfer or primaries @ref ColorEncoding
  ///         cannot name.
  core::Result<std::optional<RgbdFrame>> poll_raw() override;

  /// @return `true` if the capture was opened with
  ///         @ref OrbbecStreamOptions::raw, so its frames come through
  ///         @ref poll_raw; `false` otherwise, and on a moved-from capture.
  bool raw_frames() const noexcept override;

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
