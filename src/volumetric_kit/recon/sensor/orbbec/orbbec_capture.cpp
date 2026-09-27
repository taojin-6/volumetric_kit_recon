// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/recon/sensor/orbbec/orbbec_capture.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <exception>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <libobsensor/ObSensor.hpp>

#include "frame_conversion.hpp"
#include "volumetric_kit/recon/sensor/camera_conventions.hpp"

namespace volumetric_kit::recon::sensor {

namespace {

// The SDK reports every failure as a thrown ob::Error; this repo returns
// Status across its API and never throws. Each public entry point catches at
// its boundary and names what it was doing.
Status sdk_error(const std::string& what, const std::exception& e) {
  return Status::io_error("OrbbecCapture: " + what + ": " + e.what());
}

const char* or_empty(const char* s) { return s != nullptr ? s : ""; }

// How often open() re-asks the network while waiting for a camera. The query
// itself blocks about a second probing for Ethernet devices, so this only
// spaces out the retries.
constexpr std::chrono::milliseconds kDiscoveryRetry{250};

// Pairs in a row poll() may skip before it calls the camera broken: about a
// second at the 30 fps default. See OrbbecCapture::poll.
constexpr std::uint32_t kMaxFailedPairsInARow = 30;

// How far the SDK's intrinsics may sit from the camera a frame is stamped with
// before the first-pair check refuses them, in pixels.
constexpr float kIntrinsicsTolerance = 1e-3f;

std::string join(const std::vector<std::string>& items) {
  std::string out;
  for (const std::string& item : items) {
    if (!out.empty()) out += ", ";
    out += item;
  }
  return out;
}

// "640x576@30 Y16, 320x288@30 Y16, ..." -- what a camera offers, for the error
// that says the requested mode is not among them. It runs while that error is
// being built, so it must not throw one of its own over it: an entry that is
// not a video mode is skipped, and a list the SDK fails to read out is cut
// short and says so.
std::string list_modes(const ob::StreamProfileList& profiles) {
  std::string out;
  try {
    for (std::uint32_t i = 0; i < profiles.getCount(); ++i) {
      const auto profile = profiles.getProfile(i);
      if (!profile->is<ob::VideoStreamProfile>()) continue;
      const auto video = profile->as<ob::VideoStreamProfile>();
      if (!out.empty()) out += ", ";
      out += std::to_string(video->getWidth()) + "x" +
             std::to_string(video->getHeight()) + "@" +
             std::to_string(video->getFps()) + " " +
             ob::TypeHelper::convertOBFormatTypeToString(video->getFormat());
    }
  } catch (const std::exception&) {
    out += out.empty() ? "(none it could list)" : ", ... (the rest unreadable)";
  }
  return out;
}

// A camera that reports its image mirrored, flipped or rotated delivers pixels
// its intrinsics do not describe, and nothing downstream can tell: the
// reconstruction comes out mirrored or scrambled without an error. Refused
// rather than un-done, since undoing it would mean writing the camera's
// settings.
Status check_orientation(ob::Device& device) {
  struct BoolProp {
    OBPropertyID id;
    const char* name;
  };
  const BoolProp bool_props[] = {
      {OB_PROP_DEPTH_MIRROR_BOOL, "depth mirror"},
      {OB_PROP_DEPTH_FLIP_BOOL, "depth flip"},
      {OB_PROP_COLOR_MIRROR_BOOL, "colour mirror"},
      {OB_PROP_COLOR_FLIP_BOOL, "colour flip"},
  };
  for (const BoolProp& prop : bool_props) {
    if (device.isPropertySupported(prop.id, OB_PERMISSION_READ) &&
        device.getBoolProperty(prop.id)) {
      return Status::unsupported(std::string("OrbbecCapture: the camera has ") +
                                 prop.name +
                                 " switched on; its intrinsics describe the "
                                 "unmirrored image. Switch it off.");
    }
  }
  for (const OBPropertyID id :
       {OB_PROP_DEPTH_ROTATE_INT, OB_PROP_COLOR_ROTATE_INT}) {
    if (device.isPropertySupported(id, OB_PERMISSION_READ) &&
        device.getIntProperty(id) != 0) {
      return Status::unsupported(
          "OrbbecCapture: the camera rotates its image; its intrinsics "
          "describe the unrotated one. Set the rotation to 0.");
    }
  }
  return {};
}

// What the SDK's threads write and the polling thread reads. Captured by
// shared_ptr in both SDK callbacks, since neither is guaranteed gone when the
// capture is: the SDK calls a *copy* of the device-changed callback after
// releasing its lock, so unregistering does not wait for a call in flight, and
// a pipeline stop that throws may leave the SDK holding the frame callback.
struct Mailbox {
  std::mutex mutex;
  std::shared_ptr<ob::FrameSet> pending;  // guarded by mutex
  std::string fault;                      // guarded by mutex
  // Set, after `fault`, once the camera is gone; lets exhausted() answer
  // without the lock.
  std::atomic<bool> disconnected{false};
  std::atomic<std::uint64_t> received{0};
  std::atomic<std::uint64_t> dropped{0};

  void on_frameset(std::shared_ptr<ob::FrameSet> frameset) {
    // Runs on the SDK's thread; it must never throw back into the SDK.
    if (frameset == nullptr) return;
    std::lock_guard<std::mutex> lock(mutex);
    if (pending != nullptr) dropped.fetch_add(1, std::memory_order_relaxed);
    pending = std::move(frameset);
    received.fetch_add(1, std::memory_order_relaxed);
  }

  void on_devices_changed(const std::string& serial,
                          const ob::DeviceList& removed) {
    try {
      for (std::uint32_t i = 0; i < removed.getCount(); ++i) {
        if (serial == or_empty(removed.getSerialNumber(i))) {
          std::lock_guard<std::mutex> lock(mutex);
          fault = "OrbbecCapture: camera " + serial +
                  " disconnected; open a new capture to use it again";
          disconnected.store(true, std::memory_order_release);
          return;
        }
      }
    } catch (...) {
      // A list the SDK cannot read out is not evidence of a disconnect.
    }
  }
};

}  // namespace

bool waits_for_primary(OrbbecSyncMode mode) noexcept {
  // Not SoftwareTriggering: that waits for the host, not another camera, and
  // open() refuses it.
  switch (mode) {
    case OrbbecSyncMode::Secondary:
    case OrbbecSyncMode::SecondarySynced:
    case OrbbecSyncMode::HardwareTriggering:
      return true;
    default:
      return false;
  }
}

const char* to_string(OrbbecSyncMode mode) noexcept {
  switch (mode) {
    case OrbbecSyncMode::FreeRun:
      return "free-run";
    case OrbbecSyncMode::Standalone:
      return "standalone";
    case OrbbecSyncMode::Primary:
      return "primary";
    case OrbbecSyncMode::Secondary:
      return "secondary";
    case OrbbecSyncMode::SecondarySynced:
      return "secondary-synced";
    case OrbbecSyncMode::SoftwareTriggering:
      return "software-triggering";
    case OrbbecSyncMode::HardwareTriggering:
      return "hardware-triggering";
    case OrbbecSyncMode::Other:
      break;
  }
  return "other";
}

struct OrbbecCapture::Impl {
  // Declared first so it is destroyed last: every SDK object below belongs to
  // this context.
  std::shared_ptr<ob::Context> context;
  // Next, so a frame still pending in it is released while the context lives.
  std::shared_ptr<Mailbox> mailbox = std::make_shared<Mailbox>();
  std::shared_ptr<ob::Device> device;
  std::shared_ptr<ob::Pipeline> pipeline;
  std::shared_ptr<ob::StreamProfile> depth_profile;
  std::shared_ptr<ob::StreamProfile> color_profile;
  std::shared_ptr<ob::UnDistortionFilter> undistort_color;
  std::shared_ptr<ob::Align> align_to_color;
  bool device_callback_registered = false;
  OBCallbackId device_callback_id = 0;

  OrbbecDeviceInfo info;
  ColorCameraParams color_camera{};
  DepthCameraParams depth_camera{};

  // The polling thread's counters; the SDK's thread counts in `mailbox`.
  std::uint64_t delivered = 0;
  std::uint64_t failed = 0;
  std::uint32_t failed_in_a_row = 0;

  bool running = false;
  // Whether the first processed pair of this start has been checked against
  // the camera the frames are stamped with.
  bool first_pair_checked = false;

  // Storage the frame poll() hands out borrows.
  std::vector<float> depth_metres;
  std::vector<std::uint32_t> color_packed;

  Impl() = default;
  Impl(const Impl&) = delete;
  Impl& operator=(const Impl&) = delete;

  ~Impl() {
    stop_streaming();
    if (device_callback_registered) {
      try {
        context->unregisterDeviceChangedCallback(device_callback_id);
      } catch (...) {
        // Nothing to do at teardown; the context goes next regardless.
      }
    }
  }

  void stop_streaming() noexcept {
    if (running) {
      try {
        pipeline->stop();
      } catch (...) {
        // A camera that already went away cannot be stopped, and there is no
        // one to tell; a late frame lands in the mailbox, not in this object.
      }
      running = false;
    }
    std::lock_guard<std::mutex> lock(mailbox->mutex);
    mailbox->pending.reset();
  }
};

OrbbecCapture::OrbbecCapture(std::unique_ptr<Impl> impl) noexcept
    : impl_(std::move(impl)) {}

OrbbecCapture::OrbbecCapture(OrbbecCapture&& other) noexcept = default;
OrbbecCapture& OrbbecCapture::operator=(OrbbecCapture&& other) noexcept =
    default;
OrbbecCapture::~OrbbecCapture() = default;

Result<OrbbecCapture> OrbbecCapture::open(const Options& options) {
  VR_TRY(orbbec::validate(options));
  auto impl = std::make_unique<Impl>();
  try {
    if (options.configure_sdk_logging) {
      // One call per sink: setLoggerSeverity sets every sink, the file one
      // included, so it would switch the file log back on.
      ob::Context::setLoggerToFile(OB_LOG_SEVERITY_OFF, "");
      ob::Context::setLoggerToConsole(OB_LOG_SEVERITY_WARN);
    }
    impl->context = std::make_shared<ob::Context>();
    impl->context->enableNetDeviceEnumeration(true);

    // Discovery: re-query until the camera answers or the window closes. A
    // named camera opens the moment it answers; an unnamed one only once the
    // whole window has passed with no other camera answering.
    const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::milliseconds(options.discovery_timeout_ms);
    std::vector<std::string> answered;  // every serial any query listed
    // Unnamed: the latest list that held the one camera, opened at the end.
    std::shared_ptr<ob::DeviceList> sole;
    for (;;) {
      const auto list = impl->context->queryDeviceList();
      const std::uint32_t count = list->getCount();
      for (std::uint32_t i = 0; i < count; ++i) {
        const std::string serial = or_empty(list->getSerialNumber(i));
        if (std::find(answered.begin(), answered.end(), serial) ==
            answered.end()) {
          answered.push_back(serial);
        }
        if (!options.serial.empty() && serial == options.serial) {
          impl->device = list->getDevice(i);
          break;
        }
      }
      if (impl->device != nullptr) break;
      if (options.serial.empty()) {
        if (answered.size() > 1) {
          return Status::invalid_argument(
              "OrbbecCapture: " + std::to_string(answered.size()) +
              " cameras answered (" + join(answered) +
              "); name one in Options::serial");
        }
        if (count == 1) sole = list;
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        if (sole != nullptr) {
          impl->device = sole->getDevice(0);
          break;
        }
        const std::string wanted = options.serial.empty()
                                       ? std::string("no camera")
                                       : "camera " + options.serial + " not";
        return Status::not_found(
            "OrbbecCapture: " + wanted + " found within " +
            std::to_string(options.discovery_timeout_ms) + " ms" +
            (answered.empty() ? std::string(" (none answered)")
                              : " (answered: " + join(answered) + ")"));
      }
      std::this_thread::sleep_for(kDiscoveryRetry);
    }

    const auto device_info = impl->device->getDeviceInfo();
    impl->info.name = or_empty(device_info->getName());
    impl->info.serial = or_empty(device_info->getSerialNumber());
    impl->info.firmware_version = or_empty(device_info->getFirmwareVersion());
    impl->info.connection_type = or_empty(device_info->getConnectionType());
    impl->info.ip_address = or_empty(device_info->getIpAddress());
    const std::uint16_t sync_modes =
        impl->device->getSupportedMultiDeviceSyncModeBitmap();
    impl->info.sync_mode =
        sync_modes != 0 ? orbbec::sync_mode_from(
                              impl->device->getMultiDeviceSyncConfig().syncMode)
                        : OrbbecSyncMode::Standalone;
    if (impl->info.sync_mode == OrbbecSyncMode::SoftwareTriggering) {
      return Status::unsupported(
          "OrbbecCapture: camera " + impl->info.serial +
          " is in software-triggering mode, which captures only when the host "
          "sends a trigger, and this driver sends none. Set another sync "
          "mode on the camera.");
    }

    VR_TRY(check_orientation(*impl->device));

    impl->pipeline = std::make_shared<ob::Pipeline>(impl->device);
    const auto depth_modes =
        impl->pipeline->getStreamProfileList(OB_SENSOR_DEPTH);
    const auto color_modes =
        impl->pipeline->getStreamProfileList(OB_SENSOR_COLOR);
    // getVideoStreamProfile throws when nothing matches; the SDK's message
    // does not say what would have, so each lookup is caught on its own.
    try {
      impl->depth_profile = depth_modes->getVideoStreamProfile(
          static_cast<int>(options.depth_width),
          static_cast<int>(options.depth_height), OB_FORMAT_Y16,
          static_cast<int>(options.fps));
    } catch (const ob::Error&) {
      return Status::unsupported("OrbbecCapture: camera " + impl->info.serial +
                                 " has no depth mode " +
                                 std::to_string(options.depth_width) + "x" +
                                 std::to_string(options.depth_height) + "@" +
                                 std::to_string(options.fps) +
                                 " Y16; it offers " + list_modes(*depth_modes));
    }
    try {
      // RGB: the wire carries MJPG either way, decoded on the SDK's thread.
      // TODO(sensor): stream H.265, which the SDK passes through undecoded, so
      // 4K on three cameras needs a decoder of ours (the 2026-09-26 decision).
      impl->color_profile = color_modes->getVideoStreamProfile(
          static_cast<int>(options.color_width),
          static_cast<int>(options.color_height), OB_FORMAT_RGB,
          static_cast<int>(options.fps));
    } catch (const ob::Error&) {
      return Status::unsupported("OrbbecCapture: camera " + impl->info.serial +
                                 " has no colour mode " +
                                 std::to_string(options.color_width) + "x" +
                                 std::to_string(options.color_height) + "@" +
                                 std::to_string(options.fps) +
                                 " RGB; it offers " + list_modes(*color_modes));
    }

    // The pinhole camera of the undistorted colour image: the undistortion
    // filter keeps the stream's intrinsics and drops only the distortion, and
    // registration re-projects depth with exactly these. poll() checks both
    // claims on the first pair of every start.
    const OBCameraIntrinsic intrinsic =
        impl->color_profile->as<ob::VideoStreamProfile>()->getIntrinsic();
    VR_ASSIGN(impl->color_camera,
              orbbec::color_camera_from(intrinsic, options.cam_to_world));
    if (impl->color_camera.width != options.color_width ||
        impl->color_camera.height != options.color_height) {
      return Status::io_error(
          "OrbbecCapture: camera " + impl->info.serial +
          " reports colour intrinsics for a " +
          std::to_string(impl->color_camera.width) + "x" +
          std::to_string(impl->color_camera.height) + " image, not the " +
          std::to_string(options.color_width) + "x" +
          std::to_string(options.color_height) + " mode it opened");
    }
    VR_ASSIGN(impl->depth_camera,
              depth_from_registered_color(
                  impl->color_camera, options.color_width, options.color_height,
                  options.min_depth, options.max_depth));

    impl->undistort_color =
        std::make_shared<ob::UnDistortionFilter>(OB_STREAM_COLOR);
    impl->align_to_color = std::make_shared<ob::Align>(OB_STREAM_COLOR);
    // Registered depth at the colour image's full size -- the frame's one
    // resolution -- rather than at depth's own with colour's aspect ratio.
    // Set rather than left to the SDK's default, which poll() depends on.
    impl->align_to_color->setMatchTargetResolution(true);

    impl->device_callback_id = impl->context->registerDeviceChangedCallback(
        [mailbox = impl->mailbox, serial = impl->info.serial](
            std::shared_ptr<ob::DeviceList> removed,
            std::shared_ptr<ob::DeviceList> /*added*/) {
          if (removed != nullptr) mailbox->on_devices_changed(serial, *removed);
        });
    impl->device_callback_registered = true;
  } catch (const std::exception& e) {  // ob::Error is one
    return sdk_error("opening the camera", e);
  }
  return OrbbecCapture(std::move(impl));
}

const OrbbecDeviceInfo& OrbbecCapture::device_info() const noexcept {
  static const OrbbecDeviceInfo kEmpty{};
  return impl_ != nullptr ? impl_->info : kEmpty;
}

const ColorCameraParams& OrbbecCapture::color_camera() const noexcept {
  static const ColorCameraParams kEmpty{};
  return impl_ != nullptr ? impl_->color_camera : kEmpty;
}

OrbbecCaptureStats OrbbecCapture::stats() const noexcept {
  OrbbecCaptureStats s;
  if (impl_ == nullptr) return s;
  s.received = impl_->mailbox->received.load(std::memory_order_relaxed);
  s.delivered = impl_->delivered;
  s.dropped = impl_->mailbox->dropped.load(std::memory_order_relaxed);
  s.failed = impl_->failed;
  return s;
}

bool OrbbecCapture::exhausted() const noexcept {
  return impl_ == nullptr ||
         impl_->mailbox->disconnected.load(std::memory_order_acquire);
}

Status OrbbecCapture::start() {
  if (impl_ == nullptr) {
    return Status::invalid_argument(
        "OrbbecCapture: start on a moved-from "
        "capture");
  }
  Impl* raw = impl_.get();
  Mailbox& box = *raw->mailbox;
  {
    std::lock_guard<std::mutex> lock(box.mutex);
    // Checked before `running`: a camera that went away while streaming is
    // not "already started".
    if (!box.fault.empty()) return Status::io_error(box.fault);
    if (raw->running) return {};
    box.pending.reset();
  }
  box.received = 0;
  box.dropped = 0;
  raw->delivered = 0;
  raw->failed = 0;
  raw->failed_in_a_row = 0;
  raw->first_pair_checked = false;
  try {
    auto config = std::make_shared<ob::Config>();
    config->enableStream(raw->depth_profile);
    config->enableStream(raw->color_profile);
    // Only pairs: a frame set missing either half is never handed over.
    config->setFrameAggregateOutputMode(
        OB_FRAME_AGGREGATE_OUTPUT_ALL_TYPE_FRAME_REQUIRE);
    // Pair depth with the colour frame nearest it in time, rather than with
    // whichever arrived last.
    raw->pipeline->enableFrameSync();
    raw->pipeline->start(
        config, [mailbox = raw->mailbox](std::shared_ptr<ob::FrameSet> fs) {
          try {
            mailbox->on_frameset(std::move(fs));
          } catch (...) {
            // Never throw into the SDK's thread.
          }
        });
  } catch (const std::exception& e) {  // ob::Error is one
    return sdk_error("starting camera " + raw->info.serial, e);
  }
  raw->running = true;
  return {};
}

void OrbbecCapture::stop() noexcept {
  if (impl_ == nullptr) return;
  impl_->stop_streaming();
  // The frame the last poll handed out borrows these; stopping drops it.
  impl_->depth_metres.clear();
  impl_->color_packed.clear();
}

Result<std::optional<CapturedFrame>> OrbbecCapture::poll() {
  if (impl_ == nullptr) {
    return Status::invalid_argument(
        "OrbbecCapture: poll on a moved-from "
        "capture");
  }
  Impl& s = *impl_;
  std::shared_ptr<ob::FrameSet> frameset;
  {
    std::lock_guard<std::mutex> lock(s.mailbox->mutex);
    if (!s.mailbox->fault.empty()) return Status::io_error(s.mailbox->fault);
    frameset = std::move(s.mailbox->pending);
    s.mailbox->pending.reset();
  }
  if (!s.running || frameset == nullptr) return no_frame();

  // From here the pair is this poll's: it is delivered, or counted as failed
  // by one of these two. `refuse` is for a pair that contradicts the stream
  // open() negotiated -- every pair after it would too; `skip` for one the SDK
  // failed on, which the next may not be.
  const auto refuse = [&s](Status why) {
    ++s.failed;
    return why;
  };
  const auto skip =
      [&s](const std::string& why) -> Result<std::optional<CapturedFrame>> {
    ++s.failed;
    if (++s.failed_in_a_row < kMaxFailedPairsInARow) return no_frame();
    return Status::io_error(
        "OrbbecCapture: " + std::to_string(s.failed_in_a_row) +
        " pairs in a row could not be processed; the "
        "last: " +
        why);
  };

  const std::uint32_t width = s.color_camera.width;
  const std::uint32_t height = s.color_camera.height;
  const std::size_t pixels = static_cast<std::size_t>(width) * height;
  std::uint64_t timestamp_us = 0;
  try {
    // Undistort colour, then register depth to it (see the class comment).
    // TODO(sensor): the same steps as GPU kernels, keeping the frame on the
    // device through fusion -- which needs device-resident fusion entry
    // points, or the upload is merely moved. And measure the camera's own
    // registration (ALIGN_D2C_HW_MODE; the 2026-09-26 decision).
    const auto undistorted = s.undistort_color->process(frameset);
    if (undistorted == nullptr) {
      return skip("colour undistortion produced no frame");
    }
    const auto registered = s.align_to_color->process(undistorted);
    if (registered == nullptr) {
      return skip("depth registration produced no frame");
    }
    const auto pair = registered->as<ob::FrameSet>();
    const auto depth = pair->getDepthFrame();
    const auto color = pair->getColorFrame();
    if (depth == nullptr || color == nullptr) {
      return skip("a processed pair is missing its depth or colour frame");
    }
    if (depth->getWidth() != width || depth->getHeight() != height ||
        color->getWidth() != width || color->getHeight() != height) {
      return refuse(Status::io_error(
          "OrbbecCapture: processed pair is depth " +
          std::to_string(depth->getWidth()) + "x" +
          std::to_string(depth->getHeight()) + ", colour " +
          std::to_string(color->getWidth()) + "x" +
          std::to_string(color->getHeight()) + "; expected both " +
          std::to_string(width) + "x" + std::to_string(height)));
    }
    if (depth->getFormat() != OB_FORMAT_Y16 ||
        depth->getDataSize() < pixels * sizeof(std::uint16_t)) {
      return refuse(
          Status::io_error("OrbbecCapture: registered depth is not a full "
                           "Y16 image"));
    }
    if (color->getFormat() != OB_FORMAT_RGB ||
        color->getDataSize() < pixels * 3) {
      return refuse(
          Status::io_error("OrbbecCapture: undistorted colour is not a "
                           "full RGB image"));
    }
    if (!s.first_pair_checked) {
      // The frames are stamped with the camera computed at open; hold the SDK
      // to it once per start. Registration must have re-projected into these
      // intrinsics, and undistortion must have kept them and dropped only the
      // distortion -- a filter that re-projected to a new camera matrix would
      // leave every colour sample projected with the wrong one.
      const auto depth_video =
          depth->getStreamProfile()->as<ob::VideoStreamProfile>();
      const auto color_video =
          color->getStreamProfile()->as<ob::VideoStreamProfile>();
      if (!orbbec::same_pinhole(depth_video->getIntrinsic(), s.color_camera,
                                kIntrinsicsTolerance)) {
        return refuse(Status::io_error(
            "OrbbecCapture: registered depth reports intrinsics other than "
            "the colour camera's; the frame would be unprojected wrongly"));
      }
      if (!orbbec::same_pinhole(color_video->getIntrinsic(), s.color_camera,
                                kIntrinsicsTolerance)) {
        return refuse(Status::io_error(
            "OrbbecCapture: undistorted colour reports intrinsics other than "
            "the ones open() read; the frame would be projected wrongly"));
      }
      // `!= 0` refuses a NaN coefficient too.
      const OBCameraDistortion d = color_video->getDistortion();
      if (d.k1 != 0.0f || d.k2 != 0.0f || d.k3 != 0.0f || d.k4 != 0.0f ||
          d.k5 != 0.0f || d.k6 != 0.0f || d.p1 != 0.0f || d.p2 != 0.0f) {
        return refuse(Status::io_error(
            "OrbbecCapture: undistorted colour still reports lens "
            "distortion"));
      }
      s.first_pair_checked = true;
    }
    const float value_scale = depth->getValueScale();
    if (!std::isfinite(value_scale) || !(value_scale > 0.0f)) {
      return refuse(
          Status::io_error("OrbbecCapture: depth frame reports value scale " +
                           std::to_string(value_scale)));
    }
    s.depth_metres.resize(pixels);
    s.color_packed.resize(pixels);
    orbbec::depth_to_metres(
        reinterpret_cast<const std::uint16_t*>(depth->getData()), pixels,
        value_scale, s.depth_metres.data());
    orbbec::pack_rgb(color->getData(), pixels, s.color_packed.data());
    // The camera's own hardware clock. TODO(sensor): a multi-camera source
    // needs the cameras on one clock (the SDK's global timestamp, or a
    // synced timer) before it can group their frames.
    timestamp_us = depth->getTimeStampUs();
  } catch (const std::exception& e) {  // ob::Error is one
    return skip(std::string("the SDK failed on it: ") + e.what());
  }

  CapturedFrame frame{};
  frame.depth = s.depth_metres.data();
  frame.color = s.color_packed.data();
  frame.depth_camera = s.depth_camera;
  frame.color_camera = s.color_camera;
  // The camera's colour is ordinary 8-bit sRGB: the canonical form, declared
  // by leaving the default.
  frame.timestamp_ns = timestamp_us * 1000;
  s.failed_in_a_row = 0;
  ++s.delivered;
  return some_frame(frame);
}

}  // namespace volumetric_kit::recon::sensor
