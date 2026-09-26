// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/recon/sensor/orbbec/orbbec_capture.hpp"

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

// "640x576@30 Y16, 320x288@30 Y16, ..." -- what a camera offers, for the error
// that says the requested mode is not among them.
std::string list_modes(const ob::StreamProfileList& profiles) {
  std::string out;
  for (std::uint32_t i = 0; i < profiles.getCount(); ++i) {
    const auto video = profiles.getProfile(i)->as<ob::VideoStreamProfile>();
    if (!out.empty()) out += ", ";
    out += std::to_string(video->getWidth()) + "x" +
           std::to_string(video->getHeight()) + "@" +
           std::to_string(video->getFps()) + " " +
           ob::TypeHelper::convertOBFormatTypeToString(video->getFormat());
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

}  // namespace

bool waits_for_primary(OrbbecSyncMode mode) noexcept {
  switch (mode) {
    case OrbbecSyncMode::Secondary:
    case OrbbecSyncMode::SecondarySynced:
    case OrbbecSyncMode::SoftwareTriggering:
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

  // The SDK's frame thread -> the polling thread. The callback only swaps a
  // shared pointer in; all processing happens in poll(), so a pair nobody
  // polls costs nothing but the SDK's own decode.
  std::mutex mutex;
  std::shared_ptr<ob::FrameSet> pending;  // guarded by mutex
  std::string fault;                      // guarded by mutex
  std::atomic<std::uint64_t> received{0};
  std::atomic<std::uint64_t> delivered{0};
  std::atomic<std::uint64_t> dropped{0};

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

  void on_frameset(std::shared_ptr<ob::FrameSet> frameset) {
    // Runs on the SDK's thread; it must never throw back into the SDK.
    if (frameset == nullptr) return;
    std::lock_guard<std::mutex> lock(mutex);
    if (pending != nullptr) dropped.fetch_add(1, std::memory_order_relaxed);
    pending = std::move(frameset);
    received.fetch_add(1, std::memory_order_relaxed);
  }

  void on_devices_changed(const ob::DeviceList& removed) {
    try {
      for (std::uint32_t i = 0; i < removed.getCount(); ++i) {
        if (info.serial == or_empty(removed.getSerialNumber(i))) {
          std::lock_guard<std::mutex> lock(mutex);
          fault = "OrbbecCapture: camera " + info.serial + " disconnected";
          return;
        }
      }
    } catch (...) {
      // A list the SDK cannot read out is not evidence of a disconnect.
    }
  }

  void stop_streaming() noexcept {
    if (running) {
      try {
        pipeline->stop();
      } catch (...) {
        // Stopping is the destructor's fallback; a camera that already went
        // away cannot be stopped twice, and there is no one to tell.
      }
      running = false;
    }
    std::lock_guard<std::mutex> lock(mutex);
    pending.reset();
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

    // Discovery: re-query until the camera answers or the window closes. One
    // query is not proof of absence for an Ethernet camera.
    const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::milliseconds(options.discovery_timeout_ms);
    std::string seen;
    for (;;) {
      const auto list = impl->context->queryDeviceList();
      const std::uint32_t count = list->getCount();
      seen.clear();
      for (std::uint32_t i = 0; i < count; ++i) {
        if (!seen.empty()) seen += ", ";
        seen += or_empty(list->getSerialNumber(i));
      }
      if (options.serial.empty()) {
        if (count > 1) {
          return Status::invalid_argument(
              "OrbbecCapture: " + std::to_string(count) + " cameras found (" +
              seen + "); name one in Options::serial");
        }
        if (count == 1) {
          impl->device = list->getDevice(0);
          break;
        }
      } else {
        for (std::uint32_t i = 0; i < count; ++i) {
          if (options.serial == or_empty(list->getSerialNumber(i))) {
            impl->device = list->getDevice(i);
            break;
          }
        }
        if (impl->device != nullptr) break;
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        const std::string wanted = options.serial.empty()
                                       ? std::string("no camera")
                                       : "camera " + options.serial + " not";
        return Status::not_found("OrbbecCapture: " + wanted + " found within " +
                                 std::to_string(options.discovery_timeout_ms) +
                                 " ms" +
                                 (seen.empty() ? std::string(" (none answered)")
                                               : " (answered: " + seen + ")"));
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
      // RGB, not MJPG: the camera still sends MJPG over the wire (the link
      // carries the same ~30 Mbit/s either way at 1280x720) and the SDK
      // decodes it on its own thread, so the frame arrives as RGB triplets.
      // TODO(sensor): stream H.265 instead -- the camera encodes it (21 Mbit/s
      // at 4K against MJPG's 185, which also tops out at 16.7 of 25 fps) but
      // the SDK passes it through undecoded, so it needs a decoder of ours.
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

    Impl* raw = impl.get();
    impl->device_callback_id = impl->context->registerDeviceChangedCallback(
        [raw](std::shared_ptr<ob::DeviceList> removed,
              std::shared_ptr<ob::DeviceList> /*added*/) {
          if (removed != nullptr) raw->on_devices_changed(*removed);
        });
    impl->device_callback_registered = true;
  } catch (const ob::Error& e) {
    return sdk_error("opening the camera", e);
  } catch (const std::exception& e) {
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
  s.received = impl_->received.load(std::memory_order_relaxed);
  s.delivered = impl_->delivered.load(std::memory_order_relaxed);
  s.dropped = impl_->dropped.load(std::memory_order_relaxed);
  return s;
}

Status OrbbecCapture::start() {
  if (impl_ == nullptr) {
    return Status::invalid_argument(
        "OrbbecCapture: start on a moved-from "
        "capture");
  }
  if (impl_->running) return {};
  Impl* raw = impl_.get();
  {
    std::lock_guard<std::mutex> lock(raw->mutex);
    raw->pending.reset();
  }
  raw->received = 0;
  raw->delivered = 0;
  raw->dropped = 0;
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
    raw->pipeline->start(config, [raw](std::shared_ptr<ob::FrameSet> fs) {
      try {
        raw->on_frameset(std::move(fs));
      } catch (...) {
        // Never throw into the SDK's thread.
      }
    });
  } catch (const ob::Error& e) {
    return sdk_error("starting camera " + raw->info.serial, e);
  } catch (const std::exception& e) {
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
    std::lock_guard<std::mutex> lock(s.mutex);
    if (!s.fault.empty()) return Status::io_error(s.fault);
    frameset = std::move(s.pending);
    s.pending.reset();
  }
  if (!s.running || frameset == nullptr) return no_frame();

  const std::uint32_t width = s.color_camera.width;
  const std::uint32_t height = s.color_camera.height;
  const std::size_t pixels = static_cast<std::size_t>(width) * height;
  std::uint64_t timestamp_us = 0;
  try {
    // Undistort colour, then register depth to it; see the class comment for
    // why the order matters. Both filters are synchronous here.
    // TODO(sensor): the same three steps as GPU kernels, keeping the frame on
    // the device through fusion -- which needs device-resident overloads of
    // the fusion entry points, or the upload is merely moved. And measure the
    // camera's own registration (ALIGN_D2C_HW_MODE), deferred for shipping
    // 2.5x the depth pixels over the link and for its unmeasured treatment of
    // the colour lens (the 2026-09-26 decision).
    const auto undistorted = s.undistort_color->process(frameset);
    if (undistorted == nullptr) {
      return Status::io_error(
          "OrbbecCapture: colour undistortion produced "
          "no frame");
    }
    const auto registered = s.align_to_color->process(undistorted);
    if (registered == nullptr) {
      return Status::io_error(
          "OrbbecCapture: depth registration produced "
          "no frame");
    }
    const auto pair = registered->as<ob::FrameSet>();
    const auto depth = pair->getDepthFrame();
    const auto color = pair->getColorFrame();
    if (depth == nullptr || color == nullptr) {
      return Status::io_error(
          "OrbbecCapture: a processed pair is missing its "
          "depth or colour frame");
    }
    if (depth->getWidth() != width || depth->getHeight() != height ||
        color->getWidth() != width || color->getHeight() != height) {
      return Status::io_error("OrbbecCapture: processed pair is depth " +
                              std::to_string(depth->getWidth()) + "x" +
                              std::to_string(depth->getHeight()) + ", colour " +
                              std::to_string(color->getWidth()) + "x" +
                              std::to_string(color->getHeight()) +
                              "; expected both " + std::to_string(width) + "x" +
                              std::to_string(height));
    }
    if (depth->getFormat() != OB_FORMAT_Y16 ||
        depth->getDataSize() < pixels * sizeof(std::uint16_t)) {
      return Status::io_error(
          "OrbbecCapture: registered depth is not a full "
          "Y16 image");
    }
    if (color->getFormat() != OB_FORMAT_RGB ||
        color->getDataSize() < pixels * 3) {
      return Status::io_error(
          "OrbbecCapture: undistorted colour is not a "
          "full RGB image");
    }
    if (!s.first_pair_checked) {
      // The frames are stamped with the camera computed at open; hold the SDK
      // to it once per start. Registration must have re-projected into these
      // intrinsics, and the undistorted colour must carry no distortion.
      const auto depth_video =
          depth->getStreamProfile()->as<ob::VideoStreamProfile>();
      const auto color_video =
          color->getStreamProfile()->as<ob::VideoStreamProfile>();
      const OBCameraIntrinsic k = depth_video->getIntrinsic();
      const OBCameraDistortion d = color_video->getDistortion();
      const float tol = 1e-3f;
      if (std::fabs(k.fx - s.color_camera.fx) > tol ||
          std::fabs(k.fy - s.color_camera.fy) > tol ||
          std::fabs(k.cx - s.color_camera.cx) > tol ||
          std::fabs(k.cy - s.color_camera.cy) > tol) {
        return Status::io_error(
            "OrbbecCapture: registered depth reports intrinsics other than "
            "the colour camera's; the frame would be unprojected wrongly");
      }
      if (d.k1 != 0.0f || d.k2 != 0.0f || d.k3 != 0.0f || d.k4 != 0.0f ||
          d.k5 != 0.0f || d.k6 != 0.0f || d.p1 != 0.0f || d.p2 != 0.0f) {
        return Status::io_error(
            "OrbbecCapture: undistorted colour still reports lens distortion");
      }
      s.first_pair_checked = true;
    }
    const float value_scale = depth->getValueScale();
    if (!std::isfinite(value_scale) || !(value_scale > 0.0f)) {
      return Status::io_error(
          "OrbbecCapture: depth frame reports value "
          "scale " +
          std::to_string(value_scale));
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
  } catch (const ob::Error& e) {
    return sdk_error("processing a frame", e);
  } catch (const std::exception& e) {
    return sdk_error("processing a frame", e);
  }

  CapturedFrame frame{};
  frame.depth = s.depth_metres.data();
  frame.color = s.color_packed.data();
  frame.depth_camera = s.depth_camera;
  frame.color_camera = s.color_camera;
  // The camera's colour is ordinary 8-bit sRGB: the canonical form, declared
  // by leaving the default.
  frame.timestamp_ns = timestamp_us * 1000;
  s.delivered.fetch_add(1, std::memory_order_relaxed);
  return some_frame(frame);
}

}  // namespace volumetric_kit::recon::sensor
