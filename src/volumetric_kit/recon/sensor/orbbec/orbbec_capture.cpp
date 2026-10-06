// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/recon/sensor/orbbec/orbbec_capture.hpp"

#include <exception>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <libobsensor/ObSensor.hpp>

#include "camera_stream.hpp"
#include "frame_conversion.hpp"

namespace volumetric_kit::recon::sensor {

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

const char* to_string(OrbbecColorCodec codec) noexcept {
  return codec == OrbbecColorCodec::Hevc ? "hevc" : "mjpeg";
}

struct OrbbecCapture::Impl {
  std::unique_ptr<orbbec::CameraStream> stream;
};

OrbbecCapture::OrbbecCapture(std::unique_ptr<Impl> impl) noexcept
    : impl_(std::move(impl)) {}

OrbbecCapture::OrbbecCapture(OrbbecCapture&& other) noexcept = default;
OrbbecCapture& OrbbecCapture::operator=(OrbbecCapture&& other) noexcept =
    default;
OrbbecCapture::~OrbbecCapture() = default;

core::Result<OrbbecCapture> OrbbecCapture::open(const Options& options) {
  VKC_TRY(orbbec::validate(options));
  VKC_TRY(orbbec::check_color_codec(options, "OrbbecCapture"));
  auto impl = std::make_unique<Impl>();
  VKC_ASSIGN(impl->stream,
             orbbec::open_camera(options.serial, options.discovery_timeout_ms,
                                 options.configure_sdk_logging, options,
                                 options.cam_to_world, "OrbbecCapture"));
  return OrbbecCapture(std::move(impl));
}

const OrbbecDeviceInfo& OrbbecCapture::device_info() const noexcept {
  static const OrbbecDeviceInfo kEmpty{};
  return impl_ != nullptr ? impl_->stream->info() : kEmpty;
}

const ColorCameraParams& OrbbecCapture::color_camera() const noexcept {
  static const ColorCameraParams kEmpty{};
  return impl_ != nullptr ? impl_->stream->color_camera() : kEmpty;
}

OrbbecCaptureStats OrbbecCapture::stats() const noexcept {
  return impl_ != nullptr ? impl_->stream->stats() : OrbbecCaptureStats{};
}

bool OrbbecCapture::exhausted() const noexcept {
  return impl_ == nullptr || impl_->stream->disconnected();
}

core::Status OrbbecCapture::start() {
  if (impl_ == nullptr) {
    return core::Status::invalid_argument(
        "OrbbecCapture: start on a moved-from capture");
  }
  return impl_->stream->start();
}

void OrbbecCapture::stop() noexcept {
  if (impl_ != nullptr) impl_->stream->stop();
}

core::Result<std::optional<CapturedFrame>> OrbbecCapture::poll() {
  if (impl_ == nullptr) {
    return core::Status::invalid_argument(
        "OrbbecCapture: poll on a moved-from capture");
  }
  if (impl_->stream->raw()) {
    return core::Status::invalid_argument(
        "OrbbecCapture: opened for raw frames; take them with poll_raw");
  }
  VKC_ASSIGN(std::shared_ptr<ob::FrameSet> pair, impl_->stream->take());
  if (pair == nullptr) return no_frame();
  return impl_->stream->process(pair);
}

core::Result<std::optional<RgbdFrame>> OrbbecCapture::poll_raw() {
  if (impl_ == nullptr) {
    return core::Status::invalid_argument(
        "OrbbecCapture: poll_raw on a moved-from capture");
  }
  if (!impl_->stream->raw()) {
    return core::Status::invalid_argument(
        "OrbbecCapture: poll_raw needs a capture opened with raw = true");
  }
  VKC_ASSIGN(std::shared_ptr<ob::FrameSet> pair, impl_->stream->take());
  if (pair == nullptr) return std::optional<RgbdFrame>();
  return impl_->stream->process_raw(pair);
}

bool OrbbecCapture::raw_frames() const noexcept {
  return impl_ != nullptr && impl_->stream->raw();
}

}  // namespace volumetric_kit::recon::sensor
