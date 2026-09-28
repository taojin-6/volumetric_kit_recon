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

struct OrbbecCapture::Impl {
  // Declared first so it is destroyed last: the stream's SDK objects belong
  // to it.
  std::shared_ptr<ob::Context> context;
  std::unique_ptr<orbbec::CameraStream> stream;
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
    if (options.configure_sdk_logging) orbbec::configure_sdk_logging();
    impl->context = std::make_shared<ob::Context>();
    impl->context->enableNetDeviceEnumeration(true);
    std::vector<std::string> serials;
    if (!options.serial.empty()) serials.push_back(options.serial);
    VR_ASSIGN(const auto devices,
              orbbec::discover(*impl->context, serials,
                               options.discovery_timeout_ms, "OrbbecCapture"));
    VR_ASSIGN(impl->stream, orbbec::CameraStream::create(
                                impl->context, devices.front(), options,
                                options.cam_to_world, "OrbbecCapture"));
  } catch (const std::exception& e) {  // ob::Error is one
    return orbbec::sdk_error("OrbbecCapture", "opening the camera", e);
  }
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

Status OrbbecCapture::start() {
  if (impl_ == nullptr) {
    return Status::invalid_argument(
        "OrbbecCapture: start on a moved-from capture");
  }
  return impl_->stream->start();
}

void OrbbecCapture::stop() noexcept {
  if (impl_ != nullptr) impl_->stream->stop();
}

Result<std::optional<CapturedFrame>> OrbbecCapture::poll() {
  if (impl_ == nullptr) {
    return Status::invalid_argument(
        "OrbbecCapture: poll on a moved-from capture");
  }
  VR_ASSIGN(std::shared_ptr<ob::FrameSet> pair, impl_->stream->take());
  if (pair == nullptr) return no_frame();
  return impl_->stream->process(pair);
}

}  // namespace volumetric_kit::recon::sensor
