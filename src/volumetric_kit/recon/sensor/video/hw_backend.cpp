// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "hw_backend.hpp"

#include <string>
#include <vector>

#include "volumetric_kit/recon/core/log.hpp"

#if defined(__APPLE__)
#include <VideoToolbox/VideoToolbox.h>
#endif

namespace volumetric_kit::recon::sensor::video {

std::vector<VideoDecodeBackend> platform_hardware_order() {
#if defined(__APPLE__)
  return {VideoDecodeBackend::VideoToolbox};
#elif defined(_WIN32)
  return {VideoDecodeBackend::Cuda, VideoDecodeBackend::D3d11va};
#else
  return {VideoDecodeBackend::Cuda, VideoDecodeBackend::Vaapi};
#endif
}

AVHWDeviceType device_type(VideoDecodeBackend backend) noexcept {
  switch (backend) {
    case VideoDecodeBackend::VideoToolbox:
      return AV_HWDEVICE_TYPE_VIDEOTOOLBOX;
    case VideoDecodeBackend::Cuda:
      return AV_HWDEVICE_TYPE_CUDA;
    case VideoDecodeBackend::Vaapi:
      return AV_HWDEVICE_TYPE_VAAPI;
    case VideoDecodeBackend::D3d11va:
      return AV_HWDEVICE_TYPE_D3D11VA;
    case VideoDecodeBackend::Auto:
    case VideoDecodeBackend::Software:
      break;
  }
  return AV_HWDEVICE_TYPE_NONE;
}

AVPixelFormat hardware_pixel_format(const AVCodec* codec,
                                    AVHWDeviceType type) noexcept {
  for (int i = 0;; ++i) {
    const AVCodecHWConfig* config = avcodec_get_hw_config(codec, i);
    if (config == nullptr) return AV_PIX_FMT_NONE;
    if ((config->methods & AV_CODEC_HW_CONFIG_METHOD_HW_DEVICE_CTX) != 0 &&
        config->device_type == type) {
      return config->pix_fmt;
    }
  }
}

std::optional<bool> hardware_decodes(VideoDecodeBackend backend,
                                     AVCodecID codec) noexcept {
#if defined(__APPLE__)
  if (backend == VideoDecodeBackend::VideoToolbox) {
    switch (codec) {
      case AV_CODEC_ID_HEVC:
        return VTIsHardwareDecodeSupported(kCMVideoCodecType_HEVC) != 0;
      case AV_CODEC_ID_H264:
        return VTIsHardwareDecodeSupported(kCMVideoCodecType_H264) != 0;
      default:
        break;
    }
  }
#else
  (void)backend;
  (void)codec;
#endif
  return std::nullopt;
}

core::Result<BufferRef> open_hardware_device(VideoDecodeBackend backend,
                                             const char* name) {
  const AVHWDeviceType type = device_type(backend);
  if (type == AV_HWDEVICE_TYPE_NONE) {
    return core::Status::invalid_argument(
        std::string("no device for back end ") + to_string(backend));
  }
  AVBufferRef* device = nullptr;
  const int err = av_hwdevice_ctx_create(&device, type, name, nullptr, 0);
  if (err < 0) {
    return core::Status::unsupported(
        std::string(to_string(backend)) +
        ": no device opens: " + ffmpeg_message(err));
  }
  return BufferRef(device);
}

void warn_host_pictures(const char* who, const std::string& label,
                        const std::string& why) {
  log_message(core::LogLevel::Warning,
              (label.empty() ? std::string() : label + ": ") + who + ": " +
                  why + "; from here on its pictures come to the host");
}

bool crops_left_and_top(VideoDecodeBackend backend) noexcept {
  return backend != VideoDecodeBackend::VideoToolbox;
}

}  // namespace volumetric_kit::recon::sensor::video
