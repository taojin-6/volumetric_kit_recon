// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// The hardware back ends as FFmpeg names them: which this platform tries, in
// which order, how a decoder is attached to one, and the warning a decoder
// gives when its pictures come to the host instead. Codec-neutral. Internal.

#include <optional>
#include <string>
#include <vector>

#include "ffmpeg.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/recon/sensor/video/video_backend.hpp"

namespace volumetric_kit::recon::sensor::video {

/// @return The hardware back ends this platform tries, most preferred first:
///         VideoToolbox on Apple; Cuda, Vaapi on Linux (NVIDIA ahead
///         of an integrated GPU); Cuda, D3d11va on Windows.
std::vector<VideoDecodeBackend> platform_hardware_order();

/// @return FFmpeg's device type for a hardware @p backend;
///         AV_HWDEVICE_TYPE_NONE for Auto and Software.
AVHWDeviceType device_type(VideoDecodeBackend backend) noexcept;

/// @return The pixel format @p codec decodes to on @p type's device, or
///         AV_PIX_FMT_NONE if this FFmpeg has no such hardware path.
AVPixelFormat hardware_pixel_format(const AVCodec* codec,
                                    AVHWDeviceType type) noexcept;

/// @return The platform's own answer to whether @p backend's hardware decodes
///         @p codec, where it has one to ask: VideoToolbox's
///         VTIsHardwareDecodeSupported, for HEVC and H.264. Empty for the
///         others, which only decoding a clip can answer.
std::optional<bool> hardware_decodes(VideoDecodeBackend backend,
                                     AVCodecID codec) noexcept;

/// @return A device context for @p backend on the default device;
///         Unsupported if this FFmpeg lacks it or no device opens.
core::Result<BufferRef> open_hardware_device(VideoDecodeBackend backend,
                                             const char* name = nullptr);

/// @brief Warn, through core's log handler, that a decoder's pictures come to
///        the host from here on because of @p why: on a discrete GPU every
///        one then crosses the bus. A decoder says it once.
/// @param who    The decoder ("HevcDecoder").
/// @param label  Whose decoder it is (a camera, say), or empty.
void warn_host_pictures(const char* who, const std::string& label,
                        const std::string& why);

/// @return Whether @p backend's pictures can be cropped at the left and top.
///         Every one's can but VideoToolbox's: FFmpeg sizes its output at the
///         display size, which VideoToolbox fills from the coded picture's
///         top-left corner, and then clears the frame's crop, so a stream
///         whose display window starts right of or below that corner comes
///         out showing the wrong region.
bool crops_left_and_top(VideoDecodeBackend backend) noexcept;

}  // namespace volumetric_kit::recon::sensor::video
