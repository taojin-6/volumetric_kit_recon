// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "image_io.hpp"

#include <cstddef>

#include "stb_image.h"  // declarations only; the implementation is stb_impl.cpp

namespace vr_example {

vr::Result<std::vector<std::uint32_t>> load_color_packed(
    const std::string& path, std::uint32_t expected_w,
    std::uint32_t expected_h) {
  int w = 0;
  int h = 0;
  int channels = 0;
  // Force 3 channels: a JPEG decodes to RGB regardless of its stored channel
  // count, so packing below is uniform.
  stbi_uc* pixels = stbi_load(path.c_str(), &w, &h, &channels, 3);
  if (pixels == nullptr) {
    return vr::Status::invalid_argument("load_color_packed: " + path + ": " +
                                        stbi_failure_reason());
  }
  if (static_cast<std::uint32_t>(w) != expected_w ||
      static_cast<std::uint32_t>(h) != expected_h) {
    stbi_image_free(pixels);
    return vr::Status::invalid_argument("load_color_packed: " + path +
                                        ": size does not match the intrinsics");
  }

  const std::size_t count =
      static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  std::vector<std::uint32_t> packed(count);
  for (std::size_t i = 0; i < count; ++i) {
    const std::uint32_t r = pixels[i * 3 + 0];
    const std::uint32_t g = pixels[i * 3 + 1];
    const std::uint32_t b = pixels[i * 3 + 2];
    packed[i] = r | (g << 8) | (b << 16);
  }
  stbi_image_free(pixels);
  return packed;
}

vr::Result<std::vector<float>> load_depth_metres(const std::string& path,
                                                 std::uint32_t expected_w,
                                                 std::uint32_t expected_h,
                                                 float depth_scale) {
  if (!(depth_scale > 0.0f)) {
    return vr::Status::invalid_argument(
        "load_depth_metres: depth_scale must be > 0");
  }
  int w = 0;
  int h = 0;
  int channels = 0;
  // Depth must be a genuine 16-bit PNG. stbi_load_16 would silently upscale an
  // 8-bit source (~x257), which then divides by depth_scale into depths ~257x
  // too large; reject that up front so a mis-exported dataset fails loudly. (A
  // missing/corrupt file fails the stbi_load_16 below with the specific stb
  // reason, so this precedes that only as a header check.)
  const bool is_16bit = stbi_is_16_bit(path.c_str()) != 0;
  // 16-bit single-channel decode (Replica depth is a uint16 grayscale PNG).
  stbi_us* raw = stbi_load_16(path.c_str(), &w, &h, &channels, 1);
  if (raw == nullptr) {
    return vr::Status::invalid_argument("load_depth_metres: " + path + ": " +
                                        stbi_failure_reason());
  }
  if (!is_16bit) {
    stbi_image_free(raw);
    return vr::Status::invalid_argument("load_depth_metres: " + path +
                                        ": expected a 16-bit depth PNG");
  }
  if (static_cast<std::uint32_t>(w) != expected_w ||
      static_cast<std::uint32_t>(h) != expected_h) {
    stbi_image_free(raw);
    return vr::Status::invalid_argument("load_depth_metres: " + path +
                                        ": size does not match the intrinsics");
  }

  const std::size_t count =
      static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  const float inv_scale = 1.0f / depth_scale;
  std::vector<float> metres(count);
  for (std::size_t i = 0; i < count; ++i) {
    metres[i] = static_cast<float>(raw[i]) * inv_scale;
  }
  stbi_image_free(raw);
  return metres;
}

}  // namespace vr_example
