// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file image.hpp
/// @brief A 2-D `VkImage` whoever made it frees, such as a plane of a
///        hardware decoder's picture imported onto the device.

#include <cstdint>
#include <functional>

#include "volumetric_kit/recon/core/export.hpp"
#include "volumetric_kit/recon/core/vulkan.hpp"

namespace volumetric_kit::recon {

/// @brief Owns a 2-D, single-level `VkImage` and whatever backs it, freed
///        together by the deleter its maker supplies.
///
/// What recon does with one is copy it into a buffer
/// (`CommandBatch::copy`), which reads it in the one layout it stays in.
///
/// @warning An Image must not outlive the @ref Device it was made on.
class VR_CORE_API Image {
 public:
  /// @brief Construct an empty image (owns nothing; `valid()` is false).
  Image() noexcept = default;

  /// @brief Adopt @p handle and what backs it, freed by @p deleter.
  /// @param handle   The `VkImage`: 2-D, one mip level, one layer.
  /// @param format   Its format.
  /// @param width    Its width in texels.
  /// @param height   Its height in texels.
  /// @param usage    The `VkImageUsageFlags` it was created with.
  /// @param layout   The layout its maker put its contents in, which they
  ///                 stay in: `VK_IMAGE_LAYOUT_GENERAL` or
  ///                 `VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL`, the two a copy
  ///                 reads.
  /// @param deleter  Frees the image and what backs it exactly once.
  Image(VkImage handle, VkFormat format, std::uint32_t width,
        std::uint32_t height, VkImageUsageFlags usage, VkImageLayout layout,
        std::function<void()> deleter) noexcept;

  ~Image();
  Image(Image&& other) noexcept;
  Image& operator=(Image&& other) noexcept;
  Image(const Image&) = delete;
  Image& operator=(const Image&) = delete;

  /// @return The image handle (`VK_NULL_HANDLE` when empty).
  VkImage handle() const noexcept { return image_; }
  /// @return Its format (`VK_FORMAT_UNDEFINED` when empty).
  VkFormat format() const noexcept { return format_; }
  /// @return Its width in texels (`0` when empty).
  std::uint32_t width() const noexcept { return width_; }
  /// @return Its height in texels (`0` when empty).
  std::uint32_t height() const noexcept { return height_; }
  /// @return The usage flags it was created with (`0` when empty), recorded
  ///         because Vulkan cannot be asked, as @ref Buffer::usage is.
  VkImageUsageFlags usage() const noexcept { return usage_; }
  /// @return The layout its contents are in (`VK_IMAGE_LAYOUT_UNDEFINED`
  ///         when empty).
  VkImageLayout layout() const noexcept { return layout_; }
  /// @return `true` if this owns an image.
  bool valid() const noexcept { return image_ != VK_NULL_HANDLE; }

 private:
  void destroy() noexcept;

  VkImage image_ = VK_NULL_HANDLE;
  VkFormat format_ = VK_FORMAT_UNDEFINED;
  std::uint32_t width_ = 0;
  std::uint32_t height_ = 0;
  VkImageUsageFlags usage_ = 0;
  VkImageLayout layout_ = VK_IMAGE_LAYOUT_UNDEFINED;
  std::function<void()> deleter_;
};

}  // namespace volumetric_kit::recon
