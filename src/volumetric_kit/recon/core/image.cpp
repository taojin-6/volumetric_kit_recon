// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/recon/core/image.hpp"

#include <utility>

namespace volumetric_kit::recon {

Image::Image(VkImage handle, VkFormat format, std::uint32_t width,
             std::uint32_t height, VkImageUsageFlags usage,
             VkImageLayout layout, std::function<void()> deleter) noexcept
    : image_(handle),
      format_(format),
      width_(width),
      height_(height),
      usage_(usage),
      layout_(layout),
      deleter_(std::move(deleter)) {}

Image::~Image() { destroy(); }

Image::Image(Image&& other) noexcept
    : image_(other.image_),
      format_(other.format_),
      width_(other.width_),
      height_(other.height_),
      usage_(other.usage_),
      layout_(other.layout_),
      deleter_(std::move(other.deleter_)) {
  other.image_ = VK_NULL_HANDLE;
  other.format_ = VK_FORMAT_UNDEFINED;
  other.width_ = 0;
  other.height_ = 0;
  other.usage_ = 0;
  other.layout_ = VK_IMAGE_LAYOUT_UNDEFINED;
  other.deleter_ = nullptr;
}

Image& Image::operator=(Image&& other) noexcept {
  if (this != &other) {
    destroy();
    image_ = other.image_;
    format_ = other.format_;
    width_ = other.width_;
    height_ = other.height_;
    usage_ = other.usage_;
    layout_ = other.layout_;
    deleter_ = std::move(other.deleter_);
    other.image_ = VK_NULL_HANDLE;
    other.format_ = VK_FORMAT_UNDEFINED;
    other.width_ = 0;
    other.height_ = 0;
    other.usage_ = 0;
    other.layout_ = VK_IMAGE_LAYOUT_UNDEFINED;
    other.deleter_ = nullptr;
  }
  return *this;
}

void Image::destroy() noexcept {
  if (deleter_) deleter_();
  image_ = VK_NULL_HANDLE;
  format_ = VK_FORMAT_UNDEFINED;
  width_ = 0;
  height_ = 0;
  usage_ = 0;
  layout_ = VK_IMAGE_LAYOUT_UNDEFINED;
  deleter_ = nullptr;
}

}  // namespace volumetric_kit::recon
