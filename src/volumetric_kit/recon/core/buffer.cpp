// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/recon/core/buffer.hpp"

#include <utility>

namespace volumetric_kit::recon {

Buffer::Buffer(VkBuffer handle, VkDeviceSize size, VkBufferUsageFlags usage,
               VkSharingMode sharing, void* mapped,
               std::function<void()> deleter) noexcept
    : buffer_(handle),
      size_(size),
      usage_(usage),
      sharing_(sharing),
      mapped_(mapped),
      deleter_(std::move(deleter)) {}

Buffer::~Buffer() { destroy(); }

Buffer::Buffer(Buffer&& other) noexcept
    : buffer_(other.buffer_),
      size_(other.size_),
      usage_(other.usage_),
      sharing_(other.sharing_),
      mapped_(other.mapped_),
      deleter_(std::move(other.deleter_)) {
  other.buffer_ = VK_NULL_HANDLE;
  other.size_ = 0;
  other.usage_ = 0;
  other.sharing_ = VK_SHARING_MODE_EXCLUSIVE;
  other.mapped_ = nullptr;
  other.deleter_ = nullptr;
}

Buffer& Buffer::operator=(Buffer&& other) noexcept {
  if (this != &other) {
    destroy();
    buffer_ = other.buffer_;
    size_ = other.size_;
    usage_ = other.usage_;
    sharing_ = other.sharing_;
    mapped_ = other.mapped_;
    deleter_ = std::move(other.deleter_);
    other.buffer_ = VK_NULL_HANDLE;
    other.size_ = 0;
    other.usage_ = 0;
    other.sharing_ = VK_SHARING_MODE_EXCLUSIVE;
    other.mapped_ = nullptr;
    other.deleter_ = nullptr;
  }
  return *this;
}

void Buffer::destroy() noexcept {
  // Frees both the VkBuffer and its VMA allocation (captured in the deleter).
  if (deleter_) {
    deleter_();
  }
  buffer_ = VK_NULL_HANDLE;
  size_ = 0;
  usage_ = 0;
  sharing_ = VK_SHARING_MODE_EXCLUSIVE;
  mapped_ = nullptr;
  deleter_ = nullptr;
}

}  // namespace volumetric_kit::recon
