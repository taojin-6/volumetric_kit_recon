// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/recon/core/buffer.hpp"

#include <utility>

namespace volumetric_kit::recon {

Buffer::Buffer(VkBuffer handle, VkDeviceSize size, void* mapped,
               std::function<void()> deleter) noexcept
    : buffer_(handle),
      size_(size),
      mapped_(mapped),
      deleter_(std::move(deleter)) {}

Buffer::~Buffer() { destroy(); }

Buffer::Buffer(Buffer&& other) noexcept
    : buffer_(other.buffer_),
      size_(other.size_),
      mapped_(other.mapped_),
      deleter_(std::move(other.deleter_)) {
  other.buffer_ = VK_NULL_HANDLE;
  other.size_ = 0;
  other.mapped_ = nullptr;
  other.deleter_ = nullptr;
}

Buffer& Buffer::operator=(Buffer&& other) noexcept {
  if (this != &other) {
    destroy();
    buffer_ = other.buffer_;
    size_ = other.size_;
    mapped_ = other.mapped_;
    deleter_ = std::move(other.deleter_);
    other.buffer_ = VK_NULL_HANDLE;
    other.size_ = 0;
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
  mapped_ = nullptr;
  deleter_ = nullptr;
}

}  // namespace volumetric_kit::recon
