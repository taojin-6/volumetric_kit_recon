// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file buffer.hpp
/// @brief A `VkBuffer` plus its VMA allocation, owned and freed together.

#include <functional>

#include "volumetric_kit/recon/core/export.hpp"
#include "volumetric_kit/recon/core/vulkan.hpp"

namespace volumetric_kit::recon {

/// @brief Owns a `VkBuffer` and the VMA allocation backing it, freeing both
///        together.
///
/// Constructed only by @ref Allocator::create_buffer. The VMA allocation is
/// held inside a type-erased `std::function<void()>` deleter, so
/// `<vk_mem_alloc.h>` never reaches this header or a consumer (the
/// backend-out-of-headers rule). A buffer created with `BufferDesc::mapped`
/// exposes a persistent host pointer through @ref mapped; a device-local
/// buffer's @ref mapped is `nullptr`.
///
/// @warning A Buffer must not outlive the @ref Allocator that created it: the
///          deleter frees the `VkBuffer` and its allocation through that
///          allocator, so freeing once the Allocator is gone is a
///          use-after-free.
class VR_CORE_API Buffer {
 public:
  /// @brief Construct an empty buffer (owns nothing; `valid()` is false).
  Buffer() noexcept = default;

  /// @brief Adopt @p handle and its allocation, freed by @p deleter.
  ///
  /// Called by @ref Allocator::create_buffer; the @p deleter captures the
  /// opaque VMA handles and calls `vmaDestroyBuffer`.
  /// @param handle   The `VkBuffer`.
  /// @param size     Its size in bytes.
  /// @param mapped   Persistent host pointer, or `nullptr` if unmapped.
  /// @param deleter  Frees the buffer and its allocation exactly once.
  Buffer(VkBuffer handle, VkDeviceSize size, void* mapped,
         std::function<void()> deleter) noexcept;

  ~Buffer();
  Buffer(Buffer&& other) noexcept;
  Buffer& operator=(Buffer&& other) noexcept;
  Buffer(const Buffer&) = delete;
  Buffer& operator=(const Buffer&) = delete;

  /// @return The buffer handle (`VK_NULL_HANDLE` when empty).
  VkBuffer handle() const noexcept { return buffer_; }
  /// @return The size in bytes (`0` when empty).
  VkDeviceSize size() const noexcept { return size_; }
  /// @return The persistent host pointer, or `nullptr` when not host-mapped.
  void* mapped() const noexcept { return mapped_; }
  /// @return `true` if this owns a buffer.
  bool valid() const noexcept { return buffer_ != VK_NULL_HANDLE; }

 private:
  void destroy() noexcept;

  VkBuffer buffer_ = VK_NULL_HANDLE;
  VkDeviceSize size_ = 0;
  void* mapped_ = nullptr;
  std::function<void()> deleter_;
};

}  // namespace volumetric_kit::recon
