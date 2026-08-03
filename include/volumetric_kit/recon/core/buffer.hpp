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
  /// @param usage    The `VkBufferUsageFlags` it was created with.
  /// @param sharing  The `VkSharingMode` it was created with.
  /// @param mapped   Persistent host pointer, or `nullptr` if unmapped.
  /// @param deleter  Frees the buffer and its allocation exactly once.
  Buffer(VkBuffer handle, VkDeviceSize size, VkBufferUsageFlags usage,
         VkSharingMode sharing, void* mapped,
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
  /// @brief The usage flags this buffer was created with (`0` when empty).
  ///
  /// Recorded at creation because Vulkan offers no way to ask a `VkBuffer`
  /// what it was created with -- the same reason `AdoptedDevice` carries its
  /// enabled extension list. A consumer handed a borrowed buffer can therefore
  /// *verify* it permits the binding it is about to make, instead of trusting
  /// that whoever created it was passed the right flags.
  VkBufferUsageFlags usage() const noexcept { return usage_; }
  /// @brief The sharing mode this buffer was created with
  ///        (`VK_SHARING_MODE_EXCLUSIVE` when empty).
  ///
  /// Recorded for the same reason as @ref usage, and with more at stake:
  /// reading an EXCLUSIVE buffer from a queue family that does not own it is
  /// *undefined*, where a missing usage bit is at least a validation-layer
  /// diagnostic. A consumer handed a borrowed buffer can therefore check
  /// whether its family may read it directly, or whether the producer owes it
  /// an ownership transfer, instead of inferring that from flags restated at
  /// both ends of the seam.
  ///
  /// Deliberately the mode alone, not the family list: the mode is what decides
  /// whether an ownership transfer is needed, which is the actionable question
  /// and the bug this exists to catch. Record the indices too if a consumer
  /// ever needs the finer check ("was *my* family named"), which costs every
  /// buffer a small array for one call site's benefit.
  VkSharingMode sharing_mode() const noexcept { return sharing_; }
  /// @return The persistent host pointer, or `nullptr` when not host-mapped.
  void* mapped() const noexcept { return mapped_; }
  /// @return `true` if this owns a buffer.
  bool valid() const noexcept { return buffer_ != VK_NULL_HANDLE; }

 private:
  void destroy() noexcept;

  VkBuffer buffer_ = VK_NULL_HANDLE;
  VkDeviceSize size_ = 0;
  VkBufferUsageFlags usage_ = 0;
  VkSharingMode sharing_ = VK_SHARING_MODE_EXCLUSIVE;
  void* mapped_ = nullptr;
  std::function<void()> deleter_;
};

}  // namespace volumetric_kit::recon
