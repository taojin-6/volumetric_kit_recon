// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file buffer.hpp
/// @brief A `VkBuffer` plus its backing allocation, owned and freed together.

#include <cstdint>
#include <functional>
#include <optional>

#include "volumetric_kit/recon/core/export.hpp"
#include "volumetric_kit/recon/core/vulkan.hpp"

namespace volumetric_kit::recon {

/// @brief Actual memory type backing a buffer, queried at allocation time.
///
/// Indices refer to the allocating physical device's memory properties.
/// `HOST_VISIBLE` and `DEVICE_LOCAL` can both be set, on unified memory and
/// host-visible device heaps; mapping support does not imply system memory.
struct BufferMemoryInfo {
  VkMemoryPropertyFlags properties = 0;  ///< Selected type's property flags.
  std::uint32_t type_index = 0;          ///< Selected Vulkan memory type.
  std::uint32_t heap_index = 0;          ///< Heap backing the selected type.
};

/// @brief Owns a `VkBuffer` and the allocation backing it, freeing both
///        together.
///
/// Constructed by @ref Allocator::create_buffer or by adopting a buffer.
/// A VMA allocation is
/// held inside a type-erased `std::function<void()>` deleter, so
/// `<vk_mem_alloc.h>` never reaches this header or a consumer (the
/// backend-out-of-headers rule). A buffer created with `BufferDesc::mapped`
/// exposes a persistent host pointer through @ref mapped. An allocation
/// requested with `MemoryUsage::DeviceLocal` is never mapped; an allocation
/// requested as host-visible can also have the device-local memory flag.
///
/// A VMA-backed buffer retains the allocator's implementation until it is
/// freed, including across allocator moves. The Vulkan instance and device
/// must still outlive it. An adopted buffer's deleter must likewise retain
/// everything it needs to release the buffer and allocation.
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
  /// @param memory   Actual backing memory metadata, or `std::nullopt` when
  ///                 unknown. An adopter supplying it must query the bound
  ///                 allocation's selected memory type, not infer it from
  ///                 usage. Required, so an adopter decides: a buffer with
  ///                 unknown memory is refused as a device input.
  Buffer(VkBuffer handle, VkDeviceSize size, VkBufferUsageFlags usage,
         VkSharingMode sharing, void* mapped, std::function<void()> deleter,
         std::optional<BufferMemoryInfo> memory) noexcept;

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
  /// @return Actual backing memory metadata, or `std::nullopt` for an empty
  ///         buffer or an adopted buffer whose memory type was not supplied.
  const std::optional<BufferMemoryInfo>& memory_info() const noexcept {
    return memory_;
  }
  /// @return Whether the known backing memory has `DEVICE_LOCAL` set.
  ///         `HOST_VISIBLE` is allowed too; unknown memory returns false.
  bool is_device_local() const noexcept {
    return memory_.has_value() &&
           (memory_->properties & VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT) != 0;
  }
  /// @return `true` if this owns a buffer.
  bool valid() const noexcept { return buffer_ != VK_NULL_HANDLE; }

 private:
  void destroy() noexcept;

  VkBuffer buffer_ = VK_NULL_HANDLE;
  VkDeviceSize size_ = 0;
  VkBufferUsageFlags usage_ = 0;
  VkSharingMode sharing_ = VK_SHARING_MODE_EXCLUSIVE;
  void* mapped_ = nullptr;
  std::optional<BufferMemoryInfo> memory_;
  std::function<void()> deleter_;
};

}  // namespace volumetric_kit::recon
