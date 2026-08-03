// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file allocator.hpp
/// @brief The VMA allocator: recon's device-memory arena and the factory for
///        @ref Buffer.

#include <cstdint>
#include <memory>

#include "volumetric_kit/recon/core/export.hpp"
#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/core/vulkan.hpp"

namespace volumetric_kit::recon {

class Device;
class Buffer;

/// @brief Where a buffer's memory should live.
enum class MemoryUsage {
  Auto,         ///< Let VMA choose based on usage (`VMA_MEMORY_USAGE_AUTO`).
  DeviceLocal,  ///< Prefer device-local (GPU) memory (staged uploads).
  HostVisible,  ///< Prefer host-visible (CPU-mappable) memory.
};

/// @brief Host access pattern for a mapped buffer; selects the VMA host-access
///        allocation flag.
enum class HostAccess {
  Random,  ///< Reads and writes in any order (`..._HOST_ACCESS_RANDOM`).
  SequentialWrite,  ///< Write-once, front-to-back (`..._SEQUENTIAL_WRITE`).
};

/// @brief One memory heap's usage and budget, in bytes.
///
/// Both figures are VMA's running accounting of the heap, not a live driver
/// query: @ref usage_bytes is what VMA has allocated out of it, @ref
/// budget_bytes is how much VMA estimates is safely usable. They are heuristics
/// unless `VK_EXT_memory_budget` is enabled, which lets VMA read the driver's
/// authoritative figures.
///
/// TODO(core): enable `VK_EXT_memory_budget` on @ref Device::create (and
/// require it in @ref DeviceRequirements for the adopt path) so these become
/// the driver's own numbers rather than VMA's estimate.
struct HeapStats {
  std::uint64_t usage_bytes = 0;   ///< Bytes VMA has allocated from the heap.
  std::uint64_t budget_bytes = 0;  ///< Bytes VMA estimates are usable in it.
};

/// @brief A snapshot of per-heap memory usage across the device's memory heaps.
///
/// Allocation-free: a fixed `VK_MAX_MEMORY_HEAPS` array with a live count, so
/// it can be filled and returned by value without touching the host heap. Only
/// the leading @ref heap_count entries of @ref heaps carry valid figures.
///
/// On a unified-memory (UMA) GPU -- Apple silicon through MoltenVK, this repo's
/// primary target -- system and device memory are one pool, so the device
/// typically reports a single unified heap rather than the separate
/// device-local / host-visible heaps a discrete GPU exposes.
struct MemoryStats {
  std::uint32_t heap_count = 0;  ///< Number of valid entries in @ref heaps.
  HeapStats heaps[VK_MAX_MEMORY_HEAPS]{};  ///< Per-heap usage/budget.
};

/// @brief Parameters for @ref Allocator::create_buffer.
struct BufferDesc {
  /// Size in bytes; must be non-zero.
  VkDeviceSize size = 0;
  /// Usage flags (e.g. `VK_BUFFER_USAGE_STORAGE_BUFFER_BIT`); must be non-zero.
  VkBufferUsageFlags usage = 0;
  /// Where the memory should live.
  MemoryUsage memory = MemoryUsage::Auto;
  /// Persistently map the allocation (host-visible only). A mapped buffer's
  /// memory is reachable through @ref Buffer::mapped for the buffer's lifetime.
  bool mapped = false;
  /// Host access pattern; consulted only when @ref mapped is set.
  HostAccess host_access = HostAccess::Random;

  /// @brief The queue families that will access this buffer.
  ///
  /// Left null (the default), the buffer is `VK_SHARING_MODE_EXCLUSIVE` and is
  /// owned by whichever family first uses it -- correct for everything this
  /// library allocates for itself.
  ///
  /// It is *not* correct for a buffer a renderer will read directly (interop
  /// seam B). On Apple the reconstruction compute and the renderer are handed
  /// queues from **different families**, and reading an EXCLUSIVE buffer from a
  /// family that does not own it is undefined -- the contents are not
  /// guaranteed to be there, with no error and often no visible symptom on the
  /// device that happened to work.
  ///
  /// Enumerate the families that will touch it and this picks the mode: two or
  /// more distinct indices give `VK_SHARING_MODE_CONCURRENT`, one gives
  /// EXCLUSIVE, because a single family *is* exclusive and Vulkan rejects
  /// CONCURRENT with fewer than two. Duplicates are ignored for that count, so
  /// a caller can pass its compute and render families unconditionally and get
  /// EXCLUSIVE for free wherever the two turn out to be the same family --
  /// which is the common case off Apple, and where CONCURRENT would otherwise
  /// cost access performance for nothing.
  ///
  /// The array need only outlive the @ref Allocator::create_buffer call.
  const std::uint32_t* queue_families = nullptr;
  /// Number of entries in @ref queue_families; must be zero when it is null.
  std::uint32_t queue_family_count = 0;
};

namespace detail {

/// @brief Reduce @p families to its distinct entries, in first-seen order.
///
/// The rule @ref BufferDesc::queue_families describes, on its own so it can be
/// pinned by a host test. Its failure mode is the reason: getting it wrong
/// produces a buffer Vulkan rejects only when a validation layer happens to be
/// installed, and a *silently wrong sharing mode* when one is not -- which is
/// the same "correct on the machine that ran it" hazard the camera and colour
/// conventions are kept as pure arithmetic to avoid.
///
/// @param families      Entries to reduce; may be null only when @p count is 0.
/// @param count         Number of entries in @p families.
/// @param out           Receives the distinct entries; may be null only when
///                      @p out_capacity is 0.
/// @param out_capacity  How many entries @p out holds.
/// @return The distinct count, or `out_capacity + 1` when more distinct
///         families were found than @p out can hold -- a value the caller can
///         only treat as an error, since the reduction is then incomplete.
VR_CORE_API std::uint32_t distinct_queue_families(const std::uint32_t* families,
                                                  std::uint32_t count,
                                                  std::uint32_t* out,
                                                  std::uint32_t out_capacity);

}  // namespace detail

/// @brief Owns a `VmaAllocator` built over a `VkDevice`, and creates
///        VMA-backed @ref Buffer resources on it.
///
/// A separate object from @ref Device rather than a member of it: a device
/// obtained through @ref Device::adopt (shared with the renderer) still gets
/// its own allocator here -- VMA allocators are independent bookkeeping over
/// the same `VkDevice` memory, so each library manages its own. Build one per
/// @ref Device.
///
/// @warning The `VkInstance` and @ref Device passed to @ref create must outlive
///          this allocator; it stores their handles.
/// @warning This allocator must outlive every @ref Buffer it creates: a Buffer
///          frees its `VkBuffer` and allocation through this allocator, so a
///          Buffer destroyed after its Allocator dereferences a freed allocator
///          (a use-after-free). Destroy Buffers before their Allocator.
///
/// @code
/// Result<Allocator> alloc = Allocator::create(instance.handle(), device);
/// if (!alloc) return alloc.status();
/// @endcode
class VR_CORE_API Allocator {
 public:
  /// @brief Create a VMA allocator over @p device.
  /// @param instance  The instance @p device belongs to.
  /// @param device    The logical device to allocate on (must outlive this).
  /// @return The allocator, or a non-OK @ref Status:
  ///         @ref Status::Code::InvalidArgument for a null @p instance or an
  ///         invalid @p device; @ref Status::Code::Backend if
  ///         `vmaCreateAllocator` fails.
  static Result<Allocator> create(VkInstance instance, const Device& device);

  ~Allocator();
  Allocator(Allocator&& other) noexcept;
  Allocator& operator=(Allocator&& other) noexcept;
  Allocator(const Allocator&) = delete;
  Allocator& operator=(const Allocator&) = delete;

  /// @brief Allocate a `VkBuffer` and its backing memory per @p desc.
  /// @param desc  Size, usage, memory location, and mapping request.
  /// @return The buffer, or a non-OK @ref Status:
  ///         @ref Status::Code::InvalidArgument for a zero size/usage, a
  ///         `mapped` device-local request, or a host-visible request that is
  ///         not `mapped`; @ref Status::Code::Backend if VMA fails.
  Result<Buffer> create_buffer(const BufferDesc& desc);

  /// @brief Sample per-heap memory usage and budget across the device's heaps.
  ///
  /// The figures cover what *this* allocator has allocated -- a device shared
  /// with the renderer (@ref Device::adopt) gives each library its own VMA
  /// allocator, so each reports only its own share of the same `VkDevice`
  /// memory.
  ///
  /// @return A @ref MemoryStats whose @ref MemoryStats::heap_count names the
  ///         device's memory heaps and whose first that-many @ref
  ///         MemoryStats::heaps entries carry each heap's usage/budget in
  ///         bytes. A moved-from allocator reports `heap_count == 0`.
  /// @note The byte figures are VMA heuristics unless `VK_EXT_memory_budget` is
  ///       enabled; see @ref HeapStats. On UMA (Apple) GPUs expect one heap.
  MemoryStats memory_stats() const;

  /// @return `true` if this owns an allocator (`false` when moved-from).
  bool valid() const noexcept { return impl_ != nullptr; }

 private:
  Allocator() = default;

  // pImpl so the VmaAllocator handle -- and thus <vk_mem_alloc.h> -- stays out
  // of this public header. ~Impl frees the allocator, which is what lets the
  // move operations below default correctly.
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace volumetric_kit::recon
