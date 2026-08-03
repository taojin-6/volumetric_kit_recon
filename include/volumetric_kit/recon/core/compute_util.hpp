// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file core/compute_util.hpp
/// @brief Small host-side helpers every compute tier repeats: the dispatch
///        group-count ceil-divide and host-visible storage-buffer
///        create/upload.
///
/// These sit alongside @ref dispatch / @ref KernelSetBuilder (the 2026-07-06
/// "mechanism lives in core because every compute tier repeats the shape"
/// decision): the group-count math and the "make a mapped storage buffer"
/// pattern had been copied verbatim into every tier. Hoisted here so a tier
/// declares neither. The **policy** (which buffers, which bindings) stays in
/// the tier.

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "volumetric_kit/recon/core/allocator.hpp"
#include "volumetric_kit/recon/core/buffer.hpp"
#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/core/vulkan.hpp"

namespace volumetric_kit::recon {

/// @brief Workgroup count for a 1-D dispatch of @p items threads at
///        @p local_size threads per group: `ceil(items / local_size)`.
///
/// Computed as `items / local_size + (items % local_size != 0)` rather than the
/// `(items + local_size - 1) / local_size` idiom, which overflows `uint32` for
/// @p items near `UINT32_MAX` (wrapping to ~0 groups -> a silent no-op
/// dispatch).
/// @param items       Thread count (one per work item).
/// @param local_size  Threads per workgroup (the shader's `local_size_x`).
/// @return The number of workgroups to dispatch.
inline std::uint32_t group_count(std::uint32_t items,
                                 std::uint32_t local_size) {
  return items / local_size + (items % local_size != 0u ? 1u : 0u);
}

/// @brief Create a host-visible, host-mapped storage buffer of @p bytes.
/// @param allocator    The allocator to create on.
/// @param bytes        Size in bytes (must be non-zero).
/// @param access       Host access pattern (@ref HostAccess::Random when the
///                     host both writes and reads back; @ref
///                     HostAccess::SequentialWrite for write-once inputs).
/// @param extra_usage  Usage bits added beyond `STORAGE_BUFFER`, for a
///                     *consumer* that binds the same allocation some other way
///                     (a renderer taking it as a vertex buffer). A compute
///                     tier's own kernels need only the default.
/// @param queue_families      Families that will access the buffer; see @ref
///                            BufferDesc::queue_families. Null (the default)
///                            leaves it `VK_SHARING_MODE_EXCLUSIVE`, which is
///                            what a tier allocating for its own kernels wants.
///                            A buffer a *sibling* reads must name both
///                            families, or the cross-family read is undefined.
/// @param queue_family_count  Entries in @p queue_families.
/// @return The buffer, or a non-OK @ref Status if creation fails.
///
/// @note The allocation is deliberately `HostVisible` + mapped: this helper
///       exists for buffers the host fills or reads back. That is the right
///       trade for inputs and for a counter the host must read every dispatch,
///       and the wrong one for a large output a device-local consumer streams
///       -- such a consumer wants its own allocation, not a parameter here.
inline Result<Buffer> storage_buffer(
    Allocator& allocator, VkDeviceSize bytes,
    HostAccess access = HostAccess::Random, VkBufferUsageFlags extra_usage = 0,
    const std::uint32_t* queue_families = nullptr,
    std::uint32_t queue_family_count = 0) {
  BufferDesc desc;
  desc.size = bytes;
  desc.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | extra_usage;
  desc.memory = MemoryUsage::HostVisible;
  desc.mapped = true;
  desc.host_access = access;
  desc.queue_families = queue_families;
  desc.queue_family_count = queue_family_count;
  return allocator.create_buffer(desc);
}

/// @brief @ref storage_buffer of @p bytes, filled from @p src.
/// @param allocator  The allocator to create on.
/// @param src        Source bytes to copy in (at least @p bytes long).
/// @param bytes      Size in bytes (must be non-zero).
/// @param access     Host access pattern; defaults to
///                   @ref HostAccess::SequentialWrite (write-once inputs).
/// @return The filled buffer, or a non-OK @ref Status if creation fails.
inline Result<Buffer> upload_storage_buffer(
    Allocator& allocator, const void* src, VkDeviceSize bytes,
    HostAccess access = HostAccess::SequentialWrite) {
  VR_ASSIGN(Buffer buf, storage_buffer(allocator, bytes, access));
  std::memcpy(buf.mapped(), src, static_cast<std::size_t>(bytes));
  return buf;
}

}  // namespace volumetric_kit::recon
