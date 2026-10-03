// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file core/external_memory.hpp
/// @brief Device memory another API on the same GPU writes into: CUDA writing
///        a hardware decoder's picture, which the kernels then read in place;
///        and the memory type such a resource is bound to.

#include <cstdint>
#include <optional>

#include "volumetric_kit/recon/core/buffer.hpp"
#include "volumetric_kit/recon/core/export.hpp"
#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/core/vulkan.hpp"

namespace volumetric_kit::recon {

class Device;

/// @brief A buffer on memory of its own, and a file descriptor for that
///        memory.
struct ExportedBuffer {
  /// A device-local storage buffer, with transfer usage too, on dedicated
  /// memory. It frees the memory with itself.
  Buffer buffer;
  /// An opaque file descriptor for the memory (`VK_KHR_external_memory_fd`).
  /// An import that succeeds owns it, as CUDA's does; until then the caller
  /// does, and closes it.
  int fd = -1;
  /// The memory's size, which an importer is told.
  VkDeviceSize memory_size = 0;
};

/// @brief The memory type to bind a resource to that is not allocated through
///        the @ref Allocator: the first that @p type_bits allows with every
///        flag of @p required and none of @p excluded.
///
/// Vulkan orders a memory type ahead of any whose flags strictly contain its
/// own, so the first match is the plainest. Asked for device-local alone, it
/// is a type the host cannot map wherever @p type_bits allows one, which keeps
/// the resource out of a ReBAR system's host-visible BAR heap. Protected,
/// lazily allocated and AMD device-coherent types are always skipped: the
/// features they need are not enabled by @ref Device::create.
/// @param device    The device whose memory types are searched.
/// @param type_bits `VkMemoryRequirements::memoryTypeBits` of the resource.
/// @param required  Flags the type must have; 0 for any.
/// @param excluded  Flags the type must not have; 0 for none.
/// @return The type's index, or empty when no allowed type fits.
VR_CORE_API std::optional<std::uint32_t> find_memory_type(
    const Device& device, std::uint32_t type_bits,
    VkMemoryPropertyFlags required, VkMemoryPropertyFlags excluded = 0);

/// @brief Make a buffer that another API on the same GPU imports and writes.
///
/// Its memory comes from Vulkan directly, not from the @ref Allocator: an
/// exported allocation needs memory of its own, and the few a decoder keeps
/// need no pool.
/// @param device  A device that exports memory (@ref Device::exports_memory).
/// @param bytes   The buffer's size; not 0.
/// @return The buffer and its descriptor; Unsupported where @p device does not
///         export memory, cannot export this buffer, or has no device-local
///         memory it can use;
///         InvalidArgument for 0 bytes; otherwise a Vulkan failure.
VR_CORE_API Result<ExportedBuffer> create_exported_buffer(const Device& device,
                                                          VkDeviceSize bytes);

}  // namespace volumetric_kit::recon
