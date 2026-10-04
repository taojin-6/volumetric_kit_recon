// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file allocator.hpp
/// @brief recon's names for the core's VMA-backed allocator and its memory
///        placements.
///
/// recon defines no Vulkan foundation of its own: it uses volumetric_kit_core's
/// vulkan tier (DECISIONS.md, 2026-10-04, "The Vulkan foundation comes from
/// volumetric_kit_core"), so a recon object is the same type an embedder holds
/// -- and gfx, once it adopts the core -- and one `VkDevice` serves them all.
/// The using-declarations below let recon and its consumers keep writing these
/// names in recon's namespace. The contract is the core header's,
/// `volumetric_kit/core/vulkan/allocator.hpp`.
///
/// Every buffer and image the GPU works on is placed with
/// @ref MemoryUsage::DeviceOnly, so a discrete GPU never reads it across the
/// bus; only staging and readback buffers live in system memory.
///
/// @code
/// VR_ASSIGN(Allocator allocator, Allocator::create(instance.handle(),
/// device)); VR_ASSIGN(Buffer grid, allocator.create_buffer(
///                           {bytes, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
///                            MemoryUsage::DeviceOnly}));
/// @endcode

#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/recon/core/result.hpp"

namespace volumetric_kit::recon {

using core::Allocator;
using core::BufferDesc;
using core::check_queue_family_count;
using core::HeapStats;
using core::HostAccess;
using core::ImageDesc;
using core::MemoryStats;
using core::MemoryUsage;

}  // namespace volumetric_kit::recon
