// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file allocator.hpp
/// @brief recon's names for the core's VMA-backed allocator and its memory
///        placements.
///
/// Re-exported from volumetric_kit_core (DECISIONS.md, 2026-10-04); the
/// contract is the core header's, `volumetric_kit/core/vulkan/allocator.hpp`.
///
/// Every buffer and image the GPU works on is placed with
/// @ref MemoryUsage::DeviceOnly, so a discrete GPU never reads it across the
/// bus; only staging and readback buffers live in system memory.
///
/// @code
/// VR_ASSIGN(Allocator allocator,
///           Allocator::create(instance.handle(), device));
/// BufferDesc desc;
/// desc.size = bytes;
/// desc.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;  // DeviceOnly by default
/// VR_ASSIGN(Buffer grid, allocator.create_buffer(desc));
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
