// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file compute_util.hpp
/// @brief recon's names for the core's compute helpers: group counts,
///        storage-buffer limits and placement, and persistent inputs.
///
/// recon defines no Vulkan foundation of its own: it uses volumetric_kit_core's
/// vulkan tier (DECISIONS.md, 2026-10-04, "The Vulkan foundation comes from
/// volumetric_kit_core"), so a recon object is the same type an embedder holds
/// -- and gfx, once it adopts the core -- and one `VkDevice` serves them all.
/// The using-declarations below let recon and its consumers keep writing these
/// names in recon's namespace. The contract is the core header's,
/// `volumetric_kit/core/vulkan/compute_util.hpp`.

#include "volumetric_kit/core/vulkan/compute_util.hpp"
#include "volumetric_kit/recon/core/result.hpp"

namespace volumetric_kit::recon {

using core::check_storage_buffer_range;
using core::device_storage_buffer;
using core::ensure_device_scratch;
using core::group_count;
using core::mapped_storage_buffer;
using core::max_storage_buffer_range;
using core::StorageInput;
using core::upload_storage_buffer;

}  // namespace volumetric_kit::recon
