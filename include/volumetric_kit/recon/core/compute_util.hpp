// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file compute_util.hpp
/// @brief recon's names for the core's compute helpers: group counts,
///        storage-buffer limits and placement, and persistent inputs.
///
/// Re-exported from volumetric_kit_core (DECISIONS.md, 2026-10-04); the
/// contract is the core header's,
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
