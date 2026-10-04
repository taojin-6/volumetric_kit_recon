// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file vk_result.hpp
/// @brief recon's names for the core's bridge from a `VkResult` to a backend
///        `Status`.
///
/// Re-exported from volumetric_kit_core (DECISIONS.md, 2026-10-04); the
/// contract is the core header's, `volumetric_kit/core/vulkan/vk_result.hpp`.

#include "volumetric_kit/core/vulkan/vk_result.hpp"
#include "volumetric_kit/recon/core/result.hpp"

namespace volumetric_kit::recon {

using core::vk_error;
using core::vk_result;

}  // namespace volumetric_kit::recon

// TODO: rename VR_VK_TRY to the core's VKC_VK_TRY with VR_TRY (result.hpp).

/// @brief Early-return a backend `Status` from a failed `VkResult`; the core's
///        `VKC_VK_TRY` under recon's name.
/// @param expr  An expression yielding a `VkResult`.
#define VR_VK_TRY(expr) VKC_VK_TRY(expr)
