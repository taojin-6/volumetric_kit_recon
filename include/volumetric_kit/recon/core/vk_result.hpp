// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file vk_result.hpp
/// @brief Bridge a Vulkan `VkResult` into the backend-neutral core `Status`.
///
/// @ref result.hpp stays free of any GPU-API include so the `Status`/`Result`
/// idiom serves every tier. This header is the one place the Vulkan core turns
/// a failed `VkResult` into a `Status` (domain @ref Status::Code::Backend,
/// detail = the `VkResult` value), and where @ref VR_VK_TRY lives -- the
/// `VkResult` analogue of @ref VR_TRY.

#include <cstdint>
#include <string>
#include <string_view>

#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/core/vulkan.hpp"

namespace volumetric_kit::recon {

/// @brief Wrap a non-success `VkResult` as a backend @ref Status.
/// @param code  The `VkResult` returned by a failed Vulkan call.
/// @param what  Human-readable context (e.g. the failing call site).
/// @return A non-OK `Status` (domain @ref Status::Code::Backend) carrying
///         @p code as its @ref Status::detail.
inline Status vk_error(VkResult code, std::string_view what) {
  return Status::backend_error(static_cast<std::int64_t>(code),
                               std::string(what));
}

}  // namespace volumetric_kit::recon

/// @brief Evaluate a `VkResult` expression and early-return a backend `Status`
///        if it is not `VK_SUCCESS`.
/// @param expr  An expression yielding a `VkResult`.
///
/// Usable only inside a function returning `Status` or `Result<T>`. The `#expr`
/// stringization gives the failure message the failing call site.
///
/// @code
/// Status init() {
///   VR_VK_TRY(vkCreateInstance(&ci, nullptr, &instance_));
///   return {};
/// }
/// @endcode
#define VR_VK_TRY(expr)                                        \
  do {                                                         \
    VkResult _vr_vk = (expr);                                  \
    if (_vr_vk != VK_SUCCESS) {                                \
      return ::volumetric_kit::recon::vk_error(_vr_vk, #expr); \
    }                                                          \
  } while (0)
