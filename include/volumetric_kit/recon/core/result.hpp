// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file result.hpp
/// @brief recon's error handling: volumetric_kit_core's `Status` and `Result`,
///        named in this namespace.
///
/// recon defines no error types of its own. It uses the family's shared ones,
/// from volumetric_kit_core's base tier (DECISIONS.md, 2026-10-03, "Error
/// handling comes from volumetric_kit_core"), so a `Status` from recon is the
/// same type as one from calib or gfx and passes between them unchanged. The
/// using-declarations below let recon and its consumers keep writing `Status`
/// and `Result<T>` in this namespace.
///
/// No exceptions cross the API boundary: mobile consumers build with
/// `-fno-exceptions`. Fallible calls return `Status` or `Result<T>`, both
/// `[[nodiscard]]`. `Status` is backend-neutral: a failed Vulkan call is
/// `Status::Code::Backend` with the `VkResult` as its `detail()` (see
/// vk_result.hpp). Reading the value of an error `Result` is a programmer error
/// and aborts. The full contract is in the core's
/// `volumetric_kit/core/base/result.hpp`.
///
/// @code
/// Result<VoxelHashMap> r = VoxelHashMap::create(config);
/// if (!r) return r.status();      // propagate failure to our caller
/// VoxelHashMap& map = r.value();  // safe: guarded by the !r check above
/// @endcode

#include "volumetric_kit/core/base/result.hpp"

namespace volumetric_kit::recon {

using core::Result;
using core::Status;
using core::to_string;

}  // namespace volumetric_kit::recon

// TODO: rename VR_TRY / VR_ASSIGN (and VR_CHECK, check.hpp) to the core's
// VKC_TRY / VKC_ASSIGN / VKC_CHECK across recon once the branches open on
// 2026-10-03 have landed, then delete these aliases. Until then the old names
// keep those branches merging cleanly; new code may use either.

/// @brief recon's name for the core's @ref VKC_TRY: evaluate a `Status`
///        expression and early-return it if not OK.
/// @param expr  An expression yielding a `Status`.
#define VR_TRY(expr) VKC_TRY(expr)

/// @brief recon's name for the core's @ref VKC_ASSIGN: unwrap a `Result<T>`
///        into @p decl, or early-return its `Status`.
/// @param decl  A variable declaration bound to the unwrapped value.
/// @param expr  An expression yielding a `Result<T>`.
#define VR_ASSIGN(decl, expr) VKC_ASSIGN(decl, expr)
