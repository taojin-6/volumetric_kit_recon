// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file check.hpp
/// @brief recon's name for the core's fail-fast contract check.
///
/// `VR_CHECK` is volumetric_kit_core's `VKC_CHECK`: for *programmer errors*
/// (precondition violations), as distinct from recoverable runtime failures,
/// which flow through `Status` / `Result`. On failure it logs at Error through
/// the family's log sink (source `"core"`), then calls `std::abort()`, in every
/// build. Abort rather than `throw`, because mobile consumers build with
/// `-fno-exceptions` and crash reporters capture SIGABRT.

#include "volumetric_kit/core/base/check.hpp"

// TODO: rename to VKC_CHECK with VR_TRY / VR_ASSIGN (result.hpp), then delete
// this alias.

/// @brief Abort, after logging, unless @p cond holds; the core's
///        @ref VKC_CHECK under recon's name.
/// @param cond  A precondition expression that must hold.
/// @param msg   A description of the contract.
#define VR_CHECK(cond, msg) VKC_CHECK(cond, msg)
