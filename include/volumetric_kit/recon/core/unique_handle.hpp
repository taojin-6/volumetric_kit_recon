// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file unique_handle.hpp
/// @brief recon's name for the core's move-only Vulkan handle owner.
///
/// Re-exported from volumetric_kit_core (DECISIONS.md, 2026-10-04); the
/// contract is the core header's,
/// `volumetric_kit/core/vulkan/unique_handle.hpp`.

#include "volumetric_kit/core/vulkan/unique_handle.hpp"

namespace volumetric_kit::recon {

using core::UniqueHandle;

}  // namespace volumetric_kit::recon
