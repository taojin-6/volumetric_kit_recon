// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file instance.hpp
/// @brief recon's names for the core's Vulkan instance.
///
/// Re-exported from volumetric_kit_core (DECISIONS.md, 2026-10-04); the
/// contract is the core header's, `volumetric_kit/core/vulkan/instance.hpp`.

#include "volumetric_kit/core/vulkan/instance.hpp"
#include "volumetric_kit/recon/core/result.hpp"

namespace volumetric_kit::recon {

using core::Instance;
using core::InstanceConfig;

}  // namespace volumetric_kit::recon
