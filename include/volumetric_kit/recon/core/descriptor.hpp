// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file descriptor.hpp
/// @brief recon's names for the core's descriptor pool, layout and set.
///
/// Re-exported from volumetric_kit_core (DECISIONS.md, 2026-10-04); the
/// contract is the core header's, `volumetric_kit/core/vulkan/descriptor.hpp`.

#include "volumetric_kit/core/vulkan/descriptor.hpp"
#include "volumetric_kit/recon/core/result.hpp"

namespace volumetric_kit::recon {

using core::DescriptorPool;
using core::DescriptorSet;
using core::DescriptorSetLayout;

}  // namespace volumetric_kit::recon
