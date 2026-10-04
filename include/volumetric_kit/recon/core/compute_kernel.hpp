// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file compute_kernel.hpp
/// @brief recon's names for the core's compute kernel and its descriptor
///        sets.
///
/// Re-exported from volumetric_kit_core (DECISIONS.md, 2026-10-04); the
/// contract is the core header's,
/// `volumetric_kit/core/vulkan/compute_kernel.hpp`.

#include "volumetric_kit/core/vulkan/compute_kernel.hpp"
#include "volumetric_kit/recon/core/result.hpp"

namespace volumetric_kit::recon {

using core::ComputeKernel;
using core::dispatch;
using core::KernelSetBuilder;
using core::KernelSets;

}  // namespace volumetric_kit::recon
