// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file gpu_timer.hpp
/// @brief recon's names for the core's GPU timestamp spans.
///
/// Re-exported from volumetric_kit_core (DECISIONS.md, 2026-10-04); the
/// contract is the core header's, `volumetric_kit/core/vulkan/gpu_timer.hpp`.

#include "volumetric_kit/core/vulkan/gpu_timer.hpp"
#include "volumetric_kit/recon/core/result.hpp"

namespace volumetric_kit::recon {

using core::GpuSpanTag;
using core::GpuStageScope;
using core::GpuTimer;
using core::ticks_to_ms;
using core::timestamp_delta;

}  // namespace volumetric_kit::recon
