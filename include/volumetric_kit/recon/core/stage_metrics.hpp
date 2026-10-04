// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file stage_metrics.hpp
/// @brief recon's names for the core's per-stage timing table.
///
/// recon defines no timing vocabulary of its own: it uses volumetric_kit_core's
/// base tier (DECISIONS.md, 2026-10-04, "The Vulkan foundation comes from
/// volumetric_kit_core"), the table the core's `GpuTimer` reports into. The
/// using-declarations below let recon and its consumers keep writing these
/// names in recon's namespace. The contract is the core header's,
/// `volumetric_kit/core/base/stage_metrics.hpp`.
///
/// The table is Vulkan-free and header-only (the core's base tier), so a
/// pure-host consumer reports timings without a driver header.

#include "volumetric_kit/core/base/stage_metrics.hpp"
#include "volumetric_kit/recon/core/result.hpp"

namespace volumetric_kit::recon {

using core::StageMetrics;
using core::StageRow;
using core::StageScope;

}  // namespace volumetric_kit::recon
