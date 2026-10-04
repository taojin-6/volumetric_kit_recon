// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file compute_pipeline.hpp
/// @brief recon's names for the core's compute pipeline.
///
/// recon defines no Vulkan foundation of its own: it uses volumetric_kit_core's
/// vulkan tier (DECISIONS.md, 2026-10-04, "The Vulkan foundation comes from
/// volumetric_kit_core"), so a recon object is the same type an embedder holds
/// -- and gfx, once it adopts the core -- and one `VkDevice` serves them all.
/// The using-declarations below let recon and its consumers keep writing these
/// names in recon's namespace. The contract is the core header's,
/// `volumetric_kit/core/vulkan/compute_pipeline.hpp`.

#include "volumetric_kit/core/vulkan/compute_pipeline.hpp"
#include "volumetric_kit/recon/core/result.hpp"

namespace volumetric_kit::recon {

using core::ComputePipeline;
using core::ComputePipelineDesc;

}  // namespace volumetric_kit::recon
