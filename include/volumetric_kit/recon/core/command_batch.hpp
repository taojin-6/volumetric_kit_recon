// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file command_batch.hpp
/// @brief recon's name for the core's one-submit command batch.
///
/// Re-exported from volumetric_kit_core (DECISIONS.md, 2026-10-04); the
/// contract is the core header's,
/// `volumetric_kit/core/vulkan/command_batch.hpp`.

#include "volumetric_kit/core/vulkan/command_batch.hpp"
#include "volumetric_kit/recon/core/result.hpp"

namespace volumetric_kit::recon {

using core::CommandBatch;

}  // namespace volumetric_kit::recon
