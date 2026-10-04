// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file buffer.hpp
/// @brief recon's names for the core's RAII buffer.
///
/// Re-exported from volumetric_kit_core (DECISIONS.md, 2026-10-04); the
/// contract is the core header's, `volumetric_kit/core/vulkan/buffer.hpp`.

#include "volumetric_kit/core/vulkan/buffer.hpp"
#include "volumetric_kit/recon/core/result.hpp"

namespace volumetric_kit::recon {

using core::Buffer;
using core::MemoryInfo;

}  // namespace volumetric_kit::recon
