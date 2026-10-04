// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file external_memory.hpp
/// @brief recon's names for the core's exportable buffers, which hand
///        recon's memory to CUDA in place.
///
/// Re-exported from volumetric_kit_core (DECISIONS.md, 2026-10-04); the
/// contract is the core header's,
/// `volumetric_kit/core/vulkan/external_memory.hpp`.

#include "volumetric_kit/core/vulkan/external_memory.hpp"
#include "volumetric_kit/recon/core/result.hpp"

namespace volumetric_kit::recon {

using core::create_exported_buffer;
using core::ExportedBuffer;
using core::find_memory_type;
using core::UniqueFd;

}  // namespace volumetric_kit::recon
