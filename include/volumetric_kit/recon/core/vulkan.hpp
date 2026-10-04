// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file vulkan.hpp
/// @brief The single umbrella header through which first-party code includes
///        Vulkan.
///
/// Always include this header -- never `<vulkan/vulkan.h>` or a loader header
/// directly. It forwards to volumetric_kit_core's umbrella, so the loader and
/// dispatch choice (the link-time loader `Vulkan::Vulkan` today) is one
/// decision for the whole family: adopting volk for the iOS/Android loader
/// would be a change in the core with no churn at recon's call sites.

#include "volumetric_kit/core/vulkan/vulkan.hpp"  // IWYU pragma: export
