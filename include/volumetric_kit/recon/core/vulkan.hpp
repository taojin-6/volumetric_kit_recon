// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file vulkan.hpp
/// @brief The single umbrella header through which first-party code includes
///        Vulkan.
///
/// The single point where first-party code pulls in Vulkan. Always include this
/// header -- never `<vulkan/vulkan.h>` or a loader header directly -- so the
/// loader / dispatch choice (currently the link-time loader `Vulkan::Vulkan`)
/// stays a detail of this one file: adopting volk for the iOS/Android loader
/// would be a change here plus the link line, with no churn at call sites. This
/// mirrors volumetric_kit_gfx exactly, which keeps a future shared core cheap.

#include <vulkan/vulkan.h>
