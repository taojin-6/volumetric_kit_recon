// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file fuse_device.hpp
/// @brief What a fusing example asks of its device: recon's requirements and
///        the sensor package's, merged, since both run on the one device.

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/device_requirements.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"
#include "volumetric_kit/sensor/frame/device_requirements.hpp"

namespace vr_example {

/// @return recon's and the sensor package's device requirements merged with
///         the core's `merge`, or its refusal.
inline volumetric_kit::core::Result<volumetric_kit::core::DeviceRequirements>
fuse_device_requirements() {
  return volumetric_kit::core::merge(
      volumetric_kit::recon::device_requirements(),
      volumetric_kit::sensor::device_requirements());
}

}  // namespace vr_example
