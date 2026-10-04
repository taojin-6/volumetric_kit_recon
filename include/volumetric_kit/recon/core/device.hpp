// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file device.hpp
/// @brief recon's names for the core's logical device, and what recon's
///        kernels require of one.
///
/// Re-exported from volumetric_kit_core (DECISIONS.md, 2026-10-04); the
/// contract is the core header's, `volumetric_kit/core/vulkan/device.hpp`.
///
/// What recon adds is @ref device_requirements: the one statement of what
/// recon's kernels need, which a standalone caller passes to
/// `Instance::select_physical_device` and `Device::create`, and an embedder
/// merges with its own (`merge`, or a `SharedDeviceConfig`) before building a
/// shared device.
///
/// @code
/// const DeviceRequirements reqs = device_requirements();
/// VR_ASSIGN(PhysicalDeviceInfo gpu, instance.select_physical_device(reqs));
/// VR_ASSIGN(Device device, Device::create(instance, gpu, reqs));
/// @endcode

#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/core/vulkan/device_requirements.hpp"
#include "volumetric_kit/core/vulkan/physical_device_info.hpp"
#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/core/vulkan.hpp"

namespace volumetric_kit::recon {

using core::AdoptedDevice;
using core::check_device_support;
using core::debug_object_handle;
using core::Device;
using core::DeviceRequirements;
using core::DeviceSupport;
using core::merge;
using core::PhysicalDeviceInfo;

/// @brief What recon's kernels require of a device.
///
/// A compute queue with timeline semaphores (the core's defaults) and
/// `scalarBlockLayout`, the buffer ABI every recon shader declares
/// (`layout(scalar)`; DECISIONS.md, 2026-07-05). Two extensions are optional
/// and enabled where offered: `VK_KHR_external_memory_fd`, through which a
/// hardware decoder's picture lands in recon's buffers without a host trip
/// (external_memory.hpp), and `VK_EXT_metal_objects`, through which a
/// VideoToolbox picture does on Apple. They are named here, not left to the
/// device's creator, so a merged bootstrap that never hears of them does not
/// silently send every decoded picture through the host.
///
/// @warning Pass these to every `Device::create` and `Device::adopt` recon
///          runs on. The core's default requirements leave
///          `scalarBlockLayout` off, and nothing in recon can tell: its
///          kernels are created and run without an error on such a device,
///          with their buffers read at the wrong offsets.
///
/// @code
/// VR_ASSIGN(Device device,
///           Device::create(instance, gpu, device_requirements()));
/// @endcode
/// @return The requirements; a caller may add to them before creating.
// TODO(core): refuse a device without scalarBlockLayout when a kernel is
// created, once the core's Device records the features it enabled.
inline DeviceRequirements device_requirements() {
  DeviceRequirements reqs;
  reqs.scalar_block_layout = true;
  reqs.optional_extensions = {VK_KHR_EXTERNAL_MEMORY_FD_EXTENSION_NAME,
                              "VK_EXT_metal_objects"};
  return reqs;
}

}  // namespace volumetric_kit::recon
