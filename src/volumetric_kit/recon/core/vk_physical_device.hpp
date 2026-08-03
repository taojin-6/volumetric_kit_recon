// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file vk_physical_device.hpp
/// @brief Internal (non-installed) physical-device query helpers shared by the
///        instance and device translation units.
///
/// The "enumerate queue families and scan for a compute family" and "read the
/// device's API version" idioms are needed in both `instance.cpp` (device
/// selection) and `device.cpp` (create/adopt), so they live here once rather
/// than copy-pasted per call site. This header stays under `src/` and is
/// included only by core `.cpp` files -- it is deliberately not part of the
/// installed public API.

#include <cstdint>
#include <optional>
#include <vector>

#include "volumetric_kit/recon/core/vulkan.hpp"

namespace volumetric_kit::recon::detail {

/// @return The queue-family properties of @p physical (one driver enumeration).
inline std::vector<VkQueueFamilyProperties> queue_families(
    VkPhysicalDevice physical) {
  std::uint32_t count = 0;
  vkGetPhysicalDeviceQueueFamilyProperties(physical, &count, nullptr);
  std::vector<VkQueueFamilyProperties> families(count);
  vkGetPhysicalDeviceQueueFamilyProperties(physical, &count, families.data());
  return families;
}

/// @brief The first compute-capable queue family on @p physical, if any.
/// @return The family index, or `std::nullopt` when none is compute-capable. A
///         compute-capable family implicitly supports transfer, so one queue
///         covers both of recon's needs.
inline std::optional<std::uint32_t> find_compute_family(
    VkPhysicalDevice physical) {
  const std::vector<VkQueueFamilyProperties> families =
      queue_families(physical);
  for (std::uint32_t i = 0; i < families.size(); ++i) {
    if ((families[i].queueFlags & VK_QUEUE_COMPUTE_BIT) != 0) {
      return i;
    }
  }
  return std::nullopt;
}

/// @return Whether @p family is a valid index on @p physical and its queue
///         family is compute-capable.
inline bool queue_family_has_compute(VkPhysicalDevice physical,
                                     std::uint32_t family) {
  const std::vector<VkQueueFamilyProperties> families =
      queue_families(physical);
  return family < families.size() &&
         (families[family].queueFlags & VK_QUEUE_COMPUTE_BIT) != 0;
}

/// @brief The capabilities @p family advertises on @p physical.
///
/// Read once at device create/adopt and cached, because a pipeline barrier may
/// only name stages the recording command buffer's queue family supports --
/// `VK_PIPELINE_STAGE_VERTEX_INPUT_BIT` requires `VK_QUEUE_GRAPHICS_BIT`, which
/// a compute-only family does not have.
/// @return The family's `queueFlags`, or `0` when @p family is out of range.
inline VkQueueFlags queue_family_flags(VkPhysicalDevice physical,
                                       std::uint32_t family) {
  const std::vector<VkQueueFamilyProperties> families =
      queue_families(physical);
  return family < families.size() ? families[family].queueFlags : 0;
}

/// @return The packed Vulkan API version @p physical reports. Compared with
///         `<` against a `VK_API_VERSION_*` constant, which is valid for the
///         standard (variant 0) packing.
inline std::uint32_t physical_api_version(VkPhysicalDevice physical) {
  VkPhysicalDeviceProperties props{};
  vkGetPhysicalDeviceProperties(physical, &props);
  return props.apiVersion;
}

}  // namespace volumetric_kit::recon::detail
