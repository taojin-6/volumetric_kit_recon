// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/viewer/shared_device.hpp
/// @brief The neutral bootstrap: one `VkInstance` + one `VkDevice` built from
///        the union of recon's and gfx's published requirements, for both to
///        adopt.
///
/// This is the embedder half of the create/adopt seam both libraries expose.
/// Neither owns the device -- an app that wants them to share one builds it
/// from `Device::requirements` on each side, then hands the same handles to
/// `recon::Device::adopt` and `gfx::app::WindowedApp::adopt`. That is what
/// makes zero-copy possible at all: a `VkBuffer` is valid only on the device
/// that created it, so recon geometry can be drawn by gfx *only* if there is
/// one device.
///
/// Deliberately a plain function over raw Vulkan rather than a wrapper type:
/// it is the one place in this tree that is neither library's, and both
/// libraries' RAII owners start *after* it.

#include <cstdio>
#include <cstring>
#include <mutex>
#include <vector>

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#include "volumetric_kit/gfx/app/windowed_app.hpp"
#include "volumetric_kit/gfx/core/device.hpp"
#include "volumetric_kit/recon/core/device.hpp"

namespace fuse_viewer {

namespace vr = volumetric_kit::recon;
namespace vg = volumetric_kit::gfx;

/// @brief One Vulkan device both libraries adopt, plus everything each needs
///        to adopt it.
///
/// Owns the raw instance/device (destroyed in @ref ~SharedDevice, after both
/// adopters have released their wrappers) and the surface the swapchain
/// presents to.
struct SharedDevice {
  VkInstance instance = VK_NULL_HANDLE;
  VkPhysicalDevice physical_device = VK_NULL_HANDLE;
  VkDevice device = VK_NULL_HANDLE;
  /// Created here so device selection can test present support, then handed
  /// to the renderer, which owns and destroys it -- see @ref release_surface.
  VkSurfaceKHR surface = VK_NULL_HANDLE;

  /// The one graphics+compute+present family both libraries draw queues from.
  /// Using a single family avoids any queue-family ownership transfer on a
  /// resource that recon writes and gfx reads.
  std::uint32_t family = 0;
  VkQueue graphics_queue = VK_NULL_HANDLE;  ///< gfx's (also presents).
  VkQueue compute_queue = VK_NULL_HANDLE;   ///< recon's.

  /// Non-null only when the family exposed a single queue and both libraries
  /// share it -- Vulkan requires queue submits be externally synchronized, and
  /// both `adopt` paths route through this when set.
  std::mutex* submit_mutex = nullptr;

  /// Exactly what was enabled on @ref device, for each `adopt` to verify.
  std::vector<const char*> enabled_extensions;
  VkPhysicalDeviceFeatures enabled_features{};

  ~SharedDevice() {
    // Both libraries' wrappers must already be gone: they borrow these.
    if (device != VK_NULL_HANDLE) vkDestroyDevice(device, nullptr);
    if (surface != VK_NULL_HANDLE && instance != VK_NULL_HANDLE) {
      vkDestroySurfaceKHR(instance, surface, nullptr);
    }
    if (instance != VK_NULL_HANDLE) vkDestroyInstance(instance, nullptr);
  }

  /// @brief Give up ownership of @ref surface to the caller.
  ///
  /// gfx's surface factory contract is that the app adopts and later destroys
  /// what the factory returns, but this bootstrap had to create the surface
  /// first -- picking a physical device requires testing present support
  /// against a real surface. So it is created here and handed over, and this
  /// stops tracking it; destroying it twice would be a use-after-free at
  /// teardown.
  VkSurfaceKHR release_surface() {
    VkSurfaceKHR released = surface;
    surface = VK_NULL_HANDLE;
    return released;
  }

  SharedDevice() = default;
  SharedDevice(const SharedDevice&) = delete;
  SharedDevice& operator=(const SharedDevice&) = delete;
  SharedDevice(SharedDevice&&) = delete;
  SharedDevice& operator=(SharedDevice&&) = delete;

 private:
  std::mutex owned_submit_mutex_;
  friend bool build_shared_device(GLFWwindow* window, SharedDevice& out);
};

/// @brief Build one instance + device satisfying both libraries, and the
///        window's surface on it.
///
/// @param window  The GLFW window to present to; its required instance
///                extensions are enabled and its surface created here.
/// @param out     Filled on success; destroys what it holds on scope exit.
/// @return `false` (with a message on stderr) when no device can satisfy the
///         union -- the caller should fall back to separate devices rather
///         than treat it as fatal.
inline bool build_shared_device(GLFWwindow* window, SharedDevice& out) {
  // --- 1. Merge what each library publishes. --------------------------------
  // Neither is consulted about the other: each states its needs, the embedder
  // satisfies the union. gfx needs present; recon needs compute.
  vg::DeviceConfig gfx_config;
  gfx_config.needs_present = true;
  const vr::DeviceRequirements recon_req = vr::Device::requirements({});
  const vg::DeviceRequirements gfx_req = vg::Device::requirements(gfx_config);

  // The higher floor wins. This must NOT inherit recon's 1.2: MoltenVK caps a
  // physical device's advertised apiVersion to whatever the instance asked
  // for, so an instance created at recon's floor would make a 1.3-capable
  // device look 1.2 and fail gfx's check.
  const std::uint32_t api_version = recon_req.api_version > gfx_req.api_version
                                        ? recon_req.api_version
                                        : gfx_req.api_version;

  std::vector<const char*> extensions;
  auto add_extension = [&extensions](const char* name) {
    for (const char* have : extensions) {
      if (std::strcmp(have, name) == 0) return;
    }
    extensions.push_back(name);
  };
  for (const char* name : recon_req.device_extensions) add_extension(name);
  for (const char* name : gfx_req.device_extensions) add_extension(name);

  // --- 2. Instance: the window's extensions, at the merged floor. -----------
  std::uint32_t glfw_extension_count = 0;
  const char** glfw_extensions =
      glfwGetRequiredInstanceExtensions(&glfw_extension_count);
  if (glfw_extensions == nullptr) {
    std::fprintf(stderr, "shared device: GLFW reports no Vulkan support\n");
    return false;
  }
  std::vector<const char*> instance_extensions(
      glfw_extensions, glfw_extensions + glfw_extension_count);
#ifdef VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME
  // MoltenVK is a portability driver; without this the loader enumerates no
  // physical device at all.
  instance_extensions.push_back(VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME);
#endif

  VkApplicationInfo app_info{};
  app_info.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
  app_info.pApplicationName = "fuse_viewer";
  app_info.apiVersion = api_version;

  VkInstanceCreateInfo instance_info{};
  instance_info.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
  instance_info.pApplicationInfo = &app_info;
  instance_info.enabledExtensionCount =
      static_cast<std::uint32_t>(instance_extensions.size());
  instance_info.ppEnabledExtensionNames = instance_extensions.data();
#ifdef VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME
  instance_info.flags |= VK_INSTANCE_CREATE_ENUMERATE_PORTABILITY_BIT_KHR;
#endif
  if (vkCreateInstance(&instance_info, nullptr, &out.instance) != VK_SUCCESS) {
    std::fprintf(stderr, "shared device: vkCreateInstance failed\n");
    return false;
  }

  if (glfwCreateWindowSurface(out.instance, window, nullptr, &out.surface) !=
      VK_SUCCESS) {
    std::fprintf(stderr, "shared device: glfwCreateWindowSurface failed\n");
    return false;
  }

  // --- 3. One family that can do all three. --------------------------------
  std::uint32_t device_count = 0;
  vkEnumeratePhysicalDevices(out.instance, &device_count, nullptr);
  std::vector<VkPhysicalDevice> devices(device_count);
  vkEnumeratePhysicalDevices(out.instance, &device_count, devices.data());
  const VkQueueFlags needed_flags =
      recon_req.queue_flags | gfx_req.queue_flags | VK_QUEUE_GRAPHICS_BIT;

  bool found = false;
  for (VkPhysicalDevice candidate : devices) {
    std::uint32_t family_count = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(candidate, &family_count, nullptr);
    std::vector<VkQueueFamilyProperties> families(family_count);
    vkGetPhysicalDeviceQueueFamilyProperties(candidate, &family_count,
                                             families.data());
    for (std::uint32_t i = 0; i < family_count; ++i) {
      if ((families[i].queueFlags & needed_flags) != needed_flags) continue;
      VkBool32 present = VK_FALSE;
      vkGetPhysicalDeviceSurfaceSupportKHR(candidate, i, out.surface, &present);
      if (present != VK_TRUE) continue;
      out.physical_device = candidate;
      out.family = i;
      // Two queues from the one family when it has them -- gfx and recon then
      // submit independently; otherwise both share one under a mutex.
      out.submit_mutex =
          families[i].queueCount >= 2 ? nullptr : &out.owned_submit_mutex_;
      found = true;
      break;
    }
    if (found) break;
  }
  if (!found) {
    std::fprintf(stderr,
                 "shared device: no graphics+compute+present family found\n");
    return false;
  }

  // --- 4. The device, satisfying the union. --------------------------------
  // Portability drivers require this extension whenever they expose it.
  std::uint32_t available_count = 0;
  vkEnumerateDeviceExtensionProperties(out.physical_device, nullptr,
                                       &available_count, nullptr);
  std::vector<VkExtensionProperties> available(available_count);
  vkEnumerateDeviceExtensionProperties(out.physical_device, nullptr,
                                       &available_count, available.data());
  for (const VkExtensionProperties& extension : available) {
    if (std::strcmp(extension.extensionName, "VK_KHR_portability_subset") ==
        0) {
      add_extension("VK_KHR_portability_subset");
      break;
    }
  }
  out.enabled_extensions = extensions;

  const float priorities[2] = {1.0f, 1.0f};
  VkDeviceQueueCreateInfo queue_info{};
  queue_info.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
  queue_info.queueFamilyIndex = out.family;
  queue_info.queueCount = out.submit_mutex == nullptr ? 2u : 1u;
  queue_info.pQueuePriorities = priorities;

  // The union of both libraries' feature requests. Chained rather than merged
  // by hand so each library's published requirement is visible at the call.
  VkPhysicalDeviceDynamicRenderingFeatures dynamic_rendering{};
  dynamic_rendering.sType =
      VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_DYNAMIC_RENDERING_FEATURES;
  dynamic_rendering.dynamicRendering =
      gfx_req.dynamic_rendering ? VK_TRUE : VK_FALSE;
  VkPhysicalDeviceTimelineSemaphoreFeatures timeline{};
  timeline.sType =
      VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_TIMELINE_SEMAPHORE_FEATURES;
  timeline.timelineSemaphore =
      (recon_req.timeline_semaphore || gfx_req.timeline_semaphore) ? VK_TRUE
                                                                   : VK_FALSE;
  timeline.pNext = &dynamic_rendering;
  // recon reads its buffers through scalar block layout (its shader ABI), so
  // the shared device must enable it even though gfx does not ask.
  VkPhysicalDeviceScalarBlockLayoutFeatures scalar_layout{};
  scalar_layout.sType =
      VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_SCALAR_BLOCK_LAYOUT_FEATURES;
  scalar_layout.scalarBlockLayout =
      recon_req.scalar_block_layout ? VK_TRUE : VK_FALSE;
  scalar_layout.pNext = &timeline;

  VkPhysicalDeviceFeatures2 features{};
  features.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
  features.pNext = &scalar_layout;

  VkDeviceCreateInfo device_info{};
  device_info.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
  device_info.pNext = &features;
  device_info.queueCreateInfoCount = 1;
  device_info.pQueueCreateInfos = &queue_info;
  device_info.enabledExtensionCount =
      static_cast<std::uint32_t>(out.enabled_extensions.size());
  device_info.ppEnabledExtensionNames = out.enabled_extensions.data();
  if (vkCreateDevice(out.physical_device, &device_info, nullptr, &out.device) !=
      VK_SUCCESS) {
    std::fprintf(stderr, "shared device: vkCreateDevice failed\n");
    return false;
  }

  vkGetDeviceQueue(out.device, out.family, 0, &out.graphics_queue);
  vkGetDeviceQueue(out.device, out.family,
                   out.submit_mutex == nullptr ? 1u : 0u, &out.compute_queue);

  std::printf("shared device: one VkDevice, family %u, %s\n", out.family,
              out.submit_mutex == nullptr ? "2 queues (gfx + recon)"
                                          : "1 queue shared under a mutex");
  return true;
}

/// @brief The payload `recon::Device::adopt` needs from a @ref SharedDevice.
inline vr::AdoptedDevice recon_adopt_payload(const SharedDevice& shared) {
  vr::AdoptedDevice adopted;
  adopted.instance = shared.instance;
  adopted.physical_device = shared.physical_device;
  adopted.device = shared.device;
  adopted.compute_family = shared.family;
  adopted.compute_queue = shared.compute_queue;
  adopted.submit_mutex = shared.submit_mutex;
  adopted.enabled_device_extensions = shared.enabled_extensions.data();
  adopted.enabled_device_extension_count =
      static_cast<std::uint32_t>(shared.enabled_extensions.size());
  adopted.enabled_features = shared.enabled_features;
  // Both were enabled in the feature chain above; recon verifies its own
  // requirements against these rather than re-querying the device.
  adopted.enabled_timeline_semaphore = true;
  adopted.enabled_scalar_block_layout = true;
  return adopted;
}

/// @brief The payload `gfx::app::WindowedApp::adopt` needs from a
///        @ref SharedDevice.
inline vg::AdoptedDevice gfx_adopt_payload(const SharedDevice& shared) {
  vg::AdoptedDevice adopted;
  adopted.instance = shared.instance;
  adopted.physical_device = shared.physical_device;
  adopted.device = shared.device;
  adopted.graphics_family = shared.family;
  adopted.graphics_queue = shared.graphics_queue;
  // One family that presents, so the present queue is the graphics queue.
  adopted.has_present = true;
  adopted.present_family = shared.family;
  adopted.present_queue = shared.graphics_queue;
  adopted.submit_mutex = shared.submit_mutex;
  adopted.enabled_device_extensions = shared.enabled_extensions.data();
  adopted.enabled_device_extension_count =
      static_cast<std::uint32_t>(shared.enabled_extensions.size());
  return adopted;
}

}  // namespace fuse_viewer
