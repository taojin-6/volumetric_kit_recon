// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// Vulkan availability smoke: proves recon's build finds Vulkan, the
// core/vulkan.hpp umbrella compiles, and the platform's Vulkan driver (MoltenVK
// on Apple) can create an instance and expose a COMPUTE-capable queue family.
// Compute -- not graphics -- is the capability this backend actually needs, so
// the check targets it directly. This is the "validate the GPU path early"
// de-risk before the Vulkan core (device/allocator/compute pipeline) is built.

#include <cstdint>
#include <cstdio>
#include <vector>

#include "volumetric_kit/recon/core/vulkan.hpp"

namespace {

// Create an instance. On Apple, MoltenVK is a portability driver, so we first
// try with the portability-enumeration extension/flag (which makes MoltenVK
// devices visible); if the loader lacks it, fall back to a plain instance so
// the smoke still exercises loader linkage.
VkResult create_instance(bool with_portability, VkInstance* out) {
  VkApplicationInfo app{};
  app.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
  app.pApplicationName = "recon_vulkan_smoke";
  app.apiVersion = VK_API_VERSION_1_1;

  VkInstanceCreateInfo ci{};
  ci.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
  ci.pApplicationInfo = &app;

  // Portability enumeration (what makes MoltenVK visible on Apple) only exists
  // in Vulkan headers >= 1.3.216. Older SDKs -- e.g. Ubuntu 22.04's 1.3.204 --
  // lack the symbols, and there the flag is unnecessary anyway: a Linux ICD
  // (lavapipe or a real driver) is not a portability driver. Guard on the
  // extension macro the header defines so the smoke compiles on both.
#ifdef VK_KHR_portability_enumeration
  const char* exts[] = {VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME};
  if (with_portability) {
    ci.flags |= VK_INSTANCE_CREATE_ENUMERATE_PORTABILITY_BIT_KHR;
    ci.enabledExtensionCount = 1;
    ci.ppEnabledExtensionNames = exts;
  }
#else
  (void)with_portability;
#endif
  return vkCreateInstance(&ci, nullptr, out);
}

// Whether any queue family on `physical` reports compute support.
bool has_compute_family(VkPhysicalDevice physical) {
  std::uint32_t count = 0;
  vkGetPhysicalDeviceQueueFamilyProperties(physical, &count, nullptr);
  std::vector<VkQueueFamilyProperties> families(count);
  vkGetPhysicalDeviceQueueFamilyProperties(physical, &count, families.data());
  for (const VkQueueFamilyProperties& f : families) {
    if ((f.queueFlags & VK_QUEUE_COMPUTE_BIT) != 0) {
      return true;
    }
  }
  return false;
}

}  // namespace

int main() {
  VkInstance instance = VK_NULL_HANDLE;
  VkResult r = create_instance(/*with_portability=*/true, &instance);
  if (r == VK_ERROR_EXTENSION_NOT_PRESENT ||
      r == VK_ERROR_INCOMPATIBLE_DRIVER) {
    r = create_instance(/*with_portability=*/false, &instance);
  }
  if (r != VK_SUCCESS || instance == VK_NULL_HANDLE) {
    // No Vulkan driver on this machine (e.g. a headless CI runner without an
    // ICD). Treat as a skip, not a failure: the smoke gates on driver
    // availability, which is environmental, not a code defect.
    std::fprintf(stderr, "no Vulkan instance (VkResult %d); skipping\n",
                 static_cast<int>(r));
    return 0;
  }

  std::uint32_t device_count = 0;
  vkEnumeratePhysicalDevices(instance, &device_count, nullptr);
  if (device_count == 0) {
    std::fprintf(stderr, "no physical devices; skipping\n");
    vkDestroyInstance(instance, nullptr);
    return 0;
  }
  std::vector<VkPhysicalDevice> devices(device_count);
  vkEnumeratePhysicalDevices(instance, &device_count, devices.data());

  bool any_compute = false;
  for (VkPhysicalDevice d : devices) {
    if (has_compute_family(d)) {
      any_compute = true;
      break;
    }
  }

  std::printf("Vulkan instance created; %u device(s); compute-capable: %s\n",
              device_count, any_compute ? "yes" : "no");
  vkDestroyInstance(instance, nullptr);

  if (!any_compute) {
    std::fprintf(stderr, "no compute-capable queue family found\n");
    return 1;
  }
  std::puts("recon Vulkan compute smoke passed");
  return 0;
}
