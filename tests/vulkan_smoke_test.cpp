// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// Vulkan availability smoke: proves recon's build finds Vulkan, the
// core/vulkan.hpp umbrella compiles, and the platform's Vulkan driver (MoltenVK
// on Apple) can create an instance and expose a COMPUTE-capable queue family.
// Compute -- not graphics -- is the capability this backend actually needs, so
// the check targets it directly. This is the "validate the GPU path early"
// de-risk before the Vulkan core (device/allocator/compute pipeline) is built.
//
// It drives the same Instance::create + select_physical_device path the rest of
// the core uses, rather than re-implementing the portability-enumeration and
// compute-family dance, so the loader detail stays in one place (instance.cpp).

#include <cstdint>
#include <cstdio>

#include "volumetric_kit/recon/core/instance.hpp"
#include "volumetric_kit/recon/core/vulkan.hpp"

namespace vr = volumetric_kit::recon;

int main() {
  vr::Result<vr::Instance> instance = vr::Instance::create({});
  if (!instance) {
    // No Vulkan driver on this machine (e.g. a headless CI runner without an
    // ICD). Treat as a skip, not a failure: the smoke gates on driver
    // availability, which is environmental, not a code defect.
    std::fprintf(stderr, "no Vulkan instance (%s); skipping\n",
                 instance.status().message().c_str());
    return 0;
  }

  // Distinguish "no devices at all" (environmental -> skip) from "devices exist
  // but none is compute-capable" (a real failure of the path we depend on).
  std::uint32_t device_count = 0;
  vkEnumeratePhysicalDevices(instance.value().handle(), &device_count, nullptr);
  if (device_count == 0) {
    std::fprintf(stderr, "no physical devices; skipping\n");
    return 0;
  }

  vr::Result<VkPhysicalDevice> gpu = instance.value().select_physical_device();
  if (!gpu) {
    std::fprintf(stderr, "%u device(s) but none compute-capable (%s)\n",
                 device_count, gpu.status().message().c_str());
    return 1;
  }

  std::printf("Vulkan instance created; %u device(s); compute-capable: yes\n",
              device_count);
  std::puts("recon Vulkan compute smoke passed");
  return 0;
}
