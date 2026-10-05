// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// Vulkan availability smoke: proves recon's build finds Vulkan, the
// core/vulkan.hpp umbrella compiles, and the platform's Vulkan driver (MoltenVK
// on Apple) offers a device that meets recon's requirements
// (core/device.hpp's device_requirements) and creates one on it. The Vulkan
// foundation behind it is volumetric_kit_core's, tested there; what this pins
// is recon's side: that the requirements recon states are ones a real driver
// meets.

#include <cstdint>
#include <cstdio>

#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/core/vulkan/instance.hpp"
#include "volumetric_kit/core/vulkan/vulkan.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"

namespace vr = volumetric_kit::recon;
namespace vkc = volumetric_kit::core;

int main() {
  vkc::Result<vkc::Instance> instance = vkc::Instance::create({});
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

  // recon's requirements, not the core's defaults: scalarBlockLayout is what
  // every recon kernel's buffer ABI needs, so a device the core accepts can
  // still be one recon cannot run on.
  const vkc::DeviceRequirements reqs = vr::device_requirements();
  vkc::Result<vkc::PhysicalDeviceInfo> gpu =
      instance.value().select_physical_device(reqs);
  if (!gpu) {
    std::fprintf(stderr,
                 "%u device(s) but none meets recon's requirements (%s)\n",
                 device_count, gpu.status().message().c_str());
    return 1;
  }
  vkc::Result<vkc::Device> device =
      vkc::Device::create(instance.value(), gpu.value(), reqs);
  if (!device) {
    std::fprintf(stderr, "device create failed on %s: %s\n",
                 gpu.value().properties().deviceName,
                 device.status().message().c_str());
    return 1;
  }

  std::printf("Vulkan instance created; %u device(s); recon runs on %s\n",
              device_count, gpu.value().properties().deviceName);
  std::puts("recon Vulkan compute smoke passed");
  return 0;
}
