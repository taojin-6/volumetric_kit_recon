// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// Vulkan availability smoke: proves recon's build finds Vulkan, the core's
// core/vulkan/vulkan.hpp umbrella compiles, and the platform's Vulkan driver
// (MoltenVK on Apple) offers a device that meets recon's requirements
// (core/device_requirements.hpp) and creates one on it. The Vulkan
// foundation behind it is volumetric_kit_core's, tested there; what this pins
// is recon's side: that the requirements recon states are ones a real driver
// meets. gpu_test.hpp fails it where a device exists but falls short of them.

#include <cstdio>

#include "volumetric_kit/core/vulkan/vulkan.hpp"

#include "gpu_test.hpp"

namespace {

int gpu_main(vr_test::GpuContext& gpu) {
  std::printf("recon runs on %s\n", gpu.physical.properties().deviceName);
  std::puts("recon Vulkan compute smoke passed");
  return 0;
}

}  // namespace

int main() { return vr_test::run_on_gpu(gpu_main); }
