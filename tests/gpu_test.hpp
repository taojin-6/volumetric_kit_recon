// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// What a GPU test runs on, under the core's test policy
// (volumetric_kit/core/testing/vulkan_policy.hpp): VKC_REQUIRE_VULKAN_DEVICE,
// VKC_TEST_VALIDATION and VKC_TEST_SYNC_VALIDATION mean here what they mean in
// the core's tests, every error the validation layer reports fails the test,
// and a test skips by returning the core's test::kSkipExitCode, every test's
// SKIP_RETURN_CODE (tests/CMakeLists.txt).
//
//   int gpu_main(vr_test::GpuContext& gpu) {
//     ... // make, test and destroy everything on gpu.device
//     return 0;
//   }
//   int main() { return vr_test::run_on_gpu(gpu_main); }

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "buffer_readback.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/testing/vulkan_policy.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/core/vulkan/instance.hpp"
#include "volumetric_kit/core/vulkan/physical_device_info.hpp"
#include "volumetric_kit/core/vulkan/vulkan.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"

namespace vr_test {

namespace vr = volumetric_kit::recon;
namespace vkc = volumetric_kit::core;

// A Gpu, with the instance and the physical device its device was made on.
struct GpuContext : Gpu {
  vkc::Instance& instance;
  const vkc::PhysicalDeviceInfo& physical;
};

namespace detail {

// Makes the context and runs `test` on it. Everything made here is destroyed
// before this returns, while the caller's log capture still counts what the
// validation layer reports, a leaked object included.
template <typename Test>
int run_on_gpu(Test& test) {
  vkc::Result<vkc::Instance> instance =
      vkc::Instance::create(vkc::test::instance_config());
  if (!instance) {
    return vkc::test::no_device_exit_code("no Vulkan instance (" +
                                          instance.status().message() + ")");
  }
  if (const vkc::Status loaded = vkc::test::check_layer_loaded(*instance);
      !loaded.ok()) {
    std::fprintf(stderr, "%s\n", loaded.message().c_str());
    return 1;
  }
  std::uint32_t count = 0;
  vkEnumeratePhysicalDevices(instance->handle(), &count, nullptr);
  if (count == 0) return vkc::test::no_device_exit_code("no physical devices");
  // A device that falls short of recon's requirements fails rather than
  // skips: they are the requirements recon states a real driver meets.
  vkc::Result<vkc::PhysicalDeviceInfo> physical =
      instance->select_physical_device(vr::device_requirements());
  if (!physical) {
    std::fprintf(stderr,
                 "%u device(s), but none meets recon's requirements: %s\n",
                 count, physical.status().message().c_str());
    return 1;
  }
  vkc::Result<vkc::Device> device =
      vkc::Device::create(*instance, *physical, vr::device_requirements());
  if (!device) {
    std::fprintf(stderr, "device create failed on %s: %s\n",
                 physical->properties().deviceName,
                 device.status().message().c_str());
    return 1;
  }
  vkc::Result<vkc::Allocator> allocator =
      vkc::Allocator::create(instance->handle(), *device);
  if (!allocator) {
    std::fprintf(stderr, "allocator create failed: %s\n",
                 allocator.status().message().c_str());
    return 1;
  }
  GpuContext gpu{{*device, *allocator}, *instance, *physical};
  return test(gpu);
}

}  // namespace detail

// Runs `test`, an `int(GpuContext&)` returning the test's exit code, on a
// device that meets recon's requirements, under the validation the
// environment asks for. Returns the process's exit code: the test's, or 1
// where the layer reported an error; test::kSkipExitCode, or 1 under
// VKC_REQUIRE_VULKAN_DEVICE, where there is no device.
template <typename Test>
int run_on_gpu(Test&& test) {
  const vkc::test::ValidationSession validation;
  const vkc::test::LogCapture log;
  return log.exit_code(detail::run_on_gpu(test));
}

// What a decoder test returns when the decoders have no device path here (no
// GPU this build's hardware path reaches): it skips, or fails where CI sets
// VR_TEST_HEVC_BACKEND, as on every leg that promises one.
inline int no_decoder(const std::string& why) {
  const char* promise = std::getenv("VR_TEST_HEVC_BACKEND");
  const bool required = promise != nullptr && *promise != '\0';
  std::fprintf(stderr, "no device path for the decoders (%s); %s\n",
               why.c_str(), required ? "failing" : "skipping");
  return required ? 1 : vkc::test::kSkipExitCode;
}

// As no_decoder for JpegDecoder, except that a GPU with no hardware JPEG
// engine always skips: NVIDIA's are few (the RTX 5090 has one, the RTX 4090
// on one CI host none), and nothing stands in for it.
inline int no_jpeg_decoder(const std::string& why) {
  if (why.find("no hardware JPEG engine") != std::string::npos) {
    std::fprintf(stderr, "%s; skipping\n", why.c_str());
    return vkc::test::kSkipExitCode;
  }
  return no_decoder(why);
}

// `gpu`'s device again, adopted with none of its optional extensions declared,
// so a decoder handed it can keep no picture on the GPU: the device path fails
// to open on any machine, which is how a test reaches a decoder's refusal of
// it. Borrowed: `gpu`'s device must outlive the result.
inline vkc::Result<vkc::Device> bare_device(const GpuContext& gpu) {
  vkc::AdoptedDevice view;
  view.instance = gpu.instance.handle();
  view.instance_api_version = gpu.instance.api_version();
  view.physical_device = gpu.device.physical_device();
  view.device = gpu.device.handle();
  view.queue_family = gpu.device.queue_family();
  view.queue = gpu.device.queue();
  view.submit_mutex = gpu.device.submit_mutex();
  view.enabled_features.timeline_semaphore = true;
  view.enabled_features.scalar_block_layout = true;
  return vkc::Device::adopt(view, vr::device_requirements());
}

}  // namespace vr_test
