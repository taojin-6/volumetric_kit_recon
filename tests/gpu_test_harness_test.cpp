// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The GPU tests' harness (gpu_test.hpp) under the Khronos validation layer:
// a test the layer reports nothing for passes, one that skips skips, and one
// that returns 0 fails when the layer reports an error, whether in the test's
// own calls or for an object the test leaked, which the layer reports as the
// harness destroys the device; under synchronization validation, a missing
// barrier fails it too. Then the decoders' skips. Skips where the
// layer or a device is missing, unless VKC_REQUIRE_VULKAN_DEVICE is set.
//
// Under LeakSanitizer on Linux, with the sanitizer job's suppressions
// (tools/lsan.supp), a block leaked where the layer reports an error -- in the
// error handler the core's debug messenger callback reaches, under the
// loader's and the layer's frames -- is reported.

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string_view>

#if defined(__has_feature)
#if __has_feature(address_sanitizer)
#define VR_TEST_LSAN 1
#endif
#endif
#if defined(__SANITIZE_ADDRESS__)
#define VR_TEST_LSAN 1
#endif
#if defined(VR_TEST_LSAN) && defined(__linux__)
#include <sanitizer/lsan_interface.h>
#else
#undef VR_TEST_LSAN
#endif

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/testing/vulkan_policy.hpp"
#include "volumetric_kit/core/vulkan/buffer.hpp"
#include "volumetric_kit/core/vulkan/compute_util.hpp"
#include "volumetric_kit/core/vulkan/vulkan.hpp"

#include "gpu_test.hpp"

namespace vkc = volumetric_kit::core;
namespace test = volumetric_kit::core::test;

#define CHECK(cond)                                                        \
  do {                                                                     \
    if (!(cond)) {                                                         \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      return 1;                                                            \
    }                                                                      \
  } while (0)

namespace {

int passes(vr_test::GpuContext& /*gpu*/) { return 0; }

int skips(vr_test::GpuContext& /*gpu*/) { return test::kSkipExitCode; }

// A buffer with no usage, which the layer refuses before the driver sees it.
int misuses(vr_test::GpuContext& gpu) {
  VkBufferCreateInfo info{};
  info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  info.size = 256;
  info.usage = 0;
  info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  VkBuffer buffer = VK_NULL_HANDLE;
  if (vkCreateBuffer(gpu.device.handle(), &info, nullptr, &buffer) ==
      VK_SUCCESS) {
    vkDestroyBuffer(gpu.device.handle(), buffer, nullptr);
  }
  return 0;
}

// A semaphore never destroyed, which the layer reports only as the device is.
int leaks(vr_test::GpuContext& gpu) {
  VkSemaphoreCreateInfo info{};
  info.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
  VkSemaphore semaphore = VK_NULL_HANDLE;
  // 2, not 1, so a failure here is not taken for the layer's report.
  if (vkCreateSemaphore(gpu.device.handle(), &info, nullptr, &semaphore) !=
      VK_SUCCESS) {
    return 2;
  }
  return 0;
}

// Two fills of one buffer with no barrier between them: a write after a
// write, which synchronization validation reports and plain validation does
// not.
int races(vr_test::GpuContext& gpu) {
  vkc::Result<vkc::Buffer> buffer =
      vkc::device_storage_buffer(gpu.allocator, 256);
  if (!buffer) return 2;
  const VkBuffer handle = buffer->handle();
  const vkc::Status submitted =
      gpu.device.submit_single_time([&](VkCommandBuffer cmd) {
        vkCmdFillBuffer(cmd, handle, 0, VK_WHOLE_SIZE, 1);
        vkCmdFillBuffer(cmd, handle, 0, VK_WHOLE_SIZE, 2);
      });
  return submitted.ok() ? 0 : 2;
}

#ifdef VR_TEST_LSAN
// The block leak_a_block leaks, its address XORed so that LSan's scan of the
// globals finds no pointer to it.
constexpr std::uintptr_t kHide =
    static_cast<std::uintptr_t>(0xa5a5a5a5a5a5a5a5u);
volatile std::uintptr_t hidden_block = 0;

void leak_a_block(std::string_view /*message*/) {
  if (hidden_block == 0) {
    hidden_block = reinterpret_cast<std::uintptr_t>(std::malloc(64)) ^ kHide;
  }
}

// Overwrites the stack the leaking call used, so that no stale copy of the
// block's address there makes it reachable. Uninstrumented, so the array is
// on the stack rather than ASan's fake stack.
__attribute__((noinline, no_sanitize("address"))) void scrub_stack() {
  volatile unsigned char scratch[256 * 1024];
  for (volatile unsigned char& byte : scratch) byte = 0;
}

int lsan_reports_leak_in_error_handler() {
  scrub_stack();
  CHECK(__lsan_do_recoverable_leak_check() == 0);  // none before the test's
  CHECK(vr_test::run_on_gpu(misuses, leak_a_block) == 1);
  CHECK(hidden_block != 0);
  scrub_stack();
  const bool reported = __lsan_do_recoverable_leak_check() != 0;
  std::free(reinterpret_cast<void*>(hidden_block ^ kHide));
  CHECK(reported);
  return 0;
}
#endif

int decoder_skips() {
  {
    const test::ScopedEnv none("VR_TEST_HEVC_BACKEND", nullptr);
    CHECK(vr_test::no_decoder("none here") == test::kSkipExitCode);
    CHECK(vr_test::no_jpeg_decoder("none here") == test::kSkipExitCode);
  }
  const test::ScopedEnv promised("VR_TEST_HEVC_BACKEND", "cuda");
  CHECK(vr_test::no_decoder("none here") == 1);
  CHECK(vr_test::no_jpeg_decoder("none here") == 1);
  CHECK(vr_test::no_jpeg_decoder("no hardware JPEG engine") ==
        test::kSkipExitCode);
  return 0;
}

}  // namespace

int main() {
  if (decoder_skips() != 0) return 1;
  if (test::validation_layer_version() == 0) {
    return test::no_device_exit_code("no Khronos validation layer");
  }
  const test::ScopedEnv validation("VKC_TEST_VALIDATION", "1");
  const int clean = vr_test::run_on_gpu(passes);
  if (clean == test::kSkipExitCode) return clean;  // no device
  CHECK(clean == 0);
  CHECK(vr_test::run_on_gpu(skips) == test::kSkipExitCode);
  CHECK(vr_test::run_on_gpu(misuses) == 1);
  CHECK(vr_test::run_on_gpu(leaks) == 1);
  {
    const test::ScopedEnv plain("VKC_TEST_SYNC_VALIDATION", nullptr);
    CHECK(vr_test::run_on_gpu(races) == 0);
  }
  const test::ScopedEnv sync("VKC_TEST_SYNC_VALIDATION", "1");
  CHECK(vr_test::run_on_gpu(races) == 1);
#ifdef VR_TEST_LSAN
  if (lsan_reports_leak_in_error_handler() != 0) return 1;
#endif
  std::puts("gpu_test_harness: OK");
  return 0;
}
