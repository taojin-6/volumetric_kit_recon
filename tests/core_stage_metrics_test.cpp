// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

/// @file tests/core_stage_metrics_test.cpp
/// @brief The reporting vocabulary and the GPU timer behind it.
///
/// Two halves, deliberately. StageMetrics is pure host arithmetic and runs
/// everywhere, including the Linux CI legs with no GPU. GpuTimer needs a device
/// and skips without one -- but the assertion that actually matters there is
/// not "a number came back", it is that the number is **smaller than the wall
/// clock around the same submit**. That inequality is the entire point of the
/// class: every recon timing before it was wall clock around a fence-blocked
/// dispatch, and a GPU span equal to it would mean nothing had been separated.

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <utility>

#include "volumetric_kit/recon/core/allocator.hpp"
#include "volumetric_kit/recon/core/buffer.hpp"
#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/gpu_timer.hpp"
#include "volumetric_kit/recon/core/instance.hpp"
#include "volumetric_kit/recon/core/stage_metrics.hpp"
#include "volumetric_kit/recon/core/vulkan.hpp"

namespace vr = volumetric_kit::recon;

#define CHECK(cond)                                                          \
  do {                                                                       \
    if (!(cond)) {                                                           \
      std::fprintf(stderr, "FAIL (%s:%d): %s\n", __FILE__, __LINE__, #cond); \
      return 1;                                                              \
    }                                                                        \
  } while (0)

namespace {

// A buffer big enough that filling it is unambiguously more than one timestamp
// tick, on any GPU this repo targets. At even 50 GB/s this is ~2.5 ms, so the
// `gpu_ms > 0` assertion below is a real assertion rather than one that passes
// because the work rounded to zero.
constexpr VkDeviceSize kFillBytes = 128u * 1024u * 1024u;

int host_only_checks() {
  // --- accumulation, and matching by CONTENT rather than pointer ------------
  {
    vr::StageMetrics m;
    m.add_cpu("allocate", 1.5);
    m.add_cpu("allocate", 2.5);
    CHECK(m.rows().size() == 1);
    CHECK(m.rows()[0].cpu_ms == 4.0);

    // Two distinct pointers with equal content must land in ONE row. Identical
    // string literals are not guaranteed to be pooled to one address, and
    // across translation units they routinely are not -- pointer matching would
    // silently split one stage into two plausible-looking rows. Built at
    // runtime so the compiler cannot merge them.
    char built[] = "allocate";
    m.add_cpu(built, 1.0);
    CHECK(m.rows().size() == 1);
    CHECK(m.rows()[0].cpu_ms == 5.0);
  }

  // --- has_gpu is set by add_gpu alone --------------------------------------
  {
    vr::StageMetrics m;
    m.add_cpu("integrate", 3.0);
    CHECK(m.rows()[0].has_gpu == false);
    CHECK(m.rows()[0].gpu_ms == 0.0);

    m.add_gpu("integrate", 1.25);
    CHECK(m.rows().size() == 1);  // same row, not a second one
    CHECK(m.rows()[0].has_gpu == true);
    CHECK(m.rows()[0].cpu_ms == 3.0);
    CHECK(m.rows()[0].gpu_ms == 1.25);

    // The reverse order too: a GPU span may resolve for a stage the host never
    // timed, and it must not lose its cpu_ms when one arrives later.
    vr::StageMetrics n;
    n.add_gpu("meshing", 2.0);
    n.add_cpu("meshing", 8.0);
    CHECK(n.rows().size() == 1);
    CHECK(n.rows()[0].gpu_ms == 2.0);
    CHECK(n.rows()[0].cpu_ms == 8.0);
    CHECK(n.rows()[0].has_gpu == true);
  }

  // --- seed keeps a table's shape stable ------------------------------------
  {
    vr::StageMetrics m;
    m.seed("texture");
    CHECK(m.rows().size() == 1);
    CHECK(m.rows()[0].cpu_ms == 0.0);
    CHECK(m.rows()[0].has_gpu == false);  // seeding is not a GPU measurement
    m.add_cpu("texture", 2.0);
    CHECK(m.rows().size() == 1);
  }

  // --- breakdown rows are excluded from the totals --------------------------
  //
  // The one with teeth: a breakdown restates time already counted by the stage
  // it decomposes, so summing it double-counts. Dropping the prefix skip here
  // makes total_cpu_ms report 30 instead of 20.
  {
    vr::StageMetrics m;
    m.add_cpu("extract", 10.0);
    m.add_cpu("  ..meshing", 8.0);
    m.add_cpu("  ..readback", 2.0);
    m.add_cpu("integrate", 10.0);
    CHECK(m.rows().size() == 4);
    CHECK(m.total_cpu_ms() == 20.0);

    // ...and `exclude` removes one further row on top of that.
    CHECK(m.total_cpu_ms("integrate") == 10.0);
    // A name that matches nothing removes nothing.
    CHECK(m.total_cpu_ms("nosuchstage") == 20.0);

    // The GPU total obeys the same structure, and counts only measured rows.
    m.add_gpu("extract", 6.0);
    m.add_gpu("  ..meshing", 6.0);
    CHECK(m.total_gpu_ms() == 6.0);
  }

  // --- clear keeps the object usable ----------------------------------------
  {
    vr::StageMetrics m;
    m.add_cpu("a", 1.0);
    m.clear();
    CHECK(m.empty());
    CHECK(m.total_cpu_ms() == 0.0);
    m.add_cpu("b", 2.0);
    CHECK(m.rows().size() == 1);
  }

  // --- an opted-out scope records nothing -----------------------------------
  //
  // "Nothing is measured when the caller passes null" is a promise every tier
  // entry point will make, and OptionalStageScope is what makes it structural
  // rather than a convention each site re-implements with an `if`.
  {
    vr::StageMetrics m;
    {
      vr::OptionalStageScope inert(nullptr, "never");
    }
    CHECK(m.empty());
    {
      vr::OptionalStageScope live(&m, "counted");
    }
    CHECK(m.rows().size() == 1);
    CHECK(m.rows()[0].cpu_ms >= 0.0);
  }

  // --- the tick maths, including the wrap the naive version gets wrong ------
  {
    CHECK(vr::timestamp_delta(100, 150, 64) == 50);
    // Zero valid bits is "no usable counter", not a 2^64 span.
    CHECK(vr::timestamp_delta(100, 150, 0) == 0);
    // A 32-bit counter that wrapped between the two queries: masking the
    // endpoints BEFORE subtracting reports 2^64 - 20 here instead of 20.
    const std::uint64_t begin = (std::uint64_t{1} << 32) - 10;
    CHECK(vr::timestamp_delta(begin, begin + 20, 32) == 20);
    CHECK(vr::ticks_to_ms(1'000'000, 1.0f) == 1.0);  // 1e6 ns == 1 ms
    CHECK(vr::ticks_to_ms(1'000'000, 0.0f) == 0.0);  // no period, no claim
  }

  return 0;
}

}  // namespace

int main() {
  if (host_only_checks() != 0) return 1;

  vr::Result<vr::Instance> instance = vr::Instance::create({});
  if (!instance) {
    std::fprintf(stderr, "no Vulkan instance (%s); host checks passed\n",
                 instance.status().message().c_str());
    return 0;
  }
  vr::Result<VkPhysicalDevice> gpu = instance.value().select_physical_device();
  if (!gpu) {
    std::fprintf(stderr, "no compute device (%s); host checks passed\n",
                 gpu.status().message().c_str());
    return 0;
  }
  vr::Result<vr::Device> device =
      vr::Device::create(instance.value().handle(), gpu.value(), {});
  if (!device) {
    std::fprintf(stderr, "no device (%s); host checks passed\n",
                 device.status().message().c_str());
    return 0;
  }
  vr::Result<vr::Allocator> allocator =
      vr::Allocator::create(instance.value().handle(), device.value());
  CHECK(allocator);

  vr::Result<vr::GpuTimer> timer_result = vr::GpuTimer::create(device.value());
  CHECK(timer_result);
  vr::GpuTimer timer = std::move(timer_result).value();
  CHECK(timer.valid());

  vr::BufferDesc desc;
  desc.size = kFillBytes;
  desc.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
  desc.memory = vr::MemoryUsage::DeviceLocal;
  vr::Result<vr::Buffer> target = allocator.value().create_buffer(desc);
  CHECK(target);

  const auto record = [&](VkCommandBuffer cmd) {
    vkCmdFillBuffer(cmd, target.value().handle(), 0, kFillBytes, 0xA5A5A5A5u);
  };

  // The wall clock around the whole blocking submit -- allocate, begin, record,
  // submit, fence wait, free. This is exactly what every recon stage timing has
  // measured to date.
  const auto wall_start = std::chrono::steady_clock::now();
  const vr::Status submitted =
      device.value().submit_single_time(record, &timer, "fill");
  const double wall_ms = std::chrono::duration<double, std::milli>(
                             std::chrono::steady_clock::now() - wall_start)
                             .count();
  CHECK(submitted);

  vr::StageMetrics metrics;
  timer.report_into(metrics);

  if (!timer.available()) {
    // A queue family reporting zero timestampValidBits is a supported
    // configuration, not a failure: the timer must degrade to contributing no
    // rows rather than publishing zeros that read as instant dispatches.
    std::fprintf(stderr,
                 "timestamps unavailable on this compute family; "
                 "checked the degradation path instead\n");
    CHECK(metrics.empty());
    CHECK(timer.count() == 0);
    return 0;
  }

  // One resolved span, labelled, carrying a real measurement.
  CHECK(metrics.rows().size() == 1);
  CHECK(metrics.rows()[0].has_gpu);
  CHECK(std::string(metrics.rows()[0].name) == "fill");
  CHECK(metrics.rows()[0].gpu_ms > 0.0);

  // THE assertion. If the span were the wall clock relabelled -- or if the
  // timestamps resolved against the wrong period -- this is what catches it.
  // The submit's host overhead is strictly outside the recorded region, so the
  // device span must come in under it, and on a 128 MB fill it must be a
  // visible fraction of it rather than noise.
  CHECK(metrics.rows()[0].gpu_ms < wall_ms);

  // The per-window bound is enforced rather than overflowing the pool: a
  // one-span timer refuses the second, and refusing means kNoSpan, not a write
  // past the query range (which no validation layer would catch as a logic
  // error).
  vr::Result<vr::GpuTimer> tiny = vr::GpuTimer::create(device.value(), 1);
  CHECK(tiny);
  VkCommandBuffer null_cmd = VK_NULL_HANDLE;
  CHECK(tiny.value().begin(null_cmd, "a") == vr::GpuTimer::kNoSpan);

  // reset() returns the window to empty and the timer stays usable.
  timer.reset();
  CHECK(timer.count() == 0);
  vr::StageMetrics after_reset;
  timer.report_into(after_reset);
  CHECK(after_reset.empty());

  std::fprintf(stderr,
               "gpu %.3f ms vs wall %.3f ms for a %llu MB fill "
               "(%.1f%% of the blocking submit was device time)\n",
               metrics.rows()[0].gpu_ms, wall_ms,
               static_cast<unsigned long long>(kFillBytes / (1024 * 1024)),
               100.0 * metrics.rows()[0].gpu_ms / wall_ms);
  return 0;
}
