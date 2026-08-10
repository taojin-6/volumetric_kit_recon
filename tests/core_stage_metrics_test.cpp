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
#include <string>
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

    // The GPU total counts only measured rows -- but, unlike the host total,
    // it counts breakdowns too. A host scope spans a whole call including
    // everything it invokes, so a sub-row's host time is already inside its
    // parent's; a device span covers one dispatch, so a breakdown carrying one
    // is a kernel no parent span contains. Skipping it here dropped that time
    // from every total while its host half stayed counted through the parent,
    // which is how the active-set compaction went missing from the device
    // column it was added to fill.
    m.add_gpu("extract", 6.0);
    m.add_gpu("  ..meshing", 6.0);
    CHECK(m.total_gpu_ms() == 12.0);
    // `exclude` still drops a row by name, breakdown or not.
    CHECK(m.total_gpu_ms("extract") == 6.0);
    CHECK(m.total_gpu_ms("  ..meshing") == 6.0);
    // ...and a row with no device measurement contributes nothing either way.
    CHECK(m.total_gpu_ms("integrate") == 12.0);
  }

  // --- a row's class follows the nesting it was added in ---------------------
  //
  // in_stage is what lets a callee invoked from both positions name its row
  // correctly. Hard-coding the prefix instead hands a top-level caller a row
  // total_cpu_ms skips -- their only stage, absent from their own total while
  // still sitting in rows() looking accounted for.
  {
    vr::StageMetrics m;
    CHECK(!m.in_stage());
    {
      vr::StageScope outer(m, "integrate");
      CHECK(m.in_stage());
      {
        vr::StageScope inner(m, "  ..active set");
        CHECK(m.in_stage());
      }
      // Still open: the inner scope closing does not end the outer one.
      CHECK(m.in_stage());
    }
    CHECK(!m.in_stage());

    // A null metrics pointer is inert here as everywhere else -- the scope must
    // not touch a set it was not given.
    {
      vr::StageScope none(nullptr, "nowhere");
      CHECK(!m.in_stage());
    }
    CHECK(!m.in_stage());
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
  // entry point will make, and the pointer constructor is what makes it
  // structural rather than a convention each site re-implements with an `if`.
  // The inert scope is given the same `m` the live one writes to, so this
  // asserts that nothing was recorded rather than asserting on a metrics the
  // scope was never handed.
  {
    vr::StageMetrics m;
    {
      vr::StageScope inert(static_cast<vr::StageMetrics*>(nullptr), "never");
    }
    CHECK(m.empty());
    {
      vr::StageScope live(&m, "counted");
    }
    CHECK(m.rows().size() == 1);
    CHECK(std::string(m.rows()[0].name) == "counted");
    CHECK(m.rows()[0].cpu_ms >= 0.0);
    // ...and the reference constructor is the same scope, not a second one.
    {
      vr::StageScope by_ref(m, "counted");
    }
    CHECK(m.rows().size() == 1);
  }

  // --- merge carries BOTH halves --------------------------------------------
  //
  // The one with teeth: `add_cpu(row.name, row.cpu_ms)` is the natural way to
  // write this loop by hand and silently drops gpu_ms/has_gpu, and the loss is
  // invisible downstream because an absent has_gpu is indistinguishable from a
  // genuinely host-only stage.
  {
    vr::StageMetrics dst;
    dst.add_cpu("extract", 1.0);

    vr::StageMetrics src;
    src.add_cpu("extract", 2.0);
    src.add_gpu("extract", 0.5);
    src.add_cpu("texture", 3.0);  // host-only: must NOT gain has_gpu

    dst.merge(src);
    CHECK(dst.rows().size() == 2);
    CHECK(dst.rows()[0].cpu_ms == 3.0);
    CHECK(dst.rows()[0].gpu_ms == 0.5);
    CHECK(dst.rows()[0].has_gpu == true);
    CHECK(dst.rows()[1].cpu_ms == 3.0);
    CHECK(dst.rows()[1].has_gpu == false);

    // Merging a set into itself would iterate the rows it is appending to.
    dst.merge(dst);
    CHECK(dst.rows().size() == 2);
    CHECK(dst.rows()[0].cpu_ms == 3.0);
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
  if (!target) {
    // Every other resource step above skips rather than failing, and this one
    // is no different: a 128 MB device-local allocation can legitimately fail
    // on a contended runner or a memory-constrained target, and failing the leg
    // over it would report a diagnostic-only feature as broken. The size is
    // chosen only so `gpu_ms > 0` is not vacuous.
    std::fprintf(stderr, "no %llu MB device buffer (%s); host checks passed\n",
                 static_cast<unsigned long long>(kFillBytes / (1024 * 1024)),
                 target.status().message().c_str());
    return 0;
  }

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

  // Publishing ended the window, so the timer starts the next one empty. This
  // is what keeps max_spans a per-window bound: without it a timer created once
  // and submitted through forever fills up and then times nothing, silently.
  CHECK(timer.count() == 0);

  // The per-window bound is enforced rather than overflowing the pool: a
  // one-span timer refuses the second, and refusing means kNoSpan, not a write
  // past the query range (which no validation layer would catch as a logic
  // error). Driven through real submits, so the bound itself is exercised --
  // a null command buffer would short-circuit begin() before reaching it and
  // the check would pass with the guard deleted.
  vr::Result<vr::GpuTimer> tiny = vr::GpuTimer::create(device.value(), 1);
  CHECK(tiny);
  CHECK(device.value().submit_single_time(record, &tiny.value(), "a"));
  CHECK(tiny.value().count() == 1);
  CHECK(device.value().submit_single_time(record, &tiny.value(), "b"));
  CHECK(tiny.value().count() == 1);  // refused, and the submit still succeeded

  // A window reports its OWN cost, not a running total: StageMetrics
  // accumulates by name, so a re-published span would add frame N's cost on top
  // of every earlier frame's and present the sum as the current one.
  vr::StageMetrics first_window;
  tiny.value().report_into(first_window);
  CHECK(first_window.rows().size() == 1);
  CHECK(std::string(first_window.rows()[0].name) == "a");
  CHECK(tiny.value().count() == 0);

  // ...and the timer is usable again rather than stuck at its bound for the
  // rest of its life.
  CHECK(device.value().submit_single_time(record, &tiny.value(), "c"));
  vr::StageMetrics second_window;
  tiny.value().report_into(second_window);
  CHECK(second_window.rows().size() == 1);
  CHECK(std::string(second_window.rows()[0].name) == "c");
  CHECK(second_window.rows()[0].gpu_ms > 0.0);

  // Publishing an already-published window adds nothing. This is the assertion
  // with teeth: a report that re-emitted its spans would put "a" in here too,
  // and in the realistic shape -- one label per stage, reported every frame --
  // it would grow one row without bound instead of showing a second one.
  vr::StageMetrics republished;
  tiny.value().report_into(republished);
  CHECK(republished.empty());

  // create() rejects a max_spans whose doubled query count would wrap, rather
  // than building a pool smaller than the bound begin() enforces.
  CHECK(!vr::GpuTimer::create(device.value(), 0));
  CHECK(!vr::GpuTimer::create(device.value(), 0x80000000u));
  CHECK(!vr::GpuTimer::create(device.value(), vr::GpuTimer::kMaxSpans + 1));
  CHECK(vr::GpuTimer::create(device.value(), vr::GpuTimer::kMaxSpans));

  // reset() discards a window instead of publishing it, and the timer stays
  // usable.
  CHECK(device.value().submit_single_time(record, &timer, "discarded"));
  CHECK(timer.count() == 1);
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
