// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file core/gpu_timer.hpp
/// @brief Timestamp spans around compute work: what the *device* spent, as
///        distinct from the wall clock around a blocking submit.

#include <cstdint>
#include <string>
#include <vector>

#include "volumetric_kit/recon/core/export.hpp"
#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/core/stage_metrics.hpp"
#include "volumetric_kit/recon/core/unique_handle.hpp"
#include "volumetric_kit/recon/core/vulkan.hpp"

namespace volumetric_kit::recon {

class Device;

/// @brief Elapsed ticks between two timestamp queries, correct across a counter
///        wrap.
/// @param begin       The earlier query's raw tick value.
/// @param end         The later query's raw tick value.
/// @param valid_bits  `VkQueueFamilyProperties::timestampValidBits` for the
///                    queue the queries were written on (0 yields 0; >= 64
///                    means the full counter).
/// @return `(end - begin)` reduced modulo `2^valid_bits`.
///
/// Only the low @p valid_bits of each tick are meaningful, so the counter is an
/// N-bit ring and the true span is `(end - begin) mod 2^N`. Because `2^N`
/// divides `2^64`, the wrapped 64-bit subtraction is already congruent and one
/// mask recovers the answer. Masking the two endpoints *before* subtracting
/// instead leaves the result modulo `2^64`, which reports `2^64 - (begin -
/// end)` whenever the counter wrapped between the queries -- a span of several
/// thousand years, reported without complaint.
constexpr std::uint64_t timestamp_delta(std::uint64_t begin, std::uint64_t end,
                                        std::uint32_t valid_bits) noexcept {
  if (valid_bits == 0) return 0;
  const std::uint64_t mask = valid_bits >= 64
                                 ? ~std::uint64_t{0}
                                 : (std::uint64_t{1} << valid_bits) - 1;
  return (end - begin) & mask;
}

/// @brief Converts a tick delta to milliseconds.
/// @param ticks                End-minus-start ticks (see @ref
///                             timestamp_delta).
/// @param timestamp_period_ns  Nanoseconds per tick
///                             (`VkPhysicalDeviceLimits::timestampPeriod`).
/// @return The span in milliseconds, or `0.0` when the period is not positive.
///
/// Computed in `double`, so a large delta does not overflow the way a 64-bit
/// integer nanosecond intermediate could.
constexpr double ticks_to_ms(std::uint64_t ticks,
                             float timestamp_period_ns) noexcept {
  if (timestamp_period_ns <= 0.0f) return 0.0;
  return static_cast<double>(ticks) * static_cast<double>(timestamp_period_ns) *
         1e-6;
}

/// @brief Records timestamp spans into a command buffer and resolves them to
///        milliseconds once the submit has completed.
///
/// Every recon timing to date has been wall clock around
/// `Device::submit_single_time`, which blocks on a fence -- so host record,
/// submit, the fence stall and device execution collapse into one number, and
/// "the kernel is slow" cannot be told from "we are waiting". This separates
/// them. On an M5 iPad Pro that distinction is the difference between
/// optimising a marching-cubes dispatch and optimising a stall, on a stage
/// measured at 373 ms.
///
/// **recon resolves more cheaply than a renderer can.** A render loop runs
/// frames ahead of the GPU, so its profiler must buffer per in-flight slot and
/// publish a snapshot that lags. Every recon dispatch is already fence-blocked,
/// so the timestamps are readable the instant the submit returns: no ring, no
/// deferred publish, no lag. The blocking design that costs recon throughput
/// pays for itself here.
///
/// **Availability is not an error.** A queue family may report
/// `timestampValidBits == 0` -- MoltenVK does on some configurations, and a
/// compute-only family on a discrete GPU may too. @ref create then succeeds and
/// @ref available is false: @ref begin returns @ref kNoSpan, @ref end does
/// nothing, and @ref report_into contributes no rows, so a caller writes the
/// same code either way and simply gets host timings. Failing instead would
/// make an optional diagnostic able to break a scan.
///
/// @warning The @p device passed to @ref create must outlive the timer (it owns
///          a query pool freed through that device).
/// @warning Not internally synchronized. A timer records into one command
///          buffer at a time and is resolved before the next
///          @ref reset, which the @ref Device::submit_single_time overload
///          guarantees; sharing one across threads does not.
///
/// @code
/// VR_ASSIGN(GpuTimer timer, GpuTimer::create(device));
/// // The safe path: the overload begins, ends, and resolves the span itself,
/// // so it cannot be read before the fence.
/// VR_TRY(device.submit_single_time(record, &timer, "meshing"));
///
/// StageMetrics metrics;
/// timer.report_into(metrics);   // adds a "meshing" gpu_ms row
/// @endcode
class VR_CORE_API GpuTimer {
 public:
  /// @brief Returned by @ref begin when no span was started -- timing is
  ///        unavailable on this device, or the per-collection bound is
  ///        exhausted.
  static constexpr std::uint32_t kNoSpan = 0xFFFFFFFFu;

  /// @brief Construct an empty timer (owns nothing; `valid()` is false).
  GpuTimer() noexcept = default;

  /// @brief Create a timer for @p device's compute queue family.
  /// @param device     The device whose compute family is timed; supplies the
  ///                   timestamp validity and period. Must outlive the timer.
  /// @param max_spans  Upper bound on spans between @ref reset calls; sizes the
  ///                   pool at `2 * max_spans` queries. Must be >= 1.
  /// @return The timer on success -- including where the family supports no
  ///         timestamps, in which case @ref available is false -- or a non-OK
  ///         @ref Status: @ref Status::Code::InvalidArgument for a zero
  ///         @p max_spans or an invalid @p device, otherwise a Vulkan-domain
  ///         failure from creating the query pool.
  static Result<GpuTimer> create(const Device& device,
                                 std::uint32_t max_spans = 32);

  ~GpuTimer() = default;
  GpuTimer(GpuTimer&&) noexcept = default;
  GpuTimer& operator=(GpuTimer&&) noexcept = default;
  GpuTimer(const GpuTimer&) = delete;
  GpuTimer& operator=(const GpuTimer&) = delete;

  /// @return Whether this device can actually time spans. False leaves every
  ///         operation below a well-defined no-op; see the class note.
  bool available() const noexcept { return valid_bits_ != 0 && pool_.valid(); }

  /// @return `true` if this owns a timer (false when moved-from). A timer on a
  ///         device without timestamp support is still valid; it is
  ///         @ref available that is false.
  bool valid() const noexcept { return device_ != VK_NULL_HANDLE; }

  /// @brief Discard recorded spans and start a new collection window.
  ///
  /// Does not touch the device: the pool's queries are reset inside the command
  /// buffer by @ref begin, which is where a reset is legal and correctly
  /// ordered against the writes.
  void reset() noexcept { spans_.clear(); }

  /// @brief Open a span: reset this span's two queries and write its start
  ///        timestamp into @p cmd.
  /// @param cmd   A recording command buffer, outside any render pass (where
  ///              `vkCmdResetQueryPool` is legal -- always true of recon's
  ///              compute submits).
  /// @param name  Span label. Copied, unlike @ref StageRow::name, because a
  ///              caller labelling a dispatch often builds the string.
  /// @return The span id to pass to @ref end, or @ref kNoSpan when timing is
  ///         unavailable or `max_spans` is exhausted. Passing @ref kNoSpan back
  ///         to @ref end is harmless.
  std::uint32_t begin(VkCommandBuffer cmd, const char* name);

  /// @brief Close a span: write its end timestamp into @p cmd.
  /// @param cmd   The same command buffer @ref begin recorded into.
  /// @param span  The id @ref begin returned; @ref kNoSpan does nothing.
  void end(VkCommandBuffer cmd, std::uint32_t span);

  /// @brief Read the recorded spans back and convert them to milliseconds.
  ///
  /// @pre The submit carrying the command buffer has **completed** -- its fence
  ///      signalled. Called earlier the results are simply not available yet;
  ///      this reports them as unmeasured rather than blocking, because a
  ///      diagnostic must not be able to hang a dispatch.
  /// @return OK, or a Vulkan-domain @ref Status if the read itself failed.
  Status resolve();

  /// @return How many spans the current window recorded.
  std::size_t count() const noexcept { return spans_.size(); }

  /// @brief Add each resolved span to @p out as a GPU row.
  ///
  /// Spans that did not resolve (see @ref resolve) are skipped rather than
  /// reported as `0.0`, so a row's absence means "not measured" and a `0.00`
  /// means "measured, and fast". Contributes nothing when @ref available is
  /// false.
  ///
  /// @warning @ref StageRow stores its label **by pointer** while this class
  ///          copies it, so the rows added here borrow *this timer's* strings:
  ///          they are valid until the next @ref reset or the timer's
  ///          destruction, not for the lifetime of the metrics.
  void report_into(StageMetrics& out) const;

 private:
  struct Span {
    std::string name;
    double ms = 0.0;
    bool resolved = false;
  };

  VkDevice device_ = VK_NULL_HANDLE;
  UniqueHandle<VkQueryPool, vkDestroyQueryPool> pool_;
  std::uint32_t max_spans_ = 0;
  // Read from the driver at create, never declared by a caller: capabilities
  // are queryable, so nothing here can drift the way an enabled-extension list
  // can (the 2026-08-03 rule).
  std::uint32_t valid_bits_ = 0;
  float period_ns_ = 0.0f;
  std::vector<Span> spans_;
};

}  // namespace volumetric_kit::recon
