// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file core/gpu_timer.hpp
/// @brief Timestamp spans around compute work: what the *device* spent, as
///        distinct from the wall clock around a blocking submit.

#include <cstddef>
#include <cstdint>
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
/// make an optional diagnostic able to break a scan -- so a query pool this
/// *cannot allocate* degrades the same way rather than propagating, and
/// @ref abandon retires a pool mid-run on the same terms. @ref create refuses
/// only what a caller got wrong: an invalid device, or a @p max_spans out of
/// range.
///
/// **The window has one owner: @ref report_into publishes it and ends it.**
/// Spans accumulate between windows, which is what lets several dispatches be
/// timed into one frame's metrics -- but *nothing* would end the window if
/// publishing did not, and both failure modes are silent. A timer created once
/// and submitted through forever would fill `max_spans` and then stop timing
/// with no error, and every publish would report a running total of every frame
/// since creation under this frame's label. So a caller creates the timer once,
/// submits as often as it likes, and publishes per frame; @ref reset is for
/// discarding a window instead of publishing it.
///
/// @warning The @p device passed to @ref create must outlive the timer (it owns
///          a query pool freed through that device).
/// @warning Not internally synchronized: one timer records into one command
///          buffer at a time. Sharing one across threads does not work.
///
/// @code
/// VR_ASSIGN(GpuTimer timer, GpuTimer::create(device));   // once
/// for (;;) {
///   StageMetrics metrics;
///   // The safe path: the overload begins, ends, and resolves the span itself,
///   // so it cannot be read before the fence.
///   VR_TRY(device.submit_single_time(record, &timer, "meshing"));
///   timer.report_into(metrics);  // adds a "meshing" gpu_ms row, ends the
///                                // window
/// }
/// @endcode
class VR_CORE_API GpuTimer {
 public:
  /// @brief Returned by @ref begin when no span was started -- timing is
  ///        unavailable on this device, or the per-window bound is exhausted.
  static constexpr std::uint32_t kNoSpan = 0xFFFFFFFFu;

  /// @brief Hard ceiling on @ref create's `max_spans`.
  ///
  /// The pool is `2 * max_spans` queries, and `VkQueryPoolCreateInfo` counts
  /// them in a `uint32_t`: without a ceiling the doubling wraps, and a
  /// `max_spans` of `0x80000000` would create a **zero**-query pool that @ref
  /// begin still believes has room for two billion spans. Far above any real
  /// window (recon records one span per dispatch), so it only ever rejects a
  /// number that was already a mistake.
  static constexpr std::uint32_t kMaxSpans = 4096u;

  /// @brief Construct an empty timer (owns nothing; `valid()` is false).
  GpuTimer() noexcept = default;

  /// @brief Create a timer for @p device's compute queue family.
  /// @param device     The device whose compute family is timed; supplies the
  ///                   timestamp validity and period. Must outlive the timer.
  /// @param max_spans  Upper bound on spans in one window; sizes the pool at
  ///                   `2 * max_spans` queries. In `[1, kMaxSpans]`.
  /// @return The timer on success -- including where the family supports no
  ///         timestamps *and* where the query pool could not be allocated, in
  ///         both of which @ref available is false and the failure is logged --
  ///         or @ref Status::Code::InvalidArgument for a @p max_spans outside
  ///         `[1, kMaxSpans]` or an invalid @p device.
  ///
  /// A pool that will not allocate is exactly the case the availability rule
  /// above is for: a tier creating its timer in its own `create()` would
  /// otherwise refuse to construct -- failing the whole reconstruction spine
  /// for a caller who never asked for timing -- because a diagnostic ran out of
  /// host memory.
  static Result<GpuTimer> create(const Device& device,
                                 std::uint32_t max_spans = 32);

  ~GpuTimer() = default;
  GpuTimer(GpuTimer&& other) noexcept;
  GpuTimer& operator=(GpuTimer&& other) noexcept;
  GpuTimer(const GpuTimer&) = delete;
  GpuTimer& operator=(const GpuTimer&) = delete;

  /// @return Whether this device can actually time spans. False leaves every
  ///         operation below a well-defined no-op; see the class note.
  bool available() const noexcept { return valid_bits_ != 0 && pool_.valid(); }

  /// @return `true` if this owns a timer (false when moved-from). A timer on a
  ///         device without timestamp support is still valid; it is
  ///         @ref available that is false.
  bool valid() const noexcept { return device_ != VK_NULL_HANDLE; }

  /// @brief Discard the window's spans without publishing them.
  ///
  /// Does not touch the device: the pool's queries are reset inside the command
  /// buffer by @ref begin, which is where a reset is legal and correctly
  /// ordered against the writes. @ref report_into ends a window too, and is
  /// what a caller that wants the numbers uses instead.
  void reset() noexcept { end_window(); }

  /// @brief Stop timing for good: a submission carrying this window's queries
  ///        was abandoned while it may still be executing.
  ///
  /// @ref Device::submit_single_time leaks its command buffer when the fence
  /// wait fails, precisely because the GPU may still run it. That buffer resets
  /// and writes the span's two queries, so handing those indices to the next
  /// @ref begin would record a `vkCmdResetQueryPool` over queries an in-flight
  /// buffer is about to write (VUID-vkCmdResetQueryPool-None-02841), and could
  /// resolve the abandoned submit's timestamps under the new span's label.
  /// Nothing can say when that buffer drains, so the pool retires with it:
  /// @ref available is false from here on and every later call is the same
  /// well-defined no-op as on a device without timestamp support. Long-lived
  /// per-tier timers are what make this reachable -- a throwaway timer died
  /// with the failed dispatch.
  void abandon() noexcept;

  // TODO(core): pair a span with a VK_EXT_debug_utils label, so a capture in
  // RenderDoc / Xcode names the same regions this reports. Deferred because
  // recon enables the extension only when validation is on, so a label needs
  // Device to record whether it was enabled -- the declare/verify shape of the
  // enabled-extension list, and its own change.

  /// @brief Open a span: reset this span's two queries and write its start
  ///        timestamp into @p cmd.
  /// @param cmd   A recording command buffer, outside any render pass (where
  ///              `vkCmdResetQueryPool` is legal -- always true of recon's
  ///              compute submits).
  /// @param name  Span label, stored **by pointer** on exactly the terms
  ///              @ref StageRow::name states -- it is published straight into
  ///              a row, so one lifetime rule covers the whole vocabulary. Null
  ///              labels the span `"gpu"`.
  /// @return The span id to pass to @ref end, or @ref kNoSpan when timing is
  ///         unavailable or `max_spans` is exhausted. Passing @ref kNoSpan back
  ///         to @ref end is harmless.
  std::uint32_t begin(VkCommandBuffer cmd, const char* name);

  /// @brief Close a span: write its end timestamp into @p cmd.
  /// @param cmd   The same command buffer @ref begin recorded into.
  /// @param span  The id @ref begin returned; @ref kNoSpan does nothing.
  void end(VkCommandBuffer cmd, std::uint32_t span);

  /// @brief Drop @p span because the command buffer carrying it will not
  ///        execute.
  ///
  /// A span's two queries are reset and written by commands *inside* a command
  /// buffer, so a buffer abandoned before submission (a failed
  /// `vkEndCommandBuffer`, a failed fence create, a failed submit) leaves them
  /// untouched. They are then uninitialized -- reading one is undefined per
  /// VUID-vkGetQueryPoolResults-None-09401, and worse if the pool was used
  /// before, since the pair still holds an *earlier* window's value with its
  /// availability bit set and resolves as a plausible duration under the
  /// abandoned span's label. Dropping the span is what keeps @ref resolve
  /// reading only queries a submitted buffer actually wrote.
  ///
  /// Only the most recently opened, still-unresolved span can be dropped, which
  /// is exactly what a failed submit leaves behind; anything else is ignored,
  /// as is @ref kNoSpan.
  ///
  /// @param span  The id @ref begin returned.
  void discard(std::uint32_t span) noexcept;

  /// @brief Read the spans recorded since the last resolve back, and convert
  ///        them to milliseconds.
  ///
  /// Reads only the still-unresolved tail, so calling this after every submit
  /// -- which @ref Device::submit_single_time does -- costs one readback per
  /// new span rather than re-reading the whole window each time.
  ///
  /// @pre The submits carrying those spans have **completed** -- their fences
  ///      signalled. This never blocks (a diagnostic must not be able to hang a
  ///      dispatch), and it cannot detect a premature call either: a query pair
  ///      being reused by a new window still holds the *previous* window's
  ///      value with its availability bit set until the new command buffer
  ///      executes, so an early read resolves a stale duration under the new
  ///      label rather than reporting nothing. Prefer the
  ///      @ref Device::submit_single_time overload, which sequences this behind
  ///      the fence for you.
  /// @return OK, or a Vulkan-domain @ref Status if the read itself failed.
  ///         Spans left unresolved by a failure are skipped by
  ///         @ref report_into rather than published as zero.
  Status resolve();

  /// @return How many spans the current window recorded.
  std::size_t count() const noexcept { return spans_.size(); }

  /// @brief Publish each resolved span to @p out as a GPU row, and end the
  ///        window.
  ///
  /// Spans that did not resolve (see @ref resolve) are skipped rather than
  /// reported as `0.0`, so a row's absence means "not measured" and a `0.00`
  /// means "measured, and fast". Contributes nothing when @ref available is
  /// false.
  ///
  /// **Publishing ends the window** (@ref count returns to zero) because
  /// @ref StageMetrics accumulates by name: re-publishing a span already
  /// reported would add this frame's cost on top of every earlier frame's and
  /// present the running total as the current one. Draining here is what makes
  /// a create-once, submit-per-frame caller correct with no reset discipline --
  /// and what keeps `max_spans` a per-frame bound rather than a lifetime one.
  ///
  /// @param out  The metrics to add the rows to.
  void report_into(StageMetrics& out);

 private:
  struct Span {
    // Borrowed on @ref StageRow::name's terms -- see begin().
    const char* name = nullptr;
    double ms = 0.0;
    bool resolved = false;
  };

  // Zero every member, so a moved-from timer is indistinguishable from a
  // default-constructed one and its accessors stay consistent with valid().
  void clear_state() noexcept;

  // End the window: drop its spans and re-arm the full-window warning. The
  // warning latch is per window, not per timer -- latched for a lifetime it
  // would fire once on a create-once timer and every later exhaustion, each of
  // which freezes that tier's GPU column, would be silent.
  void end_window() noexcept {
    spans_.clear();
    warned_full_ = false;
  }

  VkDevice device_ = VK_NULL_HANDLE;
  UniqueHandle<VkQueryPool, vkDestroyQueryPool> pool_;
  std::uint32_t max_spans_ = 0;
  // Read from the driver at create, never declared by a caller: capabilities
  // are queryable, so nothing here can drift the way an enabled-extension list
  // can (the 2026-08-03 rule).
  std::uint32_t valid_bits_ = 0;
  float period_ns_ = 0.0f;
  // Latched so a full window warns once rather than on every refused span;
  // re-armed by end_window() so the *next* window can warn too.
  bool warned_full_ = false;
  std::vector<Span> spans_;
  // Readback scratch, reserved at create so a per-dispatch resolve allocates
  // nothing.
  std::vector<std::uint64_t> readback_;
};

/// @brief One stage's whole window: times the host span, hands @ref dispatch
///        the device timer, and publishes **both halves when the scope
///        closes**.
///
/// The pairing a tier needs, as one object, because the three parts have to be
/// got right together and hand-copying them got one wrong: a tier that opens a
/// @ref StageScope, threads its @ref GpuTimer into a dispatch, and then calls
/// @ref GpuTimer::report_into as a plain statement afterwards *skips the
/// publish on every early return between them*. The spans stay in a timer that
/// now outlives the call, so the next successful call publishes a failed
/// frame's device time under its own label -- and after `max_spans` such
/// failures @ref GpuTimer::begin refuses every span and the tier's GPU column
/// freezes for good. Being a destructor, this cannot be skipped by a return.
///
/// It is also the argument @ref dispatch takes, so a timer can no longer reach
/// a dispatch without the label it publishes under.
///
/// Null @p metrics is inert end to end: @ref timer returns null, so the
/// dispatch takes the untimed path, no query is written, and nothing is
/// published. That is the 2026-08-01 bar -- nothing measured when unasked --
/// enforced by construction rather than by an `if` at each site.
///
/// @warning The @ref StageMetrics, the @ref GpuTimer and the @p name literal
///          must all outlive the scope.
///
/// @code
/// Status Tier::run(..., StageMetrics* metrics) {
///   GpuStageScope stage(metrics, gpu_timer_, "integrate");
///   ...                                    // every return below publishes
///   return dispatch(*device_, kernel_, &push, sizeof(push), groups, max,
///   &stage);
/// }
/// @endcode
class GpuStageScope {
 public:
  /// @brief Open @p name's window on @p metrics, collecting device spans in
  ///        @p timer; inert when @p metrics is null.
  GpuStageScope(StageMetrics* metrics, GpuTimer& timer, const char* name)
      : metrics_(metrics), timer_(&timer), host_(metrics, name) {
    // Seeded up front so the row exists in first-seen order before any
    // breakdown row the stage adds beneath it -- otherwise a sub-row published
    // mid-call lands above the stage it decomposes.
    if (metrics_ != nullptr) metrics_->seed(name);
  }

  /// @brief Publish the device spans, then close the host row.
  ///
  /// Member order does the sequencing: this body runs first, then @ref host_ --
  /// declared last, so destroyed first among the members -- adds the host span.
  ~GpuStageScope() {
    if (metrics_ != nullptr) timer_->report_into(*metrics_);
  }

  GpuStageScope(const GpuStageScope&) = delete;
  GpuStageScope& operator=(const GpuStageScope&) = delete;
  GpuStageScope(GpuStageScope&&) = delete;
  GpuStageScope& operator=(GpuStageScope&&) = delete;

  /// @return The timer to record into, or `nullptr` when inert -- which is
  ///         exactly @ref dispatch's untimed path.
  GpuTimer* timer() const noexcept {
    return metrics_ != nullptr ? timer_ : nullptr;
  }

  /// @return The label device spans are published under; see
  ///         @ref StageRow::name for its lifetime.
  const char* name() const noexcept { return host_.name(); }

 private:
  StageMetrics* metrics_;
  GpuTimer* timer_;
  StageScope host_;
};

}  // namespace volumetric_kit::recon
