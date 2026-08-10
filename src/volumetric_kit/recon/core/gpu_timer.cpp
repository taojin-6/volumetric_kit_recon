// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/recon/core/gpu_timer.hpp"

#include <utility>
#include <vector>

#include "vk_physical_device.hpp"
#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/log.hpp"
#include "volumetric_kit/recon/core/vk_result.hpp"

namespace volumetric_kit::recon {
namespace {

// Nanoseconds per timestamp tick, from the driver.
float timestamp_period(VkPhysicalDevice physical) {
  VkPhysicalDeviceProperties props{};
  vkGetPhysicalDeviceProperties(physical, &props);
  return props.limits.timestampPeriod;
}

// How many low bits of a timestamp written on @p family are meaningful.
//
// Per *family*, not per device: `timestampComputeAndGraphics` says only that
// every graphics-and-compute family supports them, and recon may be handed a
// compute-only family by an embedder's bootstrap (the two-family queue plan
// does exactly that on a discrete GPU). Zero is a legitimate answer and the
// reason GpuTimer degrades rather than failing.
std::uint32_t timestamp_valid_bits(VkPhysicalDevice physical,
                                   std::uint32_t family) {
  const std::vector<VkQueueFamilyProperties> families =
      detail::queue_families(physical);
  return family < families.size() ? families[family].timestampValidBits : 0;
}

}  // namespace

GpuTimer::GpuTimer(GpuTimer&& other) noexcept
    : device_(other.device_),
      pool_(std::move(other.pool_)),
      max_spans_(other.max_spans_),
      valid_bits_(other.valid_bits_),
      period_ns_(other.period_ns_),
      warned_full_(other.warned_full_),
      spans_(std::move(other.spans_)),
      readback_(std::move(other.readback_)) {
  other.clear_state();
}

GpuTimer& GpuTimer::operator=(GpuTimer&& other) noexcept {
  if (this != &other) {
    // The pool's own move-assign frees whatever this held before adopting.
    device_ = other.device_;
    pool_ = std::move(other.pool_);
    max_spans_ = other.max_spans_;
    valid_bits_ = other.valid_bits_;
    period_ns_ = other.period_ns_;
    warned_full_ = other.warned_full_;
    spans_ = std::move(other.spans_);
    readback_ = std::move(other.readback_);
    other.clear_state();
  }
  return *this;
}

void GpuTimer::clear_state() noexcept {
  device_ = VK_NULL_HANDLE;
  max_spans_ = 0;
  valid_bits_ = 0;
  period_ns_ = 0.0f;
  warned_full_ = false;
  spans_.clear();
  readback_.clear();
}

Result<GpuTimer> GpuTimer::create(const Device& device,
                                  std::uint32_t max_spans) {
  if (device.handle() == VK_NULL_HANDLE ||
      device.physical_device() == VK_NULL_HANDLE) {
    return Status::invalid_argument("GpuTimer::create: device is not valid");
  }
  // Bounded above as well as below: `max_spans * 2` is the pool's uint32 query
  // count, so an unchecked value wraps into a pool smaller than the bound
  // begin() enforces -- see kMaxSpans.
  if (max_spans == 0 || max_spans > kMaxSpans) {
    return Status::invalid_argument(
        "GpuTimer::create: max_spans must be in [1, kMaxSpans]");
  }

  GpuTimer timer;
  timer.device_ = device.handle();
  timer.max_spans_ = max_spans;
  timer.valid_bits_ =
      timestamp_valid_bits(device.physical_device(), device.compute_family());
  timer.period_ns_ = timestamp_period(device.physical_device());

  // No usable counter: return a valid-but-unavailable timer rather than an
  // error. A diagnostic that can fail a scan is worse than one that reports
  // nothing -- see the class note.
  if (timer.valid_bits_ == 0 || timer.period_ns_ <= 0.0f) {
    return timer;
  }

  VkQueryPoolCreateInfo info{};
  info.sType = VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO;
  info.queryType = VK_QUERY_TYPE_TIMESTAMP;
  // Two queries per span: the start and the end.
  info.queryCount = max_spans * 2;
  VkQueryPool pool = VK_NULL_HANDLE;
  const VkResult created =
      vkCreateQueryPool(device.handle(), &info, nullptr, &pool);
  if (created != VK_SUCCESS) {
    // Degrade rather than propagate -- the availability rule, applied to the
    // one remaining way this can fail. Each tier creates its timer inside its
    // own create(), so returning an error here would refuse to construct the
    // reconstruction spine for a caller who never asked for timing, because a
    // diagnostic could not get a few hundred bytes. Logged, because the reason
    // every device row is missing must not be guessable only from its absence.
    log_message(LogLevel::Warning,
                "GpuTimer::create: " +
                    vk_error(created, "vkCreateQueryPool").message() +
                    "; device timings are unavailable");
    timer.valid_bits_ = 0;
    return timer;
  }
  timer.pool_ =
      UniqueHandle<VkQueryPool, vkDestroyQueryPool>(device.handle(), pool);
  // Sized once here so recording and resolving a span -- which happen inside
  // every dispatch -- never allocate.
  timer.spans_.reserve(max_spans);
  timer.readback_.reserve(static_cast<std::size_t>(max_spans) * 4);
  return timer;
}

std::uint32_t GpuTimer::begin(VkCommandBuffer cmd, const char* name) {
  if (!available() || cmd == VK_NULL_HANDLE) return kNoSpan;
  if (spans_.size() >= max_spans_) {
    // Refusing silently is how a timer whose window nothing ends stops timing
    // partway through a run while every call still returns OK, and the GPU
    // column simply freezes on stale values.
    if (!warned_full_) {
      warned_full_ = true;
      log_message(LogLevel::Warning,
                  "GpuTimer: window is full (max_spans reached); further spans "
                  "go untimed until report_into or reset ends it");
    }
    return kNoSpan;
  }
  const auto span = static_cast<std::uint32_t>(spans_.size());
  spans_.push_back(Span{name != nullptr ? name : "gpu", 0.0, false});

  // Reset this span's pair inside the command buffer rather than the whole pool
  // up front. Two reasons: `vkCmdResetQueryPool` must not run while a query it
  // covers is in flight, and resetting only what is about to be written keeps
  // spans already recorded into an *earlier* command buffer of the same window
  // readable.
  vkCmdResetQueryPool(cmd, pool_.get(), span * 2, 2);
  // BOTTOM_OF_PIPE on the *start* write too, not TOP_OF_PIPE. TOP_OF_PIPE
  // specifies no stage of execution in the first scope, so it orders against
  // nothing earlier: several spans in one command buffer would each latch near
  // the front of the buffer while each end waited for all prior work, nesting
  // rather than tiling and double-counting the total. On the shared-device seam
  // it can also latch while another library's submission is still draining, and
  // charge that tail to recon. Bottom-of-pipe makes the start "after everything
  // recorded so far", which is the only reading that composes.
  vkCmdWriteTimestamp(cmd, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, pool_.get(),
                      span * 2);
  return span;
}

void GpuTimer::end(VkCommandBuffer cmd, std::uint32_t span) {
  if (!available() || cmd == VK_NULL_HANDLE || span == kNoSpan ||
      span >= spans_.size()) {
    return;
  }
  vkCmdWriteTimestamp(cmd, VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT, pool_.get(),
                      span * 2 + 1);
}

void GpuTimer::abandon() noexcept {
  // valid_bits_ is what available() reads, so zeroing it retires the pool
  // without disturbing valid() or the handle destruction still owes.
  valid_bits_ = 0;
  end_window();
}

void GpuTimer::discard(std::uint32_t span) noexcept {
  if (span == kNoSpan || spans_.empty()) return;
  // Only the newest span can be dropped: an earlier one's queries are still
  // indexed by position, so removing it would renumber every span after it.
  if (static_cast<std::size_t>(span) + 1 != spans_.size()) return;
  if (spans_.back().resolved) return;
  spans_.pop_back();
}

Status GpuTimer::resolve() {
  if (!available()) return {};

  // Read only from the first still-unresolved span. submit_single_time resolves
  // after every dispatch, so re-reading the whole window each time would make
  // the readback quadratic in the window's length and re-convert values that
  // are already final.
  std::size_t first = 0;
  while (first < spans_.size() && spans_[first].resolved) ++first;
  if (first == spans_.size()) return {};

  // Availability bit alongside each value, and no WAIT flag. The caller's
  // contract is that the fence has signalled, so the results are ready -- but
  // asking the driver to block here would turn a missed contract into a hang
  // inside a diagnostic, where reporting "not measured" is the right failure.
  // A span begun and never ended is the other case this covers: its end query
  // is unwritten, reads back unavailable, and is skipped.
  const std::uint32_t first_query = static_cast<std::uint32_t>(first) * 2;
  const std::uint32_t query_count =
      static_cast<std::uint32_t>(spans_.size() - first) * 2;
  readback_.assign(static_cast<std::size_t>(query_count) * 2, 0);
  const VkResult got = vkGetQueryPoolResults(
      device_, pool_.get(), first_query, query_count,
      readback_.size() * sizeof(std::uint64_t), readback_.data(),
      sizeof(std::uint64_t) * 2,
      VK_QUERY_RESULT_64_BIT | VK_QUERY_RESULT_WITH_AVAILABILITY_BIT);
  // VK_NOT_READY is not a failure: it means some query in the range has no
  // result yet, and the per-query availability bits below say which.
  if (got != VK_SUCCESS && got != VK_NOT_READY) {
    return vk_error(got, "vkGetQueryPoolResults");
  }

  for (std::size_t i = first; i < spans_.size(); ++i) {
    const std::size_t at = (i - first) * 4;
    const std::uint64_t begin_value = readback_[at + 0];
    const std::uint64_t begin_avail = readback_[at + 1];
    const std::uint64_t end_value = readback_[at + 2];
    const std::uint64_t end_avail = readback_[at + 3];
    if (begin_avail == 0 || end_avail == 0) {
      continue;  // stays unresolved, and report_into skips it
    }
    spans_[i].ms = ticks_to_ms(
        timestamp_delta(begin_value, end_value, valid_bits_), period_ns_);
    spans_[i].resolved = true;
  }
  return {};
}

void GpuTimer::report_into(StageMetrics& out) {
  for (const Span& span : spans_) {
    if (span.resolved) out.add_gpu(span.name, span.ms);
  }
  // Publishing ends the window -- see the header. Without this the next publish
  // would re-add every span already reported, and `max_spans` would be a
  // lifetime bound rather than a per-window one.
  end_window();
}

}  // namespace volumetric_kit::recon
