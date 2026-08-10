// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/recon/core/gpu_timer.hpp"

#include <vector>

#include "vk_physical_device.hpp"
#include "volumetric_kit/recon/core/device.hpp"
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

Result<GpuTimer> GpuTimer::create(const Device& device,
                                  std::uint32_t max_spans) {
  if (device.handle() == VK_NULL_HANDLE ||
      device.physical_device() == VK_NULL_HANDLE) {
    return Status::invalid_argument("GpuTimer::create: device is not valid");
  }
  if (max_spans == 0) {
    return Status::invalid_argument("GpuTimer::create: max_spans must be >= 1");
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
  VR_VK_TRY(vkCreateQueryPool(device.handle(), &info, nullptr, &pool));
  timer.pool_ =
      UniqueHandle<VkQueryPool, vkDestroyQueryPool>(device.handle(), pool);
  return timer;
}

std::uint32_t GpuTimer::begin(VkCommandBuffer cmd, const char* name) {
  if (!available() || cmd == VK_NULL_HANDLE || spans_.size() >= max_spans_) {
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
  vkCmdWriteTimestamp(cmd, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT, pool_.get(),
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

Status GpuTimer::resolve() {
  if (!available() || spans_.empty()) {
    return {};
  }

  // Availability bit alongside each value, and no WAIT flag. The caller's
  // contract is that the fence has signalled, so the results are ready -- but
  // asking the driver to block here would turn a missed contract into a hang
  // inside a diagnostic, where reporting "not measured" is the right failure.
  // A span begun and never ended is the other case this covers: its end query
  // is unwritten, reads back unavailable, and is skipped.
  const std::uint32_t query_count =
      static_cast<std::uint32_t>(spans_.size()) * 2;
  std::vector<std::uint64_t> results(query_count * 2, 0);
  const VkResult got = vkGetQueryPoolResults(
      device_, pool_.get(), 0, query_count,
      results.size() * sizeof(std::uint64_t), results.data(),
      sizeof(std::uint64_t) * 2,
      VK_QUERY_RESULT_64_BIT | VK_QUERY_RESULT_WITH_AVAILABILITY_BIT);
  // VK_NOT_READY is not a failure: it means some query in the range has no
  // result yet, and the per-query availability bits below say which.
  if (got != VK_SUCCESS && got != VK_NOT_READY) {
    return vk_error(got, "vkGetQueryPoolResults");
  }

  for (std::size_t i = 0; i < spans_.size(); ++i) {
    const std::uint64_t begin_value = results[i * 4 + 0];
    const std::uint64_t begin_avail = results[i * 4 + 1];
    const std::uint64_t end_value = results[i * 4 + 2];
    const std::uint64_t end_avail = results[i * 4 + 3];
    if (begin_avail == 0 || end_avail == 0) {
      continue;  // stays unresolved, and report_into skips it
    }
    spans_[i].ms = ticks_to_ms(
        timestamp_delta(begin_value, end_value, valid_bits_), period_ns_);
    spans_[i].resolved = true;
  }
  return {};
}

void GpuTimer::report_into(StageMetrics& out) const {
  for (const Span& span : spans_) {
    if (span.resolved) out.add_gpu(span.name.c_str(), span.ms);
  }
}

}  // namespace volumetric_kit::recon
