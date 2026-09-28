// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/recon/core/command_batch.hpp"

#include <algorithm>
#include <cstring>
#include <string>
#include <utility>

#include "volumetric_kit/recon/core/allocator.hpp"
#include "volumetric_kit/recon/core/compute_kernel.hpp"
#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/gpu_timer.hpp"
#include "volumetric_kit/recon/core/log.hpp"

namespace volumetric_kit::recon {
namespace {

// Each readback's slice of the batch's one readback buffer starts on a 16-byte
// boundary, so any element a caller reads back is aligned on the host.
constexpr VkDeviceSize kReadbackAlign = 16;

// `bytes` at `offset` lie inside `buffer`, written so neither sum can wrap.
Status in_range(const Buffer& buffer, VkDeviceSize offset, VkDeviceSize bytes,
                const char* what) {
  if (!buffer.valid()) {
    return Status::invalid_argument(std::string("CommandBatch: ") + what +
                                    " buffer is empty");
  }
  if (offset > buffer.size() || bytes > buffer.size() - offset) {
    return Status::invalid_argument(std::string("CommandBatch: ") + what +
                                    " range lies past the buffer");
  }
  return {};
}

Status has_usage(const Buffer& buffer, VkBufferUsageFlags bit,
                 const char* what) {
  if ((buffer.usage() & bit) == 0) {
    return Status::invalid_argument(std::string("CommandBatch: ") + what);
  }
  return {};
}

void barrier(VkCommandBuffer cmd, VkPipelineStageFlags dst_stages,
             VkAccessFlags dst_access) {
  VkMemoryBarrier b{};
  b.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
  b.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT | VK_ACCESS_TRANSFER_WRITE_BIT;
  b.dstAccessMask = dst_access;
  vkCmdPipelineBarrier(
      cmd,
      VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_TRANSFER_BIT,
      dst_stages, 0, 1, &b, 0, nullptr, 0, nullptr);
}

}  // namespace

CommandBatch::CommandBatch(const Device& device, Allocator& allocator) noexcept
    : device_(&device), allocator_(&allocator) {
  if (!allocator.valid()) {
    status_ =
        Status::invalid_argument("CommandBatch: the allocator is moved-from");
  }
}

// The staging buffers go with the batch; its one submit waited, so nothing on
// the device still reads them.
CommandBatch::~CommandBatch() = default;

Status CommandBatch::check(Status status) {
  if (!status.ok() && status_.ok()) {
    status_ = status;
  }
  return status;
}

Status CommandBatch::usable() const {
  if (!status_.ok()) return status_;
  if (submitted_) {
    return Status::invalid_argument("CommandBatch: already submitted");
  }
  return {};
}

Result<const Buffer*> CommandBatch::stage(VkDeviceSize bytes, bool upload) {
  BufferDesc desc;
  desc.size = bytes;
  desc.usage = upload ? VK_BUFFER_USAGE_TRANSFER_SRC_BIT
                      : VK_BUFFER_USAGE_TRANSFER_DST_BIT;
  desc.memory = MemoryUsage::HostVisible;
  desc.mapped = true;
  // Write-combined for an upload; cached for a readback, which the host reads.
  desc.host_access = upload ? HostAccess::SequentialWrite : HostAccess::Random;
  VR_ASSIGN(Buffer buffer, allocator_->create_buffer(desc));
  device_->set_object_name(VK_OBJECT_TYPE_BUFFER,
                           debug_object_handle(buffer.handle()),
                           upload ? "batch.upload" : "batch.readback");
  staging_.push_back(std::move(buffer));
  return &staging_.back();
}

Status CommandBatch::upload(const Buffer& dst, VkDeviceSize offset,
                            const void* src, VkDeviceSize bytes) {
  VR_TRY(check(usable()));
  if (bytes == 0) return {};
  if (src == nullptr) {
    return check(Status::invalid_argument("CommandBatch::upload: src is null"));
  }
  VR_TRY(check(in_range(dst, offset, bytes, "upload")));
  VR_TRY(check(has_usage(dst, VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                         "upload needs a TRANSFER_DST buffer")));
  Op op;
  op.dst = dst.handle();
  op.dst_offset = offset;
  op.bytes = bytes;
  if (bytes <= kMaxInlineUpload && offset % 4 == 0 && bytes % 4 == 0) {
    // Inline: the bytes ride in the command buffer, and no staging exists.
    op.kind = Kind::Update;
    const auto* from = static_cast<const unsigned char*>(src);
    op.data.assign(from, from + bytes);
  } else {
    Result<const Buffer*> staged = stage(bytes, /*upload=*/true);
    if (!staged) return check(staged.status());
    const Buffer* staging = staged.value();
    std::memcpy(staging->mapped(), src, static_cast<std::size_t>(bytes));
    op.kind = Kind::Copy;
    op.src = staging->handle();
  }
  ops_.push_back(std::move(op));
  return {};
}

Status CommandBatch::fill(const Buffer& dst, VkDeviceSize offset,
                          VkDeviceSize bytes, std::uint32_t value) {
  VR_TRY(check(usable()));
  if (bytes == 0) return {};
  if (offset % 4 != 0 || bytes % 4 != 0) {
    return check(Status::invalid_argument(
        "CommandBatch::fill: offset and size must be multiples of 4"));
  }
  VR_TRY(check(in_range(dst, offset, bytes, "fill")));
  VR_TRY(check(has_usage(dst, VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                         "fill needs a TRANSFER_DST buffer")));
  Op op;
  op.kind = Kind::Fill;
  op.dst = dst.handle();
  op.dst_offset = offset;
  op.bytes = bytes;
  op.value = value;
  ops_.push_back(std::move(op));
  return {};
}

Status CommandBatch::copy(const Buffer& src, VkDeviceSize src_offset,
                          const Buffer& dst, VkDeviceSize dst_offset,
                          VkDeviceSize bytes) {
  VR_TRY(check(usable()));
  if (bytes == 0) return {};
  VR_TRY(check(in_range(src, src_offset, bytes, "copy source")));
  VR_TRY(check(in_range(dst, dst_offset, bytes, "copy destination")));
  VR_TRY(check(has_usage(src, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                         "copy needs a TRANSFER_SRC source")));
  VR_TRY(check(has_usage(dst, VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                         "copy needs a TRANSFER_DST destination")));
  // vkCmdCopyBuffer forbids overlapping regions of one buffer.
  if (src.handle() == dst.handle() && src_offset < dst_offset + bytes &&
      dst_offset < src_offset + bytes) {
    return check(Status::invalid_argument(
        "CommandBatch::copy: the ranges overlap in one buffer"));
  }
  Op op;
  op.kind = Kind::Copy;
  op.src = src.handle();
  op.src_offset = src_offset;
  op.dst = dst.handle();
  op.dst_offset = dst_offset;
  op.bytes = bytes;
  ops_.push_back(std::move(op));
  return {};
}

Status CommandBatch::check_dispatch(const ComputeKernel& kernel,
                                    const void* push,
                                    std::uint32_t push_size) const {
  if (!kernel.valid()) {
    return Status::invalid_argument("CommandBatch: the kernel is not built");
  }
  if (push_size > 0 && push == nullptr) {
    return Status::invalid_argument(
        "CommandBatch: push is null with a non-zero push_size");
  }
  return {};
}

CommandBatch::Op CommandBatch::dispatch_op(Kind kind,
                                           const ComputeKernel& kernel,
                                           const void* push,
                                           std::uint32_t push_size,
                                           GpuStageScope* stage) const {
  Op op;
  op.kind = kind;
  op.kernel = &kernel;
  if (push_size > 0) {
    const auto* bytes = static_cast<const unsigned char*>(push);
    op.data.assign(bytes, bytes + push_size);
  }
  op.stage = stage;
  return op;
}

Status CommandBatch::dispatch(const ComputeKernel& kernel, const void* push,
                              std::uint32_t push_size, std::uint32_t groups,
                              std::uint32_t max_groups, GpuStageScope* stage) {
  VR_TRY(check(usable()));
  VR_TRY(check(check_dispatch(kernel, push, push_size)));
  // The guard dispatch() enforces: an oversized 1-D grid is invalid usage on
  // a min-spec driver, and a clamp would drop the tail of the work silently.
  if (groups > max_groups) {
    return check(Status::invalid_argument(
        "CommandBatch::dispatch: workgroup count exceeds the device's "
        "maxComputeWorkGroupCount[0]"));
  }
  Op op = dispatch_op(Kind::Dispatch, kernel, push, push_size, stage);
  op.value = groups;
  ops_.push_back(std::move(op));
  return {};
}

Status CommandBatch::dispatch_indirect(const ComputeKernel& kernel,
                                       const void* push,
                                       std::uint32_t push_size,
                                       const Buffer& args, VkDeviceSize offset,
                                       GpuStageScope* stage) {
  VR_TRY(check(usable()));
  VR_TRY(check(check_dispatch(kernel, push, push_size)));
  if (offset % 4 != 0) {
    return check(Status::invalid_argument(
        "CommandBatch::dispatch_indirect: offset must be a multiple of 4"));
  }
  VR_TRY(check(in_range(args, offset, sizeof(VkDispatchIndirectCommand),
                        "indirect command")));
  VR_TRY(check(has_usage(args, VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT,
                         "dispatch_indirect needs an INDIRECT_BUFFER")));
  Op op = dispatch_op(Kind::DispatchIndirect, kernel, push, push_size, stage);
  op.src = args.handle();
  op.src_offset = offset;
  ops_.push_back(std::move(op));
  return {};
}

Status CommandBatch::readback(const Buffer& src, VkDeviceSize offset,
                              VkDeviceSize bytes, void* dst) {
  VR_TRY(check(usable()));
  if (bytes == 0) return {};
  if (dst == nullptr) {
    return check(
        Status::invalid_argument("CommandBatch::readback: dst is null"));
  }
  VR_TRY(check(in_range(src, offset, bytes, "readback")));
  VR_TRY(check(has_usage(src, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                         "readback needs a TRANSFER_SRC buffer")));
  Op op;
  op.kind = Kind::Readback;
  op.src = src.handle();
  op.src_offset = offset;
  op.bytes = bytes;
  op.host_dst = dst;
  ops_.push_back(std::move(op));
  return {};
}

void CommandBatch::record(VkCommandBuffer cmd) const {
  constexpr VkPipelineStageFlags kInnerStages =
      VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_TRANSFER_BIT |
      VK_PIPELINE_STAGE_DRAW_INDIRECT_BIT;
  constexpr VkAccessFlags kInnerAccess =
      VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT |
      VK_ACCESS_TRANSFER_READ_BIT | VK_ACCESS_TRANSFER_WRITE_BIT |
      VK_ACCESS_INDIRECT_COMMAND_READ_BIT;

  for (std::size_t i = 0; i < ops_.size(); ++i) {
    const Op& op = ops_[i];
    // Each command sees every write before it -- the ordering one submit per
    // dispatch used to give for free.
    if (i > 0) barrier(cmd, kInnerStages, kInnerAccess);
    switch (op.kind) {
      case Kind::Update:
        vkCmdUpdateBuffer(cmd, op.dst, op.dst_offset, op.bytes, op.data.data());
        break;
      case Kind::Copy:
      case Kind::Readback: {
        VkBufferCopy region{};
        region.srcOffset = op.src_offset;
        region.dstOffset = op.dst_offset;
        region.size = op.bytes;
        vkCmdCopyBuffer(cmd, op.src, op.dst, 1, &region);
        break;
      }
      case Kind::Fill:
        vkCmdFillBuffer(cmd, op.dst, op.dst_offset, op.bytes, op.value);
        break;
      case Kind::Dispatch:
      case Kind::DispatchIndirect: {
        // The region outside the span, as submit_single_time places them: a
        // label can cost an encoder boundary a span must not measure.
        device_->begin_debug_label(cmd, op.kernel->name);
        GpuTimer* timer = op.stage != nullptr ? op.stage->timer() : nullptr;
        const std::uint32_t span = timer != nullptr
                                       ? timer->begin(cmd, op.stage->name())
                                       : GpuTimer::kNoSpan;
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                          op.kernel->pipeline.handle());
        const VkDescriptorSet set = op.kernel->set.handle();
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                op.kernel->pipeline.layout(), 0, 1, &set, 0,
                                nullptr);
        if (!op.data.empty()) {
          vkCmdPushConstants(
              cmd, op.kernel->pipeline.layout(), VK_SHADER_STAGE_COMPUTE_BIT, 0,
              static_cast<std::uint32_t>(op.data.size()), op.data.data());
        }
        if (op.kind == Kind::Dispatch) {
          vkCmdDispatch(cmd, op.value, 1, 1);
        } else {
          vkCmdDispatchIndirect(cmd, op.src, op.src_offset);
        }
        if (timer != nullptr) timer->end(cmd, span);
        device_->end_debug_label(cmd, op.kernel->name);
        break;
      }
    }
  }
  // The last write, visible to the host, to the next batch, and to a renderer
  // drawing the result -- the stages dispatch() names, and only as far as this
  // queue family may name them.
  VkPipelineStageFlags stages = kInnerStages | VK_PIPELINE_STAGE_HOST_BIT;
  VkAccessFlags access = kInnerAccess | VK_ACCESS_HOST_READ_BIT;
  if ((device_->compute_family_flags() & VK_QUEUE_GRAPHICS_BIT) != 0) {
    stages |= VK_PIPELINE_STAGE_VERTEX_INPUT_BIT;
    access |= VK_ACCESS_VERTEX_ATTRIBUTE_READ_BIT | VK_ACCESS_INDEX_READ_BIT;
  }
  barrier(cmd, stages, access);
}

Status CommandBatch::submit() {
  if (submitted_) {
    return Status::invalid_argument("CommandBatch: already submitted");
  }
  submitted_ = true;
  if (!status_.ok()) return status_;
  if (ops_.empty()) return {};

  // Every readback lands in one host buffer, each at its own aligned slice.
  VkDeviceSize readback_bytes = 0;
  for (Op& op : ops_) {
    if (op.kind != Kind::Readback) continue;
    op.dst_offset = readback_bytes;
    readback_bytes += (op.bytes + kReadbackAlign - 1) & ~(kReadbackAlign - 1);
  }
  const Buffer* readbacks = nullptr;
  if (readback_bytes > 0) {
    VR_ASSIGN(readbacks, stage(readback_bytes, /*upload=*/false));
    for (Op& op : ops_) {
      if (op.kind == Kind::Readback) op.dst = readbacks->handle();
    }
  }

  std::vector<GpuTimer*> timers;
  for (const Op& op : ops_) {
    GpuTimer* timer = op.stage != nullptr ? op.stage->timer() : nullptr;
    if (timer != nullptr &&
        std::find(timers.begin(), timers.end(), timer) == timers.end()) {
      timers.push_back(timer);
    }
  }
  const Status submitted =
      device_->submit_single_time([this](VkCommandBuffer cmd) { record(cmd); });
  if (!submitted.ok()) {
    // Whether the device may still run the buffer (a failed wait leaks it) is
    // not visible from here, so every span it carries retires with it.
    for (GpuTimer* timer : timers) timer->abandon();
    return submitted;
  }
  for (GpuTimer* timer : timers) {
    const Status resolved = timer->resolve();
    if (!resolved.ok()) {
      log_message(LogLevel::Warning,
                  "CommandBatch::submit: GPU timestamps not resolved (" +
                      resolved.message() + "); the batch itself succeeded");
    }
  }

  if (readbacks != nullptr) {
    const auto* base = static_cast<const unsigned char*>(readbacks->mapped());
    for (const Op& op : ops_) {
      if (op.kind != Kind::Readback) continue;
      std::memcpy(op.host_dst, base + op.dst_offset,
                  static_cast<std::size_t>(op.bytes));
    }
  }
  return {};
}

}  // namespace volumetric_kit::recon
