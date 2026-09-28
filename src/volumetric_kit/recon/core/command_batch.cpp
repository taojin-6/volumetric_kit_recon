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

// Staged bytes start on a 16-byte boundary: nothing requires it of a buffer
// copy, but it keeps every slice aligned for any element a caller stages.
constexpr VkDeviceSize kSliceAlign = 16;
// The smallest chunk worth an allocation: a few counters per batch should not
// cost one each.
constexpr VkDeviceSize kMinChunk = 64 * 1024;

VkDeviceSize align_up(VkDeviceSize bytes) noexcept {
  return (bytes + kSliceAlign - 1) & ~(kSliceAlign - 1);
}

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

// --- StagingArena ------------------------------------------------------------

Result<StagingArena> StagingArena::create(Allocator& allocator) {
  if (!allocator.valid()) {
    return Status::invalid_argument(
        "StagingArena::create: allocator is moved-from");
  }
  StagingArena arena;
  arena.allocator_ = &allocator;
  return arena;
}

StagingArena::StagingArena(StagingArena&& other) noexcept
    : allocator_(other.allocator_),
      upload_(std::move(other.upload_)),
      readback_(std::move(other.readback_)),
      upload_wanted_(other.upload_wanted_),
      readback_wanted_(other.readback_wanted_),
      device_(other.device_),
      busy_(other.busy_) {
  other.allocator_ = nullptr;
  other.upload_.clear();
  other.readback_.clear();
  other.upload_wanted_ = 0;
  other.readback_wanted_ = 0;
  other.device_ = nullptr;
  other.busy_ = false;
}

StagingArena& StagingArena::operator=(StagingArena&& other) noexcept {
  if (this != &other) {
    allocator_ = other.allocator_;
    upload_ = std::move(other.upload_);
    readback_ = std::move(other.readback_);
    upload_wanted_ = other.upload_wanted_;
    readback_wanted_ = other.readback_wanted_;
    device_ = other.device_;
    busy_ = other.busy_;
    other.allocator_ = nullptr;
    other.upload_.clear();
    other.readback_.clear();
    other.upload_wanted_ = 0;
    other.readback_wanted_ = 0;
    other.device_ = nullptr;
    other.busy_ = false;
  }
  return *this;
}

VkDeviceSize StagingArena::upload_capacity() const noexcept {
  VkDeviceSize total = 0;
  for (const Chunk& chunk : upload_) total += chunk.buffer.size();
  return total;
}

VkDeviceSize StagingArena::readback_capacity() const noexcept {
  VkDeviceSize total = 0;
  for (const Chunk& chunk : readback_) total += chunk.buffer.size();
  return total;
}

Result<StagingArena::Slice> StagingArena::take(bool upload,
                                               VkDeviceSize bytes) {
  std::vector<Chunk>& pool = upload ? upload_ : readback_;
  VkDeviceSize& wanted = upload ? upload_wanted_ : readback_wanted_;
  const VkDeviceSize need = align_up(bytes);
  if (pool.empty() || pool.back().buffer.size() - pool.back().used < need) {
    // A new chunk: at least what was wanted last time, twice the previous
    // chunk, and this slice.
    const VkDeviceSize previous = pool.empty() ? 0 : pool.back().buffer.size();
    BufferDesc desc;
    desc.size = std::max({need, 2 * previous, wanted, kMinChunk});
    desc.usage = upload ? VK_BUFFER_USAGE_TRANSFER_SRC_BIT
                        : VK_BUFFER_USAGE_TRANSFER_DST_BIT;
    desc.memory = MemoryUsage::HostVisible;
    desc.mapped = true;
    // Write-combined for uploads; cached for readbacks, which the host reads.
    desc.host_access =
        upload ? HostAccess::SequentialWrite : HostAccess::Random;
    VR_ASSIGN(Buffer buffer, allocator_->create_buffer(desc));
    if (device_ != nullptr) {
      device_->set_object_name(VK_OBJECT_TYPE_BUFFER,
                               debug_object_handle(buffer.handle()),
                               upload ? "staging.upload" : "staging.readback");
    }
    wanted = 0;
    pool.push_back(Chunk{std::move(buffer), 0});
  }
  Chunk& chunk = pool.back();
  Slice slice;
  slice.buffer = chunk.buffer.handle();
  slice.offset = chunk.used;
  slice.host = static_cast<unsigned char*>(chunk.buffer.mapped()) + chunk.used;
  chunk.used += need;
  return slice;
}

void StagingArena::recycle() noexcept {
  const auto settle = [](std::vector<Chunk>& pool, VkDeviceSize& wanted) {
    if (pool.size() > 1) {
      VkDeviceSize total = 0;
      for (const Chunk& chunk : pool) total += chunk.used;
      wanted = total;
      pool.clear();
    } else if (pool.size() == 1) {
      pool.front().used = 0;
    }
  };
  settle(upload_, upload_wanted_);
  settle(readback_, readback_wanted_);
}

// --- CommandBatch ------------------------------------------------------------

CommandBatch::CommandBatch(const Device& device, StagingArena& staging) noexcept
    : device_(&device), staging_(&staging) {
  if (!staging.valid()) {
    status_ = Status::invalid_argument(
        "CommandBatch: the staging arena is moved-from");
  } else if (staging.busy_) {
    status_ = Status::invalid_argument(
        "CommandBatch: the staging arena is in use by another batch");
  } else {
    staging.busy_ = true;
    staging.device_ = &device;
    owns_staging_ = true;
  }
}

CommandBatch::~CommandBatch() { release(); }

void CommandBatch::release() noexcept {
  // Every submit waited, so nothing on the device still reads the staging.
  if (owns_staging_) {
    staging_->recycle();
    staging_->busy_ = false;
    owns_staging_ = false;
  }
}

Status CommandBatch::check(Status status) {
  if (!status.ok() && status_.ok()) {
    status_ = status;
  }
  return status;
}

Status CommandBatch::upload(const Buffer& dst, VkDeviceSize offset,
                            const void* src, VkDeviceSize bytes) {
  if (!status_.ok()) return status_;
  if (submitted_) {
    return check(Status::invalid_argument("CommandBatch: already submitted"));
  }
  if (bytes == 0) return {};
  if (src == nullptr) {
    return check(Status::invalid_argument("CommandBatch::upload: src is null"));
  }
  VR_TRY(check(in_range(dst, offset, bytes, "upload")));
  VR_TRY(check(has_usage(dst, VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                         "upload needs a TRANSFER_DST buffer")));
  // Straight into the mapping only while nothing is recorded: a command
  // recorded earlier runs after this write, and may touch these bytes.
  if (dst.mapped() != nullptr && ops_.empty()) {
    std::memcpy(static_cast<unsigned char*>(dst.mapped()) + offset, src,
                static_cast<std::size_t>(bytes));
    return {};
  }
  auto slice = staging_->take(/*upload=*/true, bytes);
  if (!slice) return check(slice.status());
  std::memcpy(slice->host, src, static_cast<std::size_t>(bytes));
  Op op;
  op.kind = Kind::Copy;
  op.src = slice->buffer;
  op.src_offset = slice->offset;
  op.dst = dst.handle();
  op.dst_offset = offset;
  op.bytes = bytes;
  ops_.push_back(std::move(op));
  return {};
}

Status CommandBatch::fill(const Buffer& dst, VkDeviceSize offset,
                          VkDeviceSize bytes, std::uint32_t value) {
  if (!status_.ok()) return status_;
  if (submitted_) {
    return check(Status::invalid_argument("CommandBatch: already submitted"));
  }
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
  if (!status_.ok()) return status_;
  if (submitted_) {
    return check(Status::invalid_argument("CommandBatch: already submitted"));
  }
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
    op.push.assign(bytes, bytes + push_size);
  }
  op.stage = stage;
  return op;
}

Status CommandBatch::dispatch(const ComputeKernel& kernel, const void* push,
                              std::uint32_t push_size, std::uint32_t groups,
                              std::uint32_t max_groups, GpuStageScope* stage) {
  if (!status_.ok()) return status_;
  if (submitted_) {
    return check(Status::invalid_argument("CommandBatch: already submitted"));
  }
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
  if (!status_.ok()) return status_;
  if (submitted_) {
    return check(Status::invalid_argument("CommandBatch: already submitted"));
  }
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
  if (!status_.ok()) return status_;
  if (submitted_) {
    return check(Status::invalid_argument("CommandBatch: already submitted"));
  }
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
  if (src.mapped() != nullptr) {
    op.mapped_src = static_cast<const unsigned char*>(src.mapped()) + offset;
  }
  ops_.push_back(std::move(op));
  return {};
}

void CommandBatch::record(VkCommandBuffer cmd) const {
  const bool family_has_graphics =
      (device_->compute_family_flags() & VK_QUEUE_GRAPHICS_BIT) != 0;
  constexpr VkPipelineStageFlags kInnerStages =
      VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_TRANSFER_BIT |
      VK_PIPELINE_STAGE_DRAW_INDIRECT_BIT;
  constexpr VkAccessFlags kInnerAccess =
      VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT |
      VK_ACCESS_TRANSFER_READ_BIT | VK_ACCESS_TRANSFER_WRITE_BIT |
      VK_ACCESS_INDIRECT_COMMAND_READ_BIT;

  bool recorded = false;
  for (const Op& op : ops_) {
    if (op.kind == Kind::Readback && op.direct) continue;
    // Each command sees every write before it -- the ordering one submit per
    // dispatch used to give for free.
    if (recorded) barrier(cmd, kInnerStages, kInnerAccess);
    recorded = true;
    switch (op.kind) {
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
        if (!op.push.empty()) {
          vkCmdPushConstants(
              cmd, op.kernel->pipeline.layout(), VK_SHADER_STAGE_COMPUTE_BIT, 0,
              static_cast<std::uint32_t>(op.push.size()), op.push.data());
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
  if (!recorded) return;
  // The last write, visible to the host, to the next batch, and to a renderer
  // drawing the result -- the stages dispatch() names, and only as far as this
  // queue family may name them.
  VkPipelineStageFlags stages = kInnerStages | VK_PIPELINE_STAGE_HOST_BIT;
  VkAccessFlags access = kInnerAccess | VK_ACCESS_HOST_READ_BIT;
  if (family_has_graphics) {
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
  // On every return below, and after the readbacks are copied out of it.
  struct Release {
    CommandBatch* batch;
    ~Release() { batch->release(); }
  } release_on_return{this};
  if (!status_.ok()) return status_;

  // A readback goes direct when nothing but readbacks follows it: then the
  // mapping holds, after the wait, exactly what a copy in its place would.
  bool gpu_after = false;
  bool needs_device = false;
  for (auto it = ops_.rbegin(); it != ops_.rend(); ++it) {
    Op& op = *it;
    if (op.kind != Kind::Readback) {
      gpu_after = true;
      needs_device = true;
      continue;
    }
    op.direct = op.mapped_src != nullptr && !gpu_after;
    if (!op.direct) {
      VR_ASSIGN(StagingArena::Slice slice,
                staging_->take(/*upload=*/false, op.bytes));
      op.dst = slice.buffer;
      op.dst_offset = slice.offset;
      op.staged = slice.host;
      needs_device = true;
    }
  }

  if (needs_device) {
    std::vector<GpuTimer*> timers;
    for (const Op& op : ops_) {
      GpuTimer* timer = op.stage != nullptr ? op.stage->timer() : nullptr;
      if (timer != nullptr &&
          std::find(timers.begin(), timers.end(), timer) == timers.end()) {
        timers.push_back(timer);
      }
    }
    const Status submitted = device_->submit_single_time(
        [this](VkCommandBuffer cmd) { record(cmd); });
    if (!submitted.ok()) {
      // Whether the device may still run the buffer (a failed wait leaks it)
      // is not visible from here, so every span it carries retires with it.
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
  }

  for (const Op& op : ops_) {
    if (op.kind != Kind::Readback) continue;
    std::memcpy(op.host_dst, op.direct ? op.mapped_src : op.staged,
                static_cast<std::size_t>(op.bytes));
  }
  return {};
}

}  // namespace volumetric_kit::recon
