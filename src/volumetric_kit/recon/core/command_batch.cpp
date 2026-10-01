// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/recon/core/command_batch.hpp"

#include <algorithm>
#include <cstring>
#include <string>
#include <utility>

#include "vk_physical_device.hpp"
#include "volumetric_kit/recon/core/allocator.hpp"
#include "volumetric_kit/recon/core/compute_kernel.hpp"
#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/gpu_timer.hpp"
#include "volumetric_kit/recon/core/image.hpp"
#include "volumetric_kit/recon/core/log.hpp"

namespace volumetric_kit::recon {
namespace {

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

Status has_usage_image(const Image& image) {
  if ((image.usage() & VK_IMAGE_USAGE_TRANSFER_SRC_BIT) == 0) {
    return Status::invalid_argument(
        "CommandBatch: copy needs a TRANSFER_SRC image");
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

CommandBatch::CommandBatch(const Device& device) noexcept
    : device_(&device), allocator_(nullptr) {
  if (device.handle() == VK_NULL_HANDLE) {
    status_ =
        Status::invalid_argument("CommandBatch: the device is moved-from");
  }
}

CommandBatch::CommandBatch(const Device& device, Allocator& allocator) noexcept
    : CommandBatch(device) {
  allocator_ = &allocator;
  if (status_.ok() && !allocator.valid()) {
    status_ =
        Status::invalid_argument("CommandBatch: the allocator is moved-from");
  }
}

// Staging still held here never reached the device, or its submit failed
// before the device had it: a successful submit frees its own, and a failed
// wait leaks it.
CommandBatch::~CommandBatch() = default;

CommandBatch::CommandBatch(CommandBatch&& other) noexcept
    : device_(std::exchange(other.device_, nullptr)),
      allocator_(std::exchange(other.allocator_, nullptr)),
      ops_(std::move(other.ops_)),
      staging_(std::move(other.staging_)),
      status_(std::move(other.status_)),
      submitted_(std::exchange(other.submitted_, false)) {
  other.ops_.clear();
  other.staging_.clear();
  other.status_ = Status{};
}

CommandBatch& CommandBatch::operator=(CommandBatch&& other) noexcept {
  if (this != &other) {
    device_ = std::exchange(other.device_, nullptr);
    allocator_ = std::exchange(other.allocator_, nullptr);
    ops_ = std::move(other.ops_);
    staging_ = std::move(other.staging_);
    status_ = std::move(other.status_);
    submitted_ = std::exchange(other.submitted_, false);
    other.ops_.clear();
    other.staging_.clear();
    other.status_ = Status{};
  }
  return *this;
}

Status CommandBatch::check(Status status) {
  if (!status.ok() && status_.ok()) {
    status_ = status;
  }
  return status;
}

Status CommandBatch::usable() const {
  if (device_ == nullptr) {
    return Status::invalid_argument("CommandBatch: the batch is moved-from");
  }
  if (!status_.ok()) return status_;
  if (submitted_) {
    return Status::invalid_argument("CommandBatch: already submitted");
  }
  return {};
}

Result<const Buffer*> CommandBatch::stage(VkDeviceSize bytes, bool upload) {
  if (allocator_ == nullptr) {
    return Status::invalid_argument(
        "CommandBatch: the batch has no allocator to stage through");
  }
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
                            const void* src, VkDeviceSize bytes,
                            GpuStageScope* stage) {
  VR_TRY(check(usable()));
  if (bytes == 0) return {};
  if (src == nullptr) {
    return check(Status::invalid_argument("CommandBatch::upload: src is null"));
  }
  if (bytes > kMaxInlineUpload || offset % 4 != 0 || bytes % 4 != 0) {
    VR_ASSIGN(void* staging, reserve_upload(dst, offset, bytes, stage));
    std::memcpy(staging, src, static_cast<std::size_t>(bytes));
    return {};
  }
  // Inline: the bytes ride in the command buffer, and no staging exists.
  VR_TRY(check(in_range(dst, offset, bytes, "upload")));
  VR_TRY(check(has_usage(dst, VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                         "upload needs a TRANSFER_DST buffer")));
  Op op;
  op.kind = Kind::Update;
  op.dst = dst.handle();
  op.dst_offset = offset;
  op.bytes = bytes;
  const auto* from = static_cast<const unsigned char*>(src);
  op.data.assign(from, from + bytes);
  op.stage = stage;
  ops_.push_back(std::move(op));
  return {};
}

Result<void*> CommandBatch::reserve_upload(const Buffer& dst,
                                           VkDeviceSize offset,
                                           VkDeviceSize bytes,
                                           GpuStageScope* stage) {
  VR_TRY(check(usable()));
  if (bytes == 0) {
    return check(
        Status::invalid_argument("CommandBatch::reserve_upload: bytes is 0"));
  }
  VR_TRY(check(in_range(dst, offset, bytes, "upload")));
  VR_TRY(check(has_usage(dst, VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                         "upload needs a TRANSFER_DST buffer")));
  Result<const Buffer*> staged = this->stage(bytes, /*upload=*/true);
  if (!staged) return check(staged.status());
  Op op;
  op.kind = Kind::Copy;
  op.src = staged.value()->handle();
  op.dst = dst.handle();
  op.dst_offset = offset;
  op.bytes = bytes;
  op.stage = stage;
  op.staged = true;
  ops_.push_back(std::move(op));
  return staged.value()->mapped();
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

Status CommandBatch::zero(const Buffer& dst, VkDeviceSize offset,
                          VkDeviceSize bytes) {
  static constexpr unsigned char kZeros[4] = {};
  VR_TRY(check(usable()));
  if (bytes == 0) return {};
  VR_TRY(check(in_range(dst, offset, bytes, "zero")));
  const VkDeviceSize end = offset + bytes;
  const VkDeviceSize first_word =
      std::min((offset + 3) & ~VkDeviceSize{3}, end);
  const VkDeviceSize last_word = std::max(end & ~VkDeviceSize{3}, first_word);
  VR_TRY(upload(dst, offset, kZeros, first_word - offset));
  VR_TRY(fill(dst, first_word, last_word - first_word, 0u));
  return upload(dst, last_word, kZeros, end - last_word);
}

Status CommandBatch::copy(const Buffer& src, VkDeviceSize src_offset,
                          const Buffer& dst, VkDeviceSize dst_offset,
                          VkDeviceSize bytes, GpuStageScope* stage) {
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
  op.stage = stage;
  ops_.push_back(std::move(op));
  return {};
}

Status CommandBatch::copy(const Image& src, std::uint32_t width,
                          std::uint32_t height, const Buffer& dst,
                          VkDeviceSize dst_offset, GpuStageScope* stage) {
  VR_TRY(check(usable()));
  if (!src.valid()) {
    return check(
        Status::invalid_argument("CommandBatch::copy: the image is empty"));
  }
  VkDeviceSize texel = 0;
  if (src.format() == VK_FORMAT_R8_UNORM) texel = 1;
  if (src.format() == VK_FORMAT_R8G8_UNORM) texel = 2;
  if (texel == 0) {
    return check(Status::invalid_argument(
        "CommandBatch::copy: copies R8_UNORM and R8G8_UNORM images only"));
  }
  if (width == 0 || height == 0 || width > src.width() ||
      height > src.height()) {
    return check(Status::invalid_argument(
        "CommandBatch::copy: the region is empty or past the image"));
  }
  if (src.layout() != VK_IMAGE_LAYOUT_GENERAL &&
      src.layout() != VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL) {
    return check(Status::invalid_argument(
        "CommandBatch::copy: the image is in a layout a copy cannot read"));
  }
  VR_TRY(check(has_usage_image(src)));
  if (dst_offset % 4 != 0) {
    return check(Status::invalid_argument(
        "CommandBatch::copy: the destination offset is not a multiple of 4"));
  }
  // The image's size is its maker's word, so the bytes may not fit 64 bits.
  if (VkDeviceSize{width} * height > ~VkDeviceSize{0} / texel) {
    return check(Status::invalid_argument(
        "CommandBatch::copy: the region is past any buffer"));
  }
  const VkDeviceSize bytes = VkDeviceSize{width} * height * texel;
  VR_TRY(check(in_range(dst, dst_offset, bytes, "copy destination")));
  VR_TRY(check(has_usage(dst, VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                         "copy needs a TRANSFER_DST destination")));
  Op op;
  op.kind = Kind::ImageCopy;
  op.image = src.handle();
  op.image_layout = src.layout();
  op.width = width;
  op.height = height;
  op.dst = dst.handle();
  op.dst_offset = dst_offset;
  op.bytes = bytes;
  op.stage = stage;
  ops_.push_back(std::move(op));
  return {};
}

Status CommandBatch::acquire(const Buffer& buffer, std::uint32_t from) {
  VR_TRY(check(usable()));
  if (!buffer.valid()) {
    return check(
        Status::invalid_argument("CommandBatch::acquire: the buffer is empty"));
  }
  const std::uint32_t own = device_->compute_family();
  const bool external = from == VK_QUEUE_FAMILY_EXTERNAL;
  if (!external && from != VK_QUEUE_FAMILY_IGNORED &&
      from >= detail::queue_families(device_->physical_device()).size()) {
    return check(Status::invalid_argument(
        "CommandBatch::acquire: queue family " + std::to_string(from) +
        " is not one of the device's"));
  }
  const bool concurrent = buffer.sharing_mode() == VK_SHARING_MODE_CONCURRENT;
  if (from == VK_QUEUE_FAMILY_IGNORED || from == own ||
      (concurrent && !external)) {
    return {};
  }
  Op op;
  op.kind = Kind::Acquire;
  op.dst = buffer.handle();
  op.value = from;
  // A CONCURRENT buffer is taken from outside Vulkan for every family at
  // once, which Vulkan spells with the destination ignored.
  op.to_family = concurrent ? VK_QUEUE_FAMILY_IGNORED : own;
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
  if (push_size % 4 != 0 || push_size > kernel.push_bytes) {
    return Status::invalid_argument(
        "CommandBatch: push_size is not a multiple of 4 or overruns the "
        "kernel's push-constant range");
  }
  return {};
}

CommandBatch::Op CommandBatch::dispatch_op(
    Kind kind, const ComputeKernel& kernel, const DescriptorSet& set,
    const void* push, std::uint32_t push_size, GpuStageScope* stage) const {
  Op op;
  op.kind = kind;
  op.kernel = &kernel;
  op.set = &set;
  op.set_writes = set.writes();
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
  return dispatch(kernel, kernel.set, push, push_size, groups, max_groups,
                  stage);
}

Status CommandBatch::dispatch(const ComputeKernel& kernel,
                              const DescriptorSet& set, const void* push,
                              std::uint32_t push_size, std::uint32_t groups,
                              std::uint32_t max_groups, GpuStageScope* stage) {
  VR_TRY(check(usable()));
  VR_TRY(check(check_dispatch(kernel, push, push_size)));
  if (!set.valid()) {
    return check(
        Status::invalid_argument("CommandBatch::dispatch: the set is empty"));
  }
  // The guard dispatch() enforces: an oversized 1-D grid is invalid usage on
  // a min-spec driver, and a clamp would drop the tail of the work silently.
  if (groups > max_groups) {
    return check(Status::invalid_argument(
        "CommandBatch::dispatch: workgroup count exceeds the device's "
        "maxComputeWorkGroupCount[0]"));
  }
  Op op = dispatch_op(Kind::Dispatch, kernel, set, push, push_size, stage);
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
  Op op = dispatch_op(Kind::DispatchIndirect, kernel, kernel.set, push,
                      push_size, stage);
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
  if (allocator_ == nullptr) {
    return check(Status::invalid_argument(
        "CommandBatch::readback: the batch has no allocator to read back "
        "through"));
  }
  Op op;
  op.kind = Kind::Readback;
  op.src = src.handle();
  op.src_offset = offset;
  op.bytes = bytes;
  op.host_dst = dst;
  ops_.push_back(std::move(op));
  return {};
}

// Two transfers run unordered unless they share a buffer one of them writes;
// a dispatch is always ordered. A readback's write is left out: its slice of
// the batch's own buffer is touched by no other command.
bool CommandBatch::needs_barrier(std::size_t first, std::size_t i) const {
  const auto dispatches = [](const Op& op) {
    return op.kind == Kind::Dispatch || op.kind == Kind::DispatchIndirect;
  };
  const auto written = [](const Op& op) -> VkBuffer {
    return op.kind == Kind::Readback ? VK_NULL_HANDLE : op.dst;
  };
  const auto touches = [](const Op& op, VkBuffer buffer) {
    return buffer != VK_NULL_HANDLE && (op.src == buffer || op.dst == buffer);
  };
  const Op& b = ops_[i];
  // An acquire is a barrier of its own, and orders what reads its buffer
  // after it.
  if (b.kind == Kind::Acquire) return false;
  // A fill or upload that starts past the end of the one before it, in the
  // same buffer, reads nothing another command writes and writes no byte the
  // run has: the run's first write there was checked against all of it. So
  // zeroing thousands of scattered blocks is one run, and costs no scan of it
  // per fill, and so is staging several images into one buffer. A staged
  // upload reads only its own staging, and a copy from another buffer reads
  // only that, so it joins the run too once nothing in the run writes its
  // source -- which, a run's commands writing only its buffer, is asked of
  // the joining copy alone. An image copy reads an image, which no command in
  // a batch writes.
  const auto plain = [](const Op& op) {
    return op.kind == Kind::Fill || op.kind == Kind::Update || op.staged ||
           (op.kind == Kind::Copy && op.src != op.dst) ||
           op.kind == Kind::ImageCopy;
  };
  if (i > first) {
    const Op& prev = ops_[i - 1];
    if (plain(prev) && plain(b) && prev.dst == b.dst &&
        b.dst_offset >= prev.dst_offset + prev.bytes) {
      bool source_written = false;
      if (b.kind == Kind::Copy && !b.staged) {
        for (std::size_t j = first; j < i && !source_written; ++j) {
          source_written =
              ops_[j].kind != Kind::Acquire && written(ops_[j]) == b.src;
        }
      }
      if (!source_written) return false;
    }
  }
  for (std::size_t j = first; j < i; ++j) {
    const Op& a = ops_[j];
    if (a.kind == Kind::Acquire) continue;
    if (dispatches(a) || dispatches(b) || touches(a, written(b)) ||
        touches(b, written(a))) {
      return true;
    }
  }
  return false;
}

void CommandBatch::record(VkCommandBuffer cmd, std::vector<Span>& spans) const {
  constexpr VkPipelineStageFlags kInnerStages =
      VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_TRANSFER_BIT |
      VK_PIPELINE_STAGE_DRAW_INDIRECT_BIT;
  constexpr VkAccessFlags kInnerAccess =
      VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT |
      VK_ACCESS_TRANSFER_READ_BIT | VK_ACCESS_TRANSFER_WRITE_BIT |
      VK_ACCESS_INDIRECT_COMMAND_READ_BIT;

  std::size_t first = 0;  // the first command since the last barrier
  for (std::size_t i = 0; i < ops_.size(); ++i) {
    const Op& op = ops_[i];
    // Each command sees every write before it -- the ordering one submit per
    // dispatch used to give for free.
    if (i > 0 && needs_barrier(first, i)) {
      barrier(cmd, kInnerStages, kInnerAccess);
      first = i;
    }
    // A kernel's region outside its span, as submit_single_time places them:
    // a label can cost an encoder boundary a span must not measure.
    if (op.kernel != nullptr) device_->begin_debug_label(cmd, op.kernel->name);
    GpuTimer* timer = op.stage != nullptr ? op.stage->timer() : nullptr;
    const std::uint32_t span = timer != nullptr
                                   ? timer->begin(cmd, op.stage->name())
                                   : GpuTimer::kNoSpan;
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
      case Kind::ImageCopy: {
        VkBufferImageCopy region{};
        region.bufferOffset = op.dst_offset;
        region.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
        region.imageExtent = {op.width, op.height, 1};
        vkCmdCopyImageToBuffer(cmd, op.image, op.image_layout, op.dst, 1,
                               &region);
        break;
      }
      case Kind::Acquire: {
        // The writer's release made its writes available, so this makes
        // them visible to everything after it, and waits on nothing before.
        VkBufferMemoryBarrier b{};
        b.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
        b.dstAccessMask = kInnerAccess;
        b.srcQueueFamilyIndex = op.value;
        b.dstQueueFamilyIndex = op.to_family;
        b.buffer = op.dst;
        b.size = VK_WHOLE_SIZE;
        vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
                             kInnerStages, 0, 0, nullptr, 1, &b, 0, nullptr);
        break;
      }
      case Kind::Dispatch:
      case Kind::DispatchIndirect: {
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                          op.kernel->pipeline.handle());
        const VkDescriptorSet set = op.set->handle();
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
        break;
      }
    }
    if (timer != nullptr) timer->end(cmd, span);
    if (span != GpuTimer::kNoSpan) spans.push_back({timer, span});
    if (op.kernel != nullptr) device_->end_debug_label(cmd, op.kernel->name);
  }
  // The last write, visible to the host, to the next batch, and to a renderer
  // drawing the result: at VERTEX_INPUT as vertices and indices, at
  // DRAW_INDIRECT as a command. VERTEX_INPUT needs a graphics family, and
  // recon may sit on a compute-only one (a discrete GPU's async-compute
  // family), where a renderer is reached through a semaphore, which carries
  // the visibility itself (the 2026-08-03 decision).
  VkPipelineStageFlags stages = kInnerStages | VK_PIPELINE_STAGE_HOST_BIT;
  VkAccessFlags access = kInnerAccess | VK_ACCESS_HOST_READ_BIT;
  if ((device_->compute_family_flags() & VK_QUEUE_GRAPHICS_BIT) != 0) {
    stages |= VK_PIPELINE_STAGE_VERTEX_INPUT_BIT;
    access |= VK_ACCESS_VERTEX_ATTRIBUTE_READ_BIT | VK_ACCESS_INDEX_READ_BIT;
  }
  barrier(cmd, stages, access);
}

Status CommandBatch::submit() {
  if (device_ == nullptr) {
    return Status::invalid_argument("CommandBatch: the batch is moved-from");
  }
  if (submitted_) {
    return Status::invalid_argument("CommandBatch: already submitted");
  }
  submitted_ = true;
  if (!status_.ok()) return status_;
  if (ops_.empty()) return {};
  // The set is bound only now, so a write since its dispatch was recorded
  // would run that dispatch on the later binding.
  for (const Op& op : ops_) {
    if (op.set != nullptr && op.set->writes() != op.set_writes) {
      return Status::invalid_argument(
          "CommandBatch::submit: a kernel's descriptor set was rewritten "
          "after its dispatch was recorded");
    }
  }

  // Every readback lands in one host buffer, each at its own slice.
  VkDeviceSize readback_bytes = 0;
  for (Op& op : ops_) {
    if (op.kind != Kind::Readback) continue;
    op.dst_offset = readback_bytes;
    readback_bytes += op.bytes;
  }
  const Buffer* readbacks = nullptr;
  if (readback_bytes > 0) {
    VR_ASSIGN(readbacks, stage(readback_bytes, /*upload=*/false));
    for (Op& op : ops_) {
      if (op.kind == Kind::Readback) op.dst = readbacks->handle();
    }
  }

  std::vector<Span> spans;
  bool in_flight = false;
  const Status submitted = device_->submit_single_time(
      [&](VkCommandBuffer cmd) { record(cmd, spans); }, nullptr, nullptr,
      nullptr, &in_flight);
  if (!submitted.ok()) {
    if (in_flight) {
      // The device may still run the buffer, so what it touches stays, as
      // submit_single_time keeps the buffer: the staging is leaked, and the
      // spans' timers retire with their queries.
      static_cast<void>(new std::vector<Buffer>(std::move(staging_)));
      for (const Span& span : spans) span.timer->abandon();
    } else {
      // Never run: the spans' queries were never written. Newest first, the
      // only order discard takes them in.
      for (auto it = spans.rbegin(); it != spans.rend(); ++it) {
        it->timer->discard(it->id);
      }
    }
    return submitted;
  }
  std::vector<GpuTimer*> timers;
  for (const Span& span : spans) {
    if (std::find(timers.begin(), timers.end(), span.timer) == timers.end()) {
      timers.push_back(span.timer);
    }
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
  staging_.clear();
  return {};
}

}  // namespace volumetric_kit::recon
