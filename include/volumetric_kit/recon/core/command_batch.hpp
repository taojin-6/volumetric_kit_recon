// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file core/command_batch.hpp
/// @brief One call's uploads, fills, copies, dispatches and readbacks,
///        recorded into one command buffer and submitted with one fence wait.
///
/// The host reaches device memory through this and nothing else: the kernels'
/// memory is device-local and unmapped on every platform, so the host only
/// records commands, and the Mac runs the same path a discrete GPU does (the
/// 2026-09-28 residency decision).

#include <cstddef>
#include <cstdint>
#include <vector>

#include "volumetric_kit/recon/core/buffer.hpp"
#include "volumetric_kit/recon/core/export.hpp"
#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/core/vulkan.hpp"

namespace volumetric_kit::recon {

class Allocator;
class Device;
class GpuStageScope;
class GpuTimer;
struct ComputeKernel;

/// @brief Records one call's device work -- uploads, fills, copies, dispatches,
///        readbacks -- into a single command buffer, submitted once and waited
///        on once by @ref submit.
///
/// The commands run in the order they were recorded, each seeing every write
/// before it: a barrier precedes any command that could see an earlier one's
/// writes -- always around a dispatch, and between two transfers when they
/// share a buffer one of them writes -- and the last makes everything visible
/// to the host and to a renderer drawing the result, as far as the queue
/// family allows. Kernels keep their debug-utils regions and their
/// @ref GpuStageScope spans. `dispatch()` is a batch of one dispatch.
///
/// **Host bytes cross only at the edges, and each way has one path.** An
/// @ref upload of up to 64 KiB, 4-byte aligned -- a frame's parameters -- is
/// written inline in the command buffer (`vkCmdUpdateBuffer`); a larger one,
/// such as a depth frame, is copied in through a host-visible staging buffer
/// the batch allocates. A @ref readback is for small results, a block count
/// or a failure tally: every readback of a batch is copied into one small
/// host buffer allocated at @ref submit. Nothing is ever read or written
/// through a mapping of the destination, so a buffer's memory type never
/// changes what a batch does.
///
/// **A failed call poisons the batch.** Each recording call returns its own
/// refusal, and every later call, @ref submit included, returns the first of
/// them and records or runs nothing, so a batch missing a command is never
/// submitted.
///
/// Every @ref Buffer recorded must stay alive until @ref submit returns, and
/// so must every readback destination. If the fence wait fails, the device
/// may still run the batch after that: the batch then keeps its own staging
/// for good, as @ref Device::submit_single_time keeps its command buffer, and
/// the device is best treated as lost. The batch borrows its @ref Device and
/// @ref Allocator, which must outlive it; like @ref
/// Device::submit_single_time it is not thread-safe.
///
/// @code
/// CommandBatch batch(device, allocator);
/// VR_TRY(batch.upload(params_buf, 0, &params, sizeof(params)));
/// VR_TRY(batch.dispatch(kernel, &push, sizeof(push), groups, max_groups,
///                       &stage));
/// VR_TRY(batch.readback(counter_buf, 0, sizeof(count), &count));
/// VR_TRY(batch.submit());
/// @endcode
class VR_CORE_API CommandBatch {
 public:
  /// Largest upload written inline in the command buffer
  /// (`vkCmdUpdateBuffer`'s limit); larger ones are staged.
  static constexpr VkDeviceSize kMaxInlineUpload = 65536;

  /// @brief Start a batch on @p device, allocating staging from
  ///        @p allocator. A moved-from device or allocator poisons it.
  CommandBatch(const Device& device, Allocator& allocator) noexcept;
  /// @brief Start a batch that allocates nothing: a staged upload and a
  ///        readback are refused. What `dispatch()` runs on.
  explicit CommandBatch(const Device& device) noexcept;
  ~CommandBatch();
  CommandBatch(const CommandBatch&) = delete;
  CommandBatch& operator=(const CommandBatch&) = delete;
  CommandBatch(CommandBatch&& other) noexcept;
  CommandBatch& operator=(CommandBatch&& other) noexcept;

  /// @brief Write @p bytes from @p src into @p dst at @p offset.
  ///
  /// Inline when @p bytes is at most @ref kMaxInlineUpload and it and
  /// @p offset are multiples of 4; staged otherwise. @p src is copied before
  /// this returns, so it need not outlive the call.
  /// @param dst     Needs `TRANSFER_DST` usage.
  /// @param offset  Byte offset into @p dst.
  /// @param src     The bytes; may be null only when @p bytes is 0.
  /// @param bytes   How many; 0 records nothing.
  /// @return OK; InvalidArgument for a range past @p dst, a missing usage
  ///         bit, a null @p src, or a staged upload on a batch with no
  ///         allocator; a staging allocation failure; or a poisoned batch's
  ///         first refusal.
  Status upload(const Buffer& dst, VkDeviceSize offset, const void* src,
                VkDeviceSize bytes);

  /// @brief Set @p bytes of @p dst at @p offset to the repeated word
  ///        @p value (`vkCmdFillBuffer`).
  /// @param dst     Needs `TRANSFER_DST` usage.
  /// @param offset  Byte offset; a multiple of 4.
  /// @param bytes   A multiple of 4; 0 records nothing.
  /// @param value   The 32-bit word written.
  /// @return OK; InvalidArgument for a misaligned or out-of-range fill or a
  ///         missing usage bit; or a poisoned batch's first refusal.
  Status fill(const Buffer& dst, VkDeviceSize offset, VkDeviceSize bytes,
              std::uint32_t value);

  /// @brief Copy @p bytes from @p src to @p dst on the device.
  /// @param src         Needs `TRANSFER_SRC` usage.
  /// @param src_offset  Byte offset into @p src.
  /// @param dst         Needs `TRANSFER_DST` usage.
  /// @param dst_offset  Byte offset into @p dst.
  /// @param bytes       How many; 0 records nothing.
  /// @return OK; InvalidArgument for a range past either buffer, ranges of
  ///         one buffer that overlap or a missing usage bit; or a poisoned
  ///         batch's first refusal.
  Status copy(const Buffer& src, VkDeviceSize src_offset, const Buffer& dst,
              VkDeviceSize dst_offset, VkDeviceSize bytes);

  /// @brief Record a 1-D dispatch of @p kernel over @p groups workgroups, as
  ///        `dispatch()` does but in this batch.
  /// @param kernel      A built kernel whose descriptor set is written. The
  ///                    set is bound when @ref submit records, so rewriting
  ///                    it before then makes @ref submit refuse the batch.
  /// @param push        Push-constant bytes, copied here.
  /// @param push_size   Their size: a multiple of 4, at most the kernel's
  ///                    @ref ComputeKernel::push_bytes; 0 pushes nothing.
  /// @param groups      Workgroups along x.
  /// @param max_groups  The device's `maxComputeWorkGroupCount[0]`.
  /// @param stage       Optional span around this dispatch, as `dispatch()`;
  ///                    it must outlive @ref submit, which records the span.
  /// @return OK; InvalidArgument for @p groups past @p max_groups, a null
  ///         @p push with a size, a push size @p kernel does not take, or an
  ///         unbuilt kernel; or a poisoned batch's first refusal.
  Status dispatch(const ComputeKernel& kernel, const void* push,
                  std::uint32_t push_size, std::uint32_t groups,
                  std::uint32_t max_groups, GpuStageScope* stage = nullptr);

  /// @brief Record a dispatch of @p kernel whose workgroup counts the device
  ///        reads from @p args at @p offset (`vkCmdDispatchIndirect`), so a
  ///        count a kernel produced sizes the next without reaching the host.
  /// @param kernel     As @ref dispatch.
  /// @param push       As @ref dispatch.
  /// @param push_size  As @ref dispatch.
  /// @param args       Holds a `VkDispatchIndirectCommand` (three `uint32_t`);
  ///                   needs `INDIRECT_BUFFER` usage. Counts past the
  ///                   device's limits are the writer's to prevent.
  /// @param offset     A multiple of 4.
  /// @param stage      As @ref dispatch.
  /// @return OK; InvalidArgument for a misaligned or out-of-range command, a
  ///         missing usage bit, or a push or kernel @ref dispatch refuses; or
  ///         a poisoned batch's first refusal.
  Status dispatch_indirect(const ComputeKernel& kernel, const void* push,
                           std::uint32_t push_size, const Buffer& args,
                           VkDeviceSize offset, GpuStageScope* stage = nullptr);

  /// @brief Read @p bytes of @p src at @p offset, as they stand at this point
  ///        in the batch, into @p dst once @ref submit has waited.
  /// @param src     Needs `TRANSFER_SRC` usage.
  /// @param offset  Byte offset into @p src.
  /// @param bytes   How many; 0 records nothing.
  /// @param dst     Host memory of at least @p bytes, written by @ref submit;
  ///                it must stay valid until then.
  /// @return OK; InvalidArgument for a range past @p src, a missing usage
  ///         bit, a null @p dst, or a batch with no allocator; or a poisoned
  ///         batch's first refusal.
  Status readback(const Buffer& src, VkDeviceSize offset, VkDeviceSize bytes,
                  void* dst);

  /// @brief Submit everything recorded as one command buffer, wait for it,
  ///        and fill every readback destination.
  ///
  /// Resolves the spans of every timer a dispatch used, and frees the
  /// batch's staging. An empty batch submits nothing. A batch is submitted at
  /// most once.
  /// @return OK; the first refusal a recording call returned; InvalidArgument
  ///         for a second submit, a moved-from batch, or a kernel whose set
  ///         was rewritten after its dispatch was recorded; or a staging or
  ///         Vulkan failure. A submit that fails before the device has the
  ///         buffer drops its spans (`GpuTimer::discard`); one whose wait
  ///         fails retires the timers they belong to (`GpuTimer::abandon`).
  Status submit();

  /// @return `true` once @ref submit has run, whatever it returned.
  bool submitted() const noexcept { return submitted_; }

 private:
  enum class Kind { Update, Copy, Fill, Dispatch, DispatchIndirect, Readback };
  struct Op {
    Kind kind = Kind::Copy;
    VkBuffer src = VK_NULL_HANDLE;
    VkBuffer dst = VK_NULL_HANDLE;
    VkDeviceSize src_offset = 0;
    VkDeviceSize dst_offset = 0;
    VkDeviceSize bytes = 0;
    std::uint32_t value = 0;  // fill word, or workgroup count
    const ComputeKernel* kernel = nullptr;
    std::uint64_t set_writes = 0;     // the kernel's set, when recorded
    std::vector<unsigned char> data;  // push constants, or an inline upload
    GpuStageScope* stage = nullptr;
    void* host_dst = nullptr;  // a readback's destination
  };

  struct Span {
    GpuTimer* timer;
    std::uint32_t id;
  };

  Status check(Status status);
  Status usable() const;
  Status check_dispatch(const ComputeKernel& kernel, const void* push,
                        std::uint32_t push_size) const;
  Op dispatch_op(Kind kind, const ComputeKernel& kernel, const void* push,
                 std::uint32_t push_size, GpuStageScope* stage) const;
  // A host-visible buffer of `bytes`, held until the submit's wait is done.
  Result<const Buffer*> stage(VkDeviceSize bytes, bool upload);
  bool needs_barrier(std::size_t first, std::size_t i) const;
  void record(VkCommandBuffer cmd, std::vector<Span>& spans) const;

  const Device* device_;
  Allocator* allocator_;
  std::vector<Op> ops_;
  std::vector<Buffer> staging_;
  Status status_;
  bool submitted_ = false;
};

}  // namespace volumetric_kit::recon
