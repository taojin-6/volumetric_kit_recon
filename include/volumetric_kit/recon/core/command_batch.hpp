// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file core/command_batch.hpp
/// @brief One call's uploads, fills, copies, dispatches and readbacks,
///        recorded into one command buffer and submitted with one fence wait,
///        over staging memory the submitting object keeps between calls.
///
/// The host reaches device memory through this: the kernels' memory is
/// device-local and unmapped on every platform, so the Mac runs the same
/// staged path a discrete GPU does (the 2026-09-28 residency decision).

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

/// @brief Host-visible memory a @ref CommandBatch stages uploads and readbacks
///        through, kept by the object that submits them.
///
/// Grow-only and lazy: it allocates nothing until a batch first needs to
/// stage, grows when a batch needs more, and after a batch that needed more
/// than one allocation it keeps a single one of the combined size, so a call
/// whose traffic is steady allocates nothing after the first. One batch uses
/// it at a time.
///
/// @warning The @ref Allocator passed to @ref create must outlive this.
class VR_CORE_API StagingArena {
 public:
  /// @brief An empty arena over @p allocator.
  /// @param allocator  Where staging memory comes from (must outlive this).
  /// @return The arena, or @ref Status::Code::InvalidArgument for a moved-from
  ///         allocator.
  static Result<StagingArena> create(Allocator& allocator);

  StagingArena() noexcept = default;
  ~StagingArena() = default;
  StagingArena(StagingArena&& other) noexcept;
  StagingArena& operator=(StagingArena&& other) noexcept;
  StagingArena(const StagingArena&) = delete;
  StagingArena& operator=(const StagingArena&) = delete;

  /// @return Bytes of upload staging held (`0` before the first staged
  ///         upload).
  VkDeviceSize upload_capacity() const noexcept;
  /// @return Bytes of readback staging held (`0` before the first staged
  ///         readback).
  VkDeviceSize readback_capacity() const noexcept;
  /// @return `true` if this has an allocator (`false` when moved-from).
  bool valid() const noexcept { return allocator_ != nullptr; }

 private:
  friend class CommandBatch;

  // One allocation of a pool, and how much of it this batch has handed out.
  struct Chunk {
    Buffer buffer;
    VkDeviceSize used = 0;
  };
  // Where `bytes` staged bytes go: a chunk with room, or a new one.
  struct Slice {
    VkBuffer buffer = VK_NULL_HANDLE;
    VkDeviceSize offset = 0;
    void* host = nullptr;
  };
  Result<Slice> take(bool upload, VkDeviceSize bytes);
  // End of a batch. A pool that needed several chunks is freed and remembers
  // their total, so its next chunk holds the whole batch.
  void recycle() noexcept;

  Allocator* allocator_ = nullptr;
  std::vector<Chunk> upload_;
  std::vector<Chunk> readback_;
  VkDeviceSize upload_wanted_ = 0;
  VkDeviceSize readback_wanted_ = 0;
  const Device* device_ = nullptr;  // names new chunks; set by each batch
  bool busy_ = false;
};

/// @brief Records one call's device work -- uploads, fills, copies, dispatches,
///        readbacks -- into a single command buffer, submitted once and waited
///        on once by @ref submit.
///
/// The commands run in the order they were recorded, each finishing its
/// writes before the next reads: a barrier separates every two, and the last
/// makes everything visible to the host and to a renderer drawing the result,
/// as far as the queue family allows (the scope `dispatch()` uses). Kernels
/// keep their debug-utils regions and their @ref GpuStageScope spans.
///
/// **A device-local buffer is always staged; a mapped (host-visible) one skips
/// staging only where that cannot change the result.** An @ref upload into a
/// mapped buffer is a plain copy on the host when nothing has been recorded
/// before it (so no earlier command can touch the bytes); otherwise it is
/// staged and copied in its place. A @ref readback from a mapped buffer reads
/// it directly after the wait when nothing but other readbacks follows it;
/// otherwise it is copied out in its place. So a batch that only touches
/// host-visible buffers needs no submit at all. The usage flags are checked as
/// if staged either way -- `TRANSFER_DST` for an upload, `TRANSFER_SRC` for a
/// readback -- so moving a buffer into device memory later cannot turn up a
/// missing bit.
///
/// **A failed call poisons the batch.** Each recording call returns its own
/// refusal, and @ref submit then returns the first of them and runs nothing,
/// so a batch missing a command is never submitted. An upload that already
/// went straight into a mapped buffer stays written.
///
/// Every @ref Buffer recorded must stay alive until @ref submit returns, and
/// so must every readback destination. The batch borrows its @ref Device and
/// @ref StagingArena, which must outlive it; like @ref
/// Device::submit_single_time it is not thread-safe.
///
/// @code
/// CommandBatch batch(device, staging);
/// VR_TRY(batch.upload(depth_buf, 0, depth, depth_bytes));
/// VR_TRY(batch.dispatch(kernel, &push, sizeof(push), groups, max_groups,
///                       &stage));
/// VR_TRY(batch.readback(counter_buf, 0, sizeof(count), &count));
/// VR_TRY(batch.submit());
/// @endcode
class VR_CORE_API CommandBatch {
 public:
  /// @brief Start a batch on @p device, staging through @p staging.
  ///
  /// A moved-from arena, or one another live batch is using, poisons the
  /// batch: its first call and @ref submit return InvalidArgument.
  CommandBatch(const Device& device, StagingArena& staging) noexcept;
  ~CommandBatch();
  CommandBatch(const CommandBatch&) = delete;
  CommandBatch& operator=(const CommandBatch&) = delete;
  CommandBatch(CommandBatch&&) = delete;
  CommandBatch& operator=(CommandBatch&&) = delete;

  /// @brief Write @p bytes from @p src into @p dst at @p offset.
  ///
  /// @p src is copied before this returns, so it need not outlive the call.
  /// @param dst     Needs `TRANSFER_DST` usage.
  /// @param offset  Byte offset into @p dst.
  /// @param src     The bytes; may be null only when @p bytes is 0.
  /// @param bytes   How many; 0 records nothing.
  /// @return OK, or InvalidArgument for a range past @p dst, a missing usage
  ///         bit, a null @p src or a poisoned batch; or a staging allocation
  ///         failure.
  Status upload(const Buffer& dst, VkDeviceSize offset, const void* src,
                VkDeviceSize bytes);

  /// @brief Set @p bytes of @p dst at @p offset to the repeated word
  ///        @p value (`vkCmdFillBuffer`).
  /// @param dst     Needs `TRANSFER_DST` usage.
  /// @param offset  Byte offset; a multiple of 4.
  /// @param bytes   A multiple of 4; 0 records nothing.
  /// @param value   The 32-bit word written.
  /// @return OK, or InvalidArgument for a misaligned or out-of-range fill, a
  ///         missing usage bit or a poisoned batch.
  Status fill(const Buffer& dst, VkDeviceSize offset, VkDeviceSize bytes,
              std::uint32_t value);

  /// @brief Copy @p bytes from @p src to @p dst on the device.
  /// @param src         Needs `TRANSFER_SRC` usage.
  /// @param src_offset  Byte offset into @p src.
  /// @param dst         Needs `TRANSFER_DST` usage.
  /// @param dst_offset  Byte offset into @p dst.
  /// @param bytes       How many; 0 records nothing.
  /// @return OK, or InvalidArgument for a range past either buffer, ranges
  ///         of one buffer that overlap, a missing usage bit or a poisoned
  ///         batch.
  Status copy(const Buffer& src, VkDeviceSize src_offset, const Buffer& dst,
              VkDeviceSize dst_offset, VkDeviceSize bytes);

  /// @brief Record a 1-D dispatch of @p kernel over @p groups workgroups, as
  ///        `dispatch()` does but in this batch.
  /// @param kernel      A built kernel whose descriptor set is written; it
  ///                    must not be rewritten before @ref submit returns.
  /// @param push        Push-constant bytes, copied here.
  /// @param push_size   Their size; 0 pushes nothing.
  /// @param groups      Workgroups along x.
  /// @param max_groups  The device's `maxComputeWorkGroupCount[0]`.
  /// @param stage       Optional span around this dispatch, as `dispatch()`;
  ///                    it must outlive @ref submit, which records the span.
  /// @return OK, or InvalidArgument for @p groups past @p max_groups, a null
  ///         @p push with a size, an invalid kernel or a poisoned batch.
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
  /// @return OK, or InvalidArgument for a misaligned or out-of-range command,
  ///         a missing usage bit, a null @p push with a size, an invalid
  ///         kernel or a poisoned batch.
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
  /// @return OK, or InvalidArgument for a range past @p src, a missing usage
  ///         bit, a null @p dst or a poisoned batch.
  Status readback(const Buffer& src, VkDeviceSize offset, VkDeviceSize bytes,
                  void* dst);

  /// @brief Submit everything recorded as one command buffer, wait for it,
  ///        and fill every readback destination.
  ///
  /// Submits nothing when nothing needs the device (every upload and readback
  /// went direct), and resolves the spans of every timer a dispatch used. A
  /// batch is submitted at most once, and submitting it releases the arena,
  /// so the next batch may start while this one is still in scope.
  /// @return OK; the first refusal a recording call returned; InvalidArgument
  ///         for a second submit; or a staging or Vulkan failure. A failed
  ///         submit retires the timers it used (@ref GpuTimer::abandon),
  ///         since it cannot tell whether the device may still run them.
  Status submit();

  /// @return `true` once @ref submit has run, whatever it returned.
  bool submitted() const noexcept { return submitted_; }

 private:
  enum class Kind { Copy, Fill, Dispatch, DispatchIndirect, Readback };
  struct Op {
    Kind kind = Kind::Copy;
    VkBuffer src = VK_NULL_HANDLE;
    VkBuffer dst = VK_NULL_HANDLE;
    VkDeviceSize src_offset = 0;
    VkDeviceSize dst_offset = 0;
    VkDeviceSize bytes = 0;
    std::uint32_t value = 0;  // fill word, or workgroup count
    const ComputeKernel* kernel = nullptr;
    std::vector<unsigned char> push;
    GpuStageScope* stage = nullptr;
    const void* mapped_src = nullptr;  // a readback's direct source
    const void* staged = nullptr;      // a staged readback's host copy
    void* host_dst = nullptr;          // a readback's destination
    bool direct = false;               // a readback read without a copy
  };

  Status check(Status status);
  Status check_dispatch(const ComputeKernel& kernel, const void* push,
                        std::uint32_t push_size) const;
  Op dispatch_op(Kind kind, const ComputeKernel& kernel, const void* push,
                 std::uint32_t push_size, GpuStageScope* stage) const;
  void record(VkCommandBuffer cmd) const;
  // Hands the arena back, once: at the end of submit, or on destruction.
  void release() noexcept;

  const Device* device_;
  StagingArena* staging_;
  std::vector<Op> ops_;
  Status status_;
  bool submitted_ = false;
  bool owns_staging_ = false;
};

}  // namespace volumetric_kit::recon
