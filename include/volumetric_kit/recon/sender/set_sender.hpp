// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file sender/set_sender.hpp
/// @brief A sender's set step: a set's fusion, its free pass and its frame,
///        in two batches.

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/base/stage_metrics.hpp"
#include "volumetric_kit/core/vulkan/command_batch.hpp"
#include "volumetric_kit/core/vulkan/fwd.hpp"
#include "volumetric_kit/core/vulkan/gpu_timer.hpp"
#include "volumetric_kit/recon/sender/codec_stream.hpp"
#include "volumetric_kit/recon/sender/export.hpp"
#include "volumetric_kit/recon/sensor/utils/gpu_frame_prep.hpp"
#include "volumetric_kit/recon/tsdf/fuser.hpp"
#include "volumetric_kit/recon/tsdf/tsdf_integrator.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"

namespace volumetric_kit::recon::sender {

/// @brief What a sender does with a set once its prep is recorded into the
///        set's front batch: fuse it, free stale blocks on schedule, record
///        its frame's front half, submit and wait; then code the frame's rANS
///        half in a batch of its own, wait, and write, serve and decode the
///        frame (@ref CodecStream). A coded set is two submits and two waits,
///        none a readback; a set not coded, one.
///
/// The free pass runs before the frame's front half, so the coder scans the
/// smaller set; the frame is the same either way, since the pass frees only
/// blocks without an observed voxel, which the coder leaves out. A fusion the
/// device refused is done again in the fuser's finish
/// (@ref tsdf::FuseReport::redone), and the frame's front half, which saw the
/// grid without it, again after it (`"wait: redo"`). The frames are the ones
/// @ref CodecStream::code makes of the same grids.
///
/// @code
/// VKC_ASSIGN(sender::SetSender out,
///            sender::SetSender::create(device, fuser, grid, &stream, {}));
/// core::CommandBatch front(device, allocator);
/// front.time(out.timer());
/// VKC_ASSIGN(const sensor::DeviceFrameSet prepared,
///            array.record_process(front, set, &rows));
/// VKC_ASSIGN(const sender::SetSender::Sent sent,
///            out.send(front, prepared.frames, std::move(pictures), index,
///                     pts_ns, &rows));
/// @endcode
class VR_SENDER_API SetSender {
 public:
  /// @brief How a sender fuses, frees and codes its sets.
  struct Config {
    /// The running-average cap (@ref tsdf::Fuser::record_fuse).
    float max_weight = 5.0f;
    /// The fusion's mode (@ref tsdf::Fuser::record_fuse).
    tsdf::IntegrationMode mode = tsdf::IntegrationMode::Classic;
    /// A free pass every `free_after` sets
    /// (@ref volume::VoxelBlockGrid::record_free_stale), of the blocks
    /// neither an allocation nor an observed voxel stamped for more than
    /// `free_after` sets; 0 or less never frees.
    int free_after = 0;
    /// Set i is coded when `(i + 1) % code_every == 0`; 0 or less codes
    /// none.
    int code_every = 1;
  };

  /// @brief What @ref send did with a set.
  struct Sent {
    tsdf::FuseReport fused;  ///< What the fusion did to the grid.
    /// A free pass's refusal, or its settle's; the set goes on.
    core::Status freed;
    /// The set's frame; none for a set not coded.
    std::optional<SentFrame> coded;
  };

  /// @brief A sender over @p fuser, @p grid and @p stream, its front
  ///        batches' timer made on @p device here.
  /// @param device  The device the timer is made on.
  /// @param fuser   The fuser.
  /// @param grid    The volume the sets fuse into.
  /// @param stream  The stream the sets are coded into, or null to fuse
  ///                alone.
  /// @param config  How it fuses, frees and codes.
  /// @return The sender, which borrows @p device, @p fuser, @p grid and
  ///         @p stream (each must outlive it); or the timer's failure.
  static core::Result<SetSender> create(const core::Device& device,
                                        tsdf::Fuser& fuser,
                                        volume::VoxelBlockGrid& grid,
                                        CodecStream* stream, Config config);

  /// @return Whether set @p index is coded: with a stream, every
  ///         @ref Config::code_every th set.
  bool code_due(std::size_t index) const noexcept;

  /// @brief Record the set's fusion, its free pass when due and its frame's
  ///        front half into @p front after its prep, submit it and wait once
  ///        (`"wait: front"`), and finish the fusion; then, for a coded set,
  ///        finish its frame: the back half in a batch of its own and its
  ///        wait (`"wait: frame"`), the write and the serve, and the decode
  ///        back, as @ref CodecStream::code does. The frame is a key when a
  ///        viewer waits for one, its fusion done again or not.
  /// @param front   Holds the set's prep, timed in @ref timer; submitted
  ///                here.
  /// @param frames  The prepared frames; the caller keeps them for as long
  ///                as it likes.
  /// @param inputs  What the prep's commands read (the set's pictures), held
  ///                until @p front has run, then dropped.
  /// @param index   The set's index in the run: its frame's set, above every
  ///                frame's coded before.
  /// @param pts_ns  Its frame's time, no earlier than the last frame's
  ///                (@ref CodecStream::last_pts_ns).
  /// @param rows    Optional: the set's rows -- the device halves of its
  ///                front batch, the fusion's, the free pass's, its frame's
  ///                encode, `"crc"`, `"write"` and `"publish"`, and the
  ///                waits.
  /// @param sent    Optional: runs once the set is published -- for a coded
  ///                set once its frame is written and served, before the
  ///                decode back; for a set not coded, after its fusion.
  /// @return What the set did; `Status::Code::InvalidArgument`, before
  ///         anything is recorded, for a set due a frame of a moved-from
  ///         stream; or the first failure of a call it makes, the fusion's
  ///         and the frame's tokens dropped.
  core::Result<Sent> send(
      core::CommandBatch& front,
      const std::vector<std::optional<sensor::DeviceFrame>>& frames,
      std::shared_ptr<const void> inputs, std::size_t index,
      std::uint64_t pts_ns, core::StageMetrics* rows = nullptr,
      const std::function<void()>& sent = {});

  /// @return The timer a set's front batch spans its stages in, sized for
  ///         them; the back batch spans in the encoder's own.
  core::GpuTimer& timer() noexcept { return timer_; }

 private:
  SetSender(tsdf::Fuser& fuser, volume::VoxelBlockGrid& grid,
            CodecStream* stream, Config config, core::GpuTimer timer);

  tsdf::Fuser* fuser_;
  volume::VoxelBlockGrid* grid_;
  CodecStream* stream_;
  Config config_;
  core::GpuTimer timer_;
};

}  // namespace volumetric_kit::recon::sender
