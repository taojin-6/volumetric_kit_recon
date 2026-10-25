// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/set_codec.hpp
/// @brief The senders' per-set step (fuse_orbbec's and codec_replica's),
///        apart from their sources, so its test runs on every build.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "codec_config.hpp"
#include "codec_stream.hpp"
#include "config.hpp"
#include "fuse_frame.hpp"
#include "set_log.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/base/stage_metrics.hpp"
#include "volumetric_kit/core/vulkan/command_batch.hpp"
#include "volumetric_kit/core/vulkan/gpu_timer.hpp"
#include "volumetric_kit/recon/sensor/utils/gpu_frame_prep.hpp"
#include "volumetric_kit/recon/tsdf/fuser.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"

namespace vr_example {

/// @brief What fuse_orbbec and codec_replica do with a set once its prep is
///        recorded into the set's front batch: fuse it, free stale blocks on
///        schedule, record its frame's front half, submit and wait; then code
///        the frame's rANS half in a batch of its own, wait, and write and
///        publish the frame. A coded set is two submits and two waits, none a
///        readback; a set not coded, one.
///
/// The free pass runs before the frame's front half, so the coder scans the
/// smaller set; the frame is the same either way, since the pass frees only
/// blocks without an observed voxel, which the coder leaves out. A fusion
/// the device refused is done again in the fuser's finish
/// (`FuseReport::redone`), and the frame's front half, which saw the grid
/// without it, again after it.
class SetSender {
 public:
  /// @brief How a sender fuses, frees and codes its sets.
  struct Config {
    float max_weight = 5.0f;  ///< `fusion.max_weight`.
    vr::tsdf::IntegrationMode mode = vr::tsdf::IntegrationMode::Classic;
    /// `fusion.free_after` (@ref free_due); 0 never frees.
    int free_after = 0;
    /// Code set i when `(i + 1) % code_every == 0`; 0 codes none.
    int code_every = 1;
    const char* program = "";  ///< Names a free pass's failure on stderr.
  };

  /// @brief A sender over @p fuser, @p grid and @p stream, its front batches'
  ///        timer made on @p device here.
  /// @param stream  The stream, or null to fuse alone.
  /// @return The sender, which borrows all of @p device, @p fuser, @p grid
  ///         and @p stream (each must outlive it); or the timer's failure.
  static vkc::Result<SetSender> create(const vkc::Device& device,
                                       vr::tsdf::Fuser& fuser,
                                       vr::volume::VoxelBlockGrid& grid,
                                       CodecStream* stream, Config config) {
    VKC_ASSIGN(vkc::GpuTimer timer, vkc::GpuTimer::create(device, kSpans));
    return SetSender(fuser, grid, stream, config, std::move(timer));
  }

  /// @return Whether set @p index is coded.
  bool code_due(std::size_t index) const noexcept {
    return stream_ != nullptr && config_.code_every > 0 &&
           (index + 1) % static_cast<std::size_t>(config_.code_every) == 0;
  }

  /// @brief Record the set's fusion, its free pass when due and its frame's
  ///        front half into @p front after its prep, submit it and wait
  ///        (`"wait: front"`), finish the fusion; then, for a coded set, the
  ///        back half, its wait (`"wait: frame"`), the write and the publish
  ///        (`CodecStream::finish`). The set is published once its frame is,
  ///        or after its wait when it is not coded.
  /// @param front   Holds the set's prep, timed in @ref timer; submitted
  ///                here.
  /// @param frames  The prepared frames.
  /// @param inputs  What the prep's commands read (the set's pictures), held
  ///                until @p front has run, then dropped.
  /// @param index   The set's index in the run: the frame's number, above
  ///                every frame coded before.
  /// @param pts_ns  Its frame's time, no earlier than the last frame's.
  /// @param set     Its row in the run's log.
  /// @return OK; or the first failure of a call it makes, the fusion's and
  ///         the frame's tokens dropped.
  vkc::Status send(
      vkc::CommandBatch& front,
      const std::vector<std::optional<vr::sensor::DeviceFrame>>& frames,
      std::shared_ptr<const void> inputs, std::size_t index,
      std::uint64_t pts_ns, SetLog::Set& set) {
    vkc::StageMetrics& r = set.rows;
    const FuseInputs in = fuse_inputs(frames);
    VKC_ASSIGN(const vr::tsdf::PendingFuse fused,
               fuser_->record_fuse(front, *grid_, in.frames, config_.max_weight,
                                   config_.mode, &r));
    // Housekeeping: a pass refused is printed and the set goes on.
    std::optional<vr::volume::RecordedRounds> freed;
    if (free_due(index + 1, config_.free_after)) {
      vkc::Result<vr::volume::RecordedRounds> pass = grid_->record_free_stale(
          front, static_cast<std::uint32_t>(config_.free_after), &r);
      if (pass.ok()) {
        freed.emplace(std::move(pass).value());
      } else {
        std::fprintf(stderr, "%s: free blocks: %s\n", config_.program,
                     pass.status().message().c_str());
      }
    }
    std::optional<vr::codec::PendingEncode> coded;
    if (code_due(index)) {
      vkc::Result<vr::codec::PendingEncode> recorded =
          stream_->record(front, *grid_);
      if (!recorded.ok()) {
        fuser_->abandon(fused);
        return recorded.status();
      }
      coded = std::move(recorded).value();
    }
    // Prep, fusion, free pass and the frame's front half: one submit.
    const vkc::Status ran = [&]() -> vkc::Status {
      VKC_ASSIGN(vkc::PendingBatch done, front.submit_async({}, {}));
      {
        // Its counts feed the frame's tables on the host.
        const vkc::WaitScope wait(&r, "wait: front");
        VKC_TRY(done.wait());
      }
      if (stream_ != nullptr) {
        stream_->front_ran(done.device_rows(), &r);
      } else {
        r.merge_device(done.device_rows());
      }
      return {};
    }();
    inputs.reset();  // the prep was the pictures' last reader
    if (!ran.ok()) {
      fuser_->abandon(fused);
      return ran;
    }
    VKC_ASSIGN(const vr::tsdf::FuseReport report,
               fuser_->finish(fused, *grid_, &r));
    set.redone = report.redone;
    if (freed) {
      const vkc::Status settled = grid_->map().settle(*freed).status();
      if (!settled.ok()) {
        std::fprintf(stderr, "%s: free blocks: %s\n", config_.program,
                     settled.message().c_str());
      }
    }
    print_fuse_report(report);
    if (!coded) {
      set.published();
      return {};
    }
    if (report.redone) {
      VKC_ASSIGN(coded, stream_->redo(*coded, *grid_));
    }
    return stream_->finish(*coded, index, pts_ns, set);
  }

  /// @return The timer a set's front batch tiles in, sized for its stages;
  ///         the back batch tiles in the encoder's own.
  vkc::GpuTimer& timer() noexcept { return timer_; }

 private:
  // A front batch's tiles: the prep's, the fusion's, the free pass's and the
  // frame's front half's, with room for more batches in flight.
  static constexpr std::uint32_t kSpans = 128;

  SetSender(vr::tsdf::Fuser& fuser, vr::volume::VoxelBlockGrid& grid,
            CodecStream* stream, Config config, vkc::GpuTimer timer)
      : fuser_(&fuser),
        grid_(&grid),
        stream_(stream),
        config_(config),
        timer_(std::move(timer)) {}

  vr::tsdf::Fuser* fuser_;
  vr::volume::VoxelBlockGrid* grid_;
  CodecStream* stream_;
  Config config_;
  vkc::GpuTimer timer_;
};

/// @return @p opt's sender config (@ref SetSender::Config), its free pass's
///         failure printed under @p program: fusion as `fusion` says, and
///         every @p code_every-th set coded.
inline SetSender::Config sender_config(const config::Config& opt,
                                       const char* program, int code_every) {
  SetSender::Config c;
  c.max_weight = opt.fusion.max_weight;
  c.mode = integration_mode(opt.fusion.dynamic);
  c.free_after = opt.fusion.free_after;
  c.code_every = code_every;
  c.program = program;
  return c;
}

/// @return The outputs of @p opt's stream (@ref codec_outputs), decoding each
///         frame only with `output.csv` or `codec.sweep`, which read the
///         decode: a recording or a server alone needs none.
inline CodecOutputs set_codec_outputs(const config::Config& opt) {
  CodecOutputs outputs = codec_outputs(opt.output);
  outputs.decode = !opt.output.csv.empty() || opt.codec.sweep;
  return outputs;
}

/// @brief Refuse to write @p path over a file @p outputs name
///        (@ref check_not_same_file), once they exist.
/// @return OK, or the first refusal.
inline vkc::Status check_apart(const std::string& path,
                               const CodecOutputs& outputs) {
  for (const std::string* output : {&outputs.stream, &outputs.csv}) {
    if (!output->empty()) VKC_TRY(check_not_same_file(path, *output));
  }
  return {};
}

}  // namespace vr_example
