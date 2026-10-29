// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/recon/sender/set_sender.hpp"

#include <utility>

#include "volumetric_kit/recon/sender/fuse_inputs.hpp"

namespace volumetric_kit::recon::sender {

namespace {

// A front batch's spans: the prep's, the fusion's, the free pass's and the
// frame's front half's, with room for more batches in flight.
constexpr std::uint32_t kSpans = 128;

}  // namespace

core::Result<SetSender> SetSender::create(
    const core::Device& device, tsdf::Fuser& fuser,
    volumetric_kit::grid::volume::VoxelBlockGrid& grid,
    volumetric_kit::grid::codec_stream::CodecStream* stream, Config config) {
  VKC_ASSIGN(core::GpuTimer timer, core::GpuTimer::create(device, kSpans));
  return SetSender(fuser, grid, stream, config, std::move(timer));
}

SetSender::SetSender(tsdf::Fuser& fuser,
                     volumetric_kit::grid::volume::VoxelBlockGrid& grid,
                     volumetric_kit::grid::codec_stream::CodecStream* stream,
                     Config config, core::GpuTimer timer)
    : fuser_(&fuser),
      grid_(&grid),
      stream_(stream),
      config_(config),
      timer_(std::move(timer)) {}

bool SetSender::code_due(std::size_t index) const noexcept {
  return stream_ != nullptr && config_.code_every > 0 &&
         (index + 1) % static_cast<std::size_t>(config_.code_every) == 0;
}

core::Result<SetSender::Sent> SetSender::send(
    core::CommandBatch& front,
    const std::vector<std::optional<sensor::DeviceFrame>>& frames,
    std::shared_ptr<const void> inputs, std::size_t index, std::uint64_t pts_ns,
    core::StageMetrics* rows, const std::function<void()>& sent) {
  // A coded set's frame goes through the stream in halves.
  volumetric_kit::grid::codec_stream::CodecStream* const coder =
      code_due(index) ? stream_ : nullptr;
  if (coder != nullptr && !coder->valid()) {
    return core::Status::invalid_argument(
        "SetSender: the stream is moved from");
  }
  Sent out;
  const FuseInputs in = fuse_inputs(frames);
  VKC_ASSIGN(const tsdf::PendingFuse fused,
             fuser_->record_fuse(front, *grid_, in.frames, config_.max_weight,
                                 config_.mode, rows));
  // Housekeeping: a pass refused is reported and the set goes on.
  std::optional<volumetric_kit::grid::volume::RecordedRounds> freed;
  if (config_.free_after > 0 &&
      (index + 1) % static_cast<std::size_t>(config_.free_after) == 0) {
    core::Result<volumetric_kit::grid::volume::RecordedRounds> pass =
        grid_->record_free_stale(
            front, static_cast<std::uint32_t>(config_.free_after), rows);
    if (pass.ok()) {
      freed.emplace(std::move(pass).value());
    } else {
      out.freed = pass.status();
    }
  }
  std::optional<volumetric_kit::grid::codec::PendingEncode> coded;
  if (coder != nullptr) {
    core::Result<volumetric_kit::grid::codec::PendingEncode> recorded =
        coder->record_front(front, *grid_);
    if (!recorded.ok()) {
      fuser_->abandon(fused);
      return recorded.status();
    }
    coded = std::move(recorded).value();
  }
  // Prep, fusion, free pass and the frame's front half: one submit.
  const core::Status ran = [&]() -> core::Status {
    VKC_ASSIGN(core::PendingBatch done, front.submit_async({}, {}));
    {
      // Its counts feed the frame's tables on the host.
      const core::WaitScope wait(rows, "wait: front");
      VKC_TRY(done.wait());
    }
    if (coder != nullptr) {
      coder->front_ran(done.device_rows(), rows);
    } else if (rows != nullptr) {
      rows->merge_device(done.device_rows());
    }
    return {};
  }();
  inputs.reset();  // the prep was the pictures' last reader
  if (!ran.ok()) {
    fuser_->abandon(fused);
    return ran;
  }
  VKC_ASSIGN(out.fused, fuser_->finish(fused, *grid_, rows));
  if (freed) {
    const core::Status settled = grid_->map().settle(*freed).status();
    if (!settled.ok()) out.freed = settled;
  }
  if (!coded) {
    if (sent) sent();
    return out;
  }
  if (out.fused.redone) {
    VKC_ASSIGN(coded, coder->redo(*coded, *grid_, rows));
  }
  VKC_ASSIGN(out.coded, coder->finish_frame(*coded, index, pts_ns, rows, sent));
  return out;
}

}  // namespace volumetric_kit::recon::sender
