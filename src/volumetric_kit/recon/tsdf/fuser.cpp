// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/recon/tsdf/fuser.hpp"

#include <atomic>
#include <new>
#include <utility>
#include <vector>

#include "volumetric_kit/core/vulkan/compute_util.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/core/vulkan/vk_result.hpp"
#include "volumetric_kit/recon/volume/grid_growth.hpp"

namespace volumetric_kit::recon::tsdf {
namespace {

constexpr std::uint32_t kGrowthRetryTicks = 60;
constexpr int kLockRetries = 4;

bool out_of_memory(const core::Status& status) {
  if (status.domain() == core::Status::Code::OutOfMemory) return true;
  const auto vk = core::vk_result(status);
  return vk && (*vk == VK_ERROR_OUT_OF_DEVICE_MEMORY ||
                *vk == VK_ERROR_OUT_OF_HOST_MEMORY);
}

// A record's serial, unique in the process, so a token names one fuser's
// record and no other's.
std::uint64_t next_serial() noexcept {
  static std::atomic<std::uint64_t> counter{0};
  return counter.fetch_add(1, std::memory_order_relaxed) + 1;
}

}  // namespace

// What finish needs of a recorded set: its frames and fusion, to fuse again;
// whether the fuse was gated on the round; the round's result slot; the
// report so far; and the grid.
struct Fuser::Recorded {
  std::uint64_t serial = 0;
  std::vector<FrameInput> frames;
  float max_weight = 5.0f;
  IntegrationMode mode = IntegrationMode::Classic;
  bool gated = false;
  bool may_grow = true;
  volume::RecordedRounds round;
  FuseReport report;
  const volume::VoxelBlockGrid* grid = nullptr;
};

core::Result<Fuser> Fuser::create(core::Device& device,
                                  core::Allocator& allocator,
                                  FuserConfig config) try {
  if (config.max_grows_per_set < 0) {
    return core::Status::invalid_argument(
        "Fuser::create: max_grows_per_set must be >= 0");
  }
  VKC_ASSIGN(TsdfIntegrator integrator,
             TsdfIntegrator::create(device, allocator));
  Fuser fuser(device, allocator, std::move(integrator), config);
  VKC_ASSIGN(fuser.gate_,
             core::device_storage_buffer(allocator, sizeof(std::uint32_t)));
  device.set_object_name(VK_OBJECT_TYPE_BUFFER,
                         core::debug_object_handle(fuser.gate_.handle()),
                         "tsdf.fuse_gate");
  VKC_ASSIGN(fuser.timer_, core::GpuTimer::create(device));
  return fuser;
} catch (const std::bad_alloc&) {
  return core::Status::out_of_memory("Fuser::create: host allocation failed");
}

Fuser::Fuser(core::Device& device, core::Allocator& allocator,
             TsdfIntegrator integrator, FuserConfig config)
    : device_(&device),
      allocator_(&allocator),
      integrator_(std::move(integrator)),
      config_(config) {}

Fuser::~Fuser() = default;
Fuser::Fuser(Fuser&&) noexcept = default;

core::Result<bool> Fuser::grow(volume::VoxelBlockGrid& grid, FuseReport& report,
                               bool& may_grow, core::StageMetrics* metrics) {
  if (!may_grow || report.grows >= config_.max_grows_per_set) return false;
  core::Status status = volume::grow_grid(grid, 0, metrics);
  if (!status.ok()) {
    if (!out_of_memory(status)) return status;
    // Growing is opportunistic: allocation may still complete in the current
    // grid. Strict mode reports this error only if it cannot.
    report.growth_error = std::move(status);
    may_grow = false;
    if (config_.allow_partial) {
      refused_at_ = grid.grid().num_buckets;
      refused_tick_ = grid.map().tick();
    }
    return false;
  }
  refused_at_ = 0;
  ++report.grows;
  return true;
}

core::Status Fuser::fuse_blocking(volume::VoxelBlockGrid& grid,
                                  const std::vector<FrameInput>& frames,
                                  float max_weight, IntegrationMode mode,
                                  FuseReport& report, bool& may_grow,
                                  core::StageMetrics* metrics) {
  const std::vector<volume::DepthInput> depths(frames.begin(), frames.end());
  int lock_retries = 0;
  for (;;) {
    VKC_ASSIGN(report.dropped, grid.map().allocate_from_depth(
                                   depths, &report.failures, metrics));
    if (report.dropped == 0) break;
    if (report.failures.capacity_limited()) {
      VKC_ASSIGN(const bool grew, grow(grid, report, may_grow, metrics));
      if (grew) continue;
    } else if (!config_.allow_partial && lock_retries++ < kLockRetries) {
      continue;
    }
    if (!config_.allow_partial) {
      if (!report.growth_error.ok()) return report.growth_error;
      return core::Status::out_of_memory(
          "Fuser::fuse: could not allocate the complete truncation band");
    }
    break;
  }
  return integrator_.integrate(grid, frames, max_weight, mode, metrics);
}

core::Result<FuseReport> Fuser::fuse(volume::VoxelBlockGrid& grid,
                                     const std::vector<FrameInput>& frames,
                                     float max_weight, IntegrationMode mode,
                                     core::StageMetrics* metrics) try {
  core::CommandBatch batch(*device_, *allocator_);
  VKC_ASSIGN(const PendingFuse pending,
             record_fuse(batch, grid, frames, max_weight, mode, metrics));
  const core::Status ran = [&]() -> core::Status {
    VKC_ASSIGN(core::PendingBatch done, batch.submit_async({}, {}));
    {
      // The set's wait is its fusion's.
      const core::StageScope fusing(metrics, "integrate");
      VKC_TRY(done.wait());
    }
    if (metrics != nullptr) metrics->merge_device(done.device_rows());
    return {};
  }();
  if (!ran.ok()) {
    abandon(pending);
    return ran;
  }
  return finish(pending, grid, metrics);
} catch (const std::bad_alloc&) {
  return core::Status::out_of_memory("Fuser::fuse: host allocation failed");
}

core::Result<PendingFuse> Fuser::record_fuse(
    core::CommandBatch& batch, volume::VoxelBlockGrid& grid,
    const std::vector<FrameInput>& frames, float max_weight,
    IntegrationMode mode, core::StageMetrics* metrics) try {
  if (!valid()) {
    return core::Status::invalid_argument("Fuser::fuse: moved-from fuser");
  }
  // The fuse's refusals before the round is recorded, or a grow run: a
  // refused set changes neither the batch nor the grid.
  VKC_TRY(integrator_.check_frames(grid, frames));
  // Made before an unfinished record is superseded, so a host allocation
  // failure leaves that token finishable.
  auto recorded = std::make_unique<Recorded>();
  FuseReport& report = recorded->report;
  VKC_ASSIGN(report.load_factor, grid.map().load_factor());
  report.from_buckets = report.to_buckets = grid.grid().num_buckets;
  recorded->frames = frames;
  recorded->max_weight = max_weight;
  recorded->mode = mode;
  recorded->grid = &grid;
  recorded->may_grow =
      !(config_.allow_partial && refused_at_ == grid.grid().num_buckets &&
        std::uint32_t(grid.map().tick() - refused_tick_) < kGrowthRetryTicks);
  pending_.reset();
  if (!frames.empty()) {
    if (report.load_factor > volume::VoxelHashMap::kGrowThreshold) {
      VKC_TRY(grow(grid, report, recorded->may_grow, metrics).status());
    }
    // A set whose fuse needs its whole band waits on the round's gate: a
    // strict fuser's, and the first set of a grid holding no block.
    recorded->gated =
        !config_.allow_partial || grid.map().allocated_blocks() == 0;
    const volume::DeviceCount gate{&gate_, 0, 1};
    const volume::DeviceCount* gated = recorded->gated ? &gate : nullptr;
    const std::vector<volume::DepthInput> depths(frames.begin(), frames.end());
    // One row for the set, the round and the compaction its breakdowns with
    // their device halves alone, as its host time holds theirs. The rows are
    // placed now, the stage's first, so a batch's later stages, whose device
    // halves come back with theirs, do not come between them.
    core::GpuStageScope stage(metrics, timer_, "integrate");
    core::StageMetrics parts;
    if (metrics != nullptr) {
      metrics->add_cpu("integrate", 0.0);
      metrics->add_cpu("  ..allocate", 0.0);
    }
    if (gated != nullptr) {
      VKC_TRY(batch.fill(gate_, 0, sizeof(std::uint32_t), 1u));
    }
    {
      core::GpuStageScope round(metrics != nullptr ? &parts : nullptr, timer_,
                                "  ..allocate");
      VKC_ASSIGN(recorded->round, grid.map().record_allocate_from_depth(
                                      batch, depths, gated, &round));
    }
    VKC_TRY(integrator_.record_integrate(batch, grid, frames, max_weight, mode,
                                         gated, &stage, metrics));
  }
  recorded->serial = next_serial();
  pending_ = std::move(recorded);
  return PendingFuse{pending_->serial};
} catch (const std::bad_alloc&) {
  return core::Status::out_of_memory("Fuser::fuse: host allocation failed");
}

core::Result<FuseReport> Fuser::finish(const PendingFuse& pending,
                                       volume::VoxelBlockGrid& grid,
                                       core::StageMetrics* metrics) try {
  if (!holds(pending)) {
    return core::Status::invalid_argument(
        "Fuser::finish: the pending set is not this fuser's last record "
        "(finished, abandoned, superseded, or another fuser's)");
  }
  if (&grid != pending_->grid) {
    return core::Status::invalid_argument(
        "Fuser::finish: the grid is not the one the set was recorded into");
  }
  const std::unique_ptr<Recorded> set = std::move(pending_);
  FuseReport& report = set->report;
  if (!set->frames.empty()) {
    VKC_ASSIGN(report.failures, grid.map().settle(set->round));
    report.dropped = report.failures.total + report.failures.terminal;
    if (set->gated && report.dropped != 0) {
      // The round left the band incomplete and the fuse ran nothing: the set
      // again, blocking, as a set was fused before it recorded, at the clock
      // it was recorded at -- the fuse's tick taken back -- so its stamps,
      // and every later set's, are a set fused once's.
      report.redone = true;
      grid.map().rewind_tick();
      VKC_TRY(fuse_blocking(grid, set->frames, set->max_weight, set->mode,
                            report, set->may_grow, metrics));
    } else if (report.failures.capacity_limited()) {
      // A partial set fused what the round placed; the next finds room.
      VKC_TRY(grow(grid, report, set->may_grow, metrics).status());
    }
  }
  report.to_buckets = grid.grid().num_buckets;
  VKC_ASSIGN(report.load_factor, grid.map().load_factor());
  return report;
} catch (const std::bad_alloc&) {
  return core::Status::out_of_memory("Fuser::fuse: host allocation failed");
}

void Fuser::abandon(const PendingFuse& pending) noexcept {
  if (holds(pending)) pending_.reset();
}

bool Fuser::holds(const PendingFuse& pending) const noexcept {
  return pending_ != nullptr && pending.serial != 0 &&
         pending.serial == pending_->serial;
}

}  // namespace volumetric_kit::recon::tsdf
