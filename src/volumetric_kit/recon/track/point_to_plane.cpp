// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// FitKind::PointToPlane: projective point-to-plane ICP of K frames against
// one stacked ray-cast view of a grid, in the caller's batch.
//
// Layout. The fitter owns pyramid entries, grown to the largest window it
// has recorded: each holds one frame's 3-level pyramid (depth, vertex and
// normal maps), stacked by entry in one buffer per level, as a ModelView
// stacks views. Every descriptor set but the ray caster's ring is written
// once, at create or when the entries grow, when no batch of this fitter is
// pending; a kernel reaches an entry through its push constants, so a record
// rewrites no set (core refuses a batch whose set was rewritten after it was
// recorded).
//
// A record: for each frame without a committed entry, a copy of its depth
// into a free entry and its pyramid (6 dispatches); the scale (2); then each
// pass: the cast cameras moved to the estimate (after the first), one
// stacked cast of the K views, and for each level, coarse to fine, each
// iteration one reduce over all K views and one solve. One readback of the
// state. Entries a record fills count once its finish returns; an abandon or
// a failed record empties them.

#include "point_to_plane.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <new>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "fit_abi.hpp"
#include "frame_check.hpp"
#include "se3.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/buffer.hpp"
#include "volumetric_kit/core/vulkan/command_batch.hpp"
#include "volumetric_kit/core/vulkan/compute_kernel.hpp"
#include "volumetric_kit/core/vulkan/compute_util.hpp"
#include "volumetric_kit/core/vulkan/descriptor.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/core/vulkan/gpu_timer.hpp"
#include "volumetric_kit/core/vulkan/vulkan.hpp"
#include "volumetric_kit/grid/volume/ray_caster.hpp"
#include "volumetric_kit/grid/volume/voxel_hash_map.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"

#include "track_down_comp.spv.hpp"
#include "track_filter_comp.spv.hpp"
#include "track_pose_comp.spv.hpp"
#include "track_reduce_point_to_plane_comp.spv.hpp"
#include "track_scale_comp.spv.hpp"
#include "track_solve_comp.spv.hpp"
#include "track_vmap_comp.spv.hpp"

namespace volumetric_kit::recon::track::detail {
namespace {

namespace vol = volumetric_kit::grid::volume;

constexpr int kLevels = 3;
constexpr std::uint32_t kLocal = 64;
// The reduce's partials: at most this many workgroups a dispatch.
constexpr std::uint32_t kMaxReduceGroups = 1024;
// About this many pixels a reduce thread, before the group cap.
constexpr std::uint32_t kPixelsPerThread = 8;
// The scale's histogram: 1 cm bins to 5.12 m.
constexpr std::uint32_t kScaleBins = 512;
constexpr float kScaleBin = 0.01f;
// The smallest frame: 3 levels of at least 4 x 4.
constexpr std::uint32_t kMinSize = 16;
// The solve: a step needs this many inliers and this share of the hits; a
// longer one than this (metres or radians) diverged (acc_ref's guards); the
// damping is this share of the mean diagonal.
constexpr std::uint32_t kStepMinInliers = 6;
constexpr float kStepMinRatio = 0.25f;
constexpr float kMaxStep = 0.5f;
constexpr float kDamping = 1e-4f;
// A pass that moves the estimate under this skips the remaining solves.
constexpr float kStopM = 0.01f;
constexpr float kStopDeg = 0.3f;

// FitView, FitState and ReducePush: fit_abi.hpp.

struct FilterPush {
  std::uint32_t pixels, offset;
  float min_depth, max_depth;
};
static_assert(sizeof(FilterPush) == 16, "FilterPush layout drift");

struct DownPush {
  std::uint32_t w_in, h_in, w_out, h_out, in_offset, out_offset;
};
static_assert(sizeof(DownPush) == 24, "DownPush layout drift");

struct VmapPush {
  std::uint32_t width, height, offset;
  std::int32_t edge;
  Vec4f k;
  float cos_graze;
};
static_assert(sizeof(VmapPush) == 36, "VmapPush layout drift");

struct ScalePush {
  std::uint32_t mode, pixels, views, bins;
  float bin_m;
};
static_assert(sizeof(ScalePush) == 20, "ScalePush layout drift");

struct SolvePush {
  std::uint32_t groups, groups_per_view, views, mode, finest;
  float tau, damping, max_step, min_ratio;
  std::uint32_t min_inliers;
};
static_assert(sizeof(SolvePush) == 40, "SolvePush layout drift");

struct PosePush {
  std::uint32_t views;
  float stop_m, stop_rad;
};
static_assert(sizeof(PosePush) == 12, "PosePush layout drift");

VkPushConstantRange push_range(std::uint32_t bytes) {
  VkPushConstantRange r{};
  r.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
  r.offset = 0;
  r.size = bytes;
  return r;
}

// A pyramid entry's frame: what it holds, and whether that counts yet.
struct Entry {
  enum class State : std::uint8_t { Empty, Pending, Committed };
  State state = State::Empty;
  std::weak_ptr<const core::Buffer> depth;  // the frame's depth, weakly
  std::uint64_t pts = 0;
  std::uint64_t used = 0;  // the last record that read it: evict the oldest
};

class PointToPlane final : public IFitter {
 public:
  PointToPlane(core::Device& device, core::Allocator& allocator,
               const FitterConfig& config) noexcept
      : device_(device), allocator_(allocator), config_(config) {}
  ~PointToPlane() override = default;

  core::Status init();

  core::Result<PendingFit> record(
      core::CommandBatch& batch, const vol::VoxelBlockGrid& model,
      const std::vector<sensor::DeviceFrame>& frames, const PosePrior& prior,
      core::StageMetrics* metrics, const FitRows& rows) override;
  core::Result<TrackFit> finish(const PendingFit& pending) override;
  void abandon(const PendingFit& pending) noexcept override;

 private:
  struct Outstanding {
    std::uint64_t serial = 0;
    std::vector<std::uint32_t> filled;    // entries the record fills
    std::vector<std::uint32_t> entry_of;  // each frame's entry
    std::vector<sensor::DeviceFrame> frames;
    PosePrior prior;
    Vec3f origin{0.0f};
  };

  core::Status check(const vol::VoxelBlockGrid& model,
                     const std::vector<sensor::DeviceFrame>& frames,
                     const PosePrior& prior) const;
  // Hold `views` entries of width x height, every set written; a new size
  // drops every entry.
  core::Status ensure(std::uint32_t width, std::uint32_t height,
                      std::uint32_t views);
  void write_sets();
  core::Status record_commands(core::CommandBatch& batch,
                               const vol::VoxelBlockGrid& model,
                               const Outstanding& out,
                               core::StageMetrics* metrics,
                               const FitRows& rows);
  std::uint32_t groups_per_view(int level, std::uint32_t views) const noexcept;
  void empty_filled(const Outstanding& out) noexcept;

  core::Device& device_;
  core::Allocator& allocator_;
  const FitterConfig config_;
  std::uint32_t max_groups_ = 0;

  core::DescriptorPool pool_;
  core::ComputeKernel filter_;
  core::ComputeKernel down_;
  core::ComputeKernel vmap_;
  core::ComputeKernel scale_;
  core::ComputeKernel reduce_;
  core::ComputeKernel solve_;
  core::ComputeKernel pose_;
  core::KernelSets down_sets_;
  core::KernelSets vmap_sets_;
  core::KernelSets reduce_sets_;
  std::unique_ptr<vol::RayCaster> caster_;
  core::GpuTimer timer_;

  // Sized by ensure.
  std::uint32_t w_[kLevels] = {};
  std::uint32_t h_[kLevels] = {};
  std::uint32_t capacity_ = 0;  // entries, and views of each buffer below
  core::Buffer depth_[kLevels];
  core::Buffer vtx_[kLevels];
  core::Buffer nrm_[kLevels];
  core::Buffer views_;
  core::Buffer cams_;
  vol::ModelView view_;
  // Fixed.
  core::Buffer state_;
  core::Buffer partials_;
  core::Buffer bins_;

  std::vector<Entry> entries_;
  std::optional<Outstanding> outstanding_;
  FitState readback_{};
  std::uint64_t next_serial_ = 1;
};

core::Status PointToPlane::init() {
  VKC_TRY(check_device_requirements(device_, "create_fitter"));
  const VkPushConstantRange fp = push_range(sizeof(FilterPush));
  const VkPushConstantRange dp = push_range(sizeof(DownPush));
  const VkPushConstantRange vp = push_range(sizeof(VmapPush));
  const VkPushConstantRange sp = push_range(sizeof(ScalePush));
  const VkPushConstantRange rp = push_range(sizeof(ReducePush));
  const VkPushConstantRange op = push_range(sizeof(SolvePush));
  const VkPushConstantRange pp = push_range(sizeof(PosePush));
  core::KernelSetBuilder kb(device_);
  VKC_TRY(kb.add(filter_, "track_filter", vr_track_filter_comp_spv,
                 vr_track_filter_comp_spv_size, 1, &fp));
  VKC_TRY(kb.add(down_, "track_down", vr_track_down_comp_spv,
                 vr_track_down_comp_spv_size, 2, &dp));
  VKC_TRY(kb.add(vmap_, "track_vmap", vr_track_vmap_comp_spv,
                 vr_track_vmap_comp_spv_size, 3, &vp));
  VKC_TRY(kb.add(scale_, "track_scale", vr_track_scale_comp_spv,
                 vr_track_scale_comp_spv_size, 4, &sp));
  VKC_TRY(kb.add(reduce_, "track_reduce_point_to_plane",
                 vr_track_reduce_point_to_plane_comp_spv,
                 vr_track_reduce_point_to_plane_comp_spv_size, 8, &rp));
  VKC_TRY(kb.add(solve_, "track_solve", vr_track_solve_comp_spv,
                 vr_track_solve_comp_spv_size, 2, &op));
  VKC_TRY(kb.add(pose_, "track_pose", vr_track_pose_comp_spv,
                 vr_track_pose_comp_spv_size, 3, &pp));
  VKC_ASSIGN(pool_, kb.build());
  VKC_TRY(down_sets_.reserve(device_, down_, kLevels - 1));
  VKC_TRY(vmap_sets_.reserve(device_, vmap_, kLevels));
  VKC_TRY(reduce_sets_.reserve(device_, reduce_, kLevels));
  vol::RayCasterConfig cast;
  cast.casts_per_batch = std::uint32_t(config_.icp.passes);
  VKC_ASSIGN(caster_, vol::RayCaster::create(device_, allocator_, cast));
  VKC_ASSIGN(timer_, core::GpuTimer::create(device_));
  VkPhysicalDeviceProperties props{};
  vkGetPhysicalDeviceProperties(device_.physical_device(), &props);
  max_groups_ = props.limits.maxComputeWorkGroupCount[0];
  VKC_ASSIGN(state_, core::device_storage_buffer(allocator_, sizeof(FitState)));
  VKC_ASSIGN(partials_,
             core::device_storage_buffer(
                 allocator_, VkDeviceSize(kMaxReduceGroups) * kPartial * 4));
  VKC_ASSIGN(bins_, core::device_storage_buffer(allocator_,
                                                VkDeviceSize(kScaleBins) * 4));
  return {};
}

core::Status PointToPlane::ensure(std::uint32_t width, std::uint32_t height,
                                  std::uint32_t views) {
  if (width == w_[0] && height == h_[0] && views <= capacity_) return {};
  // TODO: keep the committed entries when the stack grows; a window's first
  // hops (K = 1, 5, 10, ...) now reallocate and rebuild every pyramid.
  const bool resized = width != w_[0] || height != h_[0];
  const std::uint32_t capacity = resized ? views : std::max(views, capacity_);
  // Drop the old buffers first, so a grow never holds both.
  for (int l = 0; l < kLevels; ++l) {
    depth_[l] = {};
    vtx_[l] = {};
    nrm_[l] = {};
  }
  views_ = {};
  cams_ = {};
  view_ = {};
  entries_.assign(capacity, Entry{});
  capacity_ = 0;
  w_[0] = width;
  h_[0] = height;
  for (int l = 1; l < kLevels; ++l) {
    w_[l] = w_[l - 1] / 2;
    h_[l] = h_[l - 1] / 2;
  }
  for (int l = 0; l < kLevels; ++l) {
    const VkDeviceSize px = VkDeviceSize(w_[l]) * h_[l] * capacity;
    VKC_ASSIGN(depth_[l], core::device_storage_buffer(allocator_, px * 4));
    VKC_ASSIGN(vtx_[l], core::device_storage_buffer(allocator_, px * 16));
    VKC_ASSIGN(nrm_[l], core::device_storage_buffer(allocator_, px * 16));
  }
  VKC_ASSIGN(views_, core::device_storage_buffer(
                         allocator_, VkDeviceSize(capacity) * sizeof(FitView)));
  VKC_ASSIGN(cams_, core::device_storage_buffer(
                        allocator_,
                        VkDeviceSize(capacity) * sizeof(vol::DepthPinhole)));
  VKC_ASSIGN(view_, caster_->make_view(width, height, capacity));
  capacity_ = capacity;
  write_sets();
  return {};
}

void PointToPlane::write_sets() {
  const auto bind = [](const core::DescriptorSet& set, std::uint32_t binding,
                       const core::Buffer& buffer) {
    set.write_storage_buffer(binding, buffer.handle(), 0, VK_WHOLE_SIZE);
  };
  bind(filter_.set, 0, depth_[0]);
  for (int l = 0; l + 1 < kLevels; ++l) {
    bind(down_sets_[std::size_t(l)], 0, depth_[l]);
    bind(down_sets_[std::size_t(l)], 1, depth_[l + 1]);
  }
  for (int l = 0; l < kLevels; ++l) {
    const core::DescriptorSet& v = vmap_sets_[std::size_t(l)];
    bind(v, 0, depth_[l]);
    bind(v, 1, vtx_[l]);
    bind(v, 2, nrm_[l]);
    const core::DescriptorSet& r = reduce_sets_[std::size_t(l)];
    bind(r, 0, vtx_[l]);
    bind(r, 1, nrm_[l]);
    bind(r, 2, view_.depth);
    bind(r, 3, view_.normal);
    bind(r, 4, cams_);
    bind(r, 5, views_);
    bind(r, 6, state_);
    bind(r, 7, partials_);
  }
  bind(scale_.set, 0, vtx_[0]);
  bind(scale_.set, 1, views_);
  bind(scale_.set, 2, state_);
  bind(scale_.set, 3, bins_);
  bind(solve_.set, 0, partials_);
  bind(solve_.set, 1, state_);
  bind(pose_.set, 0, state_);
  bind(pose_.set, 1, views_);
  bind(pose_.set, 2, cams_);
}

std::uint32_t PointToPlane::groups_per_view(
    int level, std::uint32_t views) const noexcept {
  const std::uint32_t px = w_[level] * h_[level];
  const std::uint32_t want = core::group_count(px, kLocal * kPixelsPerThread);
  const std::uint32_t cap = std::max(1u, kMaxReduceGroups / views);
  return std::max(1u, std::min(want, cap));
}

core::Status PointToPlane::check(const vol::VoxelBlockGrid& model,
                                 const std::vector<sensor::DeviceFrame>& frames,
                                 const PosePrior& prior) const {
  const auto refuse = [](const std::string& why) {
    return core::Status::invalid_argument("PointToPlane::record: " + why);
  };
  if (outstanding_) return refuse("a record is outstanding");
  if (frames.empty()) return refuse("no frames");
  if (frames.size() > kMaxFitFrames) {
    return refuse(std::to_string(frames.size()) + " frames, over " +
                  std::to_string(kMaxFitFrames));
  }
  const std::uint32_t w = frames.front().depth_camera.width;
  const std::uint32_t h = frames.front().depth_camera.height;
  if (w < kMinSize || h < kMinSize) {
    return refuse("a frame of " + std::to_string(w) + "x" + std::to_string(h) +
                  ", under 16x16");
  }
  for (std::size_t i = 0; i < frames.size(); ++i) {
    const sensor::DeviceFrame& f = frames[i];
    const std::string at = "frame " + std::to_string(i) + ": ";
    if (f.depth_camera.width != w || f.depth_camera.height != h) {
      return refuse(at + "sizes are mixed");
    }
    if (i > 0 && !(f.timestamp_ns > frames[i - 1].timestamp_ns)) {
      return refuse(at + "timestamps are not ascending");
    }
    const std::string why = frame_refusal(f);
    if (!why.empty()) return refuse(at + why);
  }
  if (!rigid(prior.correction)) return refuse("the prior is not rigid");
  const auto sigma_ok = [](float s) { return std::isfinite(s) && s >= 0.0f; };
  if (!sigma_ok(prior.sigma_m) || !sigma_ok(prior.sigma_rad)) {
    return refuse("the prior's sigma is negative or not finite");
  }
  for (const std::string_view name :
       {vol::kTsdfAttribute, vol::kWeightAttribute}) {
    core::Result<vol::AttributeView> a = model.attribute(name);
    if (!a.ok() || a->element_size != 4) {
      return refuse("the model has no 4-byte '" + std::string(name) + "'");
    }
  }
  if (!config_.model_filter.attribute.empty()) {
    core::Result<vol::AttributeView> a =
        model.attribute(config_.model_filter.attribute);
    if (!a.ok() || a->element_size != 1) {
      return refuse("the model has no 1-byte '" +
                    std::string(config_.model_filter.attribute) +
                    "' for the filter");
    }
  }
  return {};
}

core::Result<PendingFit> PointToPlane::record(
    core::CommandBatch& batch, const vol::VoxelBlockGrid& model,
    const std::vector<sensor::DeviceFrame>& frames, const PosePrior& prior,
    core::StageMetrics* metrics, const FitRows& rows) try {
  VKC_TRY(check(model, frames, prior));
  const auto k = std::uint32_t(frames.size());
  VKC_TRY(ensure(frames.front().depth_camera.width,
                 frames.front().depth_camera.height, k));
  Outstanding out;
  out.serial = next_serial_++;
  out.frames = frames;
  out.prior = prior;
  // Entries: a committed one holding the same frame (its timestamp and its
  // depth buffer), else a free one, emptiest first, then the longest unused.
  std::vector<bool> taken(entries_.size(), false);
  out.entry_of.assign(k, UINT32_MAX);
  for (std::uint32_t i = 0; i < k; ++i) {
    const sensor::DeviceFrame& f = frames[i];
    for (std::uint32_t s = 0; s < entries_.size(); ++s) {
      const Entry& entry = entries_[s];
      if (entry.state != Entry::State::Committed || taken[s] ||
          entry.pts != f.timestamp_ns) {
        continue;
      }
      const std::shared_ptr<const core::Buffer> held = entry.depth.lock();
      if (held && held.get() == f.depth.get()) {
        out.entry_of[i] = s;
        taken[s] = true;
        break;
      }
    }
  }
  for (std::uint32_t i = 0; i < k; ++i) {
    if (out.entry_of[i] != UINT32_MAX) continue;
    std::optional<std::uint32_t> best;
    for (std::uint32_t s = 0; s < entries_.size(); ++s) {
      if (taken[s]) continue;
      if (!best) {
        best = s;
        continue;
      }
      const Entry& a = entries_[s];
      const Entry& b = entries_[*best];
      const bool a_empty = a.state != Entry::State::Committed;
      const bool b_empty = b.state != Entry::State::Committed;
      if ((a_empty && !b_empty) || (a_empty == b_empty && a.used < b.used)) {
        best = s;
      }
    }
    out.entry_of[i] = *best;  // capacity_ >= k: one is always free
    taken[*best] = true;
    out.filled.push_back(*best);
  }
  for (const std::uint32_t s : out.filled) {
    entries_[s].state = Entry::State::Pending;
    entries_[s].depth.reset();
  }
  for (std::uint32_t i = 0; i < k; ++i)
    entries_[out.entry_of[i]].used = out.serial;
  // The frames' mean camera centre at the prior: the twists' origin.
  Vec3d o(0.0);
  for (const sensor::DeviceFrame& f : frames) {
    o += Vec3d(Vec3f((prior.correction * f.depth_camera.cam_to_world)[3]));
  }
  out.origin = Vec3f(o / double(k));
  readback_ = FitState{};  // a batch that never runs leaves serial 0
  const core::Status recorded =
      record_commands(batch, model, out, metrics, rows);
  if (!recorded.ok()) {
    empty_filled(out);
    return recorded;
  }
  const std::uint64_t serial = out.serial;
  outstanding_ = std::move(out);
  return PendingFit{serial};
} catch (const std::bad_alloc&) {
  return core::Status::out_of_memory(
      "PointToPlane::record: host allocation failed");
}

core::Status PointToPlane::record_commands(core::CommandBatch& batch,
                                           const vol::VoxelBlockGrid& model,
                                           const Outstanding& out,
                                           core::StageMetrics* metrics,
                                           const FitRows& rows) {
  const IcpConfig& icp = config_.icp;
  const auto k = std::uint32_t(out.frames.size());
  // The rows: the fit's, its breakdowns with their device halves alone (as
  // its host time holds theirs), seeded now in this order.
  core::GpuStageScope total(metrics, timer_, rows.total);
  core::StageMetrics parts;
  core::StageMetrics* part_rows = metrics != nullptr ? &parts : nullptr;
  if (metrics != nullptr) {
    metrics->add_cpu(rows.pyramid, 0.0);
    metrics->add_cpu(rows.cast, 0.0);
    metrics->add_cpu(rows.icp, 0.0);
  }
  // The views, the state at the prior and the first cast's cameras: inline
  // uploads, under 64 KiB.
  std::vector<FitView> views(k);
  std::vector<vol::DepthPinhole> cams(k);
  for (std::uint32_t i = 0; i < k; ++i) {
    const sensor::DeviceFrame& f = out.frames[i];
    const sensor::DepthCameraParams& c = f.depth_camera;
    FitView& v = views[i];
    v.cam_to_world = c.cam_to_world;
    v.k = Vec4f(c.fx, c.fy, c.cx, c.cy);
    v.noise = Vec3f(f.depth_noise.a, f.depth_noise.b, f.depth_noise.c);
    v.entry = out.entry_of[i];
    v.min_depth = c.min_depth;
    v.max_depth = c.max_depth;
    vol::DepthPinhole& p = cams[i];
    p.fx = c.fx;
    p.fy = c.fy;
    p.cx = c.cx;
    p.cy = c.cy;
    // The cast reaches a coarse gate past the frame's range, so a point at
    // its far end still meets the surface the estimate's error put behind it.
    p.min_depth = 0.5f * c.min_depth;
    p.max_depth = c.max_depth + icp.coarse_gates_m[0];
    p.width = c.width;
    p.height = c.height;
    p.cam_to_world = out.prior.correction * c.cam_to_world;
  }
  FitState init;
  init.correction = out.prior.correction;
  init.prior = out.prior.correction;
  init.pass_start = out.prior.correction;
  init.origin = out.origin;
  init.scale = 1.0f;
  const auto info = [](float sigma) {
    return sigma > 0.0f ? 1.0f / (sigma * sigma) : 0.0f;
  };
  init.info = Vec2f(info(out.prior.sigma_m), info(out.prior.sigma_rad));
  init.n_eff = icp.prior_n_eff;
  init.frames = k;
  init.serial_lo = std::uint32_t(out.serial);
  init.serial_hi = std::uint32_t(out.serial >> 32);
  for (int i = 0; i < 6; ++i) init.dirs[7 * i] = 1.0f;

  {
    core::GpuStageScope stage(part_rows, timer_, rows.pyramid);
    for (const std::uint32_t s : out.filled) {
      std::uint32_t i = 0;
      while (out.entry_of[i] != s) ++i;
      const sensor::DeviceFrame& f = out.frames[i];
      const VkDeviceSize px0 = VkDeviceSize(w_[0]) * h_[0];
      VKC_TRY(batch.copy(*f.depth, 0, depth_[0], px0 * 4 * s, px0 * 4, &stage));
      const FilterPush fp{std::uint32_t(px0), std::uint32_t(px0) * s,
                          f.depth_camera.min_depth, f.depth_camera.max_depth};
      VKC_TRY(batch.dispatch(filter_, &fp, sizeof(fp),
                             core::group_count(std::uint32_t(px0), kLocal),
                             max_groups_, &stage));
      for (int l = 0; l + 1 < kLevels; ++l) {
        const DownPush dp{w_[l],
                          h_[l],
                          w_[l + 1],
                          h_[l + 1],
                          w_[l] * h_[l] * s,
                          w_[l + 1] * h_[l + 1] * s};
        VKC_TRY(batch.dispatch(down_, down_sets_[std::size_t(l)], &dp,
                               sizeof(dp),
                               core::group_count(w_[l + 1] * h_[l + 1], kLocal),
                               max_groups_, &stage));
      }
      Vec4f kl = views[i].k;
      for (int l = 0; l < kLevels; ++l) {
        if (l > 0) {
          kl = Vec4f(kl.x * 0.5f, kl.y * 0.5f, (kl.z - 0.5f) * 0.5f,
                     (kl.w - 0.5f) * 0.5f);
        }
        const VmapPush vp{w_[l],
                          h_[l],
                          w_[l] * h_[l] * s,
                          l == 0 ? icp.edge_px : 1,
                          kl,
                          std::cos(icp.grazing_deg * 3.14159265f / 180.0f)};
        VKC_TRY(batch.dispatch(
            vmap_, vmap_sets_[std::size_t(l)], &vp, sizeof(vp),
            core::group_count(w_[l] * h_[l], kLocal), max_groups_, &stage));
      }
    }
  }
  {
    core::GpuStageScope stage(part_rows, timer_, rows.icp);
    VKC_TRY(batch.upload(views_, 0, views.data(), sizeof(FitView) * k, &stage));
    VKC_TRY(batch.upload(state_, 0, &init, sizeof(init), &stage));
    VKC_TRY(batch.zero(bins_, 0, VkDeviceSize(kScaleBins) * 4));
    const std::uint32_t px0 = w_[0] * h_[0];
    ScalePush sp{0, px0, k, kScaleBins, kScaleBin};
    VKC_TRY(batch.dispatch(scale_, &sp, sizeof(sp),
                           core::group_count(px0 * k, kLocal), max_groups_,
                           &stage));
    sp.mode = 1;
    VKC_TRY(batch.dispatch(scale_, &sp, sizeof(sp), 1, max_groups_, &stage));
  }
  const int finest_iterations = icp.iterations[kLevels - 1];
  for (int pass = 0; pass < icp.passes; ++pass) {
    {
      core::GpuStageScope stage(part_rows, timer_, rows.cast);
      if (pass == 0) {
        VKC_TRY(batch.upload(cams_, 0, cams.data(),
                             sizeof(vol::DepthPinhole) * k, &stage));
      } else {
        const PosePush pp{k, kStopM, kStopDeg * 3.14159265f / 180.0f};
        VKC_TRY(batch.dispatch(pose_, &pp, sizeof(pp), 1, max_groups_, &stage));
      }
      VKC_TRY(caster_->record_cast(batch, model, cams_, k, config_.model_filter,
                                   view_, &stage));
    }
    core::GpuStageScope stage(part_rows, timer_, rows.icp);
    const auto iterate = [&](int level, std::uint32_t mode) -> core::Status {
      const bool finest = level == 0;
      const std::uint32_t gpv = groups_per_view(level, k);
      const std::uint32_t groups = gpv * k;
      // iterations[] runs coarsest first: level 2 is its first entry.
      const float gate = finest ? icp.finest_gate_floor_m
                                : icp.coarse_gates_m[std::size_t(2 - level)];
      const ReducePush rp{w_[level] * h_[level],
                          gpv,
                          w_[0],
                          h_[0],
                          finest ? 1u : 0u,
                          gate,
                          icp.gate_sigmas,
                          icp.normal_cos,
                          icp.huber_sigmas,
                          icp.model_sigma_m,
                          config_.see_through_m};
      VKC_TRY(batch.dispatch(reduce_, reduce_sets_[std::size_t(level)], &rp,
                             sizeof(rp), groups, max_groups_, &stage));
      const SolvePush op{groups,
                         gpv,
                         k,
                         mode,
                         finest ? 1u : 0u,
                         icp.min_relative_eigenvalue,
                         kDamping,
                         kMaxStep,
                         kStepMinRatio,
                         kStepMinInliers};
      return batch.dispatch(solve_, &op, sizeof(op), 1, max_groups_, &stage);
    };
    for (int level = kLevels - 1; level >= 0; --level) {
      const int n = icp.iterations[std::size_t(kLevels - 1 - level)];
      for (int it = 0; it < n; ++it) VKC_TRY(iterate(level, 0));
    }
    if (finest_iterations == 0) VKC_TRY(iterate(0, 1));
  }
  return batch.readback(state_, 0, sizeof(FitState), &readback_);
}

void PointToPlane::empty_filled(const Outstanding& out) noexcept {
  for (const std::uint32_t s : out.filled) {
    if (s < entries_.size()) {
      entries_[s].state = Entry::State::Empty;
      entries_[s].depth.reset();
    }
  }
}

core::Result<TrackFit> PointToPlane::finish(const PendingFit& pending) {
  if (!outstanding_ || pending.serial == 0 ||
      pending.serial != outstanding_->serial) {
    return core::Status::invalid_argument(
        "PointToPlane::finish: the pending fit is not this fitter's last "
        "record");
  }
  const std::uint64_t serial = std::uint64_t(readback_.serial_lo) |
                               (std::uint64_t(readback_.serial_hi) << 32);
  if (serial != pending.serial) {
    return core::Status::invalid_argument(
        "PointToPlane::finish: the readback is not this record's (its batch "
        "never ran)");
  }
  const Outstanding out = std::move(*outstanding_);
  outstanding_.reset();
  for (const std::uint32_t s : out.filled) {
    std::uint32_t i = 0;
    while (out.entry_of[i] != s) ++i;
    entries_[s].state = Entry::State::Committed;
    entries_[s].depth = out.frames[i].depth;
    entries_[s].pts = out.frames[i].timestamp_ns;
  }
  const FitState& r = readback_;
  TrackFit fit;
  fit.passes = std::uint8_t(config_.icp.passes);
  fit.frames = std::uint32_t(out.frames.size());
  fit.attempts = std::uint32_t(std::lround(r.attempts));
  fit.hits = std::uint32_t(std::lround(r.hits));
  fit.inliers = std::uint32_t(std::lround(r.inliers));
  fit.in_front = std::uint32_t(std::lround(r.in_front));
  fit.behind = std::uint32_t(std::lround(r.behind));
  fit.see_through_m = config_.see_through_m;
  fit.rms_m = r.inliers > 0.0f ? std::sqrt(r.raw2 / r.inliers) : 0.0f;
  fit.origin = r.origin;
  fit.scale_m = r.scale;
  const bool solved = r.evaluated != 0 && r.diverged == 0 &&
                      r.lambda_max > 0.0f && std::isfinite(r.lambda_max) &&
                      finite(r.correction);
  if (!solved) {
    fit.correction = out.prior.correction;
    for (int i = 0; i < 6; ++i) fit.directions[std::size_t(7 * i)] = 1.0f;
    fit.held = 0x3f;
    return fit;
  }
  // The device composes its steps in float: a correction that moved is made
  // orthonormal again here, in double, so one fed back as the next prior
  // stays rigid. One that did not move is the prior's, bit for bit.
  fit.correction =
      std::memcmp(&r.correction, &out.prior.correction, sizeof(Mat4f)) == 0
          ? out.prior.correction
          : orthonormalised(r.correction);
  fit.lambda_max = r.lambda_max;
  for (int i = 0; i < 6; ++i)
    fit.relative_eigenvalues[std::size_t(i)] = r.rel[i];
  for (int i = 0; i < 36; ++i) fit.directions[std::size_t(i)] = r.dirs[i];
  fit.held = std::uint8_t(r.held & 0x3fu);
  return fit;
}

void PointToPlane::abandon(const PendingFit& pending) noexcept {
  if (!outstanding_ || pending.serial == 0 ||
      pending.serial != outstanding_->serial) {
    return;
  }
  empty_filled(*outstanding_);
  outstanding_.reset();
}

}  // namespace

core::Result<std::unique_ptr<IFitter>> create_point_to_plane(
    core::Device& device, core::Allocator& allocator,
    const FitterConfig& config) try {
  auto fitter = std::make_unique<PointToPlane>(device, allocator, config);
  VKC_TRY(fitter->init());
  return std::unique_ptr<IFitter>(std::move(fitter));
} catch (const std::bad_alloc&) {
  return core::Status::out_of_memory("create_fitter: host allocation failed");
}

}  // namespace volumetric_kit::recon::track::detail
