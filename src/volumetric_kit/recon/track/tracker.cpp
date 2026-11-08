// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/recon/track/tracker.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <new>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "correction_filter.hpp"
#include "frame_check.hpp"
#include "se3.hpp"

namespace volumetric_kit::recon::track {
namespace {

bool valid_name(const std::string& name) {
  if (name.empty() || name.size() > 32) return false;
  return std::all_of(name.begin(), name.end(), [](char c) {
    return (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_' ||
           c == '@' || c == '-';
  });
}

}  // namespace

Method device_method() {
  Method m;
  m.name = "arkit";
  return m;
}

Method frame_method() {
  Method m;
  m.name = "frame";
  m.fit = FitKind::PointToPlane;
  m.judge.min_inliers = 100;
  m.judge.min_coverage = 0.4f;
  m.judge.min_inlier_ratio = 0.25f;
  return m;
}

Method window_method() {
  Method m;
  m.name = "window";
  m.fit = FitKind::PointToPlane;
  m.window_ns = 2'000'000'000;
  m.hop_ns = 500'000'000;
  return m;
}

namespace {

// Every chain's method and filter, in chain order.
struct ChainSpec {
  std::string name;
  const Method* method = nullptr;
  const volumetric_kit::grid::volume::CastFilter* filter = nullptr;
  bool driver = false;
};

std::vector<ChainSpec> chain_specs(const TrackerConfig& config) {
  std::vector<ChainSpec> out;
  for (const DriverChain& d : config.drivers) {
    out.push_back({d.name, &config.driver, &d.model_filter, true});
  }
  const volumetric_kit::grid::volume::CastFilter* first =
      config.drivers.empty() ? nullptr : &config.drivers.front().model_filter;
  for (const Method& m : config.shadows) {
    out.push_back({m.name, &m, first, false});
  }
  return out;
}

}  // namespace

core::Result<std::vector<std::unique_ptr<IFitter>>> create_fitters(
    core::Device& device, core::Allocator& allocator,
    const TrackerConfig& config) try {
  if (config.drivers.empty()) {
    return core::Status::invalid_argument("create_fitters: no driver chain");
  }
  std::vector<std::unique_ptr<IFitter>> out;
  for (const ChainSpec& c : chain_specs(config)) {
    if (!c.method->fit) {
      out.emplace_back();
      continue;
    }
    FitterConfig fc;
    fc.kind = *c.method->fit;
    fc.model_filter = *c.filter;
    fc.icp = c.method->icp;
    fc.see_through_m = c.method->judge.see_through_m;
    VKC_ASSIGN(std::unique_ptr<IFitter> fitter,
               create_fitter(device, allocator, fc));
    out.push_back(std::move(fitter));
  }
  return out;
} catch (const std::bad_alloc&) {
  return core::Status::out_of_memory("create_fitters: host allocation failed");
}

sensor::DeviceFrame corrected(sensor::DeviceFrame frame,
                              const Mat4f& correction) {
  frame.depth_camera.cam_to_world =
      correction * frame.depth_camera.cam_to_world;
  frame.color_camera.cam_to_world =
      correction * frame.color_camera.cam_to_world;
  return frame;
}

// --- The tracker
// ---------------------------------------------------------------

namespace {

// An Anchored fit, as the decisions read it: the frames it held are those of
// [first, last] on the frames' clock.
struct AnchoredFit {
  std::uint64_t serial = 0;
  std::uint64_t first = 0;
  std::uint64_t last = 0;
  Mat4f correction{1.0f};
};

struct Chain {
  std::string name;
  Method method;
  bool driver = false;
  std::unique_ptr<IFitter> fitter;
  detail::CorrectionFilter judge;
  std::optional<std::uint64_t> last_hop;  // the hop of the last fit judged
  std::vector<AnchoredFit> anchored;      // oldest first
  // The rows' names, which the metrics hold by pointer: made once.
  std::string total, pyramid, cast, icp;

  Chain(std::string n, Method m, bool d, std::unique_ptr<IFitter> f)
      : name(std::move(n)),
        method(std::move(m)),
        driver(d),
        fitter(std::move(f)),
        judge(method.judge, method.initial_correction),
        total("track: " + name),
        pyramid("  .." + name + " pyramid"),
        cast("  .." + name + " ray cast"),
        icp("  .." + name + " icp") {}

  FitRows rows() const noexcept {
    return {total.c_str(), pyramid.c_str(), cast.c_str(), icp.c_str()};
  }

  // The hop a frame of time t falls in; every frame its own for a hop of 0.
  std::uint64_t hop_of(std::uint64_t t) const noexcept {
    return method.hop_ns == 0 ? t : t / method.hop_ns;
  }
  bool due(std::uint64_t t) const noexcept {
    return !last_hop || hop_of(t) > *last_hop;
  }
};

struct Held {
  sensor::DeviceFrame frame;
  std::uint64_t pts = 0;
  std::vector<std::optional<FramePose>> poses;  // each chain's decision
  bool placed = false;                          // released or dropped
};

// One fit a step recorded or a shadow's refusal.
struct Recorded {
  std::size_t chain = 0;
  PendingFit pending;
  std::uint64_t first = 0;
  std::uint64_t last = 0;
  PosePrior prior;
  std::string refused;
};

struct Outstanding {
  std::uint64_t serial = 0;
  std::vector<Recorded> fits;
  // What the record changed, for an abandon to take back.
  std::optional<std::uint64_t> last_pts;
  std::optional<std::pair<std::uint32_t, std::uint32_t>> size;
  core::Status failed;  // a driver's refusal, or a buffer failure
};

}  // namespace

struct Tracker::State {
  TrackerConfig config;
  std::vector<std::string> names;
  std::vector<Chain> chains;  // never resized after create: the rows' names
  std::deque<Held> ring;
  std::optional<std::pair<std::uint32_t, std::uint32_t>> size;
  std::optional<std::uint64_t> last_pts;
  std::optional<Outstanding> outstanding;
  std::uint64_t next_step = 1;
  std::uint64_t next_fit = 1;

  core::Result<PendingStep> record(
      core::CommandBatch& batch,
      const volumetric_kit::grid::volume::VoxelBlockGrid& model,
      sensor::DeviceFrame frame, core::StageMetrics* metrics);
  // Record every fit due at the ring's newest frame into `out`; a driver's
  // refusal or a buffer failure lands in out.failed and stops it.
  void record_fits(core::CommandBatch& batch,
                   const volumetric_kit::grid::volume::VoxelBlockGrid& model,
                   core::StageMetrics* metrics, Outstanding& out);

  void abandon_fits(const std::vector<Recorded>& fits) noexcept {
    for (const Recorded& r : fits) {
      if (r.refused.empty() && chains[r.chain].fitter) {
        chains[r.chain].fitter->abandon(r.pending);
      }
    }
  }

  // Take back what `out`'s record did: its frame, and the clock and size it
  // set.
  void roll_back(const Outstanding& out) noexcept {
    abandon_fits(out.fits);
    if (!ring.empty()) ring.pop_back();
    last_pts = out.last_pts;
    size = out.size;
  }

  // Chain c's decision about f, once it can be made; `last` decides every
  // frame with the fits there are (flush).
  std::optional<FramePose> decide(const Chain& c, const Held& f,
                                  std::uint64_t now, bool last) const {
    FramePose pose;
    pose.decided_pts_ns = now;
    if (!c.fitter) {
      pose.correction = c.method.initial_correction;
      pose.from = PoseFrom::Device;
      return pose;
    }
    // No fit can hold f once a fit's newest frame would be window_ns past it.
    const bool closed = last || f.pts + c.method.window_ns <= now + 1;
    if (!closed) return std::nullopt;
    const AnchoredFit* held = nullptr;
    const AnchoredFit* before = nullptr;
    const AnchoredFit* after = nullptr;
    const std::uint64_t hold = config.hold_ns;
    for (const AnchoredFit& a : c.anchored) {
      if (a.first <= f.pts && f.pts <= a.last) {
        held = &a;  // the newest wins: anchored is oldest first
      } else if (a.last < f.pts && f.pts - a.last <= hold) {
        before = &a;
      } else if (a.first > f.pts && a.first - f.pts <= hold &&
                 after == nullptr) {
        after = &a;
      }
    }
    if (held != nullptr) {
      pose.correction = held->correction;
      pose.from = PoseFrom::Anchored;
      pose.fit = held->serial;
      return pose;
    }
    if (before != nullptr && after != nullptr) {
      const double s =
          double(f.pts - before->last) / double(after->first - before->last);
      pose.correction =
          detail::interpolate(before->correction, after->correction, s);
      pose.from = PoseFrom::Interpolated;
      pose.fit = after->serial;
      return pose;
    }
    if (last || f.pts + hold <= now) {
      pose.correction = c.judge.correction();
      pose.from = PoseFrom::Coasting;
      return pose;
    }
    return std::nullopt;
  }

  // Decide what can be decided, release the driver's placements, and let go
  // of frames nothing holds.
  void settle(TrackStep& step, std::uint64_t now, bool last) {
    const std::size_t drivers = config.drivers.size();
    for (Held& f : ring) {
      std::vector<bool> fresh(chains.size(), false);
      for (std::size_t c = 0; c < chains.size(); ++c) {
        if (f.poses[c]) continue;
        std::optional<FramePose> pose = decide(chains[c], f, now, last);
        if (!pose) continue;
        f.poses[c] = pose;
        fresh[c] = true;
        step.poses.push_back({c, f.frame, *pose});
      }
      if (f.placed) continue;
      std::optional<std::size_t> anchored;
      std::optional<std::size_t> interpolated;
      bool all = true;
      for (std::size_t k = 0; k < drivers; ++k) {
        const std::optional<FramePose>& p = f.poses[k];
        if (!p) {
          all = false;
          continue;
        }
        if (p->from == PoseFrom::Anchored && !anchored) anchored = k;
        if (p->from == PoseFrom::Interpolated && fresh[k] && !interpolated) {
          interpolated = k;
        }
      }
      if (anchored || interpolated) {
        const std::size_t k = anchored ? *anchored : *interpolated;
        Released r;
        r.frame = f.frame;
        r.placement = anchored ? Placement::Anchored : Placement::Interpolated;
        r.driver = k;
        r.correction = f.poses[k]->correction;
        step.released.push_back(std::move(r));
        f.placed = true;
      } else if (all) {
        step.dropped_ns.push_back(f.pts);
        f.placed = true;
      }
    }
    ring.erase(
        std::remove_if(ring.begin(), ring.end(),
                       [](const Held& f) {
                         if (!f.placed) return false;
                         return std::all_of(
                             f.poses.begin(), f.poses.end(),
                             [](const auto& p) { return p.has_value(); });
                       }),
        ring.end());
    // An Anchored fit ending more than hold_ns before every held frame
    // decides nothing more.
    const std::uint64_t oldest = ring.empty() ? now : ring.front().pts;
    for (Chain& c : chains) {
      c.anchored.erase(std::remove_if(c.anchored.begin(), c.anchored.end(),
                                      [&](const AnchoredFit& a) {
                                        return a.last + config.hold_ns < oldest;
                                      }),
                       c.anchored.end());
    }
  }
};

Tracker::Tracker(std::unique_ptr<State> state) noexcept
    : state_(std::move(state)) {}
Tracker::~Tracker() = default;

core::Result<std::unique_ptr<Tracker>> Tracker::create(
    TrackerConfig config, std::vector<std::unique_ptr<IFitter>> fitters) try {
  const auto refuse = [](const std::string& why) {
    return core::Status::invalid_argument("Tracker::create: " + why);
  };
  if (!config.driver.fit) return refuse("the driver has no fit kind");
  if (config.drivers.empty()) return refuse("no driver chain");
  if (config.hold_ns == 0) return refuse("hold_ns is 0");
  const std::vector<ChainSpec> specs = chain_specs(config);
  std::set<std::string> seen;
  for (const ChainSpec& c : specs) {
    if (!valid_name(c.name)) {
      return refuse("chain name '" + c.name + "' is not [a-z0-9_@-]{1,32}");
    }
    if (!seen.insert(c.name).second) {
      return refuse("chain name '" + c.name + "' is duplicated");
    }
    VKC_TRY(detail::check_config(c.method->judge, "Tracker::create"));
    if (!detail::rigid(c.method->initial_correction)) {
      return refuse("chain '" + c.name + "': initial_correction is not rigid");
    }
  }
  if (fitters.size() != specs.size()) {
    return refuse("fitters do not match the chains (" +
                  std::to_string(fitters.size()) + " for " +
                  std::to_string(specs.size()) + ")");
  }
  for (std::size_t i = 0; i < specs.size(); ++i) {
    if (bool(fitters[i]) != bool(specs[i].method->fit)) {
      return refuse("chain '" + specs[i].name +
                    (fitters[i] ? "' has no fit kind but a fitter"
                                : "' has a fit kind but no fitter"));
    }
  }
  auto state = std::make_unique<State>();
  state->config = std::move(config);
  // The specs point into the config just moved: take them again.
  const std::vector<ChainSpec> chains = chain_specs(state->config);
  state->chains.reserve(chains.size());
  for (std::size_t i = 0; i < chains.size(); ++i) {
    Method m = *chains[i].method;
    m.name = chains[i].name;
    state->chains.emplace_back(chains[i].name, std::move(m), chains[i].driver,
                               std::move(fitters[i]));
    state->names.push_back(chains[i].name);
  }
  return std::unique_ptr<Tracker>(new Tracker(std::move(state)));
} catch (const std::bad_alloc&) {
  return core::Status::out_of_memory("Tracker::create: host allocation failed");
}

const std::vector<std::string>& Tracker::chains() const noexcept {
  return state_->names;
}

const TrackerConfig& Tracker::config() const noexcept { return state_->config; }

bool Tracker::fit_due(std::uint64_t timestamp_ns) const noexcept {
  // The driver chains share one method, and a step records and finishes
  // their fits together or drops them all: the first speaks for them.
  return state_->chains.front().due(timestamp_ns);
}

core::Result<PendingStep> Tracker::record(
    core::CommandBatch& batch,
    const volumetric_kit::grid::volume::VoxelBlockGrid& model,
    sensor::DeviceFrame frame, core::StageMetrics* metrics) {
  try {
    return state_->record(batch, model, std::move(frame), metrics);
  } catch (const std::bad_alloc&) {
    return core::Status::out_of_memory(
        "Tracker::record: host allocation failed");
  }
}

core::Result<PendingStep> Tracker::State::record(
    core::CommandBatch& batch,
    const volumetric_kit::grid::volume::VoxelBlockGrid& model,
    sensor::DeviceFrame frame, core::StageMetrics* metrics) {
  State& s = *this;
  const auto refuse = [](const std::string& why) {
    return core::Status::invalid_argument("Tracker::record: " + why);
  };
  if (s.outstanding) return refuse("a step is outstanding");
  // The fitters' frame check, at the frame's own record: a frame taken on a
  // step where no fit is due must not make every later record refuse while
  // a window still holds it.
  const std::string why = detail::frame_refusal(frame);
  if (!why.empty()) return refuse("the frame: " + why);
  const std::pair<std::uint32_t, std::uint32_t> size{frame.depth_camera.width,
                                                     frame.depth_camera.height};
  if (s.size && *s.size != size) {
    return refuse("the frame is " + std::to_string(size.first) + "x" +
                  std::to_string(size.second) + ", the first was " +
                  std::to_string(s.size->first) + "x" +
                  std::to_string(s.size->second));
  }
  const std::uint64_t t = frame.timestamp_ns;
  if (s.last_pts && t <= *s.last_pts) {
    return refuse("the frame is not newer than the last");
  }
  Outstanding out;
  out.serial = s.next_step;
  out.last_pts = s.last_pts;
  out.size = s.size;
  Held held;
  held.pts = t;
  held.poses.resize(s.chains.size());
  held.frame = std::move(frame);
  s.ring.push_back(std::move(held));
  try {
    record_fits(batch, model, metrics, out);
  } catch (const std::bad_alloc&) {
    s.roll_back(out);
    throw;
  }
  if (!out.failed.ok()) {
    s.roll_back(out);
    return out.failed;
  }
  s.last_pts = t;
  s.size = size;
  ++s.next_step;
  s.outstanding = std::move(out);
  return PendingStep{s.outstanding->serial};
}

void Tracker::State::record_fits(
    core::CommandBatch& batch,
    const volumetric_kit::grid::volume::VoxelBlockGrid& model,
    core::StageMetrics* metrics, Outstanding& out) {
  State& s = *this;
  const std::uint64_t t = s.ring.back().pts;
  for (std::size_t c = 0; c < s.chains.size(); ++c) {
    Chain& chain = s.chains[c];
    if (!chain.fitter || !chain.due(t)) continue;
    // The newest frame and the held frames in (t - window, t], the newest
    // kMaxFitFrames.
    std::vector<sensor::DeviceFrame> frames;
    std::size_t first = s.ring.size() - 1;
    while (first > 0 && s.ring[first - 1].pts + chain.method.window_ns > t &&
           s.ring.size() - first < kMaxFitFrames) {
      --first;
    }
    frames.reserve(s.ring.size() - first);
    for (std::size_t i = first; i < s.ring.size(); ++i) {
      frames.push_back(s.ring[i].frame);
    }
    Recorded rec;
    rec.chain = c;
    rec.first = s.ring[first].pts;
    rec.last = t;
    rec.prior = chain.judge.prior(t);
    core::Result<PendingFit> fit = chain.fitter->record(
        batch, model, frames, rec.prior, metrics, chain.rows());
    if (!fit.ok()) {
      if (chain.driver ||
          fit.status().domain() != core::Status::Code::InvalidArgument) {
        out.failed = fit.status();
        return;
      }
      rec.refused = fit.status().message();
    } else {
      rec.pending = *fit;
    }
    out.fits.push_back(std::move(rec));
  }
}

core::Result<TrackStep> Tracker::finish(const PendingStep& pending) try {
  State& s = *state_;
  if (!s.outstanding || pending.serial == 0 ||
      pending.serial != s.outstanding->serial) {
    return core::Status::invalid_argument(
        "Tracker::finish: the pending step is not this tracker's last record");
  }
  Outstanding out = std::move(*s.outstanding);
  s.outstanding.reset();
  // Every fitter's finish before any judge moves: a refusal drops the step.
  std::vector<TrackFit> fits(out.fits.size());
  for (std::size_t i = 0; i < out.fits.size(); ++i) {
    const Recorded& r = out.fits[i];
    if (!r.refused.empty()) continue;
    const Chain& chain = s.chains[r.chain];
    core::Result<TrackFit> fit = chain.fitter->finish(r.pending);
    if (fit.ok() && fit->see_through_m != chain.method.judge.see_through_m) {
      fit = core::Status::invalid_argument(
          "Tracker::finish: chain '" + chain.name +
          "' counted see-through hits past " +
          std::to_string(fit->see_through_m) + " m, its judge's is " +
          std::to_string(chain.method.judge.see_through_m) + " m");
    }
    if (!fit.ok()) {
      // From fit i on: a fitter that refused its finish may still hold its
      // record (one whose readback never came), and an abandon ignores a
      // record its fitter no longer holds.
      std::vector<Recorded> rest(out.fits.begin() + std::ptrdiff_t(i),
                                 out.fits.end());
      Outstanding remaining = out;
      remaining.fits = std::move(rest);
      s.roll_back(remaining);
      return fit.status();
    }
    fits[i] = *fit;
  }
  TrackStep step;
  const std::uint64_t now = s.ring.back().pts;
  for (std::size_t i = 0; i < out.fits.size(); ++i) {
    const Recorded& r = out.fits[i];
    Chain& chain = s.chains[r.chain];
    ChainFit cf;
    cf.chain = r.chain;
    cf.serial = s.next_fit++;
    cf.first_pts_ns = r.first;
    cf.last_pts_ns = r.last;
    cf.prior = r.prior;
    cf.refused = r.refused;
    chain.last_hop = chain.hop_of(r.last);
    if (r.refused.empty()) {
      cf.fit = fits[i];
      cf.judged = chain.judge.update(cf.fit, r.prior, r.last);
      if (cf.judged.state == TrackState::Anchored) {
        chain.anchored.push_back(
            {cf.serial, r.first, r.last, cf.judged.correction});
      }
    } else {
      cf.judged = {chain.judge.correction(), TrackState::Coasting};
    }
    step.fits.push_back(std::move(cf));
  }
  s.settle(step, now, false);
  return step;
} catch (const std::bad_alloc&) {
  return core::Status::out_of_memory("Tracker::finish: host allocation failed");
}

void Tracker::abandon(const PendingStep& pending) noexcept {
  State& s = *state_;
  if (!s.outstanding || pending.serial == 0 ||
      pending.serial != s.outstanding->serial) {
    return;
  }
  const Outstanding out = std::move(*s.outstanding);
  s.outstanding.reset();
  s.roll_back(out);
}

TrackStep Tracker::flush() {
  State& s = *state_;
  TrackStep step;
  if (s.outstanding) return step;
  if (!s.ring.empty()) s.settle(step, *s.last_pts, true);
  // A split: frames recorded after it are placed by fits after it alone, so
  // no interpolation bridges a map step the frames before it may hold. So
  // too when every frame was placed already.
  for (Chain& c : s.chains) c.anchored.clear();
  return step;
}

}  // namespace volumetric_kit::recon::track
