// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "live_mobile.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include "live_mobile_internal.hpp"
#include "volumetric_kit/core/vulkan/buffer.hpp"
#include "volumetric_kit/core/vulkan/command_batch.hpp"
#include "volumetric_kit/core/vulkan/compute_util.hpp"
#include "volumetric_kit/core/vulkan/gpu_timer.hpp"
#include "volumetric_kit/recon/tsdf/source.hpp"
#include "volumetric_kit/sensor/prep/gpu_frame_prep.hpp"

namespace vr_example {

HopEstimate::HopEstimate(std::chrono::nanoseconds seed) noexcept {
  samples_.fill(seed);
}

void HopEstimate::add(std::chrono::nanoseconds wall) noexcept {
  samples_[next_] = wall;
  next_ = (next_ + 1) % kHopSamples;
}

void HopEstimate::probed(std::chrono::nanoseconds wall,
                         std::chrono::nanoseconds room) noexcept {
  if (wall <= room) {
    for (std::chrono::nanoseconds& s : samples_) {
      if (s > room) s = wall;
    }
  }
  add(wall);
}

std::chrono::nanoseconds HopEstimate::p90() const noexcept {
  // Nearest rank: the ceil(0.9 n)-th smallest.
  constexpr std::size_t kRank = (kHopSamples * 9 + 9) / 10 - 1;
  std::array<std::chrono::nanoseconds, kHopSamples> sorted = samples_;
  std::nth_element(sorted.begin(), sorted.begin() + kRank, sorted.end());
  return sorted[kRank];
}

namespace {

using detail::counts_line;
using detail::device_lost;
using detail::InFlight;
using detail::Kind;
using detail::let_go;
using detail::PhoneSession;
using detail::ran_out_of_memory;
using detail::Reaper;

// `c` counted into `to`, field by field: a field added to Counts is added
// here too, or this fails.
static_assert(sizeof(MobileLane::Counts) == 16 * sizeof(std::uint64_t),
              "add_to sums every field of MobileLane::Counts");
void add_to(MobileLane::Counts& to, const MobileLane::Counts& c) {
  to.polled += c.polled;
  to.paired += c.paired;
  to.skipped += c.skipped;
  to.passed_over += c.passed_over;
  to.splits += c.splits;
  to.errors += c.errors;
  to.anchored += c.anchored;
  to.unanchored += c.unanchored;
  to.dropped += c.dropped;
  to.fused += c.fused;
  to.unregistered += c.unregistered;
  to.shed += c.shed;
  to.unfit += c.unfit;
  to.deferred += c.deferred;
  to.over_budget += c.over_budget;
  to.pictures += c.pictures;
}

// A fill's options a lane refuses: a picture_every of 0.
vkc::Status check_fill(const FillOptions& fill, const char* op) {
  if (fill.picture_every == 0) {
    return vkc::Status::invalid_argument(
        std::string(op) + ": a picture_every of 0, which must be positive");
  }
  return {};
}

bool finite(const Mat4f& m) {
  for (int c = 0; c < 4; ++c) {
    for (int r = 0; r < 4; ++r) {
      if (!std::isfinite(m[c][r])) return false;
    }
  }
  return true;
}

}  // namespace

sensor::GpuFramePrepConfig picture_prep_config() {
  sensor::GpuFramePrepConfig config;
  config.depth_outliers.enabled = false;
  return config;
}

vkc::Status check_filler(const sender::Filler& filler) {
  if (filler.index >= tsdf::kMaxSourcesPerClass) {
    return vkc::Status::invalid_argument(
        "filler: mobile index " + std::to_string(filler.index) +
        " is past the source byte's " +
        std::to_string(tsdf::kMaxSourcesPerClass) + " sensors a class");
  }
  const sensor::DepthCameraParams& d = filler.frame.depth_camera;
  if (!std::isfinite(d.fx) || !std::isfinite(d.fy) || !std::isfinite(d.cx) ||
      !std::isfinite(d.cy) || !std::isfinite(d.min_depth) ||
      !std::isfinite(d.max_depth) || !finite(d.cam_to_world)) {
    return vkc::Status::invalid_argument(
        "filler: its depth camera has a non-finite intrinsic, depth bound or "
        "pose entry");
  }
  const sensor::ColorCameraParams& c = filler.frame.color_camera;
  if (filler.frame.has_color() &&
      (!std::isfinite(c.fx) || !std::isfinite(c.fy) || !std::isfinite(c.cx) ||
       !std::isfinite(c.cy) || !finite(c.cam_to_world))) {
    return vkc::Status::invalid_argument(
        "filler: its colour camera has a non-finite intrinsic or pose entry");
  }
  return {};
}

struct MobileLane::State {
  vkc::Device* device = nullptr;
  vkc::Allocator* allocator = nullptr;
  // The sessions in join order, those added since the last gap last; a
  // session stays here once it left, for its counts and lines.
  std::vector<std::unique_ptr<PhoneSession>> sessions;
  std::uint32_t joins = 0;     // sessions joined so far
  std::size_t drain_next = 0;  // the session the next turn of the drain is
  // What its fillers carry, and the tint pictures, by tier, at each depth
  // size a filler has had (FillColour::Tint).
  FillOptions fill;
  std::map<std::pair<std::uint32_t, std::uint32_t>,
           std::array<std::shared_ptr<const vkc::Buffer>, 2>>
      tints;
  bool closed = false;
  // A live lane's: its options, its sessions' source and its reaper, which
  // goes before the sessions it reaps.
  bool live = false;
  LaneOptions options;
  std::unique_ptr<SessionSource> source;
  std::unique_ptr<Reaper> reaper;
  // The first gap's set, which a report's times count from.
  std::optional<std::uint64_t> origin_ns;
  // The sessions whose own error was printed, by id (or name), the oldest
  // forgotten past 64.
  std::deque<std::string> warned;
  // The gap's scheduler: the session the next contested tie for the hop
  // starts at; what a live gap's hop costs; the set of the first gap whose
  // budget held a hop back since a hop last ran, the probes run and the
  // newest budgeted gap's room; and, over the deferral window from its set,
  // the records classed a prep or a hop and those of them put back.
  std::size_t hop_next = 0;
  HopEstimate hop_cost;
  std::optional<std::uint64_t> held_since;
  std::uint64_t probes = 0;
  std::optional<std::chrono::nanoseconds> last_room;
  std::optional<std::uint64_t> window_ns;
  std::uint64_t window_classed = 0;
  std::uint64_t window_deferred = 0;
  std::uint64_t window_over_budget = 0;

  State() = default;
  // The source closed and the reaper's jobs done before the sessions go.
  ~State() {
    if (source) source->close();
    reaper.reset();
  }
  State(const State&) = delete;
  State& operator=(const State&) = delete;

  // The index of the session joining as `ordinal`: (ordinal - 1) mod 63,
  // past any a session holds; none while all are held.
  std::optional<std::uint32_t> free_index(std::uint32_t ordinal) const {
    std::array<bool, tsdf::kMaxSourcesPerClass> held{};
    for (const std::unique_ptr<PhoneSession>& s : sessions) {
      if (s->holding()) held[s->index] = true;
    }
    for (std::uint32_t k = 0; k < tsdf::kMaxSourcesPerClass; ++k) {
      const std::uint32_t i = static_cast<std::uint32_t>(
          (std::uint64_t{ordinal} - 1 + k) % tsdf::kMaxSourcesPerClass);
      if (!held[i]) return i;
    }
    return std::nullopt;
  }

  // The sessions in the run now, from their join until the reaper let their
  // phone go, which frees the door's slot: what a live join's line counts
  // against kMaxPhones.
  std::size_t in_run() const {
    std::size_t n = 0;
    for (const std::unique_ptr<PhoneSession>& s : sessions) {
      const Stage st = s->stage.load(std::memory_order_acquire);
      if (st == Stage::Live || st == Stage::Away || st == Stage::Retiring ||
          (st == Stage::Reaping &&
           !s->released.load(std::memory_order_acquire))) {
        ++n;
      }
    }
    return n;
  }

  // The sessions added since the last gap, in turn, each with an index free,
  // at the set `set_ns`: its phone started, a live lane's line printed and
  // its join's row queued. A phone that does not start retires its session.
  vkc::Status adopt(std::uint64_t set_ns) {
    for (const std::unique_ptr<PhoneSession>& s : sessions) {
      if (s->stage.load(std::memory_order_acquire) != Stage::Added) continue;
      const std::optional<std::uint32_t> index = free_index(joins + 1);
      if (!index) break;  // waits for a session to be done
      const vkc::Status started = s->join(joins + 1, *index, set_ns);
      ++joins;
      if (live) {
        const RunSource& src = s->log->run().source;
        const std::string by = s->by_tag                  ? "the tag"
                               : src.registration.empty() ? "its registration"
                                                          : src.registration;
        std::printf("phones: %s%s%s joined by %s; %zu of %zu\n",
                    s->label().c_str(), src.peer.empty() ? "" : " from ",
                    src.peer.c_str(), by.c_str(), in_run(), kMaxPhones);
        if (reaper) reaper->join_row(s->row(/*join=*/true));
      }
      if (!started.ok()) {
        VKC_TRY(contain(*s, started.with_context("its phone's start")));
      }
    }
    return {};
  }

  // A session's own error retires one that comes and goes alone, printed
  // once its id, and its hearer told; out of memory, the source admits
  // nothing for a while. The device's loss returns, as does a file's
  // session's error: a replay asked for that file.
  vkc::Status contain(PhoneSession& s, const vkc::Status& error) {
    if (error.ok()) return {};
    if (device_lost(error) || !s.comes_and_goes()) return error;
    if (ran_out_of_memory(error) && source) source->out_of_memory();
    const std::string& id = s.log->run().source.session_id;
    const std::string key = id.empty() ? s.name : id;
    if (std::find(warned.begin(), warned.end(), key) == warned.end()) {
      if (warned.size() >= 64) warned.pop_front();
      warned.push_back(key);
      std::fprintf(stderr, "warning: phone %s: %s; the session retires\n",
                   s.name.c_str(), error.message().c_str());
    }
    if (std::function<void(const vkc::Status&)> told =
            std::exchange(s.on_error, {})) {
      told(error);
    }
    return s.retire("its error: " + error.message());
  }

  // The gap's work (live_mobile.hpp, the gap's scheduler): each session in
  // the run does its host work, a retiring one's phone queue let go; each
  // record due is classed; one hop of them all is kept and, within a live
  // lane's `given` budget, every prep, the rest put back; the records kept
  // are begun in join order and finished in join order; each session's own
  // error is contained once every batch is waited; then each phone's end.
  // Only the device's loss returns, or a file's session's own error.
  vkc::Status gap(const vol::VoxelBlockGrid& grid, std::uint64_t next_set_ns,
                  const GapBudget& given) {
    // A file's lane reads no budget, so a replay repeats.
    const GapBudget budget = live ? given : GapBudget{};
    const std::size_t n = sessions.size();
    std::vector<bool> paired(n, false);  // its host work done, still in it
    for (std::size_t i = 0; i < n; ++i) {
      PhoneSession& s = *sessions[i];
      if (s.active()) {
        const vkc::Status hosted = s.host(next_set_ns, options);
        VKC_TRY(contain(s, hosted));
        paired[i] = hosted.ok() && s.active();  // not retired at its grace
      } else if (s.stage.load(std::memory_order_acquire) == Stage::Retiring) {
        s.set_ns = next_set_ns;
        if (s.comes_and_goes()) VKC_TRY(s.discard());
      }
    }
    // Classed before anything of a record is shown or tracked, so one put
    // back keeps its split and its watch's sight of it.
    std::vector<Kind> kind(n, Kind::None);
    std::size_t hops = 0;
    for (std::size_t i = 0; i < n; ++i) {
      if (paired[i]) kind[i] = sessions[i]->classify();
      if (kind[i] == Kind::Hop) ++hops;
    }
    // One hop: the longest waited, ties in turn from hop_next; with a
    // budget's end, only if the estimate ends it by then (or as a probe, the
    // budget having held hops back kHopProbeNs with none run), and the preps
    // only before that end.
    const std::chrono::steady_clock::time_point chose =
        std::chrono::steady_clock::now();
    bool late = false;
    bool hop_fits = true;
    bool probe = false;
    if (budget.gap_end) {
      last_room = std::chrono::duration_cast<std::chrono::nanoseconds>(
          *budget.gap_end - chose);
      late = chose > *budget.gap_end;
      hop_fits = chose + hop_cost.p90() <= *budget.gap_end;
      if (!hop_fits && !late && hops != 0 && held_since &&
          next_set_ns >= *held_since + kHopProbeNs) {
        hop_fits = probe = true;
      }
    }
    std::size_t hop = n;  // none
    for (std::size_t k = 0; hop_fits && k < n; ++k) {
      const std::size_t i = (hop_next + k) % n;
      if (kind[i] == Kind::Hop &&
          (hop == n || sessions[i]->waited > sessions[hop]->waited)) {
        hop = i;
      }
    }
    if (hop == n && hops != 0 && !held_since) held_since = next_set_ns;
    for (std::size_t i = 0; i < n; ++i) {
      const bool hop_back = kind[i] == Kind::Hop && i != hop;
      const bool prep_back = kind[i] == Kind::Prep && late;
      if (kind[i] == Kind::Hop || kind[i] == Kind::Prep) ++window_classed;
      if (!hop_back && !prep_back) continue;
      const bool by_budget = prep_back || !hop_fits;
      if (hop_back) ++sessions[i]->waited;
      sessions[i]->defer(by_budget);
      kind[i] = Kind::None;
      ++window_deferred;
      if (by_budget) ++window_over_budget;
    }
    // Each kept record's batch submitted in join order, then each waited in
    // join order; an error is contained once none is in flight.
    struct Run {
      PhoneSession* session = nullptr;
      std::unique_ptr<InFlight> flight;
      vkc::Status error;
      bool hop = false;
    };
    std::vector<Run> runs;
    for (std::size_t i = 0; i < n; ++i) {
      if (kind[i] == Kind::None) continue;
      Run r;
      r.session = sessions[i].get();
      r.hop = i == hop;
      vkc::Result<std::unique_ptr<InFlight>> begun = r.session->begin(grid);
      if (begun.ok()) {
        r.flight = std::move(begun).value();
      } else {
        r.error = begun.status();
      }
      runs.push_back(std::move(r));
    }
    bool hop_ran = false;
    for (Run& r : runs) {
      if (!r.flight) continue;
      r.error = r.session->finish(*r.flight);
      r.flight.reset();
      hop_ran = hop_ran || (r.hop && r.error.ok());
    }
    // A hop that ran: its wall time a sample, its session's wait over, and,
    // where two or more contested the gap, the turn moved on past it. One
    // chosen and not run (its frame passed over, or an error) keeps both.
    if (hop_ran) {
      const auto wall = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now() - chose);
      if (probe) {
        ++probes;
        hop_cost.probed(wall, *last_room);
      } else {
        hop_cost.add(wall);
      }
      held_since.reset();
      sessions[hop]->waited = 0;
      if (hops > 1) hop_next = hop + 1;
    }
    for (Run& r : runs) VKC_TRY(contain(*r.session, r.error));
    for (std::size_t i = 0; i < n; ++i) {
      if (paired[i] && sessions[i]->active()) {
        VKC_TRY(contain(*sessions[i], sessions[i]->at_end()));
      }
    }
    note_deferrals(next_set_ns);
    return {};
  }

  // A warning for each deferral window, from the first gap's set, in which
  // more than half the records classed a prep or a hop were put back: how
  // many by the budget, the estimate and the newest budgeted gap's room, so
  // too many phones reads apart from a rig too slow for their hops.
  void note_deferrals(std::uint64_t set_ns) {
    if (!window_ns) window_ns = set_ns;
    if (set_ns < *window_ns + kDeferralWindowNs) return;
    if (2 * window_deferred > window_classed) {
      std::fprintf(stderr,
                   "warning: phones: %llu of %llu records deferred over %.0f s "
                   "of sets (%llu by the gap's budget, the rest one hop a "
                   "gap; a hop estimated at %.1f ms, the last gap's room "
                   "%s); their fill comes late or not at all\n",
                   static_cast<unsigned long long>(window_deferred),
                   static_cast<unsigned long long>(window_classed),
                   double(set_ns - *window_ns) * 1e-9,
                   static_cast<unsigned long long>(window_over_budget),
                   double(hop_cost.p90().count()) * 1e-6, room_text().c_str());
    }
    window_ns = set_ns;
    window_classed = 0;
    window_deferred = 0;
    window_over_budget = 0;
  }

  // The newest budgeted gap's room, for a person: "none" without a budget.
  std::string room_text() const {
    if (!last_room) return "none";
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.1f ms",
                  double(last_room->count()) * 1e-6);
    return buf;
  }

  // A filler coloured by its tier's tint, through a colour camera of its
  // depth camera's geometry; the tints at its depth size made on the device
  // once.
  vkc::Status tint(sender::Filler& f) {
    const sensor::DepthCameraParams& d = f.frame.depth_camera;
    std::array<std::shared_ptr<const vkc::Buffer>, 2>& made =
        tints[{d.width, d.height}];
    if (!made[0]) {
      const VkDeviceSize bytes = VkDeviceSize(d.width) * d.height * 4;
      vkc::CommandBatch batch(*device, *allocator);
      const std::uint32_t words[2] = {kAnchoredFillTint, kUnanchoredFillTint};
      std::array<std::shared_ptr<const vkc::Buffer>, 2> fresh;
      for (int t = 0; t < 2; ++t) {
        VKC_ASSIGN(vkc::Buffer b,
                   vkc::device_storage_buffer(*allocator, bytes));
        VKC_TRY(batch.fill(b, 0, bytes, words[t]));
        fresh[t] = std::make_shared<const vkc::Buffer>(std::move(b));
      }
      VKC_TRY(batch.submit());
      made = std::move(fresh);
    }
    f.frame.color = made[f.tier == tsdf::FillTier::Anchored ? 0 : 1];
    f.frame.color_camera = {d.fx,    d.fy,     d.cx,          d.cy,
                            d.width, d.height, d.cam_to_world};
    f.frame.color_encoding = {};
    return {};
  }

  // The next send's fillers: one a session in turn, in join order from
  // where the last send's turns stopped, kDrainCap in all; a session with
  // none waiting, or gone to the reaper, gives its turn away, and a filler
  // the send would refuse is let go. Under the tint, a tint that cannot be
  // made is the device's: out of memory, the filler is let go; its loss
  // returns. Under FillColour::Picture a filler carries its picture as it
  // is, or none.
  vkc::Status drain(std::vector<sender::Filler>* out) {
    const std::size_t n = sessions.size();
    if (n == 0) return {};
    drain_next %= n;
    for (std::size_t idle = 0; out->size() < kDrainCap && idle < n;) {
      PhoneSession& s = *sessions[drain_next];
      drain_next = (drain_next + 1) % n;
      const Stage st = s.stage.load(std::memory_order_acquire);
      if (st == Stage::Reaping || st == Stage::Left || s.queue.empty()) {
        ++idle;
        continue;
      }
      idle = 0;
      sender::Filler f = std::move(s.queue.front());
      s.queue.pop_front();
      const vkc::Status fit = check_filler(f);
      if (!fit.ok()) {
        s.unfit(fit);
        continue;
      }
      if (fill.colour == FillColour::Tint) {
        const vkc::Status tinted = tint(f);
        if (!tinted.ok()) {
          if (device_lost(tinted)) return tinted;
          s.unfit(tinted);
          continue;
        }
      }
      ++s.counts.fused;
      out->push_back(std::move(f));
    }
    return {};
  }

  // A session done, handed to the reaper: its counters kept, its presence,
  // say and error's hearer let go here; from now on the reaper's.
  void hand_over(PhoneSession& s, bool keep_log) {
    s.last_stats = s.phone ? s.phone->stats() : sensor::SensorStats{};
    s.left_ns = s.set_ns;
    s.presence = {};
    s.say = {};
    s.on_error = {};
    s.stage.store(Stage::Reaping, std::memory_order_release);
    reaper->reap(&s, keep_log);
  }
};

MobileLane::MobileLane(std::unique_ptr<State> state) noexcept
    : state_(std::move(state)) {}
MobileLane::~MobileLane() = default;

vkc::Result<std::unique_ptr<MobileLane>> MobileLane::create(
    vkc::Device& device, vkc::Allocator& allocator,
    std::unique_ptr<sensor::IRgbdSensor> phone,
    std::unique_ptr<track::Tracker> tracker, std::unique_ptr<TrackLog> log,
    std::unique_ptr<TagWatch> tags, std::function<void(gnet::ViewerNote)> say,
    FillOptions fill) {
  VKC_TRY(check_fill(fill, "MobileLane::create"));
  auto s = std::make_unique<State>();
  s->device = &device;
  s->allocator = &allocator;
  s->fill = fill;
  std::unique_ptr<MobileLane> lane(new MobileLane(std::move(s)));
  if (phone) {
    if (!tracker || !log) {
      return vkc::Status::invalid_argument(
          "MobileLane::create: a phone needs its tracker and its log");
    }
    SessionParts parts;
    parts.name = phone->info().id;
    parts.placement = tags ? Placement::Tag : Placement::Registration;
    parts.phone = std::move(phone);
    parts.tracker = std::move(tracker);
    parts.log = std::move(log);
    parts.tags = std::move(tags);
    parts.say = std::move(say);
    VKC_TRY(lane->add(std::move(parts)));
  }
  return lane;
}

vkc::Result<std::unique_ptr<MobileLane>> MobileLane::create(
    vkc::Device& device, vkc::Allocator& allocator,
    std::unique_ptr<SessionSource> source, LaneOptions options) {
  if (options.settle_ns == 0 || options.settle_ns >= options.grace_ns) {
    return vkc::Status::invalid_argument("MobileLane::create: a settle of " +
                                         std::to_string(options.settle_ns) +
                                         " ns, which must be positive and "
                                         "below the grace's " +
                                         std::to_string(options.grace_ns));
  }
  if (options.error_back_off.count() <= 0) {
    return vkc::Status::invalid_argument(
        "MobileLane::create: a back-off of " +
        std::to_string(options.error_back_off.count()) +
        " ms, which must be positive");
  }
  if (options.hop_seed.count() <= 0) {
    return vkc::Status::invalid_argument(
        "MobileLane::create: a hop's seed of " +
        std::to_string(options.hop_seed.count()) +
        " ns, which must be positive");
  }
  VKC_TRY(check_fill(options.fill, "MobileLane::create"));
  auto s = std::make_unique<State>();
  s->device = &device;
  s->allocator = &allocator;
  s->live = true;
  s->fill = options.fill;
  s->hop_cost = HopEstimate(options.hop_seed);
  s->source = std::move(source);
  // sessions.csv, made at its first row, never over one there.
  std::string csv_path;
  if (!options.log_directory.empty()) {
    csv_path = (std::filesystem::path(options.log_directory) / "sessions.csv")
                   .string();
  }
  s->options = std::move(options);
  try {
    s->reaper = std::make_unique<Reaper>(std::move(csv_path));
  } catch (const std::system_error& e) {
    return vkc::Status::io_error(
        std::string("MobileLane::create: cannot start the reaper's thread: ") +
        e.what());
  }
  return std::unique_ptr<MobileLane>(new MobileLane(std::move(s)));
}

vkc::Status MobileLane::add(SessionParts parts) {
  State& st = *state_;
  if (st.closed) {
    return vkc::Status::invalid_argument("MobileLane::add: the lane is closed");
  }
  if (!parts.phone || !parts.tracker || !parts.log) {
    return vkc::Status::invalid_argument(
        "MobileLane::add: a session needs its phone, its tracker and its log");
  }
  const bool by_tag = parts.placement == Placement::Tag;
  if (by_tag != (parts.tags != nullptr)) {
    return vkc::Status::invalid_argument(
        "MobileLane::add: a session placed by the tag needs its watch, and "
        "only one placed so takes one");
  }
  auto s = std::make_unique<PhoneSession>();
  s->device = st.device;
  s->allocator = st.allocator;
  if (parts.prep) {
    s->prep = std::move(parts.prep);
  } else {
    VKC_ASSIGN(sensor::GpuFramePrep prep,
               sensor::GpuFramePrep::create(*st.device, *st.allocator));
    s->prep.emplace(std::move(prep));
  }
  if (parts.timer) {
    s->timer = std::move(parts.timer);
  } else {
    VKC_ASSIGN(vkc::GpuTimer timer, vkc::GpuTimer::create(*st.device));
    s->timer.emplace(std::move(timer));
  }
  s->fill = st.fill;
  if (st.fill.colour == FillColour::Picture) {
    if (parts.picture_prep) {
      s->picture_prep = std::move(parts.picture_prep);
    } else {
      VKC_ASSIGN(sensor::GpuFramePrep prep,
                 sensor::GpuFramePrep::create(*st.device, *st.allocator,
                                              picture_prep_config()));
      s->picture_prep.emplace(std::move(prep));
    }
  }
  s->phone = std::move(parts.phone);
  s->tracker = std::move(parts.tracker);
  s->log = std::move(parts.log);
  s->by_tag = by_tag;
  s->placed = !by_tag;
  s->tags = std::move(parts.tags);
  if (by_tag) s->say = std::move(parts.say);
  s->lives = static_cast<bool>(parts.presence);
  s->presence = std::move(parts.presence);
  if (s->lives) s->on_error = std::move(parts.on_error);
  s->name = std::move(parts.name);
  s->placement = parts.placement;
  st.sessions.push_back(std::move(s));
  return {};
}

vkc::Result<std::vector<sender::Filler>> MobileLane::next(
    const vol::VoxelBlockGrid& grid, std::uint64_t next_set_ns,
    GapBudget budget) {
  State& st = *state_;
  std::vector<sender::Filler> out;
  if (st.closed) return out;
  if (!st.origin_ns) st.origin_ns = next_set_ns;
  // One session the source made, a gap at most.
  if (st.source) {
    VKC_ASSIGN(std::optional<SessionParts> ready, st.source->take());
    if (ready) {
      const vkc::Status added = add(std::move(*ready));
      if (!added.ok()) {
        if (device_lost(added)) return added;
        std::fprintf(stderr, "warning: phones: a session not joined: %s\n",
                     added.message().c_str());
      }
    }
  }
  VKC_TRY(st.adopt(next_set_ns));
  VKC_TRY(st.gap(grid, next_set_ns, budget));
  VKC_TRY(st.drain(&out));
  for (const std::unique_ptr<PhoneSession>& s : st.sessions) {
    if (s->active()) s->tell();
    if (s->stage.load(std::memory_order_acquire) != Stage::Retiring ||
        !s->done()) {
      continue;
    }
    if (s->comes_and_goes() && st.reaper) {
      st.hand_over(*s, /*keep_log=*/false);  // it leaves
    } else {
      // A file's, done, prepares nothing more: its preps and timer go (a
      // frame handed out holds its own buffers), and their query pools with
      // them, before its watch and phone, which stay until the close.
      s->picture_prep.reset();
      s->prep.reset();
      s->timer.reset();
    }
  }
  return out;
}

vkc::Status MobileLane::close() {
  State& st = *state_;
  if (st.closed) return {};
  st.closed = true;
  if (st.source) st.source->close();
  vkc::Status lost;
  for (const std::unique_ptr<PhoneSession>& s : st.sessions) {
    const Stage now = s->stage.load(std::memory_order_acquire);
    if (now == Stage::Added || now == Stage::Reaping || now == Stage::Left) {
      continue;
    }
    // Placed, not fused: the run is over. One retired before takes its
    // watch's news once more, as a search may have ended since.
    const vkc::Status retired =
        now == Stage::Retiring ? s->take_news() : s->retire("the run's end");
    if (lost.ok() && device_lost(retired)) lost = retired;
    if (st.reaper) {
      st.hand_over(*s, /*keep_log=*/true);
    } else {
      s->last_stats = s->phone->stats();
      s->left_ns = s->set_ns;
      let_go(*s, /*keep_log=*/true);
      s->stage.store(Stage::Left, std::memory_order_release);
    }
  }
  if (st.reaper) st.reaper->finish();
  return lost;
}

bool MobileLane::enabled() const noexcept {
  return state_->live || !state_->sessions.empty();
}

bool MobileLane::live() const noexcept { return state_->live; }

MobileLane::Counts MobileLane::counts() const noexcept {
  Counts total;
  for (const std::unique_ptr<PhoneSession>& s : state_->sessions) {
    add_to(total, s->counts);
  }
  return total;
}

std::vector<MobileLane::SessionView> MobileLane::sessions() const {
  std::vector<SessionView> out;
  for (const std::unique_ptr<PhoneSession>& s : state_->sessions) {
    SessionView v;
    v.name = s->name;
    v.placement = s->placement;
    v.ordinal = s->ordinal;
    v.index = s->index;
    v.holding = s->holding();
    v.stage = s->stage.load(std::memory_order_acquire);
    v.waiting = s->queue.size();
    // The reaper's while it lets the session go, its pictures with it.
    v.pictures = v.stage == Stage::Reaping ? 0 : s->pictures.size();
    v.counts = s->counts;
    // The reaper's while it lets the session go.
    v.log = v.stage == Stage::Reaping ? nullptr : s->log.get();
    out.push_back(std::move(v));
  }
  return out;
}

const TrackLog* MobileLane::log() const noexcept {
  if (state_->sessions.empty()) return nullptr;
  const PhoneSession& s = *state_->sessions.front();
  if (s.stage.load(std::memory_order_acquire) == Stage::Reaping) return nullptr;
  return s.log.get();
}

std::string MobileLane::report() const {
  const State& st = *state_;
  const std::vector<std::unique_ptr<PhoneSession>>& sessions = st.sessions;
  const std::uint64_t origin = st.origin_ns.value_or(0);
  if (!st.live && sessions.size() == 1) {
    return sessions.front()->report("mobile", origin);
  }
  std::string out;
  for (const std::unique_ptr<PhoneSession>& s : sessions) {
    char label[64];
    if (s->ordinal != 0) {
      std::snprintf(label, sizeof(label), "mobile %02u",
                    static_cast<unsigned>(s->ordinal));
    } else {
      std::snprintf(label, sizeof(label), "mobile --");
    }
    out += s->report(std::string(label) + " (" + s->name + ")", origin);
  }
  if (!st.live) return out;
  // The lane's totals, then its source's lines.
  sensor::SensorStats phones{};
  std::size_t waiting = 0;
  std::size_t left = 0;
  for (const std::unique_ptr<PhoneSession>& s : sessions) {
    const Stage now = s->stage.load(std::memory_order_acquire);
    const bool gone = now == Stage::Reaping || now == Stage::Left;
    if (now == Stage::Added) continue;
    const sensor::SensorStats ps = gone ? s->last_stats : s->phone->stats();
    phones.received += ps.received;
    phones.failed += ps.failed;
    waiting += s->queue.size();
    if (now == Stage::Left) ++left;
  }
  char label[96];
  std::snprintf(label, sizeof(label), "mobile, %u session(s), %zu left",
                static_cast<unsigned>(st.joins), left);
  const Counts total = counts();
  out += counts_line(label, total, phones, waiting);
  if (st.joins != 0) {
    char buf[256];
    std::snprintf(buf, sizeof(buf),
                  "  the gap: %llu deferred (%llu by its budget), %llu "
                  "probe(s); a hop estimated at %.1f ms (p90 of the last "
                  "%zu), the last gap's room %s\n",
                  static_cast<unsigned long long>(total.deferred),
                  static_cast<unsigned long long>(total.over_budget),
                  static_cast<unsigned long long>(st.probes),
                  double(st.hop_cost.p90().count()) * 1e-6, kHopSamples,
                  st.room_text().c_str());
    out += buf;
  }
  if (st.source) out += st.source->report();
  return out;
}

}  // namespace vr_example
