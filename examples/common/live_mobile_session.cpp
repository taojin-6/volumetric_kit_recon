// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "live_mobile_internal.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include "live_mobile.hpp"
#include "volumetric_kit/core/base/check.hpp"
#include "volumetric_kit/core/base/stage_metrics.hpp"
#include "volumetric_kit/core/vulkan/command_batch.hpp"
#include "volumetric_kit/core/vulkan/gpu_timer.hpp"
#include "volumetric_kit/core/vulkan/vk_result.hpp"
#include "volumetric_kit/recon/tsdf/source.hpp"
#include "volumetric_kit/sensor/camera/tracked_registration.hpp"
#include "volumetric_kit/sensor/prep/gpu_frame_prep.hpp"

namespace vr_example {

sender::Filler to_filler(track::Released&& released, std::uint32_t index) {
  const bool rig_anchored =
      released.placement == track::Placement::Anchored && released.driver == 0;
  sender::Filler f;
  f.frame = track::corrected(std::move(released.frame), released.correction);
  f.tier = rig_anchored ? tsdf::FillTier::Anchored : tsdf::FillTier::Unanchored;
  f.index = index;
  return f;
}

gnet::ViewerNote lane_note(const MobileLane::Status& status,
                           std::uint64_t set_ns) {
  using Tone = gnet::ViewerNote::Tone;
  gnet::ViewerNote note;
  if (status.registered) {
    note.tone = Tone::Good;
    note.line = "Tag found, registered";
  } else if (status.refused_ns != 0 &&
             set_ns < status.refused_ns + kRefusedShownNs) {
    note.tone = Tone::Caution;
    note.line = "Tag refused, trying again";
  } else if (status.gathered != 0) {
    note.tone = Tone::Good;
    note.line = "Tag seen: " + std::to_string(status.gathered) + " of " +
                std::to_string(kHoldStills);
  } else {
    note.tone = Tone::Neutral;
    note.line = "Looking for the tag";
  }
  return note;
}

bool note_due(const gnet::ViewerNote& now,
              const std::optional<gnet::ViewerNote>& last,
              std::uint64_t since_ns) {
  return !last || now.tone != last->tone || now.line != last->line ||
         since_ns >= kNoteAtLeastNs;
}

namespace detail {

namespace {

// A CSV field: a comma or a line break made harmless.
std::string field(std::string s) {
  std::replace(s.begin(), s.end(), ',', '_');
  std::replace(s.begin(), s.end(), '\n', ' ');
  std::replace(s.begin(), s.end(), '\r', ' ');
  return s;
}

// What a session holds let go, in the plan's order: its watch, then its
// phone (the link's slot frees as the sensor goes), then its tracker, prep
// and timer.
void release(PhoneSession& s) noexcept {
  s.tags.reset();  // its thread may hold a picture of the phone's
  if (s.phone) {
    s.phone->stop();
    s.phone.reset();
  }
  s.released.store(true, std::memory_order_release);
  s.tracker.reset();
  s.pictures.clear();
  s.picture_prep.reset();
  s.prep.reset();
  s.timer.reset();
}

}  // namespace

bool device_lost(const vkc::Status& s) {
  const std::optional<VkResult> r = vkc::vk_result(s);
  return r && *r == VK_ERROR_DEVICE_LOST;
}

bool ran_out_of_memory(const vkc::Status& s) {
  if (s.domain() == vkc::Status::Code::OutOfMemory) return true;
  const std::optional<VkResult> r = vkc::vk_result(s);
  return r && (*r == VK_ERROR_OUT_OF_DEVICE_MEMORY ||
               *r == VK_ERROR_OUT_OF_HOST_MEMORY);
}

std::string counts_line(const std::string& label, const MobileLane::Counts& c,
                        const sensor::SensorStats& st, std::size_t waiting) {
  char buf[512];
  std::snprintf(
      buf, sizeof(buf),
      "%s: %llu records (%llu received, %llu failed), %llu tracked, %llu "
      "skipped, %llu passed over, %llu splits; fill %llu anchored, %llu "
      "unanchored, %llu dropped, %llu fused, %zu waiting\n",
      label.c_str(), static_cast<unsigned long long>(c.polled),
      static_cast<unsigned long long>(st.received),
      static_cast<unsigned long long>(st.failed),
      static_cast<unsigned long long>(c.paired),
      static_cast<unsigned long long>(c.skipped),
      static_cast<unsigned long long>(c.passed_over),
      static_cast<unsigned long long>(c.splits),
      static_cast<unsigned long long>(c.anchored),
      static_cast<unsigned long long>(c.unanchored),
      static_cast<unsigned long long>(c.dropped),
      static_cast<unsigned long long>(c.fused), waiting);
  return buf;
}

std::string PhoneSession::label() const {
  char nn[16];
  std::snprintf(nn, sizeof(nn), "%02u", static_cast<unsigned>(ordinal));
  const std::string& id = log->run().source.session_id;
  return std::string(nn) + " " + (id.empty() ? name : id.substr(0, 8)) + " (" +
         name + ")";
}

vkc::Status PhoneSession::join(std::uint32_t at, std::uint32_t mobile_index,
                               std::uint64_t now_ns) {
  ordinal = at;
  index = mobile_index;
  joined_ns = now_ns;
  set_ns = now_ns;
  stage.store(Stage::Live, std::memory_order_release);
  log_written(log->joined(at, mobile_index, now_ns));
  // What the link held while the cameras opened, or before a late join,
  // comes in the first takes, passed by but for each take's newest.
  VKC_TRY(phone->set_queue_depth(kPhoneQueue));
  VKC_TRY(phone->start());
  started = true;
  return {};
}

void PhoneSession::log_written(const vkc::Status& written) {
  if (written.ok() || log_failed) return;
  log_failed = true;
  std::fprintf(stderr,
               "warning: phone %s: a write of its log failed (%s); the "
               "session goes on\n",
               name.c_str(), written.message().c_str());
}

void PhoneSession::watch_rig(const track::TrackStep& step) {
  for (const track::ChainFit& f : step.fits) {
    if (f.chain != 0 || !f.refused.empty()) continue;  // the rig chain's
    if (!by_tag || rig_anchored ||
        f.judged.state != track::TrackState::Anchored) {
      continue;
    }
    rig_anchored = true;
    const Mat4f at = registered_camera();
    const auto [mm, deg] = pose_gap(f.judged.correction * at, at);
    std::printf(
        "phone: first rig fit: %.1f cm / %.1f deg from the tag's "
        "registration\n",
        mm / 10.0, deg);
  }
}

void PhoneSession::register_by(const sensor::camera::TrackedRegistration& reg,
                               const TagHold& hold) {
  placed = true;
  tracking_to_world = reg.tracking_to_world;
  registered_after = hold.last_pts_ns - first_paired.value_or(0);
  std::string where = "tag";
  if (!log->directory().empty()) {
    where = (std::filesystem::path(log->directory()) / "registration.json")
                .string();
    const vkc::Status written = sensor::camera::write_tracked_registration(
        where, reg, vkc::WriteMode::CreateNew);
    log_written(written);
    if (!written.ok()) where = "tag";
  }
  log_written(log->registered(std::move(where), Mat4f(reg.tracking_to_world)));
}

void PhoneSession::apply(TagNews news) {
  for (const std::string& f : news.failures) {
    if (!first_tag_failure.empty()) break;
    first_tag_failure = f;
    std::printf("phone: tag search failed (counted, the run goes on): %s\n",
                f.c_str());
  }
  for (const TagHold& h : news.holds) {
    log_written(log->write(h));
    if (h.use == TagHold::Use::Refused) {
      refused_ns = set_ns;
      std::printf("phone: tag hold refused: %s: %s\n", h.miss.c_str(),
                  h.reason.c_str());
      continue;
    }
    // The watch's one hold that passes, its last news.
    VKC_CHECK(news.registered.has_value(),
              "MobileLane: a registering hold without its registration");
    register_by(*news.registered, h);
    std::printf(
        "phone: tag hold: %u stills over %.1f s, RMS %.1f px, worst %.1f "
        "px, %.1f m, %.0f deg off normal -> registered\n",
        h.stills, double(h.last_pts_ns - h.first_pts_ns) * 1e-9, h.rms_px,
        h.worst_px, h.range_m, h.off_normal_deg);
  }
  tag_counts = tags->counts();
}

vkc::Status PhoneSession::take_news() {
  if (!tags) return {};
  VKC_ASSIGN(TagNews news, tags->take());
  apply(std::move(news));
  return {};
}

void PhoneSession::tell() {
  if (!say) return;
  gnet::ViewerNote now = lane_note(status(), set_ns);
  if (!note_due(now, said, set_ns - said_ns)) return;
  say(now);
  said = std::move(now);
  said_ns = set_ns;
}

void PhoneSession::take(const track::TrackStep& step) {
  watch_rig(step);
  log_written(log->write(step));
  for (const track::Released& r : step.released) {
    track::Released copy = r;
    // Its picture, if one was kept, at its own camera: corrected with the
    // frame's depth camera below.
    const auto kept = pictures.find(copy.frame.timestamp_ns);
    if (kept != pictures.end()) {
      copy.frame.color = std::move(kept->second.color);
      copy.frame.color_camera = kept->second.color_camera;
      copy.frame.color_encoding = kept->second.color_encoding;
      pictures.erase(kept);
    }
    sender::Filler f = to_filler(std::move(copy), index);
    ++(f.tier == tsdf::FillTier::Anchored ? counts.anchored
                                          : counts.unanchored);
    queue.push_back(std::move(f));
    if (queue.size() > kPlacedCap) {
      queue.pop_front();
      ++counts.shed;
    }
  }
  for (const std::uint64_t dropped : step.dropped_ns) pictures.erase(dropped);
  counts.dropped += step.dropped_ns.size();
}

void PhoneSession::flush() {
  take(tracker->flush());
  pictures.clear();
}

bool PhoneSession::picture_due(const sensor::RgbdFrame& frame) const {
  return fill.colour == FillColour::Picture && picture_prep &&
         frame.has_color() && tracked >= next_picture;
}

vkc::Status PhoneSession::begin_picture(InFlight& f, sensor::RgbdFrame frame) {
  auto p = std::make_unique<PictureInFlight>(*device, *allocator);
  p->frame = std::move(frame);
  // No metrics: the log's rows stay those of the frame's own prep.
  VKC_ASSIGN(p->picture, picture_prep->record(p->batch, p->frame));
  // Its depth is let go: the picture alone is kept.
  p->picture.depth.reset();
  VKC_ASSIGN(p->pending,
             p->batch.submit_async(sensor::ready_points(p->frame), {}));
  f.picture = std::move(p);
  return {};
}

vkc::Status PhoneSession::finish_picture(InFlight& f) {
  if (!f.picture) return {};
  const std::unique_ptr<PictureInFlight> p = std::move(f.picture);
  const vkc::Status ran = p->pending.wait();
  if (!ran.ok()) {
    p->pending = vkc::PendingBatch();  // out of flight once it is gone
    picture_refused(ran);
    return device_lost(ran) ? ran : vkc::Status{};
  }
  ++counts.pictures;
  const std::uint64_t at = p->picture.timestamp_ns;
  pictures.insert_or_assign(at, std::move(p->picture));
  // The phone's decoded picture goes here, with the batch that read it.
  return {};
}

bool PhoneSession::take_records(std::uint64_t next_set) {
  std::vector<sensor::RgbdFrame> got;
  const vkc::Status st = phone->drain(&got);
  if (!st.ok() && counts.errors++ == 0) first_error = st.message();
  if (device_lost(st) && lost.ok()) lost = st;
  if (!got.empty()) {
    const sensor::RgbdFrame& newest = got.back();
    const std::int64_t pin =
        std::int64_t(newest.timestamp_ns) - std::int64_t(next_set);
    if (!offset || (newest.arrival_ns != 0 && pin > *offset)) offset = pin;
  }
  for (sensor::RgbdFrame& f : got) {
    ++counts.polled;
    if (last_seq && f.sequence > *last_seq + 1 &&
        f.timestamp_ns >= last_pts + kSplitGapNs) {
      splits.push_back(f.timestamp_ns);
    }
    last_seq = f.sequence;
    last_pts = f.timestamp_ns;
    held.push_back(std::move(f));
  }
  return !got.empty();
}

std::optional<sensor::RgbdFrame> PhoneSession::pair(std::uint64_t next_set) {
  std::optional<sensor::RgbdFrame> due;
  for (;;) {
    if (held.empty() && !take_records(next_set)) break;
    if (std::int64_t(held.front().timestamp_ns) >
        std::int64_t(next_set) + *offset) {
      break;
    }
    if (due) ++counts.skipped;
    due = std::move(held.front());
    held.pop_front();
  }
  return due;
}

Kind PhoneSession::classify() const {
  if (!due) return Kind::None;
  if (!placed) return Kind::Unregistered;
  return tracker->fit_due(due->timestamp_ns) ? Kind::Hop : Kind::Prep;
}

void PhoneSession::defer(bool by_budget) {
  held.push_front(std::move(*due));
  due.reset();
  ++counts.deferred;
  if (by_budget) ++counts.over_budget;
}

vkc::Result<std::unique_ptr<InFlight>> PhoneSession::begin(
    const vol::VoxelBlockGrid& grid) {
  VKC_CHECK(due.has_value(), "MobileLane: a session begun with nothing due");
  sensor::RgbdFrame frame = std::move(*due);
  due.reset();
  bool split = false;
  while (!splits.empty() && splits.front() <= frame.timestamp_ns) {
    splits.pop_front();
    split = true;
  }
  if (!first_paired) first_paired = frame.timestamp_ns;
  newest_tracking = frame.color_to_world;
  if (!placed) {
    // Shown to the watch alone, before its picture is let go here: nothing
    // is tracked before the tag, and once placed nothing is shown to the
    // watch (one tag, at the start). So no picture is both the watch's copy
    // and the fill's, each of which takes its planes over from the decoder.
    if (split) tags->split();
    VKC_TRY(tags->see(frame));
    ++counts.unregistered;
    return std::unique_ptr<InFlight>();
  }
  if (by_tag) frame.color_to_world = tracking_to_world * frame.color_to_world;
  if (split) {
    ++counts.splits;
    flush();
  }
  // Under FillColour::Picture, the whole frame of one whose picture is kept,
  // for the picture's own prep after the frame's batch is submitted.
  std::optional<sensor::RgbdFrame> whole;
  if (picture_due(frame)) whole = frame;
  // The frame's depth is prepared without its picture, whatever the fill:
  // fill fuses its tier's tint, or a picture prepared on its own path.
  frame.color = {};
  frame.color_packed = nullptr;
  frame.color_bitstream = {};
  auto f = std::make_unique<InFlight>(*device, *allocator);
  f->batch.time(*timer);
  vkc::StageMetrics& rows = log->rows();
  vkc::Result<sensor::DeviceFrame> prepared =
      prep->record(f->batch, frame, &rows);
  if (!prepared.ok()) {
    if (prepared.status().domain() != vkc::Status::Code::InvalidArgument) {
      return prepared.status();
    }
    passed_over(prepared.status());
    return std::unique_ptr<InFlight>();
  }
  vkc::Result<track::PendingStep> step =
      tracker->record(f->batch, grid, std::move(*prepared), &rows);
  if (!step.ok()) {
    if (step.status().domain() != vkc::Status::Code::InvalidArgument) {
      return step.status();
    }
    passed_over(step.status());
    // The batch goes unsubmitted, the prep's slot with it.
    return std::unique_ptr<InFlight>();
  }
  f->step = *step;
  vkc::Result<vkc::PendingBatch> submitted = f->batch.submit_async({}, {});
  if (!submitted.ok()) {
    tracker->abandon(f->step);  // none of it runs
    return submitted.status();
  }
  f->pending = std::move(submitted).value();
  if (whole) {
    // The picture's own batch, after the frame's: one refused, or one whose
    // batch fails, is let go and the frame goes on as it is.
    next_picture = tracked + fill.picture_every;
    const vkc::Status pictured = begin_picture(*f, std::move(*whole));
    if (!pictured.ok()) picture_refused(pictured);
  }
  ++tracked;
  return f;
}

vkc::Status PhoneSession::finish(InFlight& f) {
  vkc::StageMetrics& rows = log->rows();
  vkc::Status ran;
  {
    vkc::WaitScope wait(&rows, "wait: mobile");
    ran = f.pending.wait();
  }
  if (!ran.ok()) {
    f.pending = vkc::PendingBatch();  // out of flight once it is gone
    tracker->abandon(f.step);
    return ran;
  }
  rows.merge_device(f.pending.device_rows());
  VKC_ASSIGN(const track::TrackStep done, tracker->finish(f.step));
  ++counts.paired;
  // Its picture kept before the step is taken, which may place the frame.
  const vkc::Status pictured = finish_picture(f);
  take(done);
  return pictured;
}

void PhoneSession::end() {
  if (ended) return;
  ended = true;
  flush();
}

vkc::Status PhoneSession::retire(std::string why) {
  const Stage now = stage.load(std::memory_order_acquire);
  if (now == Stage::Retiring || now == Stage::Reaping || now == Stage::Left) {
    return {};
  }
  leave_reason = std::move(why);
  stage.store(Stage::Retiring, std::memory_order_release);
  due.reset();
  if (comes_and_goes()) {
    std::printf("phones: %s retires: %s\n", label().c_str(),
                leave_reason.c_str());
  }
  const vkc::Status news = take_news();
  end();
  return device_lost(news) ? news : vkc::Status{};
}

vkc::Status PhoneSession::host(std::uint64_t next_set_ns,
                               const LaneOptions& options) {
  set_ns = next_set_ns;
  if (comes_and_goes()) {
    const bool here = presence();
    const Stage now = stage.load(std::memory_order_acquire);
    if (here && now == Stage::Away) {
      stage.store(Stage::Live, std::memory_order_release);
      std::printf("phones: %s is back after %.1f s away\n", label().c_str(),
                  double(set_ns - away_since) * 1e-9);
    } else if (!here && now == Stage::Live &&
               (seen || set_ns >= joined_ns + kFirstSeenNs)) {
      // One never seen connected is away from its join.
      stage.store(Stage::Away, std::memory_order_release);
      away_since = seen ? set_ns : joined_ns;
      settled = false;
      std::printf(
          "phones: %s away; it retires at %.0f s away unless it comes "
          "back\n",
          label().c_str(), double(options.grace_ns) * 1e-9);
    }
    seen = seen || here;
    if (stage.load(std::memory_order_acquire) == Stage::Away) {
      const std::uint64_t away = set_ns > away_since ? set_ns - away_since : 0;
      if (away >= options.grace_ns) {
        char why[64];
        std::snprintf(why, sizeof(why), "away %.0f s", double(away) * 1e-9);
        return retire(why);
      }
      if (away >= options.settle_ns && !settled) {
        settled = true;  // what it holds decided, as no later frame shares
        flush();
      }
    }
  }
  VKC_TRY(take_news());
  due = pair(next_set_ns);
  if (!lost.ok()) return lost;
  return {};
}

vkc::Status PhoneSession::at_end() {
  if (!held.empty() || !phone->exhausted()) return {};
  return retire(comes_and_goes() ? "its stream ended" : "its file ended");
}

vkc::Status PhoneSession::discard() {
  held.clear();
  due.reset();
  std::vector<sensor::RgbdFrame> got;
  if (!phone || !started) return {};
  const vkc::Status st = phone->drain(&got);
  return device_lost(st) ? st : vkc::Status{};
}

std::string PhoneSession::report(const std::string& label,
                                 std::uint64_t origin_ns) const {
  const MobileLane::Counts& c = counts;
  const Stage now = stage.load(std::memory_order_acquire);
  const bool gone = now == Stage::Reaping || now == Stage::Left;
  const sensor::SensorStats st = gone ? last_stats : phone->stats();
  std::string out = counts_line(label, c, st, queue.size());
  char buf[512];
  if (by_tag) {
    const TagWatch::Counts& t = tag_counts;
    char when[96];
    if (registered_after) {
      std::snprintf(when, sizeof(when),
                    "registered at %.1f s, %llu frames before it",
                    double(*registered_after) * 1e-9,
                    static_cast<unsigned long long>(c.unregistered));
    } else {
      std::snprintf(when, sizeof(when), "not registered, %llu frames",
                    static_cast<unsigned long long>(c.unregistered));
    }
    std::snprintf(buf, sizeof(buf),
                  "  tag: %llu searched, %llu stills, %llu holds (%llu "
                  "refused), %llu replaced, %llu failed; %s\n",
                  static_cast<unsigned long long>(t.candidates),
                  static_cast<unsigned long long>(t.stills),
                  static_cast<unsigned long long>(t.holds),
                  static_cast<unsigned long long>(t.refused),
                  static_cast<unsigned long long>(t.replaced),
                  static_cast<unsigned long long>(t.failed), when);
    out += buf;
  }
  if (!first_refusal.empty()) {
    out += "  first passed over: " + first_refusal + "\n";
  }
  if (c.errors != 0) {
    out += "  " + std::to_string(c.errors) +
           " phone error(s), the first: " + first_error + "\n";
  }
  if (c.shed != 0) {
    out += "  " + std::to_string(c.shed) + " placed let go past " +
           std::to_string(kPlacedCap) + " waiting\n";
  }
  if (c.unfit != 0) {
    out += "  " + std::to_string(c.unfit) +
           " placed the send would refuse, the first: " + first_unfit + "\n";
  }
  if (c.deferred != 0) {
    out += "  " + std::to_string(c.deferred) +
           " deferred to a later gap (one hop a gap; " +
           std::to_string(c.over_budget) + " by the gap's budget)\n";
  }
  if (fill.colour == FillColour::Picture) {
    out += "  pictures: " + std::to_string(c.pictures) +
           " prepared for fill, one a " + std::to_string(fill.picture_every) +
           " frame(s) tracked; " + std::to_string(pictures_refused) +
           " let go\n";
    if (pictures_refused != 0) {
      out += "  first picture let go: " + first_picture_refusal + "\n";
    }
  }
  if (comes_and_goes() && !leave_reason.empty()) {
    const auto in = [origin_ns](std::uint64_t ns) {
      return double(ns > origin_ns ? ns - origin_ns : 0) * 1e-9;
    };
    std::snprintf(buf, sizeof(buf),
                  "  joined %.1f s in, %s %.1f s in: ", in(joined_ns),
                  now == Stage::Left ? "left" : "retired",
                  in(now == Stage::Left ? left_ns : set_ns));
    out += buf + leave_reason + "\n";
  }
  return out;
}

std::string PhoneSession::row(bool join) const {
  const RunSource& src = log->run().source;
  const std::string directory =
      log->directory().empty()
          ? std::string()
          : std::filesystem::path(log->directory()).filename().string();
  std::string s = join ? "join," : "leave,";
  s += std::to_string(join ? joined_ns : left_ns) + "," +
       std::to_string(ordinal) + "," + field(src.session_id) + "," +
       field(name) + "," + field(src.peer) + "," + std::to_string(index) + "," +
       (by_tag ? "tag" : "registration") + "," + field(directory) + ",";
  if (join) return s + "0,0,0,\n";
  std::string why = leave_reason;
  std::replace(why.begin(), why.end(), '\n', ' ');
  std::replace(why.begin(), why.end(), '\r', ' ');
  return s + std::to_string(counts.polled) + "," +
         std::to_string(counts.paired) + "," + std::to_string(counts.fused) +
         "," + why + "\n";
}

void let_go(PhoneSession& s, bool keep_log) {
  release(s);
  if (!s.log) return;
  const vkc::Status closed = s.log->close();
  s.log_written(closed);
  vkc::Result<std::string> table = summarize(s.log->run(), s.log->directory());
  if (table.ok()) {
    std::fputs(table->c_str(), stdout);
  } else {
    s.log_written(table.status());
  }
  if (!keep_log) s.log.reset();
}

void Reaper::finish() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stopping_ = true;
  }
  wake_.notify_all();
  if (thread_.joinable()) thread_.join();
}

void Reaper::push(Job job) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    jobs_.push_back(std::move(job));
  }
  wake_.notify_one();
}

void Reaper::run() {
  for (;;) {
    Job job;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      wake_.wait(lock, [this] { return stopping_ || !jobs_.empty(); });
      if (jobs_.empty()) return;  // stopping, every job done
      job = std::move(jobs_.front());
      jobs_.pop_front();
    }
    // Host memory running out on the way is printed, and the next job
    // goes on: no phone's trouble ends the run.
    if (job.session != nullptr) {
      PhoneSession& s = *job.session;
      try {
        // Its row before let_go may let its log go: the row reads it.
        job.row = s.row(/*join=*/false);
        let_go(s, job.keep_log);
      } catch (const std::exception& e) {
        release(s);
        std::fprintf(stderr, "warning: phones: letting %s go: %s\n",
                     s.name.c_str(), e.what());
      }
      s.stage.store(Stage::Left, std::memory_order_release);
    }
    try {
      write(job.row);
    } catch (const std::exception& e) {
      std::fprintf(stderr, "warning: phones: %s: %s\n", csv_path_.c_str(),
                   e.what());
    }
  }
}

void Reaper::write(const std::string& row) {
  if (csv_path_.empty() || csv_failed_ || row.empty()) return;
  if (!csv_.is_open()) {
    const std::filesystem::path path(csv_path_);
    std::error_code ec;
    if (std::filesystem::exists(std::filesystem::symlink_status(path, ec))) {
      std::fprintf(stderr,
                   "warning: phones: %s is there already; this run writes "
                   "none\n",
                   csv_path_.c_str());
      csv_failed_ = true;
      return;
    }
    std::filesystem::create_directories(path.parent_path(), ec);
    csv_.open(path, std::ios::binary | std::ios::trunc);
    csv_ << "event,set_ns,ordinal,session_id,sensor,peer,mobile_index,"
            "placement,directory,polled,paired,fused,reason\n";
  }
  csv_ << row;
  csv_.flush();
  if (!csv_) {
    std::fprintf(stderr,
                 "warning: phones: a write of %s failed; it is written no "
                 "more\n",
                 csv_path_.c_str());
    csv_.close();
    csv_failed_ = true;
  }
}

}  // namespace detail
}  // namespace vr_example
