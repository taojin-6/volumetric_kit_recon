// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The origin tag's watch (live_tag.hpp), on calib's detector and solve: the
// build with VR_WITH_CALIB.

#include <algorithm>
#include <cmath>
#include <condition_variable>
#include <mutex>
#include <new>
#include <opencv2/core/utility.hpp>
#include <thread>
#include <utility>

#include "live_tag.hpp"
#include "volumetric_kit/calib/core/target.hpp"
#include "volumetric_kit/calib/detect/tag.hpp"
#include "volumetric_kit/calib/solve/tracked_world.hpp"
#include "volumetric_kit/calib/solve/world_tag.hpp"
#include "volumetric_kit/sensor/camera/tracked_stills.hpp"
#include "volumetric_kit/sensor/prep/picture_readback.hpp"

namespace vr_example {

namespace calib = volumetric_kit::calib;
namespace camera = volumetric_kit::sensor::camera;

namespace {

// A frame chosen for a search: its picture, held until searched or let go.
struct Candidate {
  sensor::YuvImage picture;
  camera::CameraModel model;
  camera::Mat4d cam_to_tracking{1.0};
  std::uint64_t pts = 0;
  std::uint64_t epoch = 0;  // the splits before it
};

// A candidate the tag was found in once.
struct Still {
  std::uint64_t pts = 0;
  camera::CameraModel model;
  camera::Mat4d cam_to_tracking{1.0};
  calib::DetectedMarker marker;
};

bool device_picture(const sensor::YuvImage& p) {
  return p.device != nullptr || p.image[0] != nullptr;
}

double median(std::vector<double> v) {
  if (v.empty()) return 0.0;
  std::sort(v.begin(), v.end());
  const std::size_t n = v.size();
  return n % 2 == 1 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

std::string still_name(std::uint64_t pts) {
  return "still_" + std::to_string(pts);
}

}  // namespace

struct TagWatch::State {
  Options options;
  calib::OriginTag tag;  // the one sought, at the world's edge
  std::unique_ptr<sensor::PictureReadback> readback;

  // The lane's thread alone: which frames are candidates.
  std::optional<std::uint64_t> first_pts;
  std::optional<std::uint64_t> last_candidate;
  std::uint64_t epoch = 0;
  Counts published;  // what counts() returns

  // The searcher's alone (the thread, or see() without it).
  std::vector<Still> stills;
  std::uint64_t stills_epoch = 0;
  bool registered = false;

  // Shared, under the mutex.
  std::mutex mutex;
  std::condition_variable wake;
  std::optional<Candidate> box;
  Counts counts;
  TagNews news;
  vkc::Status error;  // the thread's, until taken
  bool dead = false;  // the thread stopped at an error
  bool stopping = false;
  std::thread thread;

  void fail(const std::string& message) {
    std::lock_guard<std::mutex> lock(mutex);
    ++counts.failed;
    news.failures.push_back(message);
  }

  // A hold solved, and the registration it made, if it made one.
  struct Solved {
    TagHold hold;
    std::optional<camera::TrackedRegistration> registered;
  };

  // The hold of `held` solved: refused, passed, or the session's
  // registration. Nothing shared is touched: the caller publishes both at
  // once, so a take never gets the one without the other.
  Solved solve(const std::vector<Still>& held) {
    Solved out;
    TagHold& hold = out.hold;
    hold.first_pts_ns = held.front().pts;
    hold.last_pts_ns = held.back().pts;
    hold.stills = static_cast<std::uint32_t>(held.size());
    std::vector<calib::solve::TrackedTagView> views;
    camera::TrackedStills session;
    session.sensor = options.sensor;
    session.session = options.session;
    for (const Still& s : held) {
      views.push_back({still_name(s.pts), s.model.size, s.model,
                       s.cam_to_tracking, s.marker});
      session.stills.push_back(
          {still_name(s.pts), s.pts, s.model, s.cam_to_tracking});
    }
    // ARKit's world: +y up (TrackedWorldOptions' default).
    vkc::Result<calib::solve::TrackedWorld> found =
        calib::solve::tracked_world_from_tag(views, tag, tag.edge_m);
    if (!found.ok()) {
      hold.miss = "error";
      hold.reason = found.status().message();
      return out;
    }
    if (!found->fit) {
      hold.miss = calib::solve::to_string(found->miss);
      hold.reason = found->reason;
      return out;
    }
    const calib::solve::TrackedTagFit& fit = *found->fit;
    hold.rms_px = fit.rms_px;
    hold.worst_px = fit.worst_px;
    std::vector<double> range, off;
    for (const calib::solve::TrackedStillFit& s : fit.stills) {
      range.push_back(s.range_m);
      off.push_back(s.off_normal_deg);
    }
    hold.range_m = median(range);
    hold.off_normal_deg = median(off);
    if (registered) {
      hold.use = TagHold::Use::Passed;
      hold.tracking_to_world = fit.tracking_to_world;
      return out;
    }
    vkc::Result<camera::TrackedRegistration> reg =
        calib::solve::register_tracked(session, options.world, *found);
    if (!reg.ok()) {
      hold.miss = "error";
      hold.reason = reg.status().message();
      return out;
    }
    hold.use = TagHold::Use::Registered;
    hold.tracking_to_world = fit.tracking_to_world;
    registered = true;
    out.registered = std::move(reg).value();
    return out;
  }

  // One candidate searched, counted once its news is in.
  vkc::Status search(Candidate c) {
    vkc::Status searched = search_one(std::move(c));
    std::lock_guard<std::mutex> lock(mutex);
    ++counts.candidates;
    return searched;
  }

  // One candidate's search, and a hold solved when it completes one. Only a
  // device's or memory's error returns: calib's, OpenCV's and a picture the
  // copy refuses are failures, counted.
  vkc::Status search_one(Candidate c) {
    if (c.epoch != stills_epoch) {
      stills.clear();  // a loss since the stills gathered
      stills_epoch = c.epoch;
    }
    vkc::Result<sensor::PictureView> view =
        readback->read(c.picture, sensor::PictureReadback::Planes::Luma);
    c.picture = {};  // the view is the readback's own memory
    if (!view.ok()) {
      const vkc::Status::Code code = view.status().domain();
      if (code != vkc::Status::Code::InvalidArgument &&
          code != vkc::Status::Code::Unsupported) {
        return view.status();
      }
      fail(view.status().message());
      return {};
    }
    if (view->width != c.model.size.width ||
        view->height != c.model.size.height) {
      fail("TagWatch: a picture of " + std::to_string(view->width) + " x " +
           std::to_string(view->height) + ", its camera " +
           std::to_string(c.model.size.width) + " x " +
           std::to_string(c.model.size.height));
      return {};
    }
    const calib::detect::LumaView luma{view->width, view->height, view->stride,
                                       view->luma};
    vkc::Result<calib::detect::MarkerSearch> searched =
        calib::detect::detect_tag(luma, tag);
    if (!searched.ok()) {
      fail(searched.status().message());
      return {};
    }
    if (!searched->found) return {};
    {
      std::lock_guard<std::mutex> lock(mutex);
      ++counts.stills;
    }
    // A still older than the span drops out of the hold being gathered.
    while (!stills.empty() && c.pts - stills.front().pts > kHoldSpanNs) {
      stills.erase(stills.begin());
    }
    stills.push_back({c.pts, c.model, c.cam_to_tracking, *searched->found});
    if (stills.size() < kHoldStills) return {};
    std::vector<Still> held = std::move(stills);
    stills.clear();  // spent
    Solved solved = solve(held);
    // The hold, its registration and its counts in one lock section.
    std::lock_guard<std::mutex> lock(mutex);
    ++counts.holds;
    if (solved.hold.use == TagHold::Use::Refused) ++counts.refused;
    if (solved.registered) news.registered = std::move(solved.registered);
    news.holds.push_back(std::move(solved.hold));
    return {};
  }

  void run() {
    std::unique_lock<std::mutex> lock(mutex);
    for (;;) {
      wake.wait(lock, [this] { return stopping || box.has_value(); });
      if (stopping) return;
      Candidate c = std::move(*box);
      box.reset();
      lock.unlock();
      vkc::Status searched;
      try {
        searched = search(std::move(c));
      } catch (const std::bad_alloc&) {
        searched = vkc::Status::out_of_memory("TagWatch: the search");
      }
      lock.lock();
      if (!searched.ok()) {
        error = std::move(searched);
        dead = true;
        return;
      }
    }
  }
};

vkc::Status TagWatch::check(const sensor::camera::WorldFrame& world) {
  if (!world.apriltag) {
    return vkc::Status::invalid_argument(
        "TagWatch: the calibration's world is no tag (" +
        (world.sensor.empty() ? std::string("unsaid")
                              : "sensor " + world.sensor) +
        "), so a phone needs source.mobile_registration");
  }
  const sensor::camera::AprilTagWorld& t = *world.apriltag;
  const std::string family =
      calib::apriltag_family(calib::kOriginTag.dictionary);
  // register_tracked's own world check, but for the edge, which the watch
  // takes from the world: a world it would refuse never registers.
  if (!world.sensor.empty() || t.family != family ||
      t.id != calib::kOriginTag.id) {
    return vkc::Status::invalid_argument(
        "TagWatch: the calibration's world is " + t.family + " id " +
        std::to_string(t.id) +
        (world.sensor.empty() ? std::string()
                              : " beside sensor " + world.sensor) +
        ", not the origin tag the watch finds (" + family + " id " +
        std::to_string(calib::kOriginTag.id) +
        " alone), so a phone needs source.mobile_registration");
  }
  if (!std::isfinite(t.size_m) || t.size_m <= 0.0) {
    return vkc::Status::invalid_argument(
        "TagWatch: the calibration's tag has no edge (size_m), so a phone "
        "needs source.mobile_registration");
  }
  return {};
}

vkc::Result<std::unique_ptr<TagWatch>> TagWatch::create(
    const vkc::Device& device, vkc::Allocator& allocator, Options options) try {
  VKC_TRY(check(options.world));
  if (options.sensor.empty()) {
    return vkc::Status::invalid_argument("TagWatch: no sensor id");
  }
  if (std::all_of(options.session.begin(), options.session.end(),
                  [](std::uint8_t b) { return b == 0; })) {
    return vkc::Status::invalid_argument("TagWatch: an all-zero session");
  }
  auto s = std::make_unique<State>();
  s->tag = calib::kOriginTag;
  s->tag.edge_m = options.world.apriltag->size_m;  // the printed edge
  VKC_TRY(calib::check_origin_tag(s->tag));
  VKC_ASSIGN(s->readback, sensor::PictureReadback::create(device, allocator));
  // OpenCV's detector runs on one thread, the caller's (the watch's own), not
  // its pool of a thread a core. Process-wide: the run has no other OpenCV
  // user.
  cv::setNumThreads(1);
  s->options = std::move(options);
  if (s->options.background) {
    State* state = s.get();
    s->thread = std::thread([state] { state->run(); });
  }
  return std::unique_ptr<TagWatch>(new TagWatch(std::move(s)));
} catch (const std::bad_alloc&) {
  return vkc::Status::out_of_memory("TagWatch::create");
}

TagWatch::TagWatch(std::unique_ptr<State> state) noexcept
    : state_(std::move(state)) {}

TagWatch::~TagWatch() {
  if (!state_ || !state_->thread.joinable()) return;
  {
    std::lock_guard<std::mutex> lock(state_->mutex);
    state_->stopping = true;
  }
  state_->wake.notify_all();
  state_->thread.join();
}

vkc::Status TagWatch::see(const sensor::RgbdFrame& frame) try {
  State& s = *state_;
  const std::uint64_t pts = frame.timestamp_ns;
  if (!s.first_pts) s.first_pts = pts;
  // Sums, not differences, so a frame time before the first frame's or the
  // last candidate's is no candidate.
  const bool candidate =
      device_picture(frame.color) &&
      camera::check_rigid(frame.color_to_world).ok() &&
      pts >= *s.first_pts + kSettleNs &&
      (!s.last_candidate || pts >= *s.last_candidate + kTagEveryNs);
  Candidate c;
  if (candidate) {
    s.last_candidate = pts;
    c.picture = frame.color;
    c.model = frame.color_camera;
    c.cam_to_tracking = frame.color_to_world;
    c.pts = pts;
    c.epoch = s.epoch;
  }
  if (s.options.background) {
    std::lock_guard<std::mutex> lock(s.mutex);
    ++s.counts.seen;
    if (candidate && !s.dead) {
      if (s.box) ++s.counts.replaced;  // the one it held is let go
      s.box = std::move(c);
      s.wake.notify_one();
    }
    return {};
  }
  {
    std::lock_guard<std::mutex> lock(s.mutex);
    ++s.counts.seen;
  }
  vkc::Status searched = candidate ? s.search(std::move(c)) : vkc::Status{};
  std::lock_guard<std::mutex> lock(s.mutex);
  s.published = s.counts;
  return searched;
} catch (const std::bad_alloc&) {
  return vkc::Status::out_of_memory("TagWatch::see");
}

void TagWatch::split() noexcept { ++state_->epoch; }

vkc::Result<TagNews> TagWatch::take() {
  State& s = *state_;
  std::lock_guard<std::mutex> lock(s.mutex);
  s.published = s.counts;
  if (!s.error.ok()) {
    vkc::Status e = std::move(s.error);
    s.error = {};
    return e;
  }
  TagNews out = std::move(s.news);
  s.news = {};
  return out;
}

const TagWatch::Counts& TagWatch::counts() const noexcept {
  return state_->published;
}

}  // namespace vr_example
