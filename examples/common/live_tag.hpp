// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/live_tag.hpp
/// @brief A phone placed in the rig's world by the origin tag, found live in
///        its pictures: how a run with `source.mobile` and no
///        `source.mobile_registration` registers the phone.
///
/// **Stills.** The lane shows the watch each frame it pairs, in the phone's
/// own (ARKit's) world, before it lets the frame's picture go (@ref
/// TagWatch::see). A frame is a candidate when it is @ref kSettleNs past the
/// first frame shown (ARKit settling) and at least @ref kTagEveryNs of frame
/// time after the last candidate, however fast its camera moves: the hold's
/// gates refuse a wrong pose, and the rig chain's fits correct a rough one
/// (live_mobile.hpp). A candidate's Y plane is copied to the host (sensor's
/// `PictureReadback`, `Planes::Luma`) and searched for calib's `kOriginTag`
/// (`detect_tag`): a still when the picture holds exactly one print of it.
///
/// **Holds.** @ref kHoldStills stills within @ref kHoldSpanNs, with no
/// @ref TagWatch::split between them, are a hold; its stills are then spent,
/// and a still older than the span drops out of the one being gathered.
/// calib's `tracked_world_from_tag` solves it, ARKit's +y up, at the edge of
/// the world's tag (`AprilTagWorld::size_m`: the printed edge), and its gates
/// pass or refuse it. The first hold that passes registers the session
/// (`register_tracked`, in memory: @ref TagNews::registered). Later holds
/// that pass are reported, never applied. A search or a solve that calib or
/// OpenCV refuses, or a picture the copy cannot read, is reported, never
/// fatal.
///
/// **Threads.** With @ref TagWatch::Options::background, the copy, the search
/// and the solve run on the watch's own thread: @ref TagWatch::see hands it
/// the picture through a one-slot box, newest wins, letting go of the one it
/// replaces, and never waits; @ref TagWatch::take returns what the thread
/// found, each hold with its registration in the same take. Without it, all
/// of it runs inside @ref TagWatch::see, so a replay repeats exactly. The
/// copy is one submit on the run's device, waited on by the watch alone;
/// nothing of the run waits on the watch. The search runs on that one
/// thread, never OpenCV's pool: @ref TagWatch::create sets OpenCV's threads
/// to 1 (`cv::setNumThreads`), for the whole process.
///
/// A build without `VR_WITH_CALIB` has the same header and refuses: its
/// @ref TagWatch::check and @ref TagWatch::create return
/// `Status::Code::Unsupported`.
///
/// @code
/// VKC_TRY(TagWatch::check(rig.world));  // before the wait for the phone
/// VKC_ASSIGN(std::unique_ptr<TagWatch> tags,
///            TagWatch::create(device, allocator,
///                             {id, session, rig.world, /*background=*/true}));
/// // each frame paired, its picture on the device, in the phone's world:
/// VKC_TRY(tags->see(frame));
/// VKC_ASSIGN(TagNews news, tags->take());
/// if (news.registered) place_by(news.registered->tracking_to_world);
/// @endcode

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/fwd.hpp"
#include "volumetric_kit/sensor/camera/array_calibration.hpp"
#include "volumetric_kit/sensor/camera/geometry.hpp"
#include "volumetric_kit/sensor/camera/tracked_registration.hpp"
#include "volumetric_kit/sensor/rgbd_frame.hpp"

namespace vr_example {

namespace sensor = volumetric_kit::sensor;
namespace vkc = volumetric_kit::core;

/// A hold's stills.
inline constexpr std::size_t kHoldStills = 6;
/// A hold's span, its first still's frame time to its last's, at most.
inline constexpr std::uint64_t kHoldSpanNs = 3'000'000'000;
/// A candidate per this much frame time, at most: 5 Hz.
inline constexpr std::uint64_t kTagEveryNs = 200'000'000;
/// No candidate before this much frame time after the first frame shown.
inline constexpr std::uint64_t kSettleNs = 2'000'000'000;

/// @brief One hold the watch solved: a row of the track log's `tags.csv`.
struct TagHold {
  /// @brief What the hold did.
  enum class Use : std::uint8_t {
    Registered,  ///< The first that passed: the session's registration.
    Passed,      ///< Passed later: reported, not applied.
    Refused,     ///< A gate, the solve or calib refused it.
  };
  std::uint64_t first_pts_ns = 0;  ///< Its first still's frame time.
  std::uint64_t last_pts_ns = 0;   ///< Its last still's.
  std::uint32_t stills = 0;        ///< Its stills: @ref kHoldStills.
  Use use = Use::Refused;
  /// calib's name for the miss (`to_string(TagMiss)`), or `error` for a
  /// solve or registration calib or OpenCV refused; empty when it passed.
  std::string miss;
  std::string reason;   ///< calib's words for a refusal; empty when it passed.
  double rms_px = 0.0;  ///< The fit's RMS (pixels); 0 with no fit.
  double worst_px = 0.0;        ///< Its worst corner (pixels); 0 with no fit.
  double range_m = 0.0;         ///< The stills' median distance from the tag.
  double off_normal_deg = 0.0;  ///< Their median angle off its normal.
  /// The tracking world to the rig's by this hold; the identity for a
  /// refusal.
  sensor::camera::Mat4d tracking_to_world{1.0};
};

/// @return The name `tags.csv` gives @p use: `registered`, `passed` or
///         `refused`.
inline const char* to_string(TagHold::Use use) noexcept {
  switch (use) {
    case TagHold::Use::Registered:
      return "registered";
    case TagHold::Use::Passed:
      return "passed";
    case TagHold::Use::Refused:
      return "refused";
  }
  return "?";
}

/// @brief What the watch found since the last @ref TagWatch::take.
struct TagNews {
  std::vector<TagHold> holds;  ///< Every hold solved, in order.
  /// Once: the session's registration, from the first hold that passed, as
  /// `register_tracked` gives it (`validate_tracked_registration` accepts
  /// it).
  std::optional<sensor::camera::TrackedRegistration> registered;
  /// Searches calib or OpenCV refused, and pictures the copy could not read:
  /// their messages.
  std::vector<std::string> failures;
};

/// @brief Finds the origin tag in a phone's pictures and solves its holds
///        (above). An aggregate: neither copied nor moved.
class TagWatch {
 public:
  /// @brief Whose pictures, in which world, on which thread.
  struct Options {
    std::string sensor;  ///< The phone's id (`SensorInfo::id`).
    /// Its session (`link::RgbdStreamSensor::session_id`): never all zero.
    std::array<std::uint8_t, 16> session{};
    sensor::camera::WorldFrame
        world;  ///< The rig's (`ArrayCalibration::world`).
    /// On its own thread (a live run), or inside @ref see (a replay).
    bool background = false;
  };

  /// @brief What the watch did.
  struct Counts {
    std::uint64_t seen = 0;        ///< Frames shown.
    std::uint64_t candidates = 0;  ///< Searched.
    std::uint64_t stills = 0;      ///< Searched, with the tag once.
    std::uint64_t holds = 0;       ///< Solved.
    std::uint64_t refused = 0;     ///< Of those, refused.
    /// Candidates let go unsearched: one newer took the box while the
    /// thread was busy (background only).
    std::uint64_t replaced = 0;
    /// Searches calib or OpenCV refused, and pictures the copy could not
    /// read.
    std::uint64_t failed = 0;
  };

  /// @return OK when this build registers by the tag and @p world is the
  ///         origin tag the watch finds (calib's `kOriginTag`: its family
  ///         and id, no sensor beside it); `Status::Code::Unsupported` in a
  ///         build without `VR_WITH_CALIB`, naming `-DVR_WITH_CALIB=ON` and
  ///         `source.mobile_registration`; `Status::Code::InvalidArgument`
  ///         for a world with no tag, another tag, a sensor beside it, or a
  ///         tag of no edge, naming `source.mobile_registration`. Cheap: the
  ///         lane calls it before it waits for the phone.
  static vkc::Status check(const sensor::camera::WorldFrame& world);

  /// @param device     The run's device; it and @p allocator outlive the
  ///                   watch.
  /// @param allocator  What the copy's host memory comes from.
  /// @param options    Whose pictures, in which world, on which thread.
  ///                   OpenCV's threads are set to 1 (above).
  /// @return The watch; @ref check's refusal; `Status::Code::InvalidArgument`
  ///         for an empty sensor id or an all-zero session; or the
  ///         readback's error.
  static vkc::Result<std::unique_ptr<TagWatch>> create(
      const vkc::Device& device, vkc::Allocator& allocator, Options options);

  /// Joins its thread, after the search it is running; the boxed picture is
  /// let go, and so is what that search found since the last @ref take.
  ~TagWatch();
  TagWatch(const TagWatch&) = delete;
  TagWatch& operator=(const TagWatch&) = delete;
  TagWatch(TagWatch&&) = delete;
  TagWatch& operator=(TagWatch&&) = delete;

  /// @brief A frame the lane paired, in the phone's own world, its picture
  ///        on the device; every one, in time order.
  /// @return OK; a frame with no device picture or a pose not rigid is
  ///         counted seen and skipped. Without background, the copy's device
  ///         error (a picture it refuses is a failure, counted).
  vkc::Status see(const sensor::RgbdFrame& frame);

  /// @brief A loss in the frames: the hold being gathered ends.
  void split() noexcept;

  /// @return What was found since the last take; or, once, the watch
  ///         thread's device or copy error, after which it searches no more
  ///         (calib's and OpenCV's refusals are news).
  vkc::Result<TagNews> take();

  /// @return The counts; with background, as of the last @ref take.
  const Counts& counts() const noexcept;

 private:
  struct State;  // live_tag.cpp's; live_tag_off.cpp makes none
  explicit TagWatch(std::unique_ptr<State> state) noexcept;
  std::unique_ptr<State> state_;
};

}  // namespace vr_example
