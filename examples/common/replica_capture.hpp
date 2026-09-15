// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/replica_capture.hpp
/// @brief A posed Replica-SLAM RGB-D sequence played back through the
///        `sensor::ICameraCapture` contract -- the frame source every fuse
///        example drives.
///
/// The examples consume frames through the contract rather than through a
/// dataset API of their own, so a live camera is a construction-site swap: the
/// fuse loop takes an `ICameraCapture&`, waits on an empty poll until
/// `exhausted()` says the source is done, and never learns whether its frames
/// came off a disk or a sensor. This is also the one implementer of the
/// contract this repo builds and runs on every example invocation -- the tier
/// itself ships none (the 2026-08-02 decision), and its test fake exercises
/// the interface without ever producing a real frame.

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "rgbd_frame.hpp"
#include "volumetric_kit/recon/core/camera_params.hpp"
#include "volumetric_kit/recon/core/math/vector_types.hpp"
#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/sensor/camera_capture.hpp"

namespace vr_example {

namespace vr = volumetric_kit::recon;

/// @brief A Replica-SLAM RGB-D sequence -- `<scene>/results/frameNNNNNN.jpg` +
///        `depthNNNNNN.png`, per-frame poses in `<scene>/traj.txt`, intrinsics
///        in a `cam_params.json` -- as a @ref vr::sensor::ICameraCapture.
///
/// Playback is **consumer-paced**: every @ref poll hands out the next frame of
/// the sequence, decoded on demand or served from the @ref preload cache, and
/// once the last one has gone @ref exhausted turns true and every further poll
/// is empty. The frame is a non-owning view exactly as the contract says: its
/// pixels belong to this object and are valid only until the next @ref poll
/// (or @ref stop) -- *any* next poll, including the empty one that reports the
/// end of the sequence, and a poll that fails to decode. A consumer keeping a
/// frame past that point copies it into an @ref RgbdFrame of its own.
///
/// @ref open reads the intrinsics and every pose up front, then probes which
/// of the frames it will play are actually on disk (a trajectory routinely
/// lists more poses than there are images: room0 has 2000 against 400), so
/// @ref frame_count is the number of frames the sequence will really play,
/// after the limit and stride in @ref Options. The pose file lists one
/// flattened **row-major** 4x4 camera->world matrix per line, transposed into
/// the column-major @ref vr::Mat4f the pipeline uploads verbatim.
class ReplicaCapture final : public vr::sensor::ICameraCapture {
 public:
  /// @brief Which frames to play, and the depth range to stamp on them.
  struct Options {
    /// One past the highest frame index to play (a fuse example's
    /// `--max-frames`). Clamped to the frames present on disk.
    std::size_t frame_limit = std::numeric_limits<std::size_t>::max();
    /// Play every N-th frame (`--stride`). Must be >= 1.
    std::size_t frame_stride = 1;
    /// Reject depth nearer than this (metres); stamped on every frame's depth
    /// camera. The fusion gate, so the driver's knob rather than the dataset's.
    float min_depth = 0.1f;
    /// Reject depth farther than this (metres).
    float max_depth = 8.0f;
  };

  /// @brief Open a Replica scene directory.
  ///
  /// Probes the disk once, for exactly the frames the options select: index
  /// 0, `frame_stride`, `2 * frame_stride`, ... below `frame_limit`, stopping
  /// at the first one whose images are missing -- poses are matched to
  /// `frameNNNNNN` by position, so a gap ends the sequence as a missing tail
  /// does. Indices the stride skips are never looked at, so a sequence thinned
  /// on disk to every N-th frame plays in full under `frame_stride = N`.
  ///
  /// @param scene_dir        The scene folder (contains `results/` +
  ///                         `traj.txt`).
  /// @param cam_params_path  Path to the `cam_params.json` holding
  ///                         `w,h,fx,fy,cx,cy,scale`.
  /// @param options          Frame selection + depth range.
  /// @return The capture, not yet started; or a non-OK @ref vr::Status if the
  ///         intrinsics or trajectory cannot be read/parsed, if
  ///         `options.frame_stride` is 0, or if the depth range is rejected
  ///         (negative, or `min_depth` not below `max_depth`) -- named as this
  ///         capture's, so a caller that never set one of the two knows which
  ///         default it is arguing with.
  static vr::Result<ReplicaCapture> open(const std::string& scene_dir,
                                         const std::string& cam_params_path,
                                         const Options& options);

  // Hand-written rather than defaulted so a moved-from capture is EMPTY: a
  // defaulted move empties the vectors but copies `end_`, `running_` and
  // `next_`, leaving a shell that still claims frames and, polled, indexes an
  // emptied pose table. Both must name every member; see the .cpp.
  ReplicaCapture(ReplicaCapture&& other) noexcept;
  ReplicaCapture& operator=(ReplicaCapture&& other) noexcept;
  ~ReplicaCapture() override = default;

  /// @return The colour camera every frame is stamped with -- intrinsics and
  ///         size; the pose is per frame. Replica renders depth and colour
  ///         from one registered camera, so this is the sensor's geometry.
  const vr::ColorCameraParams& color_camera() const noexcept {
    return color_camera_;
  }

  /// @return Units per metre of the raw 16-bit depth PNGs: metres = raw / this.
  float depth_scale() const noexcept { return depth_scale_; }

  /// @return How many frames this capture plays: the frames on disk, under the
  ///         limit and at the stride @ref open was given.
  std::size_t frame_count() const noexcept;

  /// @brief Decode every frame this capture will play into memory up front, so
  ///        @ref poll serves from RAM and the fuse loop is not gated by
  ///        per-frame JPEG/PNG decode.
  ///
  /// Decode dominates the streaming path: on Replica room0 one frame costs
  /// ~10 ms to decode against ~1.8 ms of GPU fusion, so a streaming fuse loop
  /// is dataloader-bound -- ~75-80% of its wall clock is the reader.
  /// Preloading trades memory for that time -- roughly `width * height * 8`
  /// bytes per frame (~6 MB at Replica's 1200x680, ~2.5 GB for 400 frames), so
  /// it suits benchmarking and short sequences, not an unbounded capture. Call
  /// @ref preload_bytes_projected first to show that cost before paying it.
  ///
  /// Call before @ref start: the cache replaces whatever a previous preload
  /// held, and a frame handed out by @ref poll may borrow from it.
  ///
  /// @param cancel  Optional flag polled once per frame; when it turns true
  ///                the decode stops early and returns what it has, so a
  ///                caller shutting down is not held up by a long preload. The
  ///                frames it did not reach still decode on demand.
  /// @return How many frames were cached, or a non-OK @ref vr::Status (the
  ///         capture is running, or a frame failed to decode).
  vr::Result<std::size_t> preload(const std::atomic<bool>* cancel = nullptr);

  /// @return Bytes @ref preload would hold, so a caller can report the cost up
  ///         front rather than discover it after a multi-gigabyte decode.
  std::size_t preload_bytes_projected() const noexcept;

  /// @return Bytes of frame payload currently held by @ref preload.
  std::size_t preloaded_bytes() const noexcept;

  /// @brief Begin playback. Idempotent: starting a running capture leaves its
  ///        position alone. After @ref stop, playback resumes from the first
  ///        frame.
  vr::Status start() override;

  /// @brief Stop playback and drop the frame the last @ref poll handed out.
  ///        Idempotent. The @ref preload cache is kept.
  void stop() noexcept override;

  /// @brief Hand out the next frame of the sequence.
  ///
  /// The frame is `frame_stride` past the previous one, decoded now unless
  /// @ref preload cached it. Its depth camera carries the intrinsics, the
  /// @ref Options depth range and this frame's pose; the colour camera is the
  /// same registered camera (Replica renders both from one), so both poses are
  /// stamped from the one trajectory entry and cannot drift apart. Colour is
  /// left declared canonical -- the renders are ordinary sRGB JPEGs -- and the
  /// timestamp is 0, since the dataset carries none.
  ///
  /// @return The frame; an empty optional once @ref exhausted, or while not
  ///         started; or the decode error of a frame that is on disk but
  ///         unreadable, which leaves the position unchanged so the next poll
  ///         retries the same frame.
  vr::Result<std::optional<vr::sensor::CapturedFrame>> poll() override;

  /// @return `true` once every frame this capture plays has been handed out
  ///         (or there were none) -- the replay's end of sequence, which its
  ///         empty poll alone cannot distinguish from a live device's "not
  ///         yet". A moved-from capture plays nothing, so it is exhausted.
  bool exhausted() const noexcept override { return next_ >= end_; }

 private:
  ReplicaCapture() = default;

  // Decode one frame straight from disk, bypassing the cache, fully stamped:
  // the poll path hands out its view as is. The preload path shares it so
  // both decode identically.
  vr::Result<RgbdFrame> load(std::size_t index) const;

  Options options_{};
  float depth_scale_ = 1.0f;  ///< Units per metre: metres = raw_uint16 / this.
  // Intrinsics + size + depth range, built once at open and what every frame
  // is stamped with; only cam_to_world is per frame. The decoder size-checks
  // against these very structs, so a frame cannot be stamped with one extent
  // and hold another.
  vr::DepthCameraParams depth_camera_{};
  vr::ColorCameraParams color_camera_{};
  std::vector<vr::Mat4f> poses_;
  std::string results_dir_;
  // One past the last index played: the last strided index whose images are on
  // disk, under the limit, plus one. Zero when nothing plays.
  std::size_t end_ = 0;
  // Indexed by frame index; an empty slot is a frame the preload skipped (or a
  // preload that never ran). Empty when streaming.
  std::vector<std::optional<RgbdFrame>> cache_;

  bool running_ = false;
  std::size_t next_ = 0;  ///< Index of the frame the next poll hands out.
  // The frame the last poll decoded, when it decoded one (streaming); the
  // storage the view it handed out borrows. Empty after a cache hit, whose
  // frame the cache owns.
  std::optional<RgbdFrame> current_owned_;
};

}  // namespace vr_example
