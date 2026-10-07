// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/replica_sensor.hpp
/// @brief A posed Replica-SLAM RGB-D sequence played back as a
///        `sensor::IRgbdSensor` -- the frame source every dataset example
///        drives.
///
/// The examples take frames through the interface rather than through a
/// dataset API of their own, so a live camera is a construction-site swap: the
/// fuse loop polls an `IRgbdSensor&`, waits on an empty poll until
/// `exhausted()` says the source is done, prepares each frame on the GPU as it
/// would a camera's (`sensor::GpuFramePrep`), and never learns whether its
/// frames came off a disk or a sensor.

#include <atomic>
#include <cstddef>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/recon/camera/geometry.hpp"
#include "volumetric_kit/recon/sensor/rgbd_frame.hpp"
#include "volumetric_kit/recon/sensor/rgbd_sensor.hpp"

namespace vr_example {

namespace vr = volumetric_kit::recon;
namespace vkc = volumetric_kit::core;

/// @brief A Replica-SLAM RGB-D sequence -- `<scene>/results/frameNNNNNN.jpg` +
///        `depthNNNNNN.png`, per-frame poses in `<scene>/traj.txt`, intrinsics
///        in a `cam_params.json` -- as a @ref vr::sensor::IRgbdSensor.
///
/// Playback is **consumer-paced**: every @ref poll hands out the next frame of
/// the sequence, decoded on demand or served from the @ref preload cache, and
/// once the last one has gone @ref exhausted turns true and every further poll
/// is empty. A frame holds its pixels: the depth PNG's samples as stored, with
/// `metres_per_unit` from the intrinsics' scale, and the colour JPEG as packed
/// words (`RgbdFrame::color_packed`), declared canonical -- the renders are
/// ordinary sRGB JPEGs.
///
/// Replica renders depth and colour from one pinhole camera, so its
/// @ref info names one model for both with an identity `depth_to_color`. Each
/// frame is posed from its trajectory entry (`PoseSource::Tracked`), carries
/// no timestamp (0), and is numbered by how many frames this sensor handed
/// out before it, so a stride is no loss.
///
/// @ref open reads the intrinsics and every pose up front, then probes which
/// of the frames it will play are actually on disk (a trajectory routinely
/// lists more poses than there are images: room0 has 2000 against 400), so
/// @ref frame_count is the number of frames the sequence will really play,
/// after the limit and stride in @ref Options. The pose file lists one
/// flattened **row-major** 4x4 camera->world matrix per line.
class ReplicaSensor final : public vr::sensor::IRgbdSensor {
 public:
  /// @brief Which frames to play, and the depth range to stamp on them.
  struct Options {
    /// One past the highest frame index to play (a fuse example's
    /// `--max-frames`). Clamped to the frames present on disk.
    std::size_t frame_limit = std::numeric_limits<std::size_t>::max();
    /// Play every N-th frame (`--stride`). Must be >= 1.
    std::size_t frame_stride = 1;
    /// Reject depth nearer than this (metres); stamped on every frame. The
    /// fusion gate, so the driver's knob rather than the dataset's. Above 0,
    /// since 0 is the GPU pass's "no return".
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
  /// @return The sensor, not yet started; or a non-OK `vkc::Status` if the
  ///         intrinsics or trajectory cannot be read/parsed, if
  ///         `options.frame_stride` is 0, or if the depth range is not finite
  ///         with `0 < min_depth < max_depth` -- named as this sensor's, so a
  ///         caller that never set one of the two knows which default it is
  ///         arguing with.
  static vkc::Result<ReplicaSensor> open(const std::string& scene_dir,
                                         const std::string& cam_params_path,
                                         const Options& options);

  // Defaulted: the frames to play move with `frames_`, so a moved-from sensor
  // plays nothing and is exhausted. Assignment would self-move a vector.
  ReplicaSensor(ReplicaSensor&&) noexcept = default;
  ReplicaSensor& operator=(ReplicaSensor&&) = delete;
  ~ReplicaSensor() override = default;

  /// @return How many frames this sensor plays: the frames on disk, under the
  ///         limit and at the stride @ref open was given.
  std::size_t frame_count() const noexcept { return frames_.size(); }

  /// @brief Decode every frame this sensor will play into memory up front, so
  ///        @ref poll serves from RAM and the fuse loop is not gated by
  ///        per-frame JPEG/PNG decode.
  ///
  /// Decode dominates the streaming path: on Replica room0 one frame costs
  /// ~10 ms to decode against ~1.8 ms of GPU fusion, so a streaming fuse loop
  /// is dataloader-bound -- ~75-80% of its wall clock is the reader.
  /// Preloading trades memory for that time -- `width * height * 6` bytes per
  /// frame (~4.9 MB at Replica's 1200x680, ~2 GB for 400 frames), so it suits
  /// benchmarking and short sequences, not an unbounded capture. Call
  /// @ref preload_bytes_projected first to show that cost before paying it.
  /// The cache replaces whatever a previous preload held.
  ///
  /// @param cancel  Optional flag polled once per frame; when it turns true
  ///                the decode stops early and returns what it has, so a
  ///                caller shutting down is not held up by a long preload. The
  ///                frames it did not reach still decode on demand.
  /// @return How many frames were cached, or the decode error of a frame,
  ///         which leaves nothing cached.
  vkc::Result<std::size_t> preload(const std::atomic<bool>* cancel = nullptr);

  /// @return Bytes @ref preload would hold, so a caller can report the cost up
  ///         front rather than discover it after a multi-gigabyte decode.
  std::size_t preload_bytes_projected() const noexcept;

  /// @return Bytes of frame payload currently held by @ref preload.
  std::size_t preloaded_bytes() const noexcept;

  /// @return What @ref open read: one camera for depth and colour.
  const vr::sensor::SensorInfo& info() const noexcept override { return info_; }

  /// @brief Accepts any depth of at least 1 before @ref start, and changes
  ///        nothing: no frame is held between polls, each decoding the next.
  /// @return OK; `Status::Code::InvalidArgument` for 0, or once started.
  vkc::Status set_queue_depth(std::size_t frames) override;

  /// @brief Begin playback. Idempotent: starting a running sensor leaves its
  ///        position alone. After @ref stop, playback resumes from the first
  ///        frame.
  vkc::Status start() override;

  /// @brief Stop playback and rewind. Idempotent. The @ref preload cache is
  ///        kept.
  void stop() noexcept override;

  /// @brief Hand out the next frame of the sequence, `frame_stride` past the
  ///        previous one, decoded now unless @ref preload cached it.
  /// @return The frame; an empty optional once @ref exhausted, or while not
  ///         started; or the decode error of a frame that is on disk but
  ///         unreadable, which leaves the position unchanged so the next poll
  ///         retries the same frame.
  vkc::Result<std::optional<vr::sensor::RgbdFrame>> poll() override;

  /// @brief Append what @ref poll would return: the next frame, or nothing.
  /// @param out  Receives the frame; not null.
  /// @return OK, or @ref poll's error.
  vkc::Status drain(std::vector<vr::sensor::RgbdFrame>* out) override;

  /// @return `true` once every frame this sensor plays has been handed out
  ///         (or there were none) -- the replay's end of sequence, which its
  ///         empty poll alone cannot distinguish from a live device's "not
  ///         yet".
  bool exhausted() const noexcept override { return next_ >= frames_.size(); }

  /// @return Frames received and delivered since the last @ref start: one
  ///         each a poll that hands a frame out.
  vr::sensor::SensorStats stats() const noexcept override { return stats_; }

 private:
  // A frame this sensor plays: its index on disk, its pose, and its decode
  // once preloaded.
  struct Entry {
    std::size_t index = 0;
    vr::camera::Mat4d pose{1.0};
    std::optional<vr::sensor::RgbdFrame> cached;
  };

  ReplicaSensor() = default;

  // Decode one frame straight from disk, bypassing the cache, stamped with
  // everything but its sequence number. The preload path shares it, so both
  // decode identically.
  vkc::Result<vr::sensor::RgbdFrame> load(const Entry& entry) const;

  // Bytes of one decoded frame: u16 depth and a packed colour word a pixel.
  std::size_t frame_bytes() const noexcept;

  Options options_{};
  float metres_per_unit_ = 0.0f;
  vr::sensor::SensorInfo info_;
  std::string results_dir_;
  std::vector<Entry> frames_;  // the frames to play, in order
  bool running_ = false;
  std::size_t next_ = 0;  // position in `frames_` of the next poll
  vr::sensor::SensorStats stats_;
};

}  // namespace vr_example
