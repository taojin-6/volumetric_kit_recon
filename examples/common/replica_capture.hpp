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
/// fuse loop takes an `ICameraCapture&` and never learns whether its frames
/// came off a disk or a sensor. This is also the one implementer of the
/// contract this repo builds and runs on every example invocation -- the tier
/// itself ships none (the 2026-08-02 decision), and its test fake exercises the
/// interface without ever producing a real frame.

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "volumetric_kit/recon/core/camera_params.hpp"
#include "volumetric_kit/recon/core/math/vector_types.hpp"
#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/sensor/camera_capture.hpp"

namespace vr_example {

namespace vr = volumetric_kit::recon;

/// @brief Pinhole intrinsics + image dimensions + depth scale shared by every
///        frame of a sequence (Replica renders depth and colour with one
///        registered camera).
struct CameraModel {
  float fx = 0.0f;           ///< Focal length x (pixels).
  float fy = 0.0f;           ///< Focal length y (pixels).
  float cx = 0.0f;           ///< Principal point x (pixels).
  float cy = 0.0f;           ///< Principal point y (pixels).
  std::uint32_t width = 0;   ///< Image width (pixels).
  std::uint32_t height = 0;  ///< Image height (pixels).
  float depth_scale = 1.0f;  ///< Units per metre: metres = raw_uint16 / this.
  // The near/far depth-range gate is a fusion knob, not a camera intrinsic, so
  // it lives on the capture's Options (the fuse example's CLI), not here.
};

/// @brief A Replica-SLAM RGB-D sequence -- `<scene>/results/frameNNNNNN.jpg` +
///        `depthNNNNNN.png`, per-frame poses in `<scene>/traj.txt`, intrinsics
///        in a `cam_params.json` -- as a @ref vr::sensor::ICameraCapture.
///
/// Playback is **consumer-paced**: every @ref poll hands out the next frame of
/// the sequence, decoded on demand or served from the @ref preload cache, and
/// an empty poll means the sequence is over -- there is no "not yet" for a
/// file source, so a fuse loop treats the first empty poll as the end. The
/// frame is a non-owning view exactly as the contract says: its pixels belong
/// to this object and stay valid until the next @ref poll (or @ref stop), so a
/// consumer that keeps the last frame it polled -- as the viewer does for its
/// final texture pass -- holds a valid view for as long as it polls no further.
///
/// @ref open reads the intrinsics and every pose up front, then probes which
/// frames are actually on disk (a trajectory routinely lists more poses than
/// there are images: room0 has 2000 against 400), so @ref frame_count is the
/// number of frames the sequence will really play, after the limit and stride
/// in @ref Options. The pose file lists one flattened **row-major** 4x4
/// camera->world matrix per line, transposed into the column-major
/// @ref vr::Mat4f the pipeline uploads verbatim.
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
  /// @param scene_dir        The scene folder (contains `results/` +
  ///                         `traj.txt`).
  /// @param cam_params_path  Path to the `cam_params.json` holding
  ///                         `w,h,fx,fy,cx,cy,scale`.
  /// @param options          Frame selection + depth range.
  /// @return The capture, not yet started; or a non-OK @ref vr::Status if the
  ///         intrinsics or trajectory cannot be read/parsed, if
  ///         `options.frame_stride` is 0, or if the depth range is rejected by
  ///         `sensor::depth_from_registered_color` (negative, or min not below
  ///         max).
  static vr::Result<ReplicaCapture> open(const std::string& scene_dir,
                                         const std::string& cam_params_path,
                                         const Options& options);

  ReplicaCapture(ReplicaCapture&&) = default;
  ReplicaCapture& operator=(ReplicaCapture&&) = default;

  /// @return The shared camera intrinsics + depth scale.
  const CameraModel& camera() const noexcept { return camera_; }

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
  /// @return The frame; an empty optional once the sequence is exhausted or
  ///         while not started; or the decode error of a frame that is on disk
  ///         but unreadable, which leaves the position unchanged.
  vr::Result<std::optional<vr::sensor::CapturedFrame>> poll() override;

 private:
  /// One decoded, posed RGB-D frame: the storage a @ref
  /// vr::sensor::CapturedFrame borrows from.
  struct RgbdFrame {
    std::vector<float> depth;          ///< Depth in metres, `w*h`, row-major.
    std::vector<std::uint32_t> color;  ///< Packed RGB (`R|G<<8|B<<16`), `w*h`.
    vr::Mat4f cam_to_world{1.0f};      ///< Camera->world pose (column-major).
  };

  ReplicaCapture() = default;

  // Decode one frame straight from disk, bypassing the cache. The preload path
  // and the poll path share it so both decode identically.
  vr::Result<RgbdFrame> load(std::size_t index) const;

  // The frame the last poll handed out. Resolved on access rather than cached
  // in a member pointer, so the defaulted move operations stay correct: a
  // pointer at `current_owned_` would dangle the moment this object moved.
  const RgbdFrame* current() const noexcept {
    return current_owned_ ? &*current_owned_ : current_borrowed_;
  }

  CameraModel camera_{};
  Options options_{};
  // Intrinsics + depth range, built once at open; only cam_to_world is
  // stamped per frame.
  vr::DepthCameraParams depth_camera_{};
  vr::ColorCameraParams color_camera_{};
  std::vector<vr::Mat4f> poses_;
  std::string results_dir_;
  // One past the last index played: the frames on disk, clamped to the limit.
  std::size_t end_ = 0;
  // Indexed by frame index; an empty slot is a frame the preload skipped (or a
  // preload that never ran). Empty when streaming.
  std::vector<std::optional<RgbdFrame>> cache_;

  bool running_ = false;
  std::size_t next_ = 0;  ///< Index of the frame the next poll hands out.
  std::optional<RgbdFrame> current_owned_;  ///< Set when poll decoded it.
  const RgbdFrame* current_borrowed_ =
      nullptr;  ///< Set when the cache owns it.
};

}  // namespace vr_example
