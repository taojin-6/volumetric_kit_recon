// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/dataset.hpp
/// @brief A posed RGB-D dataset reader for the fuse examples, in the
///        Replica-SLAM layout nvblox's `fuse_replica` consumes.

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "volumetric_kit/recon/core/math/vector_types.hpp"
#include "volumetric_kit/recon/core/result.hpp"

namespace vr_example {

namespace vr = volumetric_kit::recon;

/// @brief Pinhole intrinsics + image dimensions + depth scale shared by every
///        frame of a dataset (Replica renders depth and colour with one
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
  // it lives on the driver (the fuse example's CLI), not here.
};

/// @brief One decoded, posed RGB-D frame.
struct RgbdFrame {
  std::vector<float> depth;          ///< Depth in metres, `w*h`, row-major.
  std::vector<std::uint32_t> color;  ///< Packed RGB (`R|G<<8|B<<16`), `w*h`.
  vr::Mat4f cam_to_world{1.0f};      ///< Camera->world pose (glm column-major).
};

class ReplicaDataset;

/// @brief Read access to one frame, however it was obtained: a borrowed
///        reference into the dataset's preloaded cache, or a frame this view
///        decoded and owns.
///
/// Lets a fuse loop consume frames uniformly whether or not the caller asked
/// for a preload, without copying a preloaded frame (~6 MB at Replica's
/// resolution) on every iteration. A borrowing view stays valid as long as the
/// @ref ReplicaDataset it came from is alive and is not preloaded again.
class FrameView {
 public:
  /// @return The frame. Never null for a view obtained from a successful
  ///         @ref ReplicaDataset::frame.
  const RgbdFrame& operator*() const noexcept { return *get(); }

  /// @copydoc operator*
  const RgbdFrame* operator->() const noexcept { return get(); }

 private:
  friend class ReplicaDataset;

  // Resolved on access rather than cached in a member pointer, so the defaulted
  // copy/move stay correct: a member pointing at `owned_` would dangle the
  // moment the view is moved.
  const RgbdFrame* get() const noexcept {
    return owned_ ? &*owned_ : borrowed_;
  }

  std::optional<RgbdFrame> owned_;       ///< Set when this view decoded it.
  const RgbdFrame* borrowed_ = nullptr;  ///< Set when the cache owns it.
};

/// @brief A Replica-SLAM RGB-D sequence: `<scene>/results/frameNNNNNN.jpg` +
///        `depthNNNNNN.png`, per-frame poses in `<scene>/traj.txt`, intrinsics
///        in a `cam_params.json`.
///
/// @ref open reads the intrinsics + every pose up front (cheap); @ref frame
/// then decodes one frame's images on demand, or serves it from memory when
/// @ref preload has cached it. The pose file lists one flattened **row-major**
/// 4x4 camera->world matrix per line, transposed into the column-major
/// @ref vr::Mat4f the pipeline uploads verbatim.
class ReplicaDataset {
 public:
  /// @brief Open a Replica scene directory.
  /// @param scene_dir        The scene folder (contains `results/` +
  ///                         `traj.txt`).
  /// @param cam_params_path  Path to the `cam_params.json` holding
  ///                         `w,h,fx,fy,cx,cy,scale`.
  /// @return The dataset, or a non-OK @ref vr::Status if the intrinsics or
  ///         trajectory cannot be read/parsed.
  static vr::Result<ReplicaDataset> open(const std::string& scene_dir,
                                         const std::string& cam_params_path);

  /// @return How many posed frames the trajectory holds.
  std::size_t frame_count() const noexcept { return poses_.size(); }

  /// @return The shared camera intrinsics + depth scale.
  const CameraModel& camera() const noexcept { return camera_; }

  /// @brief Decode every frame the caller will fuse into memory up front, so
  ///        @ref frame serves from RAM and the fuse loop is not gated by
  ///        per-frame JPEG/PNG decode.
  ///
  /// Decode dominates the streaming path: on Replica room0 one frame costs
  /// ~10 ms to decode against ~3 ms of GPU fusion, so a streaming fuse loop
  /// spends ~75% of its wall clock in the reader. Preloading trades memory for
  /// that time -- roughly `width * height * 8` bytes per frame (~6 MB at
  /// Replica's 1200x680, ~2.5 GB for 400 frames), so it suits benchmarking and
  /// short sequences, not an unbounded capture.
  ///
  /// Caches indices `0, frame_stride, 2*frame_stride, ...` below
  /// @p frame_limit, matching what a strided fuse loop actually visits. Stops
  /// cleanly at the first frame absent from disk (the trajectory may list more
  /// poses than there are images); calling it again replaces any previous
  /// cache.
  ///
  /// @param frame_limit   One past the highest frame index to cache.
  /// @param frame_stride  Frame step; 1 caches every frame. Must be >= 1.
  /// @return How many frames were cached, or a non-OK @ref vr::Status (a zero
  ///         @p frame_stride, or a frame present on disk that failed to
  ///         decode).
  vr::Result<std::size_t> preload(std::size_t frame_limit,
                                  std::size_t frame_stride = 1);

  /// @return Bytes of frame payload currently held by @ref preload.
  std::size_t preloaded_bytes() const noexcept;

  /// @brief Access frame @p index (colour JPEG + depth PNG) paired with its
  ///        pose, from the preload cache if present, else by decoding it now.
  /// @param index  Frame index in `[0, frame_count())`.
  /// @return A view of the frame, or a non-OK @ref vr::Status (out-of-range
  ///         index, a frame absent from disk, or an image decode failure).
  vr::Result<FrameView> frame(std::size_t index) const;

 private:
  ReplicaDataset() = default;

  // Decode one frame straight from disk, bypassing the cache. The preload path
  // and the on-demand path share it so both decode identically.
  vr::Result<RgbdFrame> load(std::size_t index) const;

  CameraModel camera_{};
  std::vector<vr::Mat4f> poses_;
  std::string results_dir_;
  // Indexed by frame index; an empty slot is a frame the preload skipped (or a
  // preload that never ran). Empty when streaming.
  std::vector<std::optional<RgbdFrame>> cache_;
};

}  // namespace vr_example
