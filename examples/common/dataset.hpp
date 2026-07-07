// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/dataset.hpp
/// @brief A posed RGB-D dataset reader for the fuse examples, in the
///        Replica-SLAM layout nvblox's `fuse_replica` consumes.

#include <cstddef>
#include <cstdint>
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

/// @brief A Replica-SLAM RGB-D sequence: `<scene>/results/frameNNNNNN.jpg` +
///        `depthNNNNNN.png`, per-frame poses in `<scene>/traj.txt`, intrinsics
///        in a `cam_params.json`.
///
/// @ref open reads the intrinsics + every pose up front (cheap); @ref load
/// decodes one frame's images on demand. The pose file lists one flattened
/// **row-major** 4x4 camera->world matrix per line, transposed into the
/// column-major @ref vr::Mat4f the pipeline uploads verbatim.
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

  /// @brief Decode frame @p i (colour JPEG + depth PNG) and pair it with its
  ///        pose.
  /// @param i  Frame index in `[0, frame_count())`.
  /// @return The decoded frame, or a non-OK @ref vr::Status (out-of-range index
  ///         or an image decode failure).
  vr::Result<RgbdFrame> load(std::size_t i) const;

 private:
  ReplicaDataset() = default;

  CameraModel camera_{};
  std::vector<vr::Mat4f> poses_;
  std::string results_dir_;
};

}  // namespace vr_example
