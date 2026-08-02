// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "dataset.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <utility>

#include "image_io.hpp"

namespace vr_example {
namespace {

// Read a whole text file into a string, or nullopt if it cannot be opened.
std::optional<std::string> read_file(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    return std::nullopt;
  }
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

// True if the path can be opened for reading. Used to tell a frame that is
// simply absent from disk (stop cleanly) from one that is present but fails to
// decode (a real error).
bool file_exists(const std::string& path) {
  return static_cast<bool>(std::ifstream(path, std::ios::binary));
}

// Pull a numeric value for `"key"` out of a flat JSON object (find the key,
// then the number after the following colon). Enough for the tiny
// cam_params.json -- no nesting or arrays to worry about, so no JSON dependency
// is pulled in.
std::optional<float> json_number(const std::string& json,
                                 const std::string& key) {
  const std::string quoted = "\"" + key + "\"";
  std::size_t pos = json.find(quoted);
  if (pos == std::string::npos) {
    return std::nullopt;
  }
  pos = json.find(':', pos + quoted.size());
  if (pos == std::string::npos) {
    return std::nullopt;
  }
  ++pos;
  // Skip whitespace to the number, then parse a float.
  while (pos < json.size() &&
         (json[pos] == ' ' || json[pos] == '\t' || json[pos] == '\n')) {
    ++pos;
  }
  try {
    return std::stof(json.substr(pos));
  } catch (...) {
    return std::nullopt;
  }
}

// Frame image path: <results>/<prefix>NNNNNN<suffix> (Replica's zero-padded
// six-digit index).
std::string frame_path(const std::string& results_dir, const char* prefix,
                       std::size_t index, const char* suffix) {
  char name[64];
  std::snprintf(name, sizeof(name), "%s%06zu%s", prefix, index, suffix);
  return results_dir + "/" + name;
}

// True when both of frame `index`'s images are on disk. Single-sourced so the
// "is this frame present" rule that stops a load, a preload, and a preload's
// size projection cannot drift apart.
bool frame_on_disk(const std::string& results_dir, std::size_t index) {
  return file_exists(frame_path(results_dir, "frame", index, ".jpg")) &&
         file_exists(frame_path(results_dir, "depth", index, ".png"));
}

}  // namespace

// Both members are cleared on transfer, so a moved-from view resolves to
// nullptr rather than to an emptied RgbdFrame -- dereferencing one faults
// instead of silently handing a fuse loop a null depth/colour pointer.
FrameView::FrameView(FrameView&& other) noexcept
    : owned_(std::move(other.owned_)), borrowed_(other.borrowed_) {
  other.owned_.reset();
  other.borrowed_ = nullptr;
}

FrameView& FrameView::operator=(FrameView&& other) noexcept {
  if (this != &other) {
    owned_ = std::move(other.owned_);
    borrowed_ = other.borrowed_;
    other.owned_.reset();
    other.borrowed_ = nullptr;
  }
  return *this;
}

vr::Result<ReplicaDataset> ReplicaDataset::open(
    const std::string& scene_dir, const std::string& cam_params_path) {
  ReplicaDataset ds;
  ds.results_dir_ = scene_dir + "/results";

  // --- Intrinsics (cam_params.json) ---
  const std::optional<std::string> cam_json = read_file(cam_params_path);
  if (!cam_json) {
    return vr::Status::invalid_argument(
        "ReplicaDataset::open: cannot read cam params: " + cam_params_path);
  }
  const std::array<const char*, 7> keys = {"fx", "fy", "cx",   "cy",
                                           "w",  "h",  "scale"};
  std::array<float, 7> values{};
  for (std::size_t k = 0; k < keys.size(); ++k) {
    const std::optional<float> v = json_number(*cam_json, keys[k]);
    if (!v) {
      return vr::Status::invalid_argument(
          std::string("ReplicaDataset::open: cam params missing key '") +
          keys[k] + "'");
    }
    values[k] = *v;
  }
  // Validate the parsed values as finite *before* using them: json_number ->
  // std::stof parses "nan"/"inf"/negatives without error, and casting a
  // non-finite or negative float to the uint32 width/height below is undefined
  // behaviour (a negative also wraps to a huge value that would slip a `== 0`
  // check). Gate every intrinsic -- including cx/cy, whose NaN would silently
  // poison every projection -- not just the ones the old check covered.
  for (const float value : values) {
    if (!std::isfinite(value)) {
      return vr::Status::invalid_argument(
          "ReplicaDataset::open: cam params has a non-finite value");
    }
  }
  const float w = values[4];
  const float h = values[5];
  if (!(values[0] > 0.0f) || !(values[1] > 0.0f) || !(values[6] > 0.0f) ||
      !(w >= 1.0f) || !(w <= 65535.0f) || !(h >= 1.0f) || !(h <= 65535.0f)) {
    // fx/fy > 0 (a zero focal length divides by zero in the unprojection
    // x = (u - cx) * d / fx), scale > 0, and width/height in a sane [1, 65535]
    // so the uint32 cast below is well-defined.
    return vr::Status::invalid_argument(
        "ReplicaDataset::open: cam params has a zero/invalid intrinsic, "
        "dimension, or scale");
  }
  ds.camera_.fx = values[0];
  ds.camera_.fy = values[1];
  ds.camera_.cx = values[2];
  ds.camera_.cy = values[3];
  ds.camera_.width = static_cast<std::uint32_t>(w);
  ds.camera_.height = static_cast<std::uint32_t>(h);
  ds.camera_.depth_scale = values[6];

  // --- Trajectory (traj.txt): one flattened row-major 4x4 cam->world per line,
  // transposed into the column-major glm matrix the pipeline uploads. ---
  std::ifstream traj(scene_dir + "/traj.txt");
  if (!traj) {
    return vr::Status::invalid_argument(
        "ReplicaDataset::open: cannot read trajectory: " + scene_dir +
        "/traj.txt");
  }
  std::string line;
  bool seen_blank = false;
  while (std::getline(traj, line)) {
    if (line.find_first_not_of(" \t\r\n") == std::string::npos) {
      seen_blank = true;  // tolerate trailing blank line(s)
      continue;
    }
    if (seen_blank) {
      // A blank line before this data line is an interior gap: skipping it
      // would silently shift every later pose off its frame index (poses are
      // matched to frameNNNNNN by position), so reject it instead.
      return vr::Status::invalid_argument(
          "ReplicaDataset::open: blank line inside the trajectory (before "
          "pose " +
          std::to_string(ds.poses_.size()) + ")");
    }
    std::istringstream ss(line);
    std::array<float, 16> m{};
    bool ok = true;
    for (float& element : m) {
      if (!(ss >> element)) {
        ok = false;
        break;
      }
    }
    if (!ok) {
      return vr::Status::invalid_argument(
          "ReplicaDataset::open: malformed trajectory line " +
          std::to_string(ds.poses_.size()));
    }
    vr::Mat4f pose(1.0f);
    for (int row = 0; row < 4; ++row) {
      for (int col = 0; col < 4; ++col) {
        pose[col][row] = m[static_cast<std::size_t>(row) * 4 + col];
      }
    }
    ds.poses_.push_back(pose);
  }
  if (ds.poses_.empty()) {
    return vr::Status::invalid_argument(
        "ReplicaDataset::open: trajectory has no poses");
  }
  return ds;
}

vr::Result<RgbdFrame> ReplicaDataset::load(std::size_t index) const {
  if (index >= poses_.size()) {
    return vr::Status::invalid_argument("ReplicaDataset::load: index " +
                                        std::to_string(index) +
                                        " out of range");
  }
  // A frame simply absent from disk (the trajectory may list more poses than
  // there are images) is reported as NotFound, so the caller can stop cleanly;
  // a decode failure on a file that IS present stays a hard error below.
  if (!frame_on_disk(results_dir_, index)) {
    return vr::Status::not_found("ReplicaDataset::load: frame " +
                                 std::to_string(index) + " not on disk");
  }
  const std::string color_path =
      frame_path(results_dir_, "frame", index, ".jpg");
  const std::string depth_path =
      frame_path(results_dir_, "depth", index, ".png");
  RgbdFrame frame;
  frame.cam_to_world = poses_[index];
  VR_ASSIGN(frame.color,
            load_color_packed(color_path, camera_.width, camera_.height));
  VR_ASSIGN(frame.depth,
            load_depth_metres(depth_path, camera_.width, camera_.height,
                              camera_.depth_scale));
  return frame;
}

vr::Result<std::size_t> ReplicaDataset::preload(
    std::size_t frame_limit, std::size_t frame_stride,
    const std::atomic<bool>* cancel) {
  if (frame_stride == 0) {
    return vr::Status::invalid_argument(
        "ReplicaDataset::preload: frame_stride must be >= 1");
  }
  const std::size_t limit = std::min(frame_limit, poses_.size());
  // Drop any previous cache first, so a second preload does not hold two
  // sequences' worth of frames at once while it refills.
  cache_.clear();
  cache_.resize(limit);
  std::size_t cached_frames = 0;
  for (std::size_t index = 0; index < limit; index += frame_stride) {
    // Polled per frame rather than per batch: a caller tearing down waits at
    // most one frame's decode, not the whole sequence's.
    if (cancel != nullptr && cancel->load()) {
      break;
    }
    vr::Result<RgbdFrame> frame_result = load(index);
    if (!frame_result) {
      // Ran past the frames present on disk: keep what we have (the fuse loop
      // stops there anyway). A frame that IS present but failed to decode is a
      // real error, exactly as in load().
      if (frame_result.status().domain() == vr::Status::Code::NotFound) {
        break;
      }
      cache_.clear();
      return frame_result.status();
    }
    cache_[index] = std::move(frame_result).value();
    ++cached_frames;
  }
  return cached_frames;
}

std::size_t ReplicaDataset::preload_bytes_projected(
    std::size_t frame_limit, std::size_t frame_stride) const {
  if (frame_stride == 0) {
    return 0;
  }
  const std::size_t limit = std::min(frame_limit, poses_.size());
  // Counts the frames preload would really reach rather than assuming the whole
  // trajectory is backed by images: Replica's room0 lists 2000 poses against
  // 400 frames on disk, where a worst-case count would overstate the cost 5x
  // and cry wolf. Two file probes per frame -- milliseconds against a decode
  // measured in seconds.
  std::size_t frames = 0;
  for (std::size_t index = 0; index < limit; index += frame_stride) {
    if (!frame_on_disk(results_dir_, index)) {
      break;
    }
    ++frames;
  }
  return frames * static_cast<std::size_t>(camera_.width) * camera_.height *
         (sizeof(float) + sizeof(std::uint32_t));
}

std::size_t ReplicaDataset::preloaded_bytes() const noexcept {
  std::size_t bytes = 0;
  for (const std::optional<RgbdFrame>& cached : cache_) {
    if (cached) {
      bytes += cached->depth.size() * sizeof(float) +
               cached->color.size() * sizeof(std::uint32_t);
    }
  }
  return bytes;
}

vr::Result<FrameView> ReplicaDataset::frame(std::size_t index) const {
  FrameView view;
  if (index < cache_.size() && cache_[index]) {
    view.borrowed_ = &*cache_[index];
    return view;
  }
  VR_ASSIGN(view.owned_, load(index));
  return view;
}

}  // namespace vr_example
