// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "replica_capture.hpp"

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
#include "volumetric_kit/recon/sensor/camera_conventions.hpp"

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

// True if the path can be opened for reading.
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

// True when both of frame `index`'s images are on disk.
bool frame_on_disk(const std::string& results_dir, std::size_t index) {
  return file_exists(frame_path(results_dir, "frame", index, ".jpg")) &&
         file_exists(frame_path(results_dir, "depth", index, ".png"));
}

}  // namespace

vr::Result<ReplicaCapture> ReplicaCapture::open(
    const std::string& scene_dir, const std::string& cam_params_path,
    const Options& options) {
  if (options.frame_stride == 0) {
    return vr::Status::invalid_argument(
        "ReplicaCapture::open: frame_stride must be >= 1");
  }
  ReplicaCapture capture;
  capture.options_ = options;
  capture.results_dir_ = scene_dir + "/results";

  // --- Intrinsics (cam_params.json) ---
  const std::optional<std::string> cam_json = read_file(cam_params_path);
  if (!cam_json) {
    return vr::Status::invalid_argument(
        "ReplicaCapture::open: cannot read cam params: " + cam_params_path);
  }
  const std::array<const char*, 7> keys = {"fx", "fy", "cx",   "cy",
                                           "w",  "h",  "scale"};
  std::array<float, 7> values{};
  for (std::size_t k = 0; k < keys.size(); ++k) {
    const std::optional<float> v = json_number(*cam_json, keys[k]);
    if (!v) {
      return vr::Status::invalid_argument(
          std::string("ReplicaCapture::open: cam params missing key '") +
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
          "ReplicaCapture::open: cam params has a non-finite value");
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
        "ReplicaCapture::open: cam params has a zero/invalid intrinsic, "
        "dimension, or scale");
  }
  CameraModel& cam = capture.camera_;
  cam.fx = values[0];
  cam.fy = values[1];
  cam.cx = values[2];
  cam.cy = values[3];
  cam.width = static_cast<std::uint32_t>(w);
  cam.height = static_cast<std::uint32_t>(h);
  cam.depth_scale = values[6];

  // Both camera blocks once, here. Depth and colour are one registered camera
  // on Replica, so the depth camera is *derived* from the colour one exactly
  // as a registered live source's is -- which also validates the depth range
  // the options carry, before a frame is ever handed out.
  vr::ColorCameraParams& color = capture.color_camera_;
  color.fx = cam.fx;
  color.fy = cam.fy;
  color.cx = cam.cx;
  color.cy = cam.cy;
  color.width = cam.width;
  color.height = cam.height;
  VR_ASSIGN(capture.depth_camera_, vr::sensor::depth_from_registered_color(
                                       color, cam.width, cam.height,
                                       options.min_depth, options.max_depth));

  // --- Trajectory (traj.txt): one flattened row-major 4x4 cam->world per line,
  // transposed into the column-major glm matrix the pipeline uploads. ---
  std::ifstream traj(scene_dir + "/traj.txt");
  if (!traj) {
    return vr::Status::invalid_argument(
        "ReplicaCapture::open: cannot read trajectory: " + scene_dir +
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
          "ReplicaCapture::open: blank line inside the trajectory (before "
          "pose " +
          std::to_string(capture.poses_.size()) + ")");
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
          "ReplicaCapture::open: malformed trajectory line " +
          std::to_string(capture.poses_.size()));
    }
    vr::Mat4f pose(1.0f);
    for (int row = 0; row < 4; ++row) {
      for (int col = 0; col < 4; ++col) {
        pose[col][row] = m[static_cast<std::size_t>(row) * 4 + col];
      }
    }
    capture.poses_.push_back(pose);
  }
  if (capture.poses_.empty()) {
    return vr::Status::invalid_argument(
        "ReplicaCapture::open: trajectory has no poses");
  }

  // --- The frames actually present. Probed once here (two file opens per
  // frame, milliseconds) rather than discovered by the first poll to hit an
  // absent image, so frame_count() is the number of frames that will really
  // play and a viewer's "fused N / M" cannot promise 2000 against 400 on disk.
  // Contiguous from 0: poses are matched to frameNNNNNN by position, so a gap
  // ends the sequence exactly as a missing tail does.
  std::size_t on_disk = 0;
  while (on_disk < capture.poses_.size() &&
         frame_on_disk(capture.results_dir_, on_disk)) {
    ++on_disk;
  }
  capture.end_ = std::min(on_disk, options.frame_limit);
  return capture;
}

std::size_t ReplicaCapture::frame_count() const noexcept {
  return end_ == 0 ? 0 : (end_ - 1) / options_.frame_stride + 1;
}

vr::Result<ReplicaCapture::RgbdFrame> ReplicaCapture::load(
    std::size_t index) const {
  if (index >= end_) {
    return vr::Status::invalid_argument("ReplicaCapture::load: index " +
                                        std::to_string(index) +
                                        " past the sequence");
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

vr::Result<std::size_t> ReplicaCapture::preload(
    const std::atomic<bool>* cancel) {
  if (running_) {
    return vr::Status::invalid_argument(
        "ReplicaCapture::preload: call before start(); the frame the last "
        "poll handed out may borrow from the cache this replaces");
  }
  // Drop any previous cache first, so a second preload does not hold two
  // sequences' worth of frames at once while it refills.
  cache_.clear();
  cache_.resize(end_);
  std::size_t cached_frames = 0;
  for (std::size_t index = 0; index < end_; index += options_.frame_stride) {
    // Polled per frame rather than per batch: a caller tearing down waits at
    // most one frame's decode, not the whole sequence's.
    if (cancel != nullptr && cancel->load()) {
      break;
    }
    vr::Result<RgbdFrame> frame_result = load(index);
    if (!frame_result) {
      cache_.clear();
      return frame_result.status();
    }
    cache_[index] = std::move(frame_result).value();
    ++cached_frames;
  }
  return cached_frames;
}

std::size_t ReplicaCapture::preload_bytes_projected() const noexcept {
  return frame_count() * static_cast<std::size_t>(camera_.width) *
         camera_.height * (sizeof(float) + sizeof(std::uint32_t));
}

std::size_t ReplicaCapture::preloaded_bytes() const noexcept {
  std::size_t bytes = 0;
  for (const std::optional<RgbdFrame>& cached : cache_) {
    if (cached) {
      bytes += cached->depth.size() * sizeof(float) +
               cached->color.size() * sizeof(std::uint32_t);
    }
  }
  return bytes;
}

vr::Status ReplicaCapture::start() {
  running_ = true;
  return {};
}

void ReplicaCapture::stop() noexcept {
  running_ = false;
  next_ = 0;
  current_owned_.reset();
  current_borrowed_ = nullptr;
}

vr::Result<std::optional<vr::sensor::CapturedFrame>> ReplicaCapture::poll() {
  if (!running_ || next_ >= end_) {
    return no_frame();
  }
  const std::size_t index = next_;
  // A cache hit, else a disk read + JPEG/PNG decode. Decoded into a fresh
  // optional and swapped in only on success, so a failed decode leaves the
  // previous frame -- which a consumer may still be reading -- intact.
  if (index < cache_.size() && cache_[index]) {
    current_owned_.reset();
    current_borrowed_ = &*cache_[index];
  } else {
    VR_ASSIGN(RgbdFrame decoded, load(index));
    current_owned_ = std::move(decoded);
    current_borrowed_ = nullptr;
  }
  // Advance without overshooting: `index + stride` could wrap for a huge
  // stride, and `end_` is the exhausted position either way.
  next_ = (end_ - index > options_.frame_stride) ? index + options_.frame_stride
                                                 : end_;

  const RgbdFrame& stored = *current();
  vr::sensor::CapturedFrame frame{};
  frame.depth = stored.depth.data();
  frame.color = stored.color.data();
  frame.depth_camera = depth_camera_;
  frame.depth_camera.cam_to_world = stored.cam_to_world;
  frame.color_camera = color_camera_;
  frame.color_camera.cam_to_world = stored.cam_to_world;
  // color_encoding stays defaulted -- the default *is* the declaration
  // "canonical", which Replica's sRGB JPEGs are -- and timestamp_ns stays 0,
  // the contract's "the device reports none".
  return some_frame(frame);
}

}  // namespace vr_example
