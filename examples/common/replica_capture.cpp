// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "replica_capture.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <system_error>
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

// True if a file is at the path. A stat, not an open: the probe at open() runs
// once per frame that will play, and constructing an ifstream costs ~10x a
// stat for the same answer. Readability is the decoder's problem -- a present
// but unreadable frame fails its decode with stb's reason, as it should.
bool file_exists(const std::string& path) {
  std::error_code ec;
  return std::filesystem::exists(path, ec) && !ec;
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

// Every member, in declaration order, and the source left as a
// default-constructed capture: nothing on disk, not running, exhausted. A
// forgotten member here is silent -- the moved-from shell would keep its
// value and report a sequence it no longer holds -- so a new member is added
// to both of these in the same change that declares it.
ReplicaCapture::ReplicaCapture(ReplicaCapture&& other) noexcept
    : options_(other.options_),
      depth_scale_(other.depth_scale_),
      depth_camera_(other.depth_camera_),
      color_camera_(other.color_camera_),
      poses_(std::move(other.poses_)),
      results_dir_(std::move(other.results_dir_)),
      end_(other.end_),
      cache_(std::move(other.cache_)),
      running_(other.running_),
      next_(other.next_),
      current_owned_(std::move(other.current_owned_)) {
  other.options_ = Options{};
  other.depth_scale_ = 1.0f;
  other.depth_camera_ = vr::DepthCameraParams{};
  other.color_camera_ = vr::ColorCameraParams{};
  other.poses_.clear();
  other.results_dir_.clear();
  other.end_ = 0;
  other.cache_.clear();
  other.running_ = false;
  other.next_ = 0;
  other.current_owned_.reset();
}

ReplicaCapture& ReplicaCapture::operator=(ReplicaCapture&& other) noexcept {
  if (this != &other) {
    // Steal through the move constructor so the member list lives in one
    // place, then swap the stolen state in. The temporary carries this
    // object's old state out and frees it.
    ReplicaCapture taken(std::move(other));
    std::swap(options_, taken.options_);
    std::swap(depth_scale_, taken.depth_scale_);
    std::swap(depth_camera_, taken.depth_camera_);
    std::swap(color_camera_, taken.color_camera_);
    std::swap(poses_, taken.poses_);
    std::swap(results_dir_, taken.results_dir_);
    std::swap(end_, taken.end_);
    std::swap(cache_, taken.cache_);
    std::swap(running_, taken.running_);
    std::swap(next_, taken.next_);
    std::swap(current_owned_, taken.current_owned_);
  }
  return *this;
}

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
  capture.depth_scale_ = values[6];

  // Both camera blocks once, here. Depth and colour are one registered camera
  // on Replica, so the depth camera is *derived* from the colour one exactly
  // as a registered live source's is -- which also validates the depth range
  // the options carry, before a frame is ever handed out. The refusal is
  // re-labelled as this capture's: the library's message names min_depth and
  // max_depth, and a caller that exposes only one of them (the viewers take
  // --min-depth and --max-depth; a default is still a value) should hear
  // which object's range it is arguing with.
  vr::ColorCameraParams& color = capture.color_camera_;
  color.fx = values[0];
  color.fy = values[1];
  color.cx = values[2];
  color.cy = values[3];
  color.width = static_cast<std::uint32_t>(w);
  color.height = static_cast<std::uint32_t>(h);
  vr::Result<vr::DepthCameraParams> depth =
      vr::sensor::depth_from_registered_color(color, color.width, color.height,
                                              options.min_depth,
                                              options.max_depth);
  if (!depth) {
    return vr::Status::invalid_argument(
        "ReplicaCapture::open: depth range rejected (min_depth " +
        std::to_string(options.min_depth) + " m, max_depth " +
        std::to_string(options.max_depth) + " m): " + depth.status().message());
  }
  capture.depth_camera_ = std::move(depth).value();

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

  // --- The frames actually present, probed once here rather than discovered
  // by the first poll to hit an absent image, so frame_count() is the number
  // of frames that will really play. Only the frames the options select are
  // looked at -- the strided indices under the limit -- so the probe scales
  // with the frames the run will play, not with the sequence on disk (at
  // `--max-frames 5` that is 5 frames, not room0's 400), and an index the
  // stride never visits cannot end the sequence. Poses are matched to
  // frameNNNNNN by position, so a gap at a visited index ends the sequence
  // exactly as a missing tail does.
  const std::size_t bound =
      std::min(capture.poses_.size(), options.frame_limit);
  std::size_t end = 0;
  for (std::size_t index = 0; index < bound;) {
    if (!frame_on_disk(capture.results_dir_, index)) {
      break;
    }
    end = index + 1;
    // Step without overshooting: `index + stride` could wrap for a huge
    // stride, and past `bound` there is nothing to probe either way.
    if (bound - index <= options.frame_stride) {
      break;
    }
    index += options.frame_stride;
  }
  capture.end_ = end;
  return capture;
}

std::size_t ReplicaCapture::frame_count() const noexcept {
  return end_ == 0 ? 0 : (end_ - 1) / options_.frame_stride + 1;
}

vr::Result<RgbdFrame> ReplicaCapture::load(std::size_t index) const {
  if (index >= end_ || index >= poses_.size()) {
    return vr::Status::invalid_argument("ReplicaCapture::load: index " +
                                        std::to_string(index) +
                                        " past the sequence");
  }
  const std::string color_path =
      frame_path(results_dir_, "frame", index, ".jpg");
  const std::string depth_path =
      frame_path(results_dir_, "depth", index, ".png");
  RgbdFrame frame;
  // Size-checked against the camera structs the frame is stamped with, so the
  // buffer a consumer indexes by `depth_camera.width * height` is exactly that
  // long.
  VR_ASSIGN(frame.color, load_color_packed(color_path, color_camera_.width,
                                           color_camera_.height));
  VR_ASSIGN(frame.depth, load_depth_metres(depth_path, depth_camera_.width,
                                           depth_camera_.height, depth_scale_));
  // Both cameras from the one trajectory entry: depth and colour are one
  // registered camera on Replica, so the two poses cannot drift apart.
  // color_encoding stays defaulted -- the default *is* the declaration
  // "canonical", which Replica's sRGB JPEGs are -- and timestamp_ns stays 0,
  // the contract's "the device reports none".
  frame.depth_camera = depth_camera_;
  frame.depth_camera.cam_to_world = poses_[index];
  frame.color_camera = color_camera_;
  frame.color_camera.cam_to_world = poses_[index];
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
  for (std::size_t index = 0; index < end_;) {
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
    // The probe's step: past this the next strided index is beyond `end_`,
    // and `index + stride` could wrap.
    if (end_ - index <= options_.frame_stride) {
      break;
    }
    index += options_.frame_stride;
  }
  return cached_frames;
}

std::size_t ReplicaCapture::preload_bytes_projected() const noexcept {
  return frame_count() * static_cast<std::size_t>(color_camera_.width) *
         color_camera_.height * (sizeof(float) + sizeof(std::uint32_t));
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
}

vr::Result<std::optional<vr::sensor::CapturedFrame>> ReplicaCapture::poll() {
  if (!running_ || exhausted()) {
    return no_frame();
  }
  const std::size_t index = next_;
  // A cache hit, else a disk read + JPEG/PNG decode. The decode lands in a
  // local and is move-assigned over the previous streamed frame only on
  // success -- which releases that frame's storage, so a view of it is stale
  // from here (the contract's "until the next poll"). A failed decode returns
  // above the assignment and leaves the position unchanged, so the next poll
  // retries the same frame.
  const RgbdFrame* stored = nullptr;
  if (index < cache_.size() && cache_[index]) {
    current_owned_.reset();
    stored = &*cache_[index];
  } else {
    VR_ASSIGN(current_owned_, load(index));
    stored = &*current_owned_;
  }
  // Advance without overshooting: `index + stride` could wrap for a huge
  // stride, and `end_` is the exhausted position either way.
  next_ = (end_ - index > options_.frame_stride) ? index + options_.frame_stride
                                                 : end_;

  return some_frame(stored->view());
}

}  // namespace vr_example
