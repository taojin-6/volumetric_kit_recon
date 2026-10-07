// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "replica_sensor.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <system_error>
#include <utility>

#include "volumetric_kit/core/base/check.hpp"
#include "volumetric_kit/recon/io/image_io.hpp"

namespace vkc = volumetric_kit::core;

namespace vr_example {
namespace {

namespace camera = vr::camera;
namespace sensor = vr::sensor;

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

vkc::Result<ReplicaSensor> ReplicaSensor::open(
    const std::string& scene_dir, const std::string& cam_params_path,
    const Options& options) {
  if (options.frame_stride == 0) {
    return vkc::Status::invalid_argument(
        "ReplicaSensor::open: frame_stride must be >= 1");
  }
  // 0 is the GPU pass's "no return", which it refuses as a range's near end,
  // so the range is refused here, named as this sensor's, before a frame.
  if (!(options.min_depth > 0.0f && options.min_depth < options.max_depth &&
        std::isfinite(options.max_depth))) {
    return vkc::Status::invalid_argument(
        "ReplicaSensor::open: depth range rejected (min_depth " +
        std::to_string(options.min_depth) + " m, max_depth " +
        std::to_string(options.max_depth) +
        " m): needs finite 0 < min_depth < max_depth");
  }
  ReplicaSensor replica;
  replica.options_ = options;
  replica.results_dir_ = scene_dir + "/results";

  // --- Intrinsics (cam_params.json) ---
  const std::optional<std::string> cam_json = read_file(cam_params_path);
  if (!cam_json) {
    return vkc::Status::invalid_argument(
        "ReplicaSensor::open: cannot read cam params: " + cam_params_path);
  }
  const std::array<const char*, 7> keys = {"fx", "fy", "cx",   "cy",
                                           "w",  "h",  "scale"};
  std::array<float, 7> values{};
  for (std::size_t k = 0; k < keys.size(); ++k) {
    const std::optional<float> v = json_number(*cam_json, keys[k]);
    if (!v) {
      return vkc::Status::invalid_argument(
          std::string("ReplicaSensor::open: cam params missing key '") +
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
      return vkc::Status::invalid_argument(
          "ReplicaSensor::open: cam params has a non-finite value");
    }
  }
  const float w = values[4];
  const float h = values[5];
  // In float, as the depth was once scaled on the host, so metres come out
  // bit-identical.
  replica.metres_per_unit_ = 1.0f / values[6];
  if (!(values[0] > 0.0f) || !(values[1] > 0.0f) || !(values[6] > 0.0f) ||
      !std::isfinite(replica.metres_per_unit_) || !(w >= 1.0f) ||
      !(w <= 65535.0f) || !(h >= 1.0f) || !(h <= 65535.0f)) {
    // fx/fy > 0 (a zero focal length divides by zero in the unprojection
    // x = (u - cx) * d / fx), scale > 0 with a finite inverse, and
    // width/height in a sane [1, 65535] so the uint32 cast below is
    // well-defined.
    return vkc::Status::invalid_argument(
        "ReplicaSensor::open: cam params has a zero/invalid intrinsic, "
        "dimension, or scale");
  }

  // One pinhole camera for both: Replica renders depth and colour from it.
  camera::CameraModel cam;
  cam.size = {static_cast<std::uint32_t>(w), static_cast<std::uint32_t>(h)};
  cam.intrinsics = {values[0], values[1], values[2], values[3]};
  replica.info_.id = "replica";
  replica.info_.model = "Replica-SLAM sequence";
  replica.info_.color = cam;
  replica.info_.depth = cam;
  replica.info_.pose = sensor::PoseSource::Tracked;

  // --- Trajectory (traj.txt): one flattened row-major 4x4 cam->world per
  // line, transposed into the column-major glm matrix. ---
  std::ifstream traj(scene_dir + "/traj.txt");
  if (!traj) {
    return vkc::Status::invalid_argument(
        "ReplicaSensor::open: cannot read trajectory: " + scene_dir +
        "/traj.txt");
  }
  std::vector<camera::Mat4d> poses;
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
      return vkc::Status::invalid_argument(
          "ReplicaSensor::open: blank line inside the trajectory (before "
          "pose " +
          std::to_string(poses.size()) + ")");
    }
    // Parsed in float and widened: the GPU pass narrows a pose back to float
    // exactly, so the fused volume is bit-identical to the host path's, where
    // a double parse could round an element one ulp apart.
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
      return vkc::Status::invalid_argument(
          "ReplicaSensor::open: malformed trajectory line " +
          std::to_string(poses.size()));
    }
    camera::Mat4d pose(1.0);
    for (int row = 0; row < 4; ++row) {
      for (int col = 0; col < 4; ++col) {
        pose[col][row] = m[static_cast<std::size_t>(row) * 4 + col];
      }
    }
    poses.push_back(pose);
  }
  if (poses.empty()) {
    return vkc::Status::invalid_argument(
        "ReplicaSensor::open: trajectory has no poses");
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
  const std::size_t bound = std::min(poses.size(), options.frame_limit);
  for (std::size_t index = 0; index < bound;) {
    if (!frame_on_disk(replica.results_dir_, index)) {
      break;
    }
    replica.frames_.push_back(Entry{index, poses[index], std::nullopt});
    // Step without overshooting: `index + stride` could wrap for a huge
    // stride, and past `bound` there is nothing to probe either way.
    if (bound - index <= options.frame_stride) {
      break;
    }
    index += options.frame_stride;
  }
  return replica;
}

vkc::Result<sensor::RgbdFrame> ReplicaSensor::load(const Entry& entry) const {
  const camera::CameraModel& cam = *info_.color;
  // One owner of both images, which every copy of the frame holds.
  struct Pixels {
    std::vector<std::uint16_t> depth;
    std::vector<std::uint32_t> color;
  };
  auto pixels = std::make_shared<Pixels>();
  // Size-checked against the camera the frame is stamped with, so the arrays
  // a consumer indexes by its size are exactly that long.
  VKC_ASSIGN(pixels->color,
             vr::io::load_color_packed(
                 frame_path(results_dir_, "frame", entry.index, ".jpg"),
                 cam.size.width, cam.size.height));
  VKC_ASSIGN(pixels->depth,
             vr::io::load_depth_u16(
                 frame_path(results_dir_, "depth", entry.index, ".png"),
                 cam.size.width, cam.size.height));
  // color_encoding stays defaulted -- the default *is* the declaration
  // "canonical", which Replica's sRGB JPEGs are -- and depth_to_color the
  // identity, the two cameras being one.
  sensor::RgbdFrame frame;
  frame.depth = pixels->depth.data();
  frame.metres_per_unit = metres_per_unit_;
  frame.depth_camera = cam;
  frame.min_depth = options_.min_depth;
  frame.max_depth = options_.max_depth;
  frame.color_packed = pixels->color.data();
  frame.color_camera = cam;
  frame.color_to_world = entry.pose;
  frame.pixels = std::move(pixels);
  return frame;
}

std::size_t ReplicaSensor::frame_bytes() const noexcept {
  const camera::ImageSize& size = info_.color->size;
  return static_cast<std::size_t>(size.width) * size.height *
         (sizeof(std::uint16_t) + sizeof(std::uint32_t));
}

vkc::Result<std::size_t> ReplicaSensor::preload(
    const std::atomic<bool>* cancel) {
  // Drop any previous cache first, so a second preload does not hold two
  // sequences' worth of frames at once while it refills.
  const auto drop = [this] {
    for (Entry& entry : frames_) entry.cached.reset();
  };
  drop();
  std::size_t cached_frames = 0;
  for (Entry& entry : frames_) {
    // Polled per frame rather than per batch: a caller tearing down waits at
    // most one frame's decode, not the whole sequence's.
    if (cancel != nullptr && cancel->load()) {
      break;
    }
    vkc::Result<sensor::RgbdFrame> frame = load(entry);
    if (!frame) {
      drop();
      return frame.status();
    }
    entry.cached = std::move(frame).value();
    ++cached_frames;
  }
  return cached_frames;
}

std::size_t ReplicaSensor::preload_bytes_projected() const noexcept {
  return frames_.size() * frame_bytes();
}

std::size_t ReplicaSensor::preloaded_bytes() const noexcept {
  std::size_t cached = 0;
  for (const Entry& entry : frames_) {
    cached += entry.cached ? 1 : 0;
  }
  return cached * frame_bytes();
}

vkc::Status ReplicaSensor::set_queue_depth(std::size_t frames) {
  if (frames == 0 || running_) {
    return vkc::Status::invalid_argument(
        "ReplicaSensor::set_queue_depth: at least 1, before start");
  }
  return {};
}

vkc::Status ReplicaSensor::start() {
  if (!running_) {
    running_ = true;
    stats_ = {};
  }
  return {};
}

void ReplicaSensor::stop() noexcept {
  running_ = false;
  next_ = 0;
}

vkc::Result<std::optional<sensor::RgbdFrame>> ReplicaSensor::poll() {
  if (!running_ || exhausted()) {
    return no_frame();
  }
  const Entry& entry = frames_[next_];
  // A cache hit, else a disk read + JPEG/PNG decode. A failed decode returns
  // here and leaves the position unchanged, so the next poll retries the same
  // frame.
  sensor::RgbdFrame frame;
  if (entry.cached) {
    frame = *entry.cached;
  } else {
    VKC_ASSIGN(frame, load(entry));
  }
  frame.sequence = next_;  // the frames handed out before it
  ++next_;
  ++stats_.received;
  ++stats_.delivered;
  return some_frame(std::move(frame));
}

vkc::Status ReplicaSensor::drain(std::vector<sensor::RgbdFrame>* out) {
  VKC_CHECK(out != nullptr, "ReplicaSensor::drain needs somewhere to put it");
  VKC_ASSIGN(std::optional<sensor::RgbdFrame> frame, poll());
  if (frame) out->push_back(std::move(*frame));
  return {};
}

}  // namespace vr_example
