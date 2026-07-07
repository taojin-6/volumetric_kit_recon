// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "dataset.hpp"

#include <array>
#include <cstdio>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>

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

}  // namespace

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
  ds.camera_.fx = values[0];
  ds.camera_.fy = values[1];
  ds.camera_.cx = values[2];
  ds.camera_.cy = values[3];
  ds.camera_.width = static_cast<std::uint32_t>(values[4]);
  ds.camera_.height = static_cast<std::uint32_t>(values[5]);
  ds.camera_.depth_scale = values[6];
  if (ds.camera_.width == 0 || ds.camera_.height == 0 ||
      !(ds.camera_.depth_scale > 0.0f)) {
    return vr::Status::invalid_argument(
        "ReplicaDataset::open: cam params has a zero dimension or scale");
  }

  // --- Trajectory (traj.txt): one flattened row-major 4x4 cam->world per line,
  // transposed into the column-major glm matrix the pipeline uploads. ---
  std::ifstream traj(scene_dir + "/traj.txt");
  if (!traj) {
    return vr::Status::invalid_argument(
        "ReplicaDataset::open: cannot read trajectory: " + scene_dir +
        "/traj.txt");
  }
  std::string line;
  while (std::getline(traj, line)) {
    if (line.find_first_not_of(" \t\r\n") == std::string::npos) {
      continue;  // skip a blank trailing line
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

vr::Result<RgbdFrame> ReplicaDataset::load(std::size_t i) const {
  if (i >= poses_.size()) {
    return vr::Status::invalid_argument("ReplicaDataset::load: index " +
                                        std::to_string(i) + " out of range");
  }
  RgbdFrame frame;
  frame.cam_to_world = poses_[i];
  VR_ASSIGN(frame.color,
            load_color_packed(frame_path(results_dir_, "frame", i, ".jpg"),
                              camera_.width, camera_.height));
  VR_ASSIGN(
      frame.depth,
      load_depth_metres(frame_path(results_dir_, "depth", i, ".png"),
                        camera_.width, camera_.height, camera_.depth_scale));
  return frame;
}

}  // namespace vr_example
