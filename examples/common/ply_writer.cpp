// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "ply_writer.hpp"

#include <cstdint>
#include <fstream>
#include <ostream>

namespace vr_example {
namespace {

// Append the raw bytes of a trivially-copyable value to the stream (binary PLY
// is little-endian; the host is little-endian on every target we build).
template <typename T>
void put(std::ostream& out, const T& value) {
  out.write(reinterpret_cast<const char*>(&value), sizeof(T));
}

std::uint8_t to_u8(float channel) {
  const float scaled = channel * 255.0f + 0.5f;
  if (scaled <= 0.0f) {
    return 0;
  }
  if (scaled >= 255.0f) {
    return 255;
  }
  return static_cast<std::uint8_t>(scaled);
}

}  // namespace

vr::Status write_ply(const std::string& path, const vr::mesh::Mesh& mesh) {
  std::ofstream out(path, std::ios::binary);
  if (!out) {
    return vr::Status::io_error("write_ply: cannot open " + path);
  }

  const std::size_t vertex_count = mesh.vertices.size();
  const std::size_t face_count = mesh.indices.size() / 3;

  out << "ply\n"
      << "format binary_little_endian 1.0\n"
      << "comment volumetric_kit_recon fuse example\n"
      << "element vertex " << vertex_count << "\n"
      << "property float x\nproperty float y\nproperty float z\n"
      << "property float nx\nproperty float ny\nproperty float nz\n"
      << "property uchar red\nproperty uchar green\nproperty uchar blue\n"
      << "element face " << face_count << "\n"
      << "property list uchar int vertex_indices\n"
      << "end_header\n";

  for (const vr::mesh::Vertex& v : mesh.vertices) {
    put(out, v.position.x);
    put(out, v.position.y);
    put(out, v.position.z);
    put(out, v.normal.x);
    put(out, v.normal.y);
    put(out, v.normal.z);
    put(out, to_u8(v.color.x));
    put(out, to_u8(v.color.y));
    put(out, to_u8(v.color.z));
  }
  for (std::size_t f = 0; f < face_count; ++f) {
    put(out, static_cast<std::uint8_t>(3));
    put(out, static_cast<std::int32_t>(mesh.indices[f * 3 + 0]));
    put(out, static_cast<std::int32_t>(mesh.indices[f * 3 + 1]));
    put(out, static_cast<std::int32_t>(mesh.indices[f * 3 + 2]));
  }
  if (!out) {
    return vr::Status::io_error("write_ply: write failed for " + path);
  }
  return {};
}

}  // namespace vr_example
