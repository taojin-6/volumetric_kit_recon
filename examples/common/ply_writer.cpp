// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "ply_writer.hpp"

#include <cstdint>
#include <exception>
#include <fstream>
#include <ostream>
#include <vector>

#include "tinyply.h"  // declarations only; the implementation is tinyply_impl.cpp
#include "volumetric_kit/recon/core/color_space.hpp"

namespace vr_example {
namespace {

// Encode a LINEAR vertex-color channel to an 8-bit canonical-encoded code.
//
// PLY is a presentation site. `mesh::Vertex::color` is linear working values
// (the 2026-08-02 color-space decision -- what glTF calls COLOR_0), but a PLY's
// `uchar` red/green/blue properties carry no color-space tag and every external
// viewer (MeshLab, Blender, CloudCompare) reads them as sRGB. Writing the
// linear value straight out would therefore render visibly dark, so the encode
// happens here. glTF export does the opposite and passes COLOR_0 through
// unchanged, which is why the two exporters cannot share one path.
std::uint8_t to_u8(float linear_channel) {
  const float scaled = vr::linear_to_srgb(linear_channel) * 255.0f + 0.5f;
  // `!(scaled > 0.0f)` maps NaN to 0 as well as <= 0 (every NaN comparison is
  // false), so a NaN channel never reaches the undefined float->uint8_t cast.
  if (!(scaled > 0.0f)) {
    return 0;
  }
  if (scaled >= 255.0f) {
    return 255;
  }
  return static_cast<std::uint8_t>(scaled);
}

}  // namespace

vr::Status write_ply(const std::string& path, const vr::mesh::Mesh& mesh) {
  const std::size_t vertex_count = mesh.vertices.size();
  const std::size_t face_count = mesh.indices.size() / 3;

  // tinyply writes each property group from a tightly-packed array, so
  // de-interleave the Mesh's array-of-Vertex-struct into per-attribute buffers
  // (position, normal, and the u8-quantized colour). One-shot final export, so
  // the extra copies are inconsequential.
  std::vector<float> positions(vertex_count * 3);
  std::vector<float> normals(vertex_count * 3);
  std::vector<std::uint8_t> colors(vertex_count * 3);
  for (std::size_t i = 0; i < vertex_count; ++i) {
    const vr::mesh::Vertex& v = mesh.vertices[i];
    positions[i * 3 + 0] = v.position.x;
    positions[i * 3 + 1] = v.position.y;
    positions[i * 3 + 2] = v.position.z;
    normals[i * 3 + 0] = v.normal.x;
    normals[i * 3 + 1] = v.normal.y;
    normals[i * 3 + 2] = v.normal.z;
    colors[i * 3 + 0] = to_u8(v.color.x);
    colors[i * 3 + 1] = to_u8(v.color.y);
    colors[i * 3 + 2] = to_u8(v.color.z);
  }
  // PLY's conventional face-index type is `int` (the interleaved triangle-soup
  // indices, uint32 in the Mesh, always fit int32 -- a >2^31-vertex mesh is
  // unrepresentable in the uint32-index Mesh anyway).
  std::vector<std::int32_t> faces(mesh.indices.begin(), mesh.indices.end());

  tinyply::PlyFile ply;
  ply.get_comments().push_back("volumetric_kit_recon fuse example");
  ply.add_properties_to_element(
      "vertex", {"x", "y", "z"}, tinyply::Type::FLOAT32, vertex_count,
      reinterpret_cast<const std::uint8_t*>(positions.data()),
      tinyply::Type::INVALID, 0);
  ply.add_properties_to_element(
      "vertex", {"nx", "ny", "nz"}, tinyply::Type::FLOAT32, vertex_count,
      reinterpret_cast<const std::uint8_t*>(normals.data()),
      tinyply::Type::INVALID, 0);
  ply.add_properties_to_element("vertex", {"red", "green", "blue"},
                                tinyply::Type::UINT8, vertex_count,
                                colors.data(), tinyply::Type::INVALID, 0);
  ply.add_properties_to_element(
      "face", {"vertex_indices"}, tinyply::Type::INT32, face_count,
      reinterpret_cast<const std::uint8_t*>(faces.data()), tinyply::Type::UINT8,
      3);

  std::ofstream out(path, std::ios::binary);
  if (!out) {
    return vr::Status::io_error("write_ply: cannot open " + path);
  }
  try {
    ply.write(out, /*isBinary=*/true);
  } catch (const std::exception& e) {
    return vr::Status::io_error("write_ply: tinyply failed for " + path + ": " +
                                e.what());
  }
  // Flush before the final check so a failed final flush (e.g. the disk fills
  // on the last buffer) is caught here rather than silently in the destructor.
  out.flush();
  if (!out) {
    return vr::Status::io_error("write_ply: write failed for " + path);
  }
  return {};
}

}  // namespace vr_example
