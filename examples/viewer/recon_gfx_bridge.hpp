// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file recon_gfx_bridge.hpp
/// @brief Interop-seam-A converter: a reconstruction mesh (recon::mesh::Mesh)
///        into the renderer's ingestion shape (gfx::assets::Mesh). Lives in the
///        neutral demo app -- the ONLY place aware of both siblings -- so
///        neither library depends on the other.

#include <utility>

#include <glm/vec4.hpp>

#include "volumetric_kit/gfx/assets/mesh.hpp"
#include "volumetric_kit/recon/mesh/mesh.hpp"

namespace fuse_viewer {

namespace rmesh = volumetric_kit::recon::mesh;
namespace gassets = volumetric_kit::gfx::assets;

/// @brief Convert a reconstruction mesh into a renderer mesh for the
/// hybrid-mesh
///        pipeline.
///
/// recon's `Vertex{position, normal, color, uv0}` maps to gfx's
/// `Vertex{position, normal, tangent, uv0, color}`: positions and normals are
/// already world-space (what the hybrid pipeline consumes), `color` and `uv0`
/// carry through unchanged. `uv0` is whatever the texture tier left it: a real
/// atlas coordinate on a triangle a keyframe textured, else recon's `(-1, -1)`
/// sentinel, which the hybrid fragment shader reads as "no atlas, use the
/// per-vertex color". `tangent` is synthesized to the glTF identity `(1, 0, 0,
/// 1)`: the hybrid pipeline does not consume it, but the interleaved
/// `assets::Vertex` carries the slot. Indices pass through (both are 32-bit
/// triangle lists).
inline gassets::Mesh to_gfx_mesh(const rmesh::Mesh& mesh) {
  gassets::Mesh out;
  out.name = "recon_reconstruction";
  out.vertices.resize(mesh.vertices.size());
  for (std::size_t i = 0; i < mesh.vertices.size(); ++i) {
    const rmesh::Vertex& in = mesh.vertices[i];
    gassets::Vertex& v = out.vertices[i];
    v.position = in.position;
    v.normal = in.normal;
    v.tangent = glm::vec4(1.0f, 0.0f, 0.0f, 1.0f);  // unused; slot carried
    v.uv0 = in.uv0;      // atlas coord where textured, else (-1,-1) sentinel
    v.color = in.color;  // per-vertex RGBA the TSDF tier fused
  }
  out.indices = mesh.indices;
  return out;
}

}  // namespace fuse_viewer
