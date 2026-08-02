// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file recon_gfx_bridge.hpp
/// @brief Interop-seam-A converter: a reconstruction mesh (recon::mesh::Mesh)
///        into the renderer's ingestion shape (gfx::assets::Mesh). Lives in the
///        neutral demo app -- the ONLY place aware of both siblings -- so
///        neither library depends on the other.

#include <cstddef>
#include <cstring>
#include <type_traits>
#include <utility>

#include <glm/vec4.hpp>

#include "volumetric_kit/gfx/assets/mesh.hpp"
#include "volumetric_kit/recon/mesh/mesh.hpp"

namespace fuse_viewer {

namespace rmesh = volumetric_kit::recon::mesh;
namespace gassets = volumetric_kit::gfx::assets;

/// @brief Adopt a reconstruction mesh as a renderer mesh for the hybrid-mesh
///        pipeline.
///
/// recon's `mesh::Vertex` is byte-for-byte `gfx::assets::Vertex` -- the mesh
/// tier emits the renderer's layout directly (see the layout `static_assert`s
/// in `mesh/mesh.hpp`), so there is no field-by-field conversion left: this
/// bulk-copies the vertex bytes and the indices. Both structs are trivially
/// copyable PODs, and the assertions below fail the build rather than let a
/// layout change on either side turn into a silent misread.
///
/// `uv0` is whatever the texture tier left: a real atlas coordinate on a
/// triangle a keyframe textured, else recon's `(-1, -1)` sentinel, which the
/// hybrid fragment shader reads as "no atlas, use the per-vertex color".
///
/// This copy exists only because the viewer hands gfx a *host* mesh (interop
/// seam A, two devices). With a shared device the same bytes are drawn from
/// recon's own buffer and even this goes away.
inline gassets::Mesh to_gfx_mesh(const rmesh::Mesh& mesh) {
  static_assert(sizeof(rmesh::Vertex) == sizeof(gassets::Vertex),
                "recon and gfx vertex layouts have diverged in size");
  static_assert(
      offsetof(rmesh::Vertex, position) == offsetof(gassets::Vertex, position),
      "recon and gfx vertex layouts have diverged: position");
  static_assert(
      offsetof(rmesh::Vertex, normal) == offsetof(gassets::Vertex, normal),
      "recon and gfx vertex layouts have diverged: normal");
  static_assert(
      offsetof(rmesh::Vertex, tangent) == offsetof(gassets::Vertex, tangent),
      "recon and gfx vertex layouts have diverged: tangent");
  static_assert(offsetof(rmesh::Vertex, uv0) == offsetof(gassets::Vertex, uv0),
                "recon and gfx vertex layouts have diverged: uv0");
  static_assert(
      offsetof(rmesh::Vertex, color) == offsetof(gassets::Vertex, color),
      "recon and gfx vertex layouts have diverged: color");
  static_assert(std::is_trivially_copyable<rmesh::Vertex>::value &&
                    std::is_trivially_copyable<gassets::Vertex>::value,
                "vertex bulk copy requires trivially copyable layouts");

  gassets::Mesh out;
  out.name = "recon_reconstruction";
  out.vertices.resize(mesh.vertices.size());
  if (!mesh.vertices.empty()) {
    std::memcpy(out.vertices.data(), mesh.vertices.data(),
                mesh.vertices.size() * sizeof(gassets::Vertex));
  }
  out.indices = mesh.indices;
  return out;
}

}  // namespace fuse_viewer
