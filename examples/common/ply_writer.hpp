// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/ply_writer.hpp
/// @brief Write a @ref volumetric_kit::recon::mesh::Mesh to a binary PLY, for
///        inspecting a reconstruction in MeshLab / Blender / Open3D.

#include <string>

#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/mesh/mesh.hpp"

namespace vr_example {

namespace vr = volumetric_kit::recon;

/// @brief Serialize @p mesh to a binary little-endian PLY at @p path.
///
/// Writes per-vertex `x y z` (float), `nx ny nz` (float), and `red green blue`
/// (uchar, from @ref mesh::Vertex::color), then a triangle face list (the
/// interleaved index triples). Alpha and `uv0` are dropped (PLY has no standard
/// slot the common viewers read). A convenience for the examples, not a
/// production exporter.
/// @return OK on success, or a non-OK @ref vr::Status if the file cannot be
///         opened / written.
vr::Status write_ply(const std::string& path, const vr::mesh::Mesh& mesh);

}  // namespace vr_example
