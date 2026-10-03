// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// Geometry-only OBJ input for codec examples. Units and orientation are
// explicit caller choices; OBJ positions alone do not establish metres.

#include <array>
#include <cstddef>
#include <cstdint>
#include <istream>
#include <string>
#include <vector>

#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/mesh/mesh.hpp"

namespace vr_example {

namespace vr = volumetric_kit::recon;
using Point3d = std::array<double, 3>;

/// Geometry consumed by triangle allocation and MeshIntegrator.
struct ObjGeometry {
  std::vector<vr::Vec3f> positions;
  std::vector<std::uint32_t> indices;
};

/// Axis-aligned position bounds, in the positions' current units.
struct MeshBounds {
  Point3d min{};
  Point3d max{};
};

/// The exact convention used to convert an OBJ into a metre-scale mesh.
struct MeshNormalization {
  MeshBounds original;
  MeshBounds normalized;
  Point3d up{};  ///< Unit head-up vector in the input coordinates.
  double original_height = 0.0;  ///< Extent projected onto up, input units.
  double metres_per_unit = 0.0;
};

/// Indexed edge and connected-component audit, not a self-intersection test.
struct MeshTopology {
  std::size_t edges = 0;
  std::size_t boundary_edges = 0;
  std::size_t nonmanifold_edges = 0;
  std::size_t inconsistent_edges = 0;
  std::size_t degenerate_triangles = 0;
  std::size_t components = 0;
  std::size_t nonpositive_components = 0;
  double signed_volume = 0.0;

  /// True when the checked topology supports closest-face signed distance.
  bool supports_signed() const noexcept;
};

/// Read triangular OBJ geometry; ignore materials, UVs and vertex normals.
/// Supports positive and relative position indices and v/vt/vn face syntax.
/// Refuses nonfinite positions, malformed faces, polygons and invalid indices.
vr::Result<ObjGeometry> read_obj_geometry(std::istream& input);

/// Open an OBJ and read its geometry; never modifies the source file.
vr::Result<ObjGeometry> read_obj_geometry(const std::string& path);

/// Rotate head-up to +Y, centre X/Z, put minimum Y at zero, and scale to
/// height. Input units are unspecified; height is in metres. A positive
/// determinant rotation preserves winding. Invalid inputs leave the geometry
/// unchanged.
vr::Result<MeshNormalization> normalize_mesh_height(ObjGeometry& geometry,
                                                    double height,
                                                    const Point3d& up);

/// Check indexed topology and signed component volumes for a loaded mesh.
MeshTopology audit_mesh_topology(const ObjGeometry& geometry);

/// Convert positions to the evaluator/exporter's host mesh container.
vr::mesh::Mesh geometry_mesh(const ObjGeometry& geometry);

}  // namespace vr_example
