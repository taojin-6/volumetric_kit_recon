// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// Private to recon_tsdf: the connectivity MeshIntegrator's signed mode signs
// with. Host-only, so it includes no Vulkan.

#include <cstdint>
#include <vector>

#include "volumetric_kit/recon/core/math/vector_types.hpp"

namespace volumetric_kit::recon::tsdf::detail {

// What signed mode needs from a mesh beyond its positions: the angle-weighted
// pseudonormal of every vertex and every edge (Baerentzen & Aanaes, "Signed
// distance computation using the angle weighted pseudo-normal", TVCG 2005),
// over a mesh welded by position.
//
// Why pseudonormals and not the nearest face's normal, which is what the prior
// engine signed with: a voxel whose closest point is an edge or a vertex is
// equidistant from every face that meets there, and on a sharp feature those
// faces disagree about its side. Past a convex edge whose faces meet at less
// than 90 degrees, part of the edge's region reads as inside through one face
// and outside through the other, so the sign follows whichever face the loop
// found first. The pseudonormal of the feature itself is the one normal that
// is correct there for any closed, consistently wound mesh.
//
// Welding is what makes "the feature" mean anything. A mesh stored as a
// triangle soup -- every STL, and any exporter that splits vertices per face
// -- has no two triangles sharing an index, so without it every edge would
// read as a boundary.
struct MeshTopology {
  // The caller's indices with each corner redirected to the first vertex at its
  // exact position. The positions are identical, so the kernel reads the same
  // geometry through these; what changes is that shared corners now share an
  // index, which is how the kernel finds their pseudonormal.
  std::vector<std::uint32_t> indices;
  // One per vertex, indexed by welded index. Zero on a vertex that touches a
  // boundary edge, or that no used triangle references.
  std::vector<Vec3f> vertex_normals;
  // Three per triangle: entry 3t + k is the edge from corner k to corner
  // (k + 1) % 3, matching kVrFeatureEdgeAB/BC/CA in triangle_common.glsl. Zero
  // on a boundary edge, and on an edge whose two faces cancel (a fold of zero
  // thickness), neither of which has a side.
  std::vector<Vec3f> edge_normals;
  // Edges one used triangle has and no other shares: an open mesh's rim.
  // Allowed -- a voxel nearest one is left unobserved rather than signed.
  std::uint32_t boundary_edges = 0;
  // Edges three or more used triangles share. Their pseudonormal is not
  // correct for any side, so signed mode refuses a mesh that has one.
  std::uint32_t nonmanifold_edges = 0;
  // Vertices where two fans of triangles touch at a point with every edge
  // manifold (a "bowtie"). Refused for the same reason as a non-manifold edge.
  std::uint32_t nonmanifold_vertices = 0;
  // Edges whose two triangles traverse it in the same direction: the winding
  // flips across it, so "outside" means opposite things on its two faces.
  // Signed mode refuses these too.
  std::uint32_t inconsistent_edges = 0;
};

// Build the topology over the triangles `candidate_offsets` marks as used (a
// non-empty range, so exactly the set the kernels see). The offsets are
// volume::triangle_candidate_offsets' result for the same mesh, which has
// already bounds-checked every index.
MeshTopology build_mesh_topology(
    const Vec3f* vertices, std::uint32_t vertex_count,
    const std::uint32_t* indices, std::uint32_t triangle_count,
    const std::vector<std::uint32_t>& candidate_offsets);

}  // namespace volumetric_kit::recon::tsdf::detail
