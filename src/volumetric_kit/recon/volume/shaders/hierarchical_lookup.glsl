// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#ifndef VR_HIERARCHICAL_LOOKUP_GLSL
#define VR_HIERARCHICAL_LOOKUP_GLSL

// Caller defines VR_HASH_ENTRIES_BINDING and VR_HIERARCHY_NODES_BINDING.
// Read-only topology, externally serialized against allocation/split/clear.
#include "volumetric_kit/recon/volume/shaders/hash_lookup.glsl"
#include "volumetric_kit/recon/volume/shaders/hierarchical_types.glsl"

layout(set = 0, binding = VR_HIERARCHY_NODES_BINDING, scalar) readonly buffer
    VrHierarchyNodes {
  HierarchicalNode vr_hierarchy_nodes[];
};

// Find the containing leaf, or stop at an internal node of stop_level.
// An internal result means every leaf in this aligned region is finer than
// stop_level. Meshing caches these regions to reject non-owned dual cells.
bool vrHierarchyLocateAtLevel(ivec3 finest_cell, VoxelGridParams root_grid,
                              uint max_level, uint node_capacity,
                              uint stop_level, out uint node_id,
                              out ivec3 local) {
  node_id = 0u;
  local = ivec3(0);
  if (max_level > 3u || stop_level > max_level || node_capacity == 0u) return false;
  // Arithmetic right shifts are floor division by a power of two, including
  // negative cells. Every hierarchy divisor is dyadic; avoid runtime IDiv.
  ivec3 root_coord = finest_cell >> int(max_level + 3u);
  int root_ptr = vrFindBlockPtr(root_coord, root_grid.num_buckets,
                                root_grid.bucket_size, root_grid.max_chain);
  if (root_ptr < 0 || root_ptr % 512 != 0) return false;
  node_id = uint(root_ptr) / 512u;
  for (uint step = 0u; step <= max_level; ++step) {
    if (node_id >= node_capacity) return false;
    HierarchicalNode node = vr_hierarchy_nodes[node_id];
    uint level = max_level - step;
    if (node.level != level ||
        node.coord != (finest_cell >> int(level + 3u))) return false;
    if (node.children == 0u) {
      if (node.ptr < 0 || node.ptr % 512 != 0 ||
          uint(node.ptr) / 512u >= node_capacity) return false;
      local = (finest_cell >> int(level)) - node.coord * 8;
      return true;
    }
    if (level == 0u || node_capacity < 8u ||
        node.children - 1u > node_capacity - 8u) return false;
    if (level == stop_level) {
      local = (finest_cell >> int(level)) - node.coord * 8;
      return true;
    }
    ivec3 child_coord = finest_cell >> int(level + 2u);
    ivec3 octant = child_coord & ivec3(1);
    node_id = node.children - 1u + uint(octant.x + 2 * octant.y + 4 * octant.z);
  }
  return false;
}

// Find the leaf containing one finest-grid integer cell. No floating-point
// epsilon is needed at boundaries; the represented cells are half-open.
bool vrHierarchyLocate(ivec3 finest_cell, VoxelGridParams root_grid,
                       uint max_level, uint node_capacity,
                       out uint node_id, out ivec3 local) {
  return vrHierarchyLocateAtLevel(finest_cell, root_grid, max_level,
                                  node_capacity, 0u, node_id, local);
}

#endif
