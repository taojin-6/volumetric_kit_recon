// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#ifndef VR_HIERARCHICAL_TYPES_GLSL
#define VR_HIERARCHICAL_TYPES_GLSL

// Mirrors volume::HierarchicalNode under layout(scalar), 24 bytes.
struct HierarchicalNode {
  ivec3 coord;
  int ptr;
  uint level;
  uint children;
};

// Mathematical floor division, including negative world coordinates.
int vrHierarchyFloorDiv(int value, int divisor) {
  int q = value / divisor;
  return q - (value % divisor < 0 ? 1 : 0);
}

ivec3 vrHierarchyFloorDiv(ivec3 value, int divisor) {
  return ivec3(vrHierarchyFloorDiv(value.x, divisor),
               vrHierarchyFloorDiv(value.y, divisor),
               vrHierarchyFloorDiv(value.z, divisor));
}

#endif
