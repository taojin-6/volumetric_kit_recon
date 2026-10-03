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
  // Do not test the sign of `%`: GLSL lowers it to signed modulo (OpSMod),
  // whose result follows the divisor, unlike the C++ remainder operation.
  return q - (value < 0 && value != q * divisor ? 1 : 0);
}

ivec3 vrHierarchyFloorDiv(ivec3 value, int divisor) {
  return ivec3(vrHierarchyFloorDiv(value.x, divisor),
               vrHierarchyFloorDiv(value.y, divisor),
               vrHierarchyFloorDiv(value.z, divisor));
}

#endif
