// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// tsdf::ChangeDetector's mask word, a uint a pixel, as the detection kernel
// (tsdf_change_detect.comp) writes it and a kernel that reads the mask decodes
// it: bits 0-1 are the tsdf::PixelChange; for Vanished, bits 2-31 are the
// depth of the surface that left, in units of kVanishedDepthUnit (0 for the
// other kinds).

#ifndef VR_TSDF_CHANGE_MASK_GLSL
#define VR_TSDF_CHANGE_MASK_GLSL

// The kinds (mirror tsdf::PixelChange).
const uint kChangeNone = 0u;
const uint kChangeNearer = 1u;
const uint kChangeFarther = 2u;
const uint kChangeVanished = 3u;

// A Vanished word's depth unit (m), mirroring kVanishedDepthUnit in
// change_detector.hpp: 0.1 mm, so 30 bits reach 107 km.
const float kVanishedDepthUnit = 1e-4;

uint change_kind(uint word) { return word & 3u; }

// The depth (m) of the surface a Vanished word says left.
float vanished_depth(uint word) {
  return float(word >> 2) * kVanishedDepthUnit;
}

// The Vanished word for a surface that left at depth d (m), rounded to the
// unit and capped at 30 bits; d is valid, so positive and finite.
uint vanished_word(float d) {
  uint units = uint(min(d / kVanishedDepthUnit + 0.5, 1073741823.0));
  return (units << 2) | kChangeVanished;
}

#endif  // VR_TSDF_CHANGE_MASK_GLSL
