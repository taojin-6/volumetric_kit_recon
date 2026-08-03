// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The device mirror of core/color_space.hpp: the one transfer curve every
// compute tier converts through. #included by tsdf/shaders/tsdf_integrate.comp
// and mesh/shaders/marching_cubes{,_sparse}.comp -- from `core` rather than
// from a tier, because `sensor` (which owns the boundary *policy*) branches off
// `core` beside the fusion tiers and cannot be included by them, while the
// curve itself is vocabulary four tiers need. This is the only cross-tier GLSL
// include; it resolves through the shader include root vr_compile_shaders
// passes, spelled like the C++ header path so its provenance is visible at the
// include site.
//
// The rule (DESIGN.md, "Color space"): 8-bit color is encoded, float color is
// linear. Averaging is linear and a display encoding deliberately is not, so
// every blend below decodes first. The canonical encoded form is the working
// space (linear BT.709/D65) through the EXACT piecewise sRGB transfer, full
// range -- exact, not pow(x, 2.2), because hardware _SRGB sampling decodes the
// atlas with the exact curve and the renderer's hybrid pipeline picks between
// the atlas and the vertex color per triangle across one surface: an
// approximation here seams exactly where texturing stops.
//
// Keep in lockstep with core/color_space.hpp. That is not a convention but a
// tested claim -- tests/core_color_space_gpu_test.cpp runs these functions over
// all 256 codes and compares against the host ones.

#ifndef VR_CORE_COLOR_COMMON_GLSL
#define VR_CORE_COLOR_COMMON_GLSL

// Exact piecewise sRGB decode, one channel. Values outside [0,1] take the same
// branches rather than clamping, so the function stays monotone.
float vrSrgbToLinear(float e) {
  return e <= 0.04045 ? e * (1.0 / 12.92)
                      : pow((e + 0.055) * (1.0 / 1.055), 2.4);
}

// Exact piecewise sRGB encode, one channel. Inverse of vrSrgbToLinear.
float vrLinearToSrgb(float l) {
  return l <= 0.0031308 ? l * 12.92 : 1.055 * pow(l, 1.0 / 2.4) - 0.055;
}

vec3 vrSrgbToLinear(vec3 e) {
  return vec3(vrSrgbToLinear(e.r), vrSrgbToLinear(e.g), vrSrgbToLinear(e.b));
}

vec3 vrLinearToSrgb(vec3 l) {
  return vec3(vrLinearToSrgb(l.r), vrLinearToSrgb(l.g), vrLinearToSrgb(l.b));
}

// Decode a packed canonical-encoded RGB word (R,G,B in the low three bytes --
// the layout the tsdf and mesh tiers share) to linear working values. Alpha is
// ignored; see vrPackLinearToSrgb for why it is nonetheless always written.
vec3 vrUnpackSrgbToLinear(uint packed) {
  return vrSrgbToLinear(unpackUnorm4x8(packed).rgb);
}

// Encode linear working values back into a packed canonical word. Clamps and
// rounds to nearest (packUnorm4x8's contract, matching the host's +0.5 round),
// and always writes alpha 0xFF -- which is what keeps the tsdf tier's "color
// unobserved" sentinel exact, since a written color is then never 0 even for
// pure black.
uint vrPackLinearToSrgb(vec3 linear) {
  return packUnorm4x8(vec4(vrLinearToSrgb(clamp(linear, 0.0, 1.0)), 1.0));
}

#endif  // VR_CORE_COLOR_COMMON_GLSL
