// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The frame's symbol models and its mask lines, shared by the rANS kernels
// that write a frame (rans_walk.glsl) and the one that reads it
// (rans_decode.comp). Mirrors bitstream.cpp.

#ifndef VR_RANS_MODELS_GLSL
#define VR_RANS_MODELS_GLSL

// Mirrors `Model` in bitstream.cpp.
const uint kDz = 0u;
const uint kDySame = 1u;
const uint kDyFree = 2u;
const uint kDxRun = 3u;
const uint kDxFree = 4u;
const uint kMaskClass = 5u;
const uint kPlane = 6u;
const uint kLine = 7u;
const uint kByte = 10u;
const uint kFirstCoef = 13u;

// Mask classes, and a plane's or line's symbol, as in bitstream.cpp.
const uint kMaskFull = 0u;
const uint kMaskEmpty = 1u;
const uint kMaskPartial = 2u;
const uint kSame = 0u;
const uint kAllEmpty = 1u;
const uint kAllFull = 2u;
const uint kOther = 3u;
const uint kMaskWords = 16u;

// Each model's alphabet, and its first entry in the per-symbol arrays (the
// counts, the tables) in TABLES order -- five coordinate models of 33
// classes, the mask class's 3, the plane's and three lines' 4, three byte
// models' 256, then 16 classes per coefficient. Mirrors frame_model_alphabet
// in bitstream.cpp, which the device frame test holds it to.
uint model_alphabet(uint model) {
  if (model <= kDxFree) return 33u;
  if (model == kMaskClass) return 3u;
  if (model < kByte) return 4u;
  if (model < kFirstCoef) return 256u;
  return 16u;
}

uint model_base(uint model) {
  if (model <= kDxFree) return 33u * model;
  if (model == kMaskClass) return 165u;
  if (model < kByte) return 168u + 4u * (model - kPlane);
  if (model < kFirstCoef) return 184u + 256u * (model - kByte);
  return 952u + 16u * (model - kFirstCoef);
}

// The block's mask being written or read: line l = y + 8 z is byte l % 4 of
// word l / 4.
uint g_mask[kMaskWords];

uint mask_line(uint l) { return (g_mask[l / 4u] >> (8u * (l % 4u))) & 0xFFu; }

// Which of the three line models a predictor selects.
uint line_context(uint predictor) {
  return predictor == 0x00u ? 0u : predictor == 0xFFu ? 1u : 2u;
}

// The line before in its plane, else the same line of the plane before, else
// unobserved.
uint line_predictor(uint z, uint y) {
  if (y > 0u) return mask_line(8u * z + y - 1u);
  return z > 0u ? mask_line(8u * (z - 1u)) : 0x00u;
}

#endif  // VR_RANS_MODELS_GLSL
