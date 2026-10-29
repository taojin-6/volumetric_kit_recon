// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The sensor boundary's curve and the voxel colour format's, held together.
// core/color_space.hpp converts a camera's colour into the working space, and
// volume/voxel_color.hpp is what fusion stores and the mesher reads; each is a
// copy of the published sRGB curve (the 2026-08-02 colour-space decision),
// pinned to it on its own side. This test holds the two copies to each other,
// bit for bit, and lives in recon because recon is what links both: the voxel
// grid takes no colour vocabulary from the sensor side.
//
// Host only: the decode over every code, the voxel word's decode against
// color_space's unpack, and the encode and running-mean pack over a sweep of
// linear values and the edge cases.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <vector>

#include "volumetric_kit/core/testing/test_check.hpp"
#include "volumetric_kit/recon/core/color_space.hpp"
#include "volumetric_kit/recon/core/math/vector_types.hpp"
#include "volumetric_kit/recon/volume/voxel_color.hpp"

namespace vr = volumetric_kit::recon;
namespace vol = volumetric_kit::recon::volume;

namespace {

bool same_bits(float a, float b) {
  std::uint32_t x = 0;
  std::uint32_t y = 0;
  std::memcpy(&x, &a, sizeof(x));
  std::memcpy(&y, &b, sizeof(y));
  return x == y;
}

bool same_bits(const vr::Vec3f& a, const vr::Vec3f& b) {
  return same_bits(a.x, b.x) && same_bits(a.y, b.y) && same_bits(a.z, b.z);
}

}  // namespace

int main() {
  for (std::uint32_t c = 0; c < 256; ++c) {
    const float e = static_cast<float>(c) / 255.0f;
    CHECK(same_bits(vol::srgb_to_linear(e), vr::srgb_to_linear(e)));
    const std::uint32_t word = c | (c << 8) | (c << 16) | 0xFF000000u;
    CHECK(same_bits(vol::voxel_color_to_linear(word),
                    vr::unpack_srgb_to_linear(word)));
  }
  const float specials[] = {-0.0f,
                            -1.0f,
                            2.0f,
                            0.0031308f,
                            std::numeric_limits<float>::quiet_NaN(),
                            std::numeric_limits<float>::infinity(),
                            -std::numeric_limits<float>::infinity()};
  std::vector<float> linear(std::begin(specials), std::end(specials));
  for (int i = 0; i <= 1 << 16; ++i) {
    linear.push_back(static_cast<float>(i) / static_cast<float>(1 << 16));
  }
  for (const float l : linear) {
    CHECK(std::isnan(l) ||
          same_bits(vol::linear_to_srgb(l), vr::linear_to_srgb(l)));
    const vr::Vec3f v(l, 1.0f - l, 0.5f * l);
    CHECK(vol::voxel_color_from_linear(v) == vr::pack_linear_to_srgb(v));
  }
  std::puts(
      "recon tsdf voxel colour space: volume's curve matched color_space's");
  return 0;
}
