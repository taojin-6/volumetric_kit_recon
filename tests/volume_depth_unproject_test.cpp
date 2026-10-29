// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// Volume's depth unprojection (volume/shaders/depth_pinhole.glsl), the line
// depth allocation places every sample by, is a copy of the pinhole recon
// exports, held to it here on the device (shaders/depth_unproject_cases.comp):
//
//   * bit for bit core/shaders/pinhole.glsl's vr_unproject_pinhole, on every
//     case, so the copy allocates the blocks the shared one did;
//   * the published formula, x = (u - cx) z / fx and y = (v - cy) z / fy,
//     against the host's double within float rounding (a relative 5e-7, the
//     error of three float operations), z exactly, and x or y exactly 0 at
//     the principal point.
//
// The cases: a grid of pixels in and around five cameras, from 8 x 8 to
// 3840 x 2160, principal points inside and outside the image, at five depths.
// The one test that reads both copies. Skips where no device is present.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "buffer_readback.hpp"
#include "test_check.hpp"
#include "volumetric_kit/core/vulkan/compute_kernel.hpp"
#include "volumetric_kit/core/vulkan/compute_util.hpp"
#include "volumetric_kit/core/vulkan/vulkan.hpp"
#include "volumetric_kit/recon/core/math/vector_types.hpp"

#include "depth_unproject_cases_comp.spv.hpp"
#include "gpu_test.hpp"

namespace vr = volumetric_kit::recon;
namespace vkc = volumetric_kit::core;

namespace {

// Mirrors Case in shaders/depth_unproject_cases.comp under std430.
struct UnprojectCase {
  vr::Vec4f k;  // fx, fy, cx, cy
  vr::Vec2f pixel;
  float depth;
  float pad;
};
static_assert(sizeof(UnprojectCase) == 32 &&
                  offsetof(UnprojectCase, pixel) == 16 &&
                  offsetof(UnprojectCase, depth) == 24,
              "mirrors Case in shaders/depth_unproject_cases.comp");

struct Camera {
  vr::Vec4f k;
  float width, height;
};

std::vector<UnprojectCase> cases() {
  const Camera cameras[] = {
      {vr::Vec4f(525.0f, 525.0f, 319.5f, 239.5f), 640.0f, 480.0f},
      {vr::Vec4f(211.37f, 209.81f, 63.29f, 47.61f), 128.0f, 96.0f},
      {vr::Vec4f(1810.7f, 1809.3f, 1919.6f, 1079.4f), 3840.0f, 2160.0f},
      {vr::Vec4f(60.0f, 60.0f, 31.5f, 23.5f), 64.0f, 48.0f},
      {vr::Vec4f(3.3f, 7.1f, -2.6f, 40.25f), 8.0f, 8.0f}};
  const float depths[] = {0.25f, 1.0f, 2.7182817f, 7.5f, 1000.0f};
  std::vector<UnprojectCase> out;
  for (const Camera& c : cameras) {
    for (int i = 0; i <= 22; ++i) {
      for (int j = 0; j <= 16; ++j) {
        // From a pixel left of and above the image to one past it, off the
        // pixel centres by a different fraction each step.
        const float u =
            -1.0f + (c.width + 2.0f) * static_cast<float>(i) / 22.0f;
        const float v =
            -1.0f + (c.height + 2.0f) * static_cast<float>(j) / 16.0f;
        for (const float z : depths) out.push_back({c.k, {u, v}, z, 0.0f});
      }
    }
    // The principal point, where the offset is exactly 0.
    out.push_back({c.k, {c.k.z, c.k.w}, 1.5f, 0.0f});
  }
  return out;
}

bool same_bits(float a, float b) {
  std::uint32_t x = 0;
  std::uint32_t y = 0;
  std::memcpy(&x, &a, sizeof(x));
  std::memcpy(&y, &b, sizeof(y));
  return x == y;
}

// Within a relative 5e-7 of the host's double, and exactly 0 where it is.
bool near(float got, double want) {
  return std::abs(static_cast<double>(got) - want) <= 5e-7 * std::abs(want);
}

int gpu_main(vr_test::GpuContext& gpu) {
  const std::vector<UnprojectCase> all = cases();
  const auto n = static_cast<std::uint32_t>(all.size());

  vkc::ComputeKernel kernel;
  vkc::KernelSetBuilder builder(gpu.device);
  CHECK(builder
            .add(kernel, "depth_unproject_cases",
                 vr_depth_unproject_cases_comp_spv,
                 vr_depth_unproject_cases_comp_spv_size, 3)
            .ok());
  auto pool = builder.build();
  auto in = vr_test::upload_device_buffer(gpu.device, gpu.allocator, all.data(),
                                          n * sizeof(all[0]));
  auto volume_out =
      vkc::device_storage_buffer(gpu.allocator, n * sizeof(vr::Vec4f));
  auto core_out =
      vkc::device_storage_buffer(gpu.allocator, n * sizeof(vr::Vec4f));
  CHECK(pool.ok() && in.ok() && volume_out.ok() && core_out.ok());
  kernel.set.write_storage_buffer(0, in->handle(), 0, VK_WHOLE_SIZE);
  kernel.set.write_storage_buffer(1, volume_out->handle(), 0, VK_WHOLE_SIZE);
  kernel.set.write_storage_buffer(2, core_out->handle(), 0, VK_WHOLE_SIZE);
  CHECK(vkc::dispatch(gpu.device, kernel, nullptr, 0, vkc::group_count(n, 64),
                      gpu.device.caps().limits().maxComputeWorkGroupCount[0])
            .ok());
  auto volume_points =
      vr_test::read_back<vr::Vec4f>(gpu.device, gpu.allocator, *volume_out, n);
  auto core_points =
      vr_test::read_back<vr::Vec4f>(gpu.device, gpu.allocator, *core_out, n);
  CHECK(volume_points.ok() && core_points.ok());

  int failures = 0;
  for (std::uint32_t i = 0; i < n; ++i) {
    const UnprojectCase& c = all[i];
    const vr::Vec4f got = volume_points.value()[i];
    const vr::Vec4f shared = core_points.value()[i];
    const double z = c.depth;
    const double x = (double{c.pixel.x} - c.k.z) * z / c.k.x;
    const double y = (double{c.pixel.y} - c.k.w) * z / c.k.y;
    const bool bits = same_bits(got.x, shared.x) &&
                      same_bits(got.y, shared.y) && same_bits(got.z, shared.z);
    const bool formula =
        near(got.x, x) && near(got.y, y) && static_cast<double>(got.z) == z;
    if ((!bits || !formula) && ++failures <= 20) {
      std::fprintf(stderr,
                   "case %u: k (%g, %g, %g, %g) pixel (%g, %g) z %g -> volume "
                   "(%.9g, %.9g, %.9g), pinhole.glsl (%.9g, %.9g, %.9g), host "
                   "(%.9g, %.9g)\n",
                   i, c.k.x, c.k.y, c.k.z, c.k.w, c.pixel.x, c.pixel.y, c.depth,
                   got.x, got.y, got.z, shared.x, shared.y, shared.z, x, y);
    }
  }
  CHECK(failures == 0);
  std::printf(
      "recon volume depth unproject: %u cases, pinhole.glsl's bits and the "
      "formula\n",
      n);
  return 0;
}

}  // namespace

int main() { return vr_test::run_on_gpu(gpu_main); }
