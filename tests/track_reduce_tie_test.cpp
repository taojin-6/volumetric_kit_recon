// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The point-to-plane reduce's model pick (track_reduce_point_to_plane.comp),
// the kernel alone, through the fitter's own ABI (fit_abi.hpp). At a pass's
// first iteration the estimate is the cast's pose, so a coarser level's point
// lands exactly between two model pixels (u = 2 x + 0.5 at level 1,
// 4 x + 1.5 at level 2): a rounding tie that the last bit of the device's
// arithmetic would break. On NVIDIA that bit changed from one dispatch to the
// next, and devices broke the ties apart (2026-11-04). The coarser levels
// pick a quarter pixel past the point; the finest picks the nearest pixel.
//
// The scene: a plane 2 m in front of a generically posed camera, its 128x96
// cast answering only at odd columns and rows, so each pick shows in the
// hits. The host predicts every pick from the rule in double, each at least
// 0.1 px from a rounding edge, and the kernel's attempts, hits and inliers
// must be the host's, its H the host's within 1e-4:
// - Level 1 at the cast's pose, every point on a tie: each picks
//   (2x + 1, 2y + 1) and hits. A tie left to the device picks an even pixel
//   for a share of the points on every device, so this fails without the
//   quarter pixel everywhere. Dispatched 40 times in each of 25 batches, the
//   partials are the same bit for bit. That repeat is short of what NVIDIA
//   took to change its result without the quarter pixel (an RTX 4090 within
//   50k dispatches, a 5090 between 50k and 100k), and a Mac never did, so it
//   passes there either way; VR_TRACK_TIE_SOAK=1 runs 3000 batches of 100.
// - Level 1, the estimate 0.6 px along the image's x: u = 2x + 1.1 still
//   picks 2x + 1 (half a pixel would pick 2x + 2).
// - Level 0, points at odd pixels, the estimate 0.3 px along x: the nearest
//   pixel, x (a quarter pixel there too would pick x + 1).
// Skips without a device.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "fit_abi.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/testing/buffer_readback.hpp"
#include "volumetric_kit/core/testing/gpu_test.hpp"
#include "volumetric_kit/core/testing/test_check.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/command_batch.hpp"
#include "volumetric_kit/core/vulkan/compute_kernel.hpp"
#include "volumetric_kit/core/vulkan/compute_util.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/core/vulkan/vulkan.hpp"
#include "volumetric_kit/grid/volume/voxel_hash_map.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"

#include "track_reduce_point_to_plane_comp.spv.hpp"

namespace vkc = volumetric_kit::core;
namespace vol = volumetric_kit::grid::volume;
namespace detail = volumetric_kit::recon::track::detail;
using volumetric_kit::recon::Mat4f;
using volumetric_kit::recon::Vec3f;
using volumetric_kit::recon::Vec4f;
using Vec3d = glm::dvec3;
using Mat3d = glm::dmat3;

#define TIE_CAT2(a, b) a##b
#define TIE_CAT(a, b) TIE_CAT2(a, b)
#define ASSIGN_OR_FAIL(lhs, expr) \
  ASSIGN_OR_FAIL_IMPL(TIE_CAT(assigned_, __LINE__), lhs, expr)
#define ASSIGN_OR_FAIL_IMPL(tmp, lhs, expr)                      \
  auto tmp = (expr);                                             \
  if (!tmp.ok()) {                                               \
    std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, \
                 tmp.status().message().c_str());                \
    return 1;                                                    \
  }                                                              \
  lhs = std::move(*tmp)
#define OK_OR_FAIL(expr)                                           \
  do {                                                             \
    const vkc::Status ok_or_fail_ = (expr);                        \
    if (!ok_or_fail_.ok()) {                                       \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, \
                   ok_or_fail_.message().c_str());                 \
      return 1;                                                    \
    }                                                              \
  } while (0)

namespace {

constexpr double kPi = 3.14159265358979323846;
// The cast: level 0's camera, the conformance test's.
constexpr std::uint32_t kW = 128;
constexpr std::uint32_t kH = 96;
constexpr float kF = 64.0f;
constexpr float kCx = 0.5f * float(kW - 1);
constexpr float kCy = 0.5f * float(kH - 1);
constexpr float kZ = 2.0f;  // the plane, along the camera's +Z
constexpr float kCoarseGate = 0.05f;
constexpr std::uint32_t kLocal = 64;

// How often the ties' case runs: batches, and dispatches a batch.
struct Repeat {
  std::uint32_t batches = 25;
  std::uint32_t dispatches = 40;
};

Mat4f camera_pose() {  // rotated 25 deg about (0.3, 1, 0.2), then moved
  const Vec3d a = glm::normalize(Vec3d(0.3, 1.0, 0.2));
  const double t = 25.0 * kPi / 180.0;
  const Mat3d k(Vec3d(0, a.z, -a.y), Vec3d(-a.z, 0, a.x), Vec3d(a.y, -a.x, 0));
  const Mat3d r = Mat3d(1.0) + std::sin(t) * k + (1.0 - std::cos(t)) * (k * k);
  Mat4f m(1.0f);
  for (int c = 0; c < 3; ++c) {
    for (int i = 0; i < 3; ++i) m[c][i] = float(r[c][i]);
  }
  m[3] = Vec4f(0.7f, -0.4f, 1.1f, 1.0f);
  return m;
}

Mat3d rot(const Mat4f& m) {
  Mat3d r(1.0);
  for (int c = 0; c < 3; ++c) {
    for (int i = 0; i < 3; ++i) r[c][i] = double(m[c][i]);
  }
  return r;
}

Vec3d xform(const Mat4f& m, const Vec4f& p) {
  return rot(m) * Vec3d(p) + Vec3d(m[3][0], m[3][1], m[3][2]);
}

bool odd(std::uint32_t i) { return (i & 1u) != 0; }

// One reduce: its level's frame points and the estimate.
struct Case {
  const char* name;
  bool finest;
  double shift_px;  // the estimate's move along the image's x, level 0 px
};

struct Expected {
  double attempts = 0, hits = 0, inliers = 0;
  double h[21] = {};
};

// The frame's points at `c`'s level, from its own intrinsics as the
// pyramid's (level 1: (f / 2, (c - 0.5) / 2)), at the plane; at level 0 only
// odd pixels. Unit normals facing the camera.
void frame_points(const Case& c, std::vector<Vec4f>& vtx,
                  std::vector<Vec4f>& nrm, std::uint32_t& w, std::uint32_t& h) {
  const int level = c.finest ? 0 : 1;
  w = kW >> level;
  h = kH >> level;
  Vec4f k(kF, kF, kCx, kCy);
  if (level == 1)
    k = Vec4f(k.x * 0.5f, k.y * 0.5f, (k.z - 0.5f) * 0.5f, (k.w - 0.5f) * 0.5f);
  vtx.assign(std::size_t(w) * h, Vec4f(0.0f));
  nrm.assign(vtx.size(), Vec4f(0.0f, 0.0f, -1.0f, 0.0f));
  for (std::uint32_t y = 0; y < h; ++y) {
    for (std::uint32_t x = 0; x < w; ++x) {
      if (c.finest && !(odd(x) && odd(y))) continue;
      const float px = float(x);
      const float py = float(y);
      vtx[std::size_t(y) * w + x] =
          Vec4f((px - k.z) * kZ / k.x, (py - k.w) * kZ / k.y, kZ, 1.0f);
    }
  }
}

// The kernel's arithmetic in double, with the pick under test: the coarser
// levels a quarter pixel past the point, the finest the nearest pixel.
// False when a point lies under 0.1 px from a rounding edge or a hit is not
// clearly inside the gates: the case would not decide the rule.
bool expect(const Case& c, const std::vector<Vec4f>& vtx, const Mat4f& cam,
            const Mat4f& correction, const detail::FitState& st,
            const detail::ReducePush& rp, Expected& out) {
  const Mat3d r = rot(cam);
  const Vec3d t(cam[3][0], cam[3][1], cam[3][2]);
  const Vec3d n_world = r * Vec3d(0.0, 0.0, -1.0);
  const Mat4f pose = correction * cam;
  const double offset = c.finest ? 0.0 : 0.25;
  for (const Vec4f& lv : vtx) {
    if (lv.w == 0.0f) continue;
    out.attempts += 1.0;
    const Vec3d p = xform(pose, lv);
    const Vec3d pm = glm::transpose(r) * (p - t);
    const double u = double(kF) * pm.x / pm.z + double(kCx);
    const double v = double(kF) * pm.y / pm.z + double(kCy);
    const double au = u + offset + 0.5;
    const double av = v + offset + 0.5;
    const auto edge = [](double a) {
      const double f = a - std::floor(a);
      return std::min(f, 1.0 - f);
    };
    if (!(u > 0.1 && u < kW - 0.1 && v > 0.1 && v < kH - 0.1)) return false;
    if (edge(au) < 0.1 || edge(av) < 0.1) return false;
    // Clamped into the image, as vs_nearest_pixel does.
    const auto mx = std::min(std::uint32_t(std::floor(au)), kW - 1);
    const auto my = std::min(std::uint32_t(std::floor(av)), kH - 1);
    if (!(odd(mx) && odd(my))) continue;
    out.hits += 1.0;
    const Vec3d q = r * Vec3d((double(mx) - kCx) * kZ / kF,
                              (double(my) - kCy) * kZ / kF, kZ) +
                    t;
    const double z = double(lv.z);
    const double sd = 0.002 + z * z * 0.002;  // the view's noise
    const double sigma =
        std::sqrt(sd * sd + double(rp.model_sigma) * double(rp.model_sigma));
    const double gate =
        c.finest ? std::max(double(rp.gate_sigmas) * sigma, double(rp.gate))
                 : double(rp.gate);
    if (glm::length(p - q) > gate - 0.005) return false;
    out.inliers += 1.0;
    const double res = glm::dot(n_world, p - q);
    const double e = std::fabs(res) / sigma;
    const double huber =
        e <= double(rp.huber_sigmas) ? 1.0 : double(rp.huber_sigmas) / e;
    const double wt = huber / (sigma * sigma);
    const Vec3d jw = glm::cross(p - Vec3d(st.origin), n_world);
    const double j[6] = {double(st.scale) * n_world.x,
                         double(st.scale) * n_world.y,
                         double(st.scale) * n_world.z,
                         jw.x,
                         jw.y,
                         jw.z};
    std::size_t k = 0;
    for (int a = 0; a < 6; ++a) {
      for (int b = a; b < 6; ++b) out.h[k++] += wt * j[a] * j[b];
    }
  }
  return true;
}

int run_case(const vkc::test::Gpu& gpu, vkc::ComputeKernel& kernel,
             const Case& c, Repeat rep, std::uint32_t max_groups) {
  const Mat4f cam = camera_pose();
  std::vector<Vec4f> vtx;
  std::vector<Vec4f> nrm;
  std::uint32_t w = 0;
  std::uint32_t h = 0;
  frame_points(c, vtx, nrm, w, h);
  // The cast: depth at odd columns and rows only, the plane's world normal.
  std::vector<float> depth(std::size_t(kW) * kH, 0.0f);
  std::vector<float> normal(3 * depth.size(), 0.0f);
  const Vec3d n_world = rot(cam) * Vec3d(0.0, 0.0, -1.0);
  for (std::uint32_t y = 0; y < kH; ++y) {
    for (std::uint32_t x = 0; x < kW; ++x) {
      const std::size_t i = std::size_t(y) * kW + x;
      if (odd(x) && odd(y)) depth[i] = kZ;
      for (int a = 0; a < 3; ++a) normal[3 * i + a] = float(n_world[a]);
    }
  }
  vol::DepthPinhole cast{kF, kF, kCx, kCy, 0.1f, 10.0f, kW, kH, cam};
  detail::FitView view;
  view.cam_to_world = cam;
  view.k = Vec4f(kF, kF, kCx, kCy);
  view.noise = Vec3f(0.002f, 0.0f, 0.002f);
  view.min_depth = 0.1f;
  view.max_depth = 10.0f;
  // The estimate: the cast's pose moved along the camera's x, so every point
  // moves shift_px in the cast.
  const Vec3d along = rot(cam) * Vec3d(c.shift_px * kZ / kF, 0.0, 0.0);
  Mat4f correction(1.0f);
  correction[3] = Vec4f(float(along.x), float(along.y), float(along.z), 1.0f);
  detail::FitState st;
  st.correction = correction;
  st.prior = correction;
  st.pass_start = correction;
  st.origin = Vec3f(cam[3]);
  st.scale = 2.0f;
  const std::uint32_t gpv = (w * h + kLocal * 8 - 1) / (kLocal * 8);
  const detail::ReducePush rp{w * h,
                              gpv,
                              kW,
                              kH,
                              c.finest ? 1u : 0u,
                              c.finest ? 0.015f : kCoarseGate,
                              3.0f,
                              0.866f,
                              2.0f,
                              0.005f,
                              0.0f};
  Expected want;
  CHECK(expect(c, vtx, cam, correction, st, rp, want));
  CHECK(want.attempts > 0 && want.hits > 0.9 * want.attempts);

  const auto upload = [&](const void* data, std::size_t bytes) {
    return vkc::test::upload_device_buffer(gpu.device, gpu.allocator, data,
                                           VkDeviceSize(bytes));
  };
  ASSIGN_OR_FAIL(vkc::Buffer b_vtx, upload(vtx.data(), vtx.size() * 16));
  ASSIGN_OR_FAIL(vkc::Buffer b_nrm, upload(nrm.data(), nrm.size() * 16));
  ASSIGN_OR_FAIL(vkc::Buffer b_depth, upload(depth.data(), depth.size() * 4));
  ASSIGN_OR_FAIL(vkc::Buffer b_normal,
                 upload(normal.data(), normal.size() * 4));
  ASSIGN_OR_FAIL(vkc::Buffer b_cam, upload(&cast, sizeof(cast)));
  ASSIGN_OR_FAIL(vkc::Buffer b_view, upload(&view, sizeof(view)));
  ASSIGN_OR_FAIL(vkc::Buffer b_state, upload(&st, sizeof(st)));
  const VkDeviceSize part = VkDeviceSize(gpv) * detail::kPartial * 4;
  ASSIGN_OR_FAIL(vkc::Buffer b_part,
                 vkc::device_storage_buffer(gpu.allocator, part));
  ASSIGN_OR_FAIL(vkc::Buffer b_all, vkc::device_storage_buffer(
                                        gpu.allocator, part * rep.dispatches));
  const vkc::Buffer* in[8] = {&b_vtx, &b_nrm,  &b_depth, &b_normal,
                              &b_cam, &b_view, &b_state, &b_part};
  for (std::uint32_t i = 0; i < 8; ++i) {
    kernel.set.write_storage_buffer(i, in[i]->handle(), 0, VK_WHOLE_SIZE);
  }

  // rep.dispatches a batch, each one's partials copied out.
  const std::size_t floats = std::size_t(gpv) * detail::kPartial;
  std::vector<float> all(floats * rep.dispatches);
  std::vector<float> first;
  std::uint32_t differ = 0;
  for (std::uint32_t b = 0; b < rep.batches; ++b) {
    vkc::CommandBatch batch(gpu.device, gpu.allocator);
    for (std::uint32_t d = 0; d < rep.dispatches; ++d) {
      OK_OR_FAIL(batch.dispatch(kernel, &rp, sizeof(rp), gpv, max_groups));
      OK_OR_FAIL(batch.copy(b_part, 0, b_all, part * d, part));
    }
    OK_OR_FAIL(batch.readback(b_all, 0, part * rep.dispatches, all.data()));
    OK_OR_FAIL(batch.submit());
    if (first.empty()) first.assign(all.begin(), all.begin() + floats);
    for (std::uint32_t d = 0; d < rep.dispatches; ++d) {
      differ +=
          std::memcmp(all.data() + floats * d, first.data(), floats * 4) != 0;
    }
  }
  // The sums, over the groups.
  double got[detail::kPartial] = {};
  for (std::uint32_t g = 0; g < gpv; ++g) {
    for (std::uint32_t i = 0; i < detail::kPartial; ++i) {
      got[i] += double(first[g * detail::kPartial + i]);
    }
  }
  double top = 0.0;
  double worst = 0.0;
  for (int i = 0; i < 21; ++i) top = std::max(top, std::fabs(want.h[i]));
  for (int i = 0; i < 21; ++i) {
    worst = std::max(worst, std::fabs(got[i] - want.h[i]) / top);
  }
  std::printf(
      "  %s: attempts %.0f/%.0f hits %.0f/%.0f inliers %.0f/%.0f (kernel / "
      "host), H within %.1e; %u of %u dispatches differ\n",
      c.name, got[detail::kPartialAttempts], want.attempts,
      got[detail::kPartialHits], want.hits, got[detail::kPartialInliers],
      want.inliers, worst, differ, rep.batches * rep.dispatches);
  CHECK(differ == 0);
  CHECK(got[detail::kPartialAttempts] == want.attempts);
  CHECK(got[detail::kPartialHits] == want.hits);
  CHECK(got[detail::kPartialInliers] == want.inliers);
  CHECK(worst < 1e-4);
  return 0;
}

int gpu_main(vkc::test::GpuContext& gpu) {
  vkc::ComputeKernel kernel;
  VkPushConstantRange pr{};
  pr.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
  pr.size = sizeof(detail::ReducePush);
  vkc::KernelSetBuilder kb(gpu.device);
  OK_OR_FAIL(kb.add(kernel, "track_reduce_point_to_plane",
                    vrt_track_reduce_point_to_plane_comp_spv,
                    vrt_track_reduce_point_to_plane_comp_spv_size, 8, &pr));
  ASSIGN_OR_FAIL(const vkc::DescriptorPool pool, kb.build());
  VkPhysicalDeviceProperties props{};
  vkGetPhysicalDeviceProperties(gpu.device.physical_device(), &props);
  const std::uint32_t max_groups = props.limits.maxComputeWorkGroupCount[0];
  std::printf("track_reduce_point_to_plane on %s\n", props.deviceName);
  const char* soak = std::getenv("VR_TRACK_TIE_SOAK");
  Repeat ties;
  if (soak != nullptr && std::string_view(soak) == "1") ties = {3000, 100};
  const Case cases[] = {{"level 1 on the ties", false, 0.0},
                        {"level 1, 0.6 px on", false, 0.6},
                        {"level 0, 0.3 px on", true, 0.3}};
  CHECK(run_case(gpu, kernel, cases[0], ties, max_groups) == 0);
  CHECK(run_case(gpu, kernel, cases[1], {1, 40}, max_groups) == 0);
  CHECK(run_case(gpu, kernel, cases[2], {1, 40}, max_groups) == 0);
  return 0;
}

}  // namespace

int main() {
  return vkc::test::run_on_gpu(volumetric_kit::recon::device_requirements(),
                               gpu_main);
}
