// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// The tracking tests' analytic scenes: planes bounding free space, a model of
// one written straight into a grid's tsdf, weight and source byte (grid's
// grid_readback.hpp writer), poses, and frames of a scene's exact depth at
// poses the test knows, registered with an error the fit should undo.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string_view>
#include <utility>
#include <vector>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/testing/buffer_readback.hpp"
#include "volumetric_kit/core/testing/gpu_test.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/grid/testing/device_blocks.hpp"
#include "volumetric_kit/grid/testing/grid_readback.hpp"
#include "volumetric_kit/grid/volume/voxel_block_grid.hpp"
#include "volumetric_kit/recon/core/math/vector_types.hpp"
#include "volumetric_kit/sensor/frame/device_frame.hpp"

namespace track_scenes {

namespace vkc = volumetric_kit::core;
namespace vol = volumetric_kit::grid::volume;
namespace sensor = volumetric_kit::sensor;
using volumetric_kit::recon::Mat4f;
using volumetric_kit::recon::Vec3f;
using volumetric_kit::recon::Vec4f;
using Vec3i = glm::ivec3;
using Vec3d = glm::dvec3;
using Mat3d = glm::dmat3;

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr std::string_view kSource = "source";
inline constexpr std::uint8_t kRigSource = 1;

// --- Scenes
// -------------------------------------------------------------------

// A scene: planes n . p + c >= 0 bounding free space, its SDF their minimum.
// Each plane may cover only part of its plane (`within`), for a floor with a
// step.
struct Plane {
  Vec3d n;
  double c = 0.0;
};

struct Scene {
  std::vector<Plane> planes;
  Vec3f lo, hi;  // the box the grid covers
  double sdf(const Vec3d& p) const {
    double d = 1e9;
    for (const Plane& pl : planes) d = std::min(d, glm::dot(pl.n, p) + pl.c);
    return d;
  }
};

inline Scene room() {  // [0, 3] x [0, 3] x [0, 2.5]
  return {{{{1, 0, 0}, 0},
           {{-1, 0, 0}, 3},
           {{0, 1, 0}, 0},
           {{0, -1, 0}, 3},
           {{0, 0, 1}, 0},
           {{0, 0, -1}, 2.5}},
          {-0.3f, -0.3f, -0.3f},
          {3.3f, 3.3f, 2.8f}};
}

inline Scene corridor() {  // y in [0, 2], z in [0, 2.5], open along x
  return {{{{0, 1, 0}, 0}, {{0, -1, 0}, 2}, {{0, 0, 1}, 0}, {{0, 0, -1}, 2.5}},
          {-4.0f, -0.3f, -0.3f},
          {8.0f, 2.3f, 2.8f}};
}

inline Scene floor_only() {  // z >= 0
  return {{{{0, 0, 1}, 0}}, {-3.0f, -3.0f, -0.3f}, {3.0f, 3.0f, 0.3f}};
}

inline vol::VoxelGridParams grid_params(float voxel, std::int32_t buckets) {
  vol::VoxelGridParams g{};
  g.voxel_size = voxel;
  g.block_size = 8;
  g.voxels_per_block = 512;
  g.trunc_dist = 3.0f * voxel;
  g.bucket_size = 8;
  g.num_buckets = buckets;
  g.num_blocks = 8 * buckets;
  g.max_chain = 64;
  return g;
}

// A model of `scene`: its band's blocks, each voxel the clamped SDF, weight 5
// to the band's back edge, source `source`. An empty scene leaves it empty.
inline vkc::Result<vol::VoxelBlockGrid> make_model(
    const vkc::test::Gpu& gpu, const Scene& scene, float voxel,
    std::int32_t buckets, std::uint8_t source = kRigSource) {
  const vol::VoxelGridParams g = grid_params(voxel, buckets);
  const vol::AttributeSpec attrs[] = {
      {vol::kTsdfAttribute, 4}, {vol::kWeightAttribute, 4}, {kSource, 1}};
  VKC_ASSIGN(
      vol::VoxelBlockGrid grid,
      vol::VoxelBlockGrid::create(gpu.device, gpu.allocator, g, attrs, 3));
  if (scene.planes.empty()) return grid;
  const float extent = voxel * float(g.block_size);
  const Vec3i b0(glm::floor(scene.lo / extent));
  const Vec3i b1(glm::floor(scene.hi / extent));
  const double reach = double(g.trunc_dist) + 0.5 * std::sqrt(3.0) * extent;
  std::vector<vol::BlockIndex> add;
  for (int z = b0.z; z <= b1.z; ++z) {
    for (int y = b0.y; y <= b1.y; ++y) {
      for (int x = b0.x; x <= b1.x; ++x) {
        const Vec3d centre =
            (Vec3d(x, y, z) * double(g.block_size) + 3.5) * double(voxel);
        if (std::fabs(scene.sdf(centre)) <= reach) {
          vol::BlockIndex b{};
          b.coord = Vec3i(x, y, z);
          add.push_back(b);
        }
      }
    }
  }
  VKC_ASSIGN(const std::uint32_t failed,
             vgrid_test::allocate_blocks(gpu, grid.map(), add));
  if (failed != 0)
    return vkc::Status::invalid_argument("model: blocks refused");
  VKC_ASSIGN(const std::vector<vol::BlockIndex> active,
             grid.map().compact_active_blocks());
  VKC_ASSIGN(const vol::AttributeView view,
             grid.attribute(vol::kTsdfAttribute));
  const auto n = std::size_t(view.element_count);
  std::vector<float> tsdf(n, 0.0f);
  std::vector<float> weight(n, 0.0f);
  std::vector<std::uint8_t> src(n, 0);
  const int bs = g.block_size;
  const double t = g.trunc_dist;
  for (const vol::BlockIndex& b : active) {
    for (int k = 0; k < g.voxels_per_block; ++k) {
      const Vec3i v =
          b.coord * bs + Vec3i(k % bs, (k / bs) % bs, k / (bs * bs));
      const double d = scene.sdf(Vec3d(v) * double(voxel));
      const std::size_t i = std::size_t(b.ptr) + std::size_t(k);
      tsdf[i] = float(std::clamp(d, -t, t));
      weight[i] = d >= -t ? 5.0f : 0.0f;
      src[i] = source;
    }
  }
  VKC_TRY(vgrid_test::write_attribute(gpu.device, gpu.allocator, grid,
                                      vol::kTsdfAttribute, tsdf));
  VKC_TRY(vgrid_test::write_attribute(gpu.device, gpu.allocator, grid,
                                      vol::kWeightAttribute, weight));
  VKC_TRY(vgrid_test::write_attribute(gpu.device, gpu.allocator, grid, kSource,
                                      src));
  return grid;
}

// --- Poses ------------------------------------------------------------------

inline Mat4f look_at(Vec3f eye, Vec3f target) {
  const Vec3f z = glm::normalize(target - eye);
  Vec3f x = glm::cross(z, Vec3f(0, 0, 1));
  if (glm::length(x) < 1e-3f) x = Vec3f(1, 0, 0);
  x = glm::normalize(x);
  const Vec3f y = glm::cross(z, x);
  return Mat4f(Vec4f(x, 0), Vec4f(y, 0), Vec4f(z, 0), Vec4f(eye, 1));
}

inline Mat3d rot_axis(Vec3d axis, double deg) {
  axis = glm::normalize(axis);
  const double a = deg * kPi / 180.0;
  const Mat3d k(Vec3d(0, axis.z, -axis.y), Vec3d(-axis.z, 0, axis.x),
                Vec3d(axis.y, -axis.x, 0));
  return Mat3d(1.0) + std::sin(a) * k + (1.0 - std::cos(a)) * (k * k);
}

inline Mat4f rigid(const Mat3d& r, const Vec3d& t) {
  Mat4f m(1.0f);
  for (int c = 0; c < 3; ++c) {
    for (int k = 0; k < 3; ++k) m[c][k] = float(r[c][k]);
  }
  m[3] = Vec4f(float(t.x), float(t.y), float(t.z), 1.0f);
  return m;
}

// A world error: rotation `deg` about `axis` through `o`, then `shift`.
inline Mat4f error_at(Vec3d o, Vec3d shift, Vec3d axis, double deg) {
  const Mat3d r = rot_axis(axis, deg);
  return rigid(r, o - r * o + shift);
}

inline Mat3d rot_of(const Mat4f& m) {
  Mat3d r(1.0);
  for (int c = 0; c < 3; ++c) {
    for (int k = 0; k < 3; ++k) r[c][k] = double(m[c][k]);
  }
  return r;
}

inline Mat4f inverse(const Mat4f& m) {
  const Mat3d rt = glm::transpose(rot_of(m));
  return rigid(rt, -(rt * Vec3d(m[3][0], m[3][1], m[3][2])));
}

// How far a from b is at o: metres and degrees.
inline void gap(const Mat4f& a, const Mat4f& b, Vec3f o, double& m,
                double& deg) {
  const Mat4f d = a * inverse(b);
  const Vec3d od(o);
  const Vec3d moved = rot_of(d) * od + Vec3d(d[3][0], d[3][1], d[3][2]);
  m = glm::length(moved - od);
  // atan2 of the skew part over the trace's: acos of the trace alone reads a
  // float rotation's rounding as 0.02 deg.
  const Mat3d r = rot_of(d);
  const Vec3d v(r[1][2] - r[2][1], r[2][0] - r[0][2], r[0][1] - r[1][0]);
  deg = std::atan2(0.5 * glm::length(v),
                   (r[0][0] + r[1][1] + r[2][2] - 1.0) * 0.5) *
        180.0 / kPi;
}

// --- Frames -----------------------------------------------------------------

struct Camera {
  // At half the phone's 256x192: coarser, a floor seen obliquely steps past
  // the pyramid's 3 cm edge test from pixel to pixel and drops out.
  std::uint32_t w = 128, h = 96;
  float f = 64.0f;
};

// The scene's exact depth from a camera at `truth`, 0 where it misses;
// `edit(u / width, depth)`, given, replaces each pixel's depth.
inline std::vector<float> render(
    const Scene& scene, const Camera& cam, const Mat4f& truth,
    const std::function<double(double, double)>& edit = nullptr) {
  std::vector<float> depth(std::size_t(cam.w) * cam.h, 0.0f);
  const Mat3d r = rot_of(truth);
  const Vec3d o(truth[3][0], truth[3][1], truth[3][2]);
  const double cx = 0.5 * double(cam.w - 1);
  const double cy = 0.5 * double(cam.h - 1);
  for (std::uint32_t v = 0; v < cam.h; ++v) {
    for (std::uint32_t u = 0; u < cam.w; ++u) {
      const Vec3d dir = r * Vec3d((u - cx) / cam.f, (v - cy) / cam.f, 1.0);
      double best = 1e9;
      for (const Plane& pl : scene.planes) {
        const double nd = glm::dot(pl.n, dir);
        if (nd >= 0.0) continue;
        const double t = -(glm::dot(pl.n, o) + pl.c) / nd;
        if (t > 0.0) best = std::min(best, t);
      }
      if (edit) best = edit(double(u) / cam.w, best);
      if (best >= 0.2 && best <= 5.0) {
        depth[std::size_t(v) * cam.w + u] = float(best);
      }
    }
  }
  return depth;
}

inline vkc::Result<sensor::DeviceFrame> make_frame(
    const vkc::test::Gpu& gpu, const std::vector<float>& depth,
    const Camera& cam, const Mat4f& registered, std::uint64_t pts) {
  VKC_ASSIGN(vkc::Buffer buffer,
             vkc::test::upload_device_buffer(gpu.device, gpu.allocator,
                                             depth.data(), depth.size() * 4));
  sensor::DeviceFrame f;
  f.depth = std::make_shared<const vkc::Buffer>(std::move(buffer));
  f.depth_camera = {cam.f,
                    cam.f,
                    0.5f * float(cam.w - 1),
                    0.5f * float(cam.h - 1),
                    0.2f,
                    5.0f,
                    cam.w,
                    cam.h,
                    registered};
  f.color_camera = {
      cam.f, cam.f,     0.5f * float(cam.w - 1), 0.5f * float(cam.h - 1), cam.w,
      cam.h, registered};
  f.timestamp_ns = pts;
  f.depth_noise = {0.002f, 0.0f, 0.002f};
  return f;
}

// Frames of `scene` at `truths`, registered at E * truth.
inline vkc::Result<std::vector<sensor::DeviceFrame>> frames_of(
    const vkc::test::Gpu& gpu, const Scene& scene, const Camera& cam,
    const std::vector<Mat4f>& truths, const Mat4f& error,
    std::uint64_t first_pts = 1000) {
  std::vector<sensor::DeviceFrame> out;
  for (std::size_t i = 0; i < truths.size(); ++i) {
    VKC_ASSIGN(sensor::DeviceFrame f,
               make_frame(gpu, render(scene, cam, truths[i]), cam,
                          error * truths[i], first_pts + i * 100'000'000));
    out.push_back(std::move(f));
  }
  return out;
}

}  // namespace track_scenes
