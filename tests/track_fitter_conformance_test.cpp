// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// Every fit kind (kFitKinds) against analytic scenes whose tsdf, weight and
// source byte the test writes (grid_readback.hpp's writer), at 5 cm voxels:
// a box room seen into a corner (all six directions constrained), a corridor
// (free along its axis) and a floor (free in x, y and yaw). Frames are the
// scenes' exact depth, 128x96, at poses the test knows; a frame's registered
// pose is the true one moved by an error E, so the fit's answer is E^-1.
//
// - The contract: each refusal (a record outstanding, a stale pending fit, a
//   finish before its batch ran, 0 or 33 frames, mixed sizes, timestamps not
//   ascending, a prior not rigid or with a negative sigma) leaves the fitter
//   as it was; a record after an abandon works; record, abandon and record
//   the same frames gives a fresh fitter's fit bit for bit (the entries roll
//   back); the same timestamps with new depth buffers are rebuilt.
// - The fit: at the truth the correction stays within 0.1 mm / 0.01 deg;
//   from 2 cm / 1 deg it converges; the floor holds x, y and yaw; an empty
//   model solves nothing; the eigensystem's invariants.
// - The model filter: geometry whose source byte the filter excludes is not
//   there.
// - Windows: K copies of a frame give its fit with K times its counts; a
//   turning 20-frame window holds the corridor's axis; 20 frames with 5 new
//   and 3 passes pass validation and the submit check.
// - Zero iterations return the prior exactly, with the first evaluation's
//   counts. Passes: a 5 cm start converges in 3; a pass under 1 cm / 0.3 deg
//   stops the solves (3 passes at the truth give 1 pass's fit).
// - Determinism: a record twice is bit-identical; a second fitter in the
//   same batch changes nothing.
// - The see-through counters count hits in front of and behind the surface.
//
// VR_TRACK_TIMING=1 adds U3's timing (256x192 in a 1 cm room, K = 1 and 20,
// passes 1 and 3, the configurations alternated), and VR_TRACK_SWEEP=1 a
// basin sweep; both print and assert nothing. Skips without a device.

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <memory>
#include <random>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/base/stage_metrics.hpp"
#include "volumetric_kit/core/testing/buffer_readback.hpp"
#include "volumetric_kit/core/testing/gpu_test.hpp"
#include "volumetric_kit/core/testing/test_check.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/command_batch.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/grid/testing/device_blocks.hpp"
#include "volumetric_kit/grid/testing/grid_readback.hpp"
#include "volumetric_kit/grid/volume/ray_caster.hpp"
#include "volumetric_kit/grid/volume/voxel_block_grid.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"
#include "volumetric_kit/recon/track/fitter.hpp"

namespace vkc = volumetric_kit::core;
namespace vol = volumetric_kit::grid::volume;
namespace sensor = volumetric_kit::sensor;
namespace track = volumetric_kit::recon::track;
using volumetric_kit::recon::Mat4f;
using volumetric_kit::recon::Vec3f;
using volumetric_kit::recon::Vec4f;
using Vec3i = glm::ivec3;
using Vec3d = glm::dvec3;
using Mat3d = glm::dmat3;

// `lhs = *expr` for a test returning an exit code: on an error, print it and
// return 1.
#define TRACK_TEST_CAT2(a, b) a##b
#define TRACK_TEST_CAT(a, b) TRACK_TEST_CAT2(a, b)
#define ASSIGN_OR_FAIL(lhs, expr) \
  ASSIGN_OR_FAIL_IMPL(TRACK_TEST_CAT(assigned_, __LINE__), lhs, expr)
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
constexpr std::string_view kSource = "source";
constexpr std::uint8_t kRigSource = 1;

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

Scene room() {  // [0, 3] x [0, 3] x [0, 2.5]
  return {{{{1, 0, 0}, 0},
           {{-1, 0, 0}, 3},
           {{0, 1, 0}, 0},
           {{0, -1, 0}, 3},
           {{0, 0, 1}, 0},
           {{0, 0, -1}, 2.5}},
          {-0.3f, -0.3f, -0.3f},
          {3.3f, 3.3f, 2.8f}};
}

Scene corridor() {  // y in [0, 2], z in [0, 2.5], open along x
  return {{{{0, 1, 0}, 0}, {{0, -1, 0}, 2}, {{0, 0, 1}, 0}, {{0, 0, -1}, 2.5}},
          {-4.0f, -0.3f, -0.3f},
          {8.0f, 2.3f, 2.8f}};
}

Scene floor_only() {  // z >= 0
  return {{{{0, 0, 1}, 0}}, {-3.0f, -3.0f, -0.3f}, {3.0f, 3.0f, 0.3f}};
}

vol::VoxelGridParams grid_params(float voxel, std::int32_t buckets) {
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
vkc::Result<vol::VoxelBlockGrid> make_model(const vkc::test::Gpu& gpu,
                                            const Scene& scene, float voxel,
                                            std::int32_t buckets,
                                            std::uint8_t source = kRigSource) {
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

Mat4f look_at(Vec3f eye, Vec3f target) {
  const Vec3f z = glm::normalize(target - eye);
  Vec3f x = glm::cross(z, Vec3f(0, 0, 1));
  if (glm::length(x) < 1e-3f) x = Vec3f(1, 0, 0);
  x = glm::normalize(x);
  const Vec3f y = glm::cross(z, x);
  return Mat4f(Vec4f(x, 0), Vec4f(y, 0), Vec4f(z, 0), Vec4f(eye, 1));
}

Mat3d rot_axis(Vec3d axis, double deg) {
  axis = glm::normalize(axis);
  const double a = deg * kPi / 180.0;
  const Mat3d k(Vec3d(0, axis.z, -axis.y), Vec3d(-axis.z, 0, axis.x),
                Vec3d(axis.y, -axis.x, 0));
  return Mat3d(1.0) + std::sin(a) * k + (1.0 - std::cos(a)) * (k * k);
}

Mat4f rigid(const Mat3d& r, const Vec3d& t) {
  Mat4f m(1.0f);
  for (int c = 0; c < 3; ++c) {
    for (int k = 0; k < 3; ++k) m[c][k] = float(r[c][k]);
  }
  m[3] = Vec4f(float(t.x), float(t.y), float(t.z), 1.0f);
  return m;
}

// A world error: rotation `deg` about `axis` through `o`, then `shift`.
Mat4f error_at(Vec3d o, Vec3d shift, Vec3d axis, double deg) {
  const Mat3d r = rot_axis(axis, deg);
  return rigid(r, o - r * o + shift);
}

Mat3d rot_of(const Mat4f& m) {
  Mat3d r(1.0);
  for (int c = 0; c < 3; ++c) {
    for (int k = 0; k < 3; ++k) r[c][k] = double(m[c][k]);
  }
  return r;
}

Mat4f inverse(const Mat4f& m) {
  const Mat3d rt = glm::transpose(rot_of(m));
  return rigid(rt, -(rt * Vec3d(m[3][0], m[3][1], m[3][2])));
}

// How far a from b is at o: metres and degrees.
void gap(const Mat4f& a, const Mat4f& b, Vec3f o, double& m, double& deg) {
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
std::vector<float> render(
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

vkc::Result<sensor::DeviceFrame> make_frame(const vkc::test::Gpu& gpu,
                                            const std::vector<float>& depth,
                                            const Camera& cam,
                                            const Mat4f& registered,
                                            std::uint64_t pts) {
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
vkc::Result<std::vector<sensor::DeviceFrame>> frames_of(
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

// --- Fitting ----------------------------------------------------------------

struct Fit {
  track::TrackFit fit;
  vkc::StageMetrics rows;
};

// One fit, submitted and finished.
vkc::Result<Fit> fit_once(const vkc::test::Gpu& gpu, track::IFitter& fitter,
                          const vol::VoxelBlockGrid& model,
                          const std::vector<sensor::DeviceFrame>& frames,
                          const track::PosePrior& prior = {}) {
  Fit out;
  vkc::CommandBatch batch(gpu.device, gpu.allocator);
  VKC_ASSIGN(const track::PendingFit p,
             fitter.record(batch, model, frames, prior, &out.rows));
  VKC_ASSIGN(vkc::PendingBatch done, batch.submit_async({}, {}));
  VKC_TRY(done.wait());
  out.rows.merge_device(done.device_rows());
  VKC_ASSIGN(out.fit, fitter.finish(p));
  return out;
}

bool same_fit(const track::TrackFit& a, const track::TrackFit& b) {
  return std::memcmp(&a.correction, &b.correction, sizeof(Mat4f)) == 0 &&
         std::memcmp(&a.lambda_max, &b.lambda_max, sizeof(float)) == 0 &&
         a.relative_eigenvalues == b.relative_eigenvalues &&
         a.directions == b.directions && a.held == b.held &&
         a.frames == b.frames && a.attempts == b.attempts && a.hits == b.hits &&
         a.inliers == b.inliers && a.in_front == b.in_front &&
         a.behind == b.behind &&
         std::memcmp(&a.rms_m, &b.rms_m, sizeof(float)) == 0 &&
         std::memcmp(&a.scale_m, &b.scale_m, sizeof(float)) == 0;
}

track::FitterConfig config_for(track::FitKind kind) {
  track::FitterConfig c;
  c.kind = kind;
  c.model_filter.min_weight = 2.0f;
  return c;
}

struct Models {
  vol::VoxelBlockGrid room;
  vol::VoxelBlockGrid corridor;
  vol::VoxelBlockGrid floor;
  vol::VoxelBlockGrid empty;
};

const Vec3f kRoomEye(1.2f, 1.0f, 1.3f);
const Vec3f kRoomTarget(3.0f, 3.0f, 0.4f);

// --- The cases
// -----------------------------------------------------------------

int test_contract(const vkc::test::Gpu& gpu, track::FitKind kind,
                  const Models& m) {
  const Camera cam;
  const Mat4f truth = look_at(kRoomEye, kRoomTarget);
  const Mat4f error =
      error_at(Vec3d(kRoomEye), {0.01, -0.005, 0.004}, {0.3, 1, 0.2}, 0.5);
  ASSIGN_OR_FAIL(const auto frames,
                 frames_of(gpu, room(), cam, {truth, truth, truth}, error));
  ASSIGN_OR_FAIL(auto fresh, track::create_fitter(gpu.device, gpu.allocator,
                                                  config_for(kind)));
  ASSIGN_OR_FAIL(const Fit reference, fit_once(gpu, *fresh, m.room, frames));
  CHECK(reference.fit.solved());

  ASSIGN_OR_FAIL(auto fitter, track::create_fitter(gpu.device, gpu.allocator,
                                                   config_for(kind)));
  {
    vkc::CommandBatch batch(gpu.device, gpu.allocator);
    // Refusals before anything is outstanding.
    CHECK(!fitter->record(batch, m.room, {}, {}).ok());
    std::vector<sensor::DeviceFrame> many(33, frames[0]);
    for (std::size_t i = 0; i < many.size(); ++i) many[i].timestamp_ns = 10 + i;
    CHECK(!fitter->record(batch, m.room, many, {}).ok());
    std::vector<sensor::DeviceFrame> mixed = frames;
    mixed[1].depth_camera.width = 32;
    CHECK(!fitter->record(batch, m.room, mixed, {}).ok());
    std::vector<sensor::DeviceFrame> unordered = frames;
    unordered[2].timestamp_ns = unordered[1].timestamp_ns;
    CHECK(!fitter->record(batch, m.room, unordered, {}).ok());
    track::PosePrior bent;
    bent.correction[0][0] = 1.5f;
    CHECK(!fitter->record(batch, m.room, frames, bent).ok());
    track::PosePrior negative;
    negative.sigma_m = -0.01f;
    CHECK(!fitter->record(batch, m.room, frames, negative).ok());
    // A record; a second while it is outstanding; a stale pending; a finish
    // before the batch ran.
    ASSIGN_OR_FAIL(const track::PendingFit p,
                   fitter->record(batch, m.room, frames, {}));
    CHECK(!fitter->record(batch, m.room, frames, {}).ok());
    CHECK(!fitter->finish(track::PendingFit{p.serial + 1}).ok());
    CHECK(!fitter->finish(p).ok());  // never ran
    fitter->abandon(p);
  }
  // Abandoned, the frames' entries are empty again: recorded once more, the
  // fit is a fresh fitter's, bit for bit.
  ASSIGN_OR_FAIL(const Fit again, fit_once(gpu, *fitter, m.room, frames));
  CHECK(same_fit(again.fit, reference.fit));
  // Committed now: the same frames reuse their entries and give the same fit.
  ASSIGN_OR_FAIL(const Fit reused, fit_once(gpu, *fitter, m.room, frames));
  CHECK(same_fit(reused.fit, reference.fit));
  // The same timestamps over new depth (another view) are rebuilt.
  const Mat4f moved = look_at(kRoomEye + Vec3f(0.1f, 0, 0), kRoomTarget);
  ASSIGN_OR_FAIL(auto other,
                 frames_of(gpu, room(), cam, {moved, moved, moved}, error));
  ASSIGN_OR_FAIL(const Fit rebuilt, fit_once(gpu, *fitter, m.room, other));
  ASSIGN_OR_FAIL(auto fresh2, track::create_fitter(gpu.device, gpu.allocator,
                                                   config_for(kind)));
  ASSIGN_OR_FAIL(const Fit rebuilt_ref, fit_once(gpu, *fresh2, m.room, other));
  CHECK(same_fit(rebuilt.fit, rebuilt_ref.fit));
  CHECK(!same_fit(rebuilt.fit, reference.fit));
  // Configs out of range.
  track::FitterConfig bad = config_for(kind);
  bad.icp.passes = 0;
  CHECK(!track::create_fitter(gpu.device, gpu.allocator, bad).ok());
  bad = config_for(kind);
  bad.icp.min_relative_eigenvalue = 0.0f;
  CHECK(!track::create_fitter(gpu.device, gpu.allocator, bad).ok());
  bad = config_for(kind);
  bad.model_filter.min_weight = 0.0f;
  CHECK(!track::create_fitter(gpu.device, gpu.allocator, bad).ok());
  bad = config_for(kind);
  bad.kind = track::FitKind(200);
  CHECK(!track::create_fitter(gpu.device, gpu.allocator, bad).ok());
  return 0;
}

int check_eigensystem(const track::TrackFit& f, float tau) {
  CHECK(f.solved());
  for (std::size_t i = 0; i < 6; ++i) {
    CHECK(f.relative_eigenvalues[i] >= 0.0f &&
          f.relative_eigenvalues[i] <= 1.0f);
    if (i > 0)
      CHECK(f.relative_eigenvalues[i] >= f.relative_eigenvalues[i - 1]);
    CHECK(((f.held >> i) & 1u) == (f.relative_eigenvalues[i] < tau ? 1u : 0u));
    for (std::size_t j = 0; j < 6; ++j) {
      double dot = 0.0;
      for (std::size_t k = 0; k < 6; ++k) {
        dot +=
            double(f.directions[6 * i + k]) * double(f.directions[6 * j + k]);
      }
      CHECK(std::fabs(dot - (i == j ? 1.0 : 0.0)) < 1e-4);
    }
  }
  CHECK(f.relative_eigenvalues[5] == 1.0f);
  return 0;
}

// How much of unit twist `e` lies in the held directions' span.
double held_share(const track::TrackFit& f, std::size_t e) {
  double s = 0.0;
  for (std::size_t i = 0; i < 6; ++i) {
    if (((f.held >> i) & 1u) == 0) continue;
    const double d = f.directions[6 * i + e];
    s += d * d;
  }
  return s;
}

int test_fit(const vkc::test::Gpu& gpu, track::FitKind kind, const Models& m) {
  const Camera cam;
  const Mat4f truth = look_at(kRoomEye, kRoomTarget);
  ASSIGN_OR_FAIL(auto fitter, track::create_fitter(gpu.device, gpu.allocator,
                                                   config_for(kind)));
  const float tau = track::IcpConfig{}.min_relative_eigenvalue;
  // At the truth: the prior comes back.
  ASSIGN_OR_FAIL(const auto exact,
                 frames_of(gpu, room(), cam, {truth}, Mat4f(1.0f)));
  ASSIGN_OR_FAIL(const Fit at_truth, fit_once(gpu, *fitter, m.room, exact));
  double em = 0.0;
  double edeg = 0.0;
  gap(at_truth.fit.correction, Mat4f(1.0f), kRoomEye, em, edeg);
  std::printf("  %s at the truth: %.4f mm %.5f deg, held 0x%x, %u/%u/%u\n",
              track::to_string(kind), em * 1e3, edeg, at_truth.fit.held,
              at_truth.fit.inliers, at_truth.fit.hits, at_truth.fit.attempts);
  CHECK(em < 1e-4 && edeg < 0.01);
  CHECK(check_eigensystem(at_truth.fit, tau) == 0);
  CHECK(at_truth.fit.held == 0);  // the corner constrains all six
  CHECK(at_truth.fit.hits > 0.9 * at_truth.fit.attempts);
  CHECK(at_truth.fit.inliers > 0.9 * at_truth.fit.hits);
  CHECK(at_truth.fit.frames == 1 && at_truth.fit.passes == 1);
  CHECK(std::fabs(at_truth.fit.scale_m - 2.0f) < 1.0f);
  // From 2 cm / 1 deg at the camera: converged.
  const Mat4f error =
      error_at(Vec3d(kRoomEye), {0.012, -0.012, 0.01}, {0.2, 0.4, 1.0}, 1.0);
  ASSIGN_OR_FAIL(const auto off, frames_of(gpu, room(), cam, {truth}, error));
  ASSIGN_OR_FAIL(const Fit from_off, fit_once(gpu, *fitter, m.room, off));
  gap(from_off.fit.correction * error, Mat4f(1.0f), kRoomEye, em, edeg);
  std::printf("  %s from 2 cm / 1 deg: %.3f mm %.4f deg left\n",
              track::to_string(kind), em * 1e3, edeg);
  CHECK(em < 2e-3 && edeg < 0.05);
  CHECK(check_eigensystem(from_off.fit, tau) == 0);
  // The floor, seen from above: x, y and yaw are held, and keep the prior.
  const Vec3f eye(0.2f, -0.1f, 1.5f);
  const Mat4f down(Vec4f(1, 0, 0, 0), Vec4f(0, -1, 0, 0), Vec4f(0, 0, -1, 0),
                   Vec4f(eye, 1));
  const Mat4f floor_error =
      error_at(Vec3d(eye), {0.01, 0.01, 0.01}, {0.1, 0.1, 1.0}, 0.5);
  ASSIGN_OR_FAIL(const auto floor_frames,
                 frames_of(gpu, floor_only(), cam, {down}, floor_error));
  ASSIGN_OR_FAIL(const Fit floor,
                 fit_once(gpu, *fitter, m.floor, floor_frames));
  std::printf(
      "  %s floor: held 0x%x rel %.1e %.1e %.1e %.1e\n", track::to_string(kind),
      floor.fit.held, floor.fit.relative_eigenvalues[0],
      floor.fit.relative_eigenvalues[1], floor.fit.relative_eigenvalues[2],
      floor.fit.relative_eigenvalues[3]);
  CHECK(floor.fit.solved());
  CHECK(held_share(floor.fit, 0) > 0.99);  // v_x
  CHECK(held_share(floor.fit, 1) > 0.99);  // v_y
  CHECK(held_share(floor.fit, 5) > 0.99);  // w_z
  // The free directions kept the prior: the camera's x and y did not move.
  const Mat4f fixed = floor.fit.correction * floor_error * down;
  const Mat4f start = floor_error * down;
  CHECK(std::fabs(fixed[3][0] - start[3][0]) < 1e-4f);
  CHECK(std::fabs(fixed[3][1] - start[3][1]) < 1e-4f);
  // ... and z did: the floor's height is 1.5 m again.
  CHECK(std::fabs(fixed[3][2] - 1.5f) < 2e-3f);
  // An empty model solves nothing.
  ASSIGN_OR_FAIL(const Fit nothing, fit_once(gpu, *fitter, m.empty, off));
  CHECK(!nothing.fit.solved() && nothing.fit.hits == 0 &&
        nothing.fit.held == 0x3f);
  CHECK(nothing.fit.attempts > 0);
  CHECK(std::memcmp(&nothing.fit.correction, &Mat4f(1.0f)[0][0],
                    sizeof(Mat4f)) == 0 ||
        nothing.fit.correction == Mat4f(1.0f));
  return 0;
}

int test_filter(const vkc::test::Gpu& gpu, track::FitKind kind,
                const Models& m) {
  const Camera cam;
  const Mat4f truth = look_at(kRoomEye, kRoomTarget);
  ASSIGN_OR_FAIL(const auto frames,
                 frames_of(gpu, room(), cam, {truth}, Mat4f(1.0f)));
  track::FitterConfig excluded = config_for(kind);
  excluded.model_filter.attribute = kSource;
  excluded.model_filter.allow[0] &= ~(1u << kRigSource);
  ASSIGN_OR_FAIL(auto none,
                 track::create_fitter(gpu.device, gpu.allocator, excluded));
  ASSIGN_OR_FAIL(const Fit a, fit_once(gpu, *none, m.room, frames));
  CHECK(!a.fit.solved() && a.fit.hits == 0);
  track::FitterConfig allowed = config_for(kind);
  allowed.model_filter.attribute = kSource;
  allowed.model_filter.allow = {};
  allowed.model_filter.allow[0] = 1u << kRigSource;
  ASSIGN_OR_FAIL(auto rig,
                 track::create_fitter(gpu.device, gpu.allocator, allowed));
  ASSIGN_OR_FAIL(const Fit b, fit_once(gpu, *rig, m.room, frames));
  CHECK(b.fit.solved());
  // A weight floor above the model's weight hides it too.
  track::FitterConfig heavy = config_for(kind);
  heavy.model_filter.min_weight = 6.0f;
  ASSIGN_OR_FAIL(auto heavy_fitter,
                 track::create_fitter(gpu.device, gpu.allocator, heavy));
  ASSIGN_OR_FAIL(const Fit c, fit_once(gpu, *heavy_fitter, m.room, frames));
  CHECK(!c.fit.solved());
  // A filter naming an attribute the model lacks is refused.
  track::FitterConfig missing = config_for(kind);
  missing.model_filter.attribute = "no_such";
  ASSIGN_OR_FAIL(auto missing_fitter,
                 track::create_fitter(gpu.device, gpu.allocator, missing));
  vkc::CommandBatch batch(gpu.device, gpu.allocator);
  CHECK(!missing_fitter->record(batch, m.room, frames, {}).ok());
  return 0;
}

int test_windows(const vkc::test::Gpu& gpu, track::FitKind kind,
                 const Models& m) {
  const Camera cam;
  const Mat4f truth = look_at(kRoomEye, kRoomTarget);
  const Mat4f error =
      error_at(Vec3d(kRoomEye), {0.008, 0.006, -0.004}, {1, 0.2, 0.3}, 0.6);
  ASSIGN_OR_FAIL(
      auto single_fitter,
      track::create_fitter(gpu.device, gpu.allocator, config_for(kind)));
  ASSIGN_OR_FAIL(const auto one, frames_of(gpu, room(), cam, {truth}, error));
  ASSIGN_OR_FAIL(const Fit single, fit_once(gpu, *single_fitter, m.room, one));
  // Four copies of the one frame (one depth buffer, four timestamps).
  std::vector<sensor::DeviceFrame> copies(4, one[0]);
  for (std::size_t i = 0; i < copies.size(); ++i) copies[i].timestamp_ns += i;
  ASSIGN_OR_FAIL(
      auto window_fitter,
      track::create_fitter(gpu.device, gpu.allocator, config_for(kind)));
  ASSIGN_OR_FAIL(const Fit four, fit_once(gpu, *window_fitter, m.room, copies));
  double em = 0.0;
  double edeg = 0.0;
  gap(four.fit.correction, single.fit.correction, kRoomEye, em, edeg);
  std::printf(
      "  %s 4 copies against 1: %.5f mm %.6f deg, counts %u/%u/%u "
      "against %u/%u/%u\n",
      track::to_string(kind), em * 1e3, edeg, four.fit.inliers, four.fit.hits,
      four.fit.attempts, single.fit.inliers, single.fit.hits,
      single.fit.attempts);
  CHECK(em < 2e-5 && edeg < 1e-3);
  CHECK(four.fit.frames == 4 && four.fit.attempts == 4 * single.fit.attempts);
  const auto close = [](std::uint32_t a, std::uint32_t b) {
    return std::fabs(double(a) - double(b)) <= 0.002 * double(b) + 4.0;
  };
  CHECK(close(four.fit.hits, 4 * single.fit.hits));
  CHECK(close(four.fit.inliers, 4 * single.fit.inliers));
  // A turning 20-frame window in the corridor holds its axis.
  std::vector<Mat4f> turning;
  for (int i = 0; i < 20; ++i) {
    const float yaw = (-20.0f + 2.0f * float(i)) * float(kPi / 180.0);
    const Vec3f eye(2.0f + 0.01f * float(i), 0.6f, 1.2f);
    // Tilted 30 deg down, so the floor fills a third of the view.
    turning.push_back(
        look_at(eye, eye + Vec3f(std::sin(yaw), std::cos(yaw), -0.58f)));
  }
  const Vec3d centre(2.1, 0.6, 1.2);
  const Mat4f corridor_error =
      error_at(centre, {0.02, 0.008, -0.006}, {0.3, 0.5, 1.0}, 0.6);
  ASSIGN_OR_FAIL(const auto corridor_frames,
                 frames_of(gpu, corridor(), cam, turning, corridor_error));
  ASSIGN_OR_FAIL(
      auto corridor_fitter,
      track::create_fitter(gpu.device, gpu.allocator, config_for(kind)));
  ASSIGN_OR_FAIL(const Fit along,
                 fit_once(gpu, *corridor_fitter, m.corridor, corridor_frames));
  std::printf("  %s corridor: held 0x%x rel0 %.1e\n", track::to_string(kind),
              along.fit.held, along.fit.relative_eigenvalues[0]);
  CHECK(along.fit.solved() && along.fit.frames == 20);
  CHECK(held_share(along.fit, 0) > 0.99);  // v_x
  // The axis kept the prior's 2 cm; the cross directions were corrected.
  const Mat4f mid = along.fit.correction * corridor_error * turning[10];
  const Mat4f start = corridor_error * turning[10];
  CHECK(std::fabs(mid[3][0] - start[3][0]) < 1e-3f);
  CHECK(std::fabs(mid[3][1] - turning[10][3][1]) < 2e-3f);
  CHECK(std::fabs(mid[3][2] - turning[10][3][2]) < 2e-3f);
  // 20 frames, 5 new, 3 passes: validation and core's submit check pass.
  track::FitterConfig three = config_for(kind);
  three.icp.passes = 3;
  ASSIGN_OR_FAIL(auto passes_fitter,
                 track::create_fitter(gpu.device, gpu.allocator, three));
  std::vector<Mat4f> path;
  for (int i = 0; i < 25; ++i) {
    path.push_back(
        look_at(kRoomEye + Vec3f(0.01f * float(i), 0, 0), kRoomTarget));
  }
  ASSIGN_OR_FAIL(const auto path_frames,
                 frames_of(gpu, room(), cam, path, error));
  const std::vector<sensor::DeviceFrame> first(path_frames.begin(),
                                               path_frames.begin() + 20);
  const std::vector<sensor::DeviceFrame> second(path_frames.begin() + 5,
                                                path_frames.end());
  ASSIGN_OR_FAIL(const Fit w1, fit_once(gpu, *passes_fitter, m.room, first));
  ASSIGN_OR_FAIL(const Fit w2, fit_once(gpu, *passes_fitter, m.room, second));
  CHECK(w1.fit.solved() && w2.fit.solved() && w2.fit.passes == 3);
  ASSIGN_OR_FAIL(auto fresh,
                 track::create_fitter(gpu.device, gpu.allocator, three));
  ASSIGN_OR_FAIL(const Fit w2_fresh, fit_once(gpu, *fresh, m.room, second));
  CHECK(same_fit(w2.fit, w2_fresh.fit));  // reused entries give a fresh fit
  return 0;
}

int test_iterations_and_passes(const vkc::test::Gpu& gpu, track::FitKind kind,
                               const Models& m) {
  const Camera cam;
  const Mat4f truth = look_at(kRoomEye, kRoomTarget);
  const Mat4f error =
      error_at(Vec3d(kRoomEye), {0.01, 0.0, 0.005}, {0, 0, 1}, 0.5);
  ASSIGN_OR_FAIL(const auto frames,
                 frames_of(gpu, room(), cam, {truth}, error));
  // Zero iterations: the prior, exactly, scored once.
  track::FitterConfig score = config_for(kind);
  score.icp.iterations = {0, 0, 0};
  ASSIGN_OR_FAIL(auto scorer,
                 track::create_fitter(gpu.device, gpu.allocator, score));
  track::PosePrior prior;
  prior.correction = error_at(Vec3d(kRoomEye), {0.001, 0, 0}, {0, 1, 0}, 0.1);
  ASSIGN_OR_FAIL(const Fit scored,
                 fit_once(gpu, *scorer, m.room, frames, prior));
  CHECK(std::memcmp(&scored.fit.correction, &prior.correction, sizeof(Mat4f)) ==
        0);
  CHECK(scored.fit.solved());
  track::FitterConfig first = config_for(kind);
  first.icp.iterations = {0, 0, 1};
  ASSIGN_OR_FAIL(auto one_step,
                 track::create_fitter(gpu.device, gpu.allocator, first));
  ASSIGN_OR_FAIL(const Fit stepped,
                 fit_once(gpu, *one_step, m.room, frames, prior));
  CHECK(scored.fit.attempts == stepped.fit.attempts);
  CHECK(scored.fit.hits == stepped.fit.hits);
  CHECK(scored.fit.inliers == stepped.fit.inliers);
  CHECK(scored.fit.relative_eigenvalues == stepped.fit.relative_eigenvalues);
  // Passes: from 5 cm / 1 deg, 3 passes converge.
  const Mat4f far =
      error_at(Vec3d(kRoomEye), {0.03, -0.03, 0.025}, {0.5, 0.3, 1.0}, 1.0);
  ASSIGN_OR_FAIL(const auto far_frames,
                 frames_of(gpu, room(), cam, {truth}, far));
  track::FitterConfig three = config_for(kind);
  three.icp.passes = 3;
  ASSIGN_OR_FAIL(auto three_fitter,
                 track::create_fitter(gpu.device, gpu.allocator, three));
  ASSIGN_OR_FAIL(const Fit p3,
                 fit_once(gpu, *three_fitter, m.room, far_frames));
  double em = 0.0;
  double edeg = 0.0;
  gap(p3.fit.correction * far, Mat4f(1.0f), kRoomEye, em, edeg);
  std::printf("  %s 3 passes from 5 cm / 1 deg: %.3f mm %.4f deg left\n",
              track::to_string(kind), em * 1e3, edeg);
  CHECK(em < 2e-3 && edeg < 0.05);
  // At the truth a pass moves under 1 cm / 0.3 deg: 3 passes stop after the
  // first, and give 1 pass's fit.
  ASSIGN_OR_FAIL(const auto exact,
                 frames_of(gpu, room(), cam, {truth}, Mat4f(1.0f)));
  ASSIGN_OR_FAIL(
      auto one_fitter,
      track::create_fitter(gpu.device, gpu.allocator, config_for(kind)));
  ASSIGN_OR_FAIL(const Fit q1, fit_once(gpu, *one_fitter, m.room, exact));
  ASSIGN_OR_FAIL(const Fit q3, fit_once(gpu, *three_fitter, m.room, exact));
  track::TrackFit q3_as_one = q3.fit;
  q3_as_one.passes = 1;
  CHECK(same_fit(q3_as_one, q1.fit));
  return 0;
}

// The prior term: a tight prior holds the fit near its mean, and only with
// the data rescaled to n_eff points a frame; without one the fit goes all the
// way.
int test_prior(const vkc::test::Gpu& gpu, track::FitKind kind,
               const Models& m) {
  const Camera cam;
  const Mat4f truth = look_at(kRoomEye, kRoomTarget);
  const Mat4f error =
      error_at(Vec3d(kRoomEye), {0.012, -0.01, 0.008}, {0.2, 0.4, 1.0}, 0.8);
  ASSIGN_OR_FAIL(const auto frames,
                 frames_of(gpu, room(), cam, {truth}, error));
  const auto left = [&](const track::TrackFit& f) {
    double em = 0.0;
    double edeg = 0.0;
    gap(f.correction * error, Mat4f(1.0f), kRoomEye, em, edeg);
    return em;
  };
  track::PosePrior tight;
  tight.sigma_m = 0.001f;
  tight.sigma_rad = 0.001f;
  ASSIGN_OR_FAIL(auto a, track::create_fitter(gpu.device, gpu.allocator,
                                              config_for(kind)));
  ASSIGN_OR_FAIL(const Fit free_fit, fit_once(gpu, *a, m.room, frames));
  ASSIGN_OR_FAIL(const Fit held_fit, fit_once(gpu, *a, m.room, frames, tight));
  track::FitterConfig honest = config_for(kind);
  honest.icp.prior_n_eff = 1e9f;  // the data counted at every pixel: s = 1
  ASSIGN_OR_FAIL(auto b,
                 track::create_fitter(gpu.device, gpu.allocator, honest));
  ASSIGN_OR_FAIL(const Fit undeflated,
                 fit_once(gpu, *b, m.room, frames, tight));
  std::printf(
      "  %s prior: free %.3f mm, tight %.3f mm, tight undeflated %.3f "
      "mm left\n",
      track::to_string(kind), left(free_fit.fit) * 1e3,
      left(held_fit.fit) * 1e3, left(undeflated.fit) * 1e3);
  CHECK(left(free_fit.fit) < 1e-4);
  CHECK(left(held_fit.fit) > 5e-3);  // most of the 2 cm kept
  CHECK(left(undeflated.fit) < 0.5 * left(held_fit.fit));
  // The prior moves no held direction and leaves the eigensystem the data's.
  CHECK(held_fit.fit.relative_eigenvalues ==
            free_fit.fit.relative_eigenvalues ||
        held_fit.fit.held == free_fit.fit.held);
  return 0;
}

int test_determinism(const vkc::test::Gpu& gpu, track::FitKind kind,
                     const Models& m) {
  const Camera cam;
  std::vector<Mat4f> path;
  for (int i = 0; i < 6; ++i) {
    path.push_back(
        look_at(kRoomEye + Vec3f(0, 0.02f * float(i), 0), kRoomTarget));
  }
  const Mat4f error =
      error_at(Vec3d(kRoomEye), {0.01, 0.01, 0.0}, {0, 0, 1}, 0.7);
  ASSIGN_OR_FAIL(const auto frames, frames_of(gpu, room(), cam, path, error));
  track::PosePrior prior;
  prior.sigma_m = 0.005f;
  prior.sigma_rad = 0.005f;
  ASSIGN_OR_FAIL(auto a, track::create_fitter(gpu.device, gpu.allocator,
                                              config_for(kind)));
  ASSIGN_OR_FAIL(const Fit first, fit_once(gpu, *a, m.room, frames, prior));
  ASSIGN_OR_FAIL(auto b, track::create_fitter(gpu.device, gpu.allocator,
                                              config_for(kind)));
  ASSIGN_OR_FAIL(const Fit second, fit_once(gpu, *b, m.room, frames, prior));
  CHECK(same_fit(first.fit, second.fit));
  // A second fitter, on another filter, in the same batch.
  track::FitterConfig other = config_for(kind);
  other.model_filter.attribute = kSource;
  ASSIGN_OR_FAIL(auto c,
                 track::create_fitter(gpu.device, gpu.allocator, other));
  ASSIGN_OR_FAIL(auto d, track::create_fitter(gpu.device, gpu.allocator,
                                              config_for(kind)));
  vkc::CommandBatch batch(gpu.device, gpu.allocator);
  ASSIGN_OR_FAIL(const track::PendingFit pd,
                 d->record(batch, m.room, frames, prior));
  ASSIGN_OR_FAIL(const track::PendingFit pc,
                 c->record(batch, m.room, frames, {}));
  OK_OR_FAIL(batch.submit());
  ASSIGN_OR_FAIL(const track::TrackFit beside, d->finish(pd));
  ASSIGN_OR_FAIL(const track::TrackFit theirs, c->finish(pc));
  CHECK(same_fit(beside, first.fit));
  CHECK(theirs.solved());
  return 0;
}

int test_see_through(const vkc::test::Gpu& gpu, track::FitKind kind,
                     const Models& m) {
  const Camera cam;
  // Looking down at the floor: the left third sees a step 0.3 m up (in front
  // of the model's floor), the right third a pit 0.3 m down (behind it).
  const Vec3f eye(0.0f, 0.0f, 1.5f);
  const Mat4f down(Vec4f(1, 0, 0, 0), Vec4f(0, -1, 0, 0), Vec4f(0, 0, -1, 0),
                   Vec4f(eye, 1));
  const std::vector<float> depth =
      render(floor_only(), cam, down, [](double u, double t) {
        if (u < 1.0 / 3.0) return t * (1.2 / 1.5);
        if (u > 2.0 / 3.0) return t * (1.8 / 1.5);
        return t;
      });
  ASSIGN_OR_FAIL(const sensor::DeviceFrame f,
                 make_frame(gpu, depth, cam, down, 1000));
  track::FitterConfig counting = config_for(kind);
  counting.see_through_m = 0.1f;
  counting.icp.iterations = {0, 0, 0};
  ASSIGN_OR_FAIL(auto fitter,
                 track::create_fitter(gpu.device, gpu.allocator, counting));
  ASSIGN_OR_FAIL(const Fit fit, fit_once(gpu, *fitter, m.floor, {f}));
  std::printf("  %s see-through: %u in front, %u behind of %u hits\n",
              track::to_string(kind), fit.fit.in_front, fit.fit.behind,
              fit.fit.hits);
  CHECK(fit.fit.in_front > fit.fit.hits / 5 &&
        fit.fit.behind > fit.fit.hits / 5);
  CHECK(fit.fit.in_front + fit.fit.behind < fit.fit.hits);
  // Off by default: none counted.
  track::FitterConfig plain = config_for(kind);
  plain.icp.iterations = {0, 0, 0};
  ASSIGN_OR_FAIL(auto quiet,
                 track::create_fitter(gpu.device, gpu.allocator, plain));
  ASSIGN_OR_FAIL(const Fit none, fit_once(gpu, *quiet, m.floor, {f}));
  CHECK(none.fit.in_front == 0 && none.fit.behind == 0);
  return 0;
}

// --- Opt-in: timing (U3) and a basin sweep
// -------------------------------------

double device_ms(const vkc::StageMetrics& rows, const char* name) {
  for (const vkc::StageRow& r : rows.rows()) {
    if (std::string_view(r.name) == name) return r.gpu_ms;
  }
  return -1.0;
}

int timing(const vkc::test::GpuContext& gpu, track::FitKind kind) {
  const Camera cam{256, 192, 128.0f};
  // The room at 1 cm, as the lab's grid is.
  ASSIGN_OR_FAIL(const vol::VoxelBlockGrid model,
                 make_model(gpu, room(), 0.01f, 16384));
  std::vector<Mat4f> path;
  for (int i = 0; i < 25; ++i) {
    path.push_back(look_at(
        kRoomEye + Vec3f(0.01f * float(i), 0.005f * float(i), 0), kRoomTarget));
  }
  const Mat4f error =
      error_at(Vec3d(kRoomEye), {0.01, 0.01, 0.0}, {0, 0, 1}, 0.5);
  ASSIGN_OR_FAIL(const auto frames, frames_of(gpu, room(), cam, path, error));
  struct Case {
    std::uint32_t k;
    int passes;
    std::vector<double> total, pyramid, cast, icp;
    std::unique_ptr<track::IFitter> fitter;
    std::uint64_t next = 0;
  };
  std::vector<Case> cases;
  for (const std::uint32_t k : {1u, 20u}) {
    for (const int passes : {1, 3}) {
      track::FitterConfig c = config_for(kind);
      c.icp.passes = passes;
      ASSIGN_OR_FAIL(auto fitter,
                     track::create_fitter(gpu.device, gpu.allocator, c));
      cases.push_back({k, passes, {}, {}, {}, {}, std::move(fitter), 0});
    }
  }
  const int runs = std::getenv("VR_TRACK_TIMING_RUNS") != nullptr
                       ? std::atoi(std::getenv("VR_TRACK_TIMING_RUNS"))
                       : 60;
  // Alternated: every configuration once a round. A window slides one frame
  // a round, so a K = 20 fit builds 1 new pyramid and reuses 19 (the lane
  // at a hop builds 5).
  for (int round = 0; round < runs + 5; ++round) {
    for (Case& c : cases) {
      const std::size_t start = std::size_t(round) % (frames.size() - c.k + 1);
      std::vector<sensor::DeviceFrame> window(
          frames.begin() + std::ptrdiff_t(start),
          frames.begin() + std::ptrdiff_t(start + c.k));
      // Fresh timestamps each round: the window's newest frame is new.
      for (std::size_t i = 0; i < window.size(); ++i) {
        window[i].timestamp_ns = 1'000'000ull * (std::uint64_t(round) + i + 1);
      }
      ASSIGN_OR_FAIL(const Fit f, fit_once(gpu, *c.fitter, model, window));
      if (round < 5) continue;  // warm-up
      c.total.push_back(device_ms(f.rows, "track"));
      c.pyramid.push_back(device_ms(f.rows, "  ..pyramid"));
      c.cast.push_back(device_ms(f.rows, "  ..ray cast"));
      c.icp.push_back(device_ms(f.rows, "  ..icp"));
    }
  }
  const auto pct = [](std::vector<double> v, double q) {
    std::sort(v.begin(), v.end());
    return v[std::min(v.size() - 1, std::size_t(q * double(v.size())))];
  };
  std::printf(
      "U3 timing on %s, %s, 256x192, 1 cm room, %d alternated runs; "
      "device ms p50 / p95\n",
      gpu.physical.properties().deviceName, track::to_string(kind), runs);
  std::printf(
      "| K | passes | fit | pyramid | ray cast | icp "
      "|\n|---|---|---|---|---|---|\n");
  for (const Case& c : cases) {
    std::printf(
        "| %u | %d | %.3f / %.3f | %.3f / %.3f | %.3f / %.3f | %.3f / %.3f |\n",
        c.k, c.passes, pct(c.total, 0.5), pct(c.total, 0.95),
        pct(c.pyramid, 0.5), pct(c.pyramid, 0.95), pct(c.cast, 0.5),
        pct(c.cast, 0.95), pct(c.icp, 0.5), pct(c.icp, 0.95));
  }
  return 0;
}

int sweep(const vkc::test::GpuContext& gpu, track::FitKind kind,
          const Models& m) {
  const Camera cam{128, 96, 64.0f};
  std::mt19937 gen(7);
  std::normal_distribution<double> normal;
  const auto unit = [&] {
    Vec3d v(normal(gen), normal(gen), normal(gen));
    return glm::normalize(v);
  };
  std::printf(
      "Basin sweep, %s on %s, the room, 128x96: share converged to "
      "< 3 mm / 0.1 deg (of 20 starts)\n",
      track::to_string(kind), gpu.physical.properties().deviceName);
  std::printf(
      "| start | K = 1, 1 pass | K = 1, 3 passes | K = 10, 1 pass | "
      "K = 10, 3 passes |\n|---|---|---|---|---|\n");
  for (const auto& [cm, deg] : std::vector<std::pair<double, double>>{
           {1, 0.5}, {2, 1}, {5, 1}, {5, 3}, {10, 3}}) {
    std::printf("| %.0f cm / %.1f deg |", cm, deg);
    for (const std::uint32_t k : {1u, 10u}) {
      for (const int passes : {1, 3}) {
        track::FitterConfig c = config_for(kind);
        c.icp.passes = passes;
        ASSIGN_OR_FAIL(auto fitter,
                       track::create_fitter(gpu.device, gpu.allocator, c));
        int good = 0;
        for (int s = 0; s < 20; ++s) {
          std::vector<Mat4f> path;
          for (std::uint32_t i = 0; i < k; ++i) {
            path.push_back(
                look_at(kRoomEye + Vec3f(0.02f * float(i), 0, 0), kRoomTarget));
          }
          const Mat4f error =
              error_at(Vec3d(kRoomEye), unit() * (cm * 0.01), unit(), deg);
          ASSIGN_OR_FAIL(const auto frames,
                         frames_of(gpu, room(), cam, path, error,
                                   std::uint64_t(s) * 100000000000ull));
          ASSIGN_OR_FAIL(const Fit f, fit_once(gpu, *fitter, m.room, frames));
          double em = 0.0;
          double edeg = 0.0;
          gap(f.fit.correction * error, Mat4f(1.0f), kRoomEye, em, edeg);
          good += em < 3e-3 && edeg < 0.1;
        }
        std::printf(" %d/20 |", good);
      }
    }
    std::printf("\n");
  }
  return 0;
}

bool flag(const char* name) {
  const char* v = std::getenv(name);
  return v != nullptr && std::string_view(v) == "1";
}

int gpu_main(vkc::test::GpuContext& gpu) {
  const auto t0 = std::chrono::steady_clock::now();
  ASSIGN_OR_FAIL(vol::VoxelBlockGrid room_model,
                 make_model(gpu, room(), 0.05f, 512));
  ASSIGN_OR_FAIL(vol::VoxelBlockGrid corridor_model,
                 make_model(gpu, corridor(), 0.05f, 512));
  ASSIGN_OR_FAIL(vol::VoxelBlockGrid floor_model,
                 make_model(gpu, floor_only(), 0.05f, 256));
  ASSIGN_OR_FAIL(vol::VoxelBlockGrid empty_model,
                 make_model(gpu, Scene{}, 0.05f, 8));
  const Models m{std::move(room_model), std::move(corridor_model),
                 std::move(floor_model), std::move(empty_model)};
  const auto t1 = std::chrono::steady_clock::now();
  for (const track::FitKind kind : track::kFitKinds) {
    std::printf("%s on %s\n", track::to_string(kind),
                gpu.physical.properties().deviceName);
    CHECK(test_contract(gpu, kind, m) == 0);
    CHECK(test_fit(gpu, kind, m) == 0);
    CHECK(test_filter(gpu, kind, m) == 0);
    CHECK(test_windows(gpu, kind, m) == 0);
    CHECK(test_iterations_and_passes(gpu, kind, m) == 0);
    CHECK(test_prior(gpu, kind, m) == 0);
    CHECK(test_determinism(gpu, kind, m) == 0);
    CHECK(test_see_through(gpu, kind, m) == 0);
    if (flag("VR_TRACK_TIMING")) CHECK(timing(gpu, kind) == 0);
    if (flag("VR_TRACK_SWEEP")) CHECK(sweep(gpu, kind, m) == 0);
  }
  CHECK(std::string_view(track::to_string(track::FitKind::PointToPlane)) ==
        "point_to_plane");
  const auto t2 = std::chrono::steady_clock::now();
  std::printf("models %.0f ms, cases %.0f ms\n",
              std::chrono::duration<double, std::milli>(t1 - t0).count(),
              std::chrono::duration<double, std::milli>(t2 - t1).count());
  return 0;
}

}  // namespace

int main() {
  return vkc::test::run_on_gpu(volumetric_kit::recon::device_requirements(),
                               gpu_main);
}
