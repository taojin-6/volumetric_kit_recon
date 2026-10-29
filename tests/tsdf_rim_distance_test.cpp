// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The true distance near outlines (DECISIONS.md, 2026-10-29), through
// tsdf::Fuser. Depth is ray-cast and noise-free from a Femto-like camera
// (640 x 576, fx 504) into 1 cm voxels, so every outline below is sharp.
//
// One camera, in each mode, ten sets of a still scene: a wall 2.5 m away, a
// square plate facing the camera 1.2 m away, turned 20 degrees in its plane
// so no side runs along the grid, and a vertical rod 1.6 cm across, 1 m away:
//
// - the plate's outline lies at its true edge, under 2 mm off on average and
//   under 6 mm anywhere along its four sides, where the projective field
//   steps it a cell at a time (2.9 / 8.1 mm Classic, 6.5 / 12.8 mm Dynamic);
// - a voxel beside the plate's edge, within the voxel's footprint of it,
//   holds its Euclidean distance to the plate to within the pixel pitch
//   there, not +trunc (Classic) or nothing (Dynamic);
// - the rod meshes along its whole length, which Dynamic's projective
//   field leaves empty.
//
// Dynamic, one camera, the plate (unturned) moving sideways 3 cm a set and
// stopping: in a set it moves, no voxel beside its side outlines holds
// weight -- an outline that moved takes no distance, so the fresh samples of
// the moving surface are fused alone, and the distances it held beside it
// before it moved go with it -- and in the first set it is still again, its
// side outlines take their distances.
//
// Dynamic, three cameras around a sphere 0.4 m across that moves at 1 m/s
// past a wall, stops and leaves: while it moves, the mesh covers 97% of its
// visible surface in every set; once it stops, 99.5%; and in the set it
// leaves, nothing of it meshes and no voxel off the wall's band holds weight.
// Skips (exit 0) where no device is present.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <map>
#include <tuple>
#include <vector>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/testing/test_check.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/grid/mesh/marching_cubes.hpp"
#include "volumetric_kit/grid/volume/voxel_block_grid.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"
#include "volumetric_kit/recon/core/math/vector_types.hpp"
#include "volumetric_kit/recon/tsdf/fuser.hpp"
#include "volumetric_kit/recon/tsdf/fusion_grid.hpp"
#include "volumetric_kit/recon/tsdf/tsdf_integrator.hpp"

#include "volumetric_kit/core/testing/gpu_test.hpp"
#include "volumetric_kit/grid/testing/grid_readback.hpp"

namespace vr = volumetric_kit::recon;
namespace sensor = volumetric_kit::sensor;
namespace vkc = volumetric_kit::core;
namespace vol = volumetric_kit::grid::volume;
namespace tsdf = volumetric_kit::recon::tsdf;
namespace mesh = volumetric_kit::grid::mesh;

namespace {

constexpr std::uint32_t kWidth = 640;
constexpr std::uint32_t kHeight = 576;
constexpr float kFocal = 504.0f;
constexpr float kVoxel = 0.01f;
constexpr float kTrunc = 0.04f;
constexpr float kMaxWeight = 5.0f;

constexpr float kWall = 2.5f;
// The plate: a square of half side kPlateHalf at depth kPlateZ, turned
// `turn` about the optical axis around its centre. Its depth sits off the
// voxel layers, so no voxel lies on it; the layer 3 mm in front is kFront.
constexpr float kPlateZ = 1.203f;
constexpr float kFront = 1.20f;
constexpr float kPlateHalf = 0.12f;
constexpr float kTurn = 0.349f;  // 20 degrees
// The rod: a vertical cylinder of radius kRodR on the axis x = kRodX,
// z = kRodZ, from y = -kRodHalf to kRodHalf.
constexpr float kRodX = 0.22f;
constexpr float kRodZ = 1.004f;
constexpr float kRodR = 0.008f;
constexpr float kRodHalf = 0.25f;
// A rim distance's error bound: the pixel pitch at the plate.
constexpr float kPitch = kPlateZ / kFocal;

sensor::DepthCameraParams camera(const vr::Mat4f& pose) {
  sensor::DepthCameraParams cam{};
  cam.fx = cam.fy = kFocal;
  cam.cx = 319.5f;
  cam.cy = 287.5f;
  cam.min_depth = 0.2f;
  cam.max_depth = 5.0f;
  cam.width = kWidth;
  cam.height = kHeight;
  cam.cam_to_world = pose;
  return cam;
}

// A camera at `eye` looking at `target`, its image y along world y.
sensor::DepthCameraParams look_at(const vr::Vec3f& eye,
                                  const vr::Vec3f& target) {
  const vr::Vec3f z = glm::normalize(target - eye);
  const vr::Vec3f x = glm::normalize(glm::cross(vr::Vec3f(0, 1, 0), z));
  const vr::Vec3f y = glm::cross(z, x);
  return camera(vr::Mat4f(vr::Vec4f(x, 0.0f), vr::Vec4f(y, 0.0f),
                          vr::Vec4f(z, 0.0f), vr::Vec4f(eye, 1.0f)));
}

// A world ray through pixel (u, v): its origin, and a direction whose
// camera-space z is 1, so its parameter is the camera depth.
struct Ray {
  vr::Vec3f o;
  vr::Vec3f d;
};
Ray ray_of(const sensor::DepthCameraParams& cam, float u, float v) {
  const vr::Mat4f& m = cam.cam_to_world;
  const vr::Vec3f dc((u - cam.cx) / cam.fx, (v - cam.cy) / cam.fy, 1.0f);
  return {vr::Vec3f(m[3]), vr::Vec3f(m * vr::Vec4f(dc, 0.0f))};
}

// The plate: its centre and turn.
struct Plate {
  vr::Vec2f centre;
  float turn = 0.0f;

  // A world point's coordinates along the plate's two sides, from its
  // centre.
  vr::Vec2f uv(const vr::Vec3f& p) const {
    const float c = std::cos(turn);
    const float s = std::sin(turn);
    const float x = p.x - centre.x;
    const float y = p.y - centre.y;
    return {c * x + s * y, -s * x + c * y};
  }
  // Where a ray meets it, or infinity.
  float hit(const Ray& r) const {
    const float t = (kPlateZ - r.o.z) / r.d.z;
    const vr::Vec2f q = uv(r.o + t * r.d);
    return std::fabs(q.x) <= kPlateHalf && std::fabs(q.y) <= kPlateHalf
               ? t
               : std::numeric_limits<float>::infinity();
  }
  // The Euclidean distance from `p` to its face.
  float distance(const vr::Vec3f& p) const {
    const vr::Vec2f q = uv(p);
    const float da = std::max(std::fabs(q.x) - kPlateHalf, 0.0f);
    const float db = std::max(std::fabs(q.y) - kPlateHalf, 0.0f);
    const float dz = p.z - kPlateZ;
    return std::sqrt(da * da + db * db + dz * dz);
  }
};

// Where a ray first meets a sphere, or infinity.
float hit_sphere(const Ray& r, const vr::Vec3f& c, float radius) {
  const vr::Vec3f oc = r.o - c;
  const float a = glm::dot(r.d, r.d);
  const float b = glm::dot(oc, r.d);
  const float q = glm::dot(oc, oc) - radius * radius;
  const float disc = b * b - a * q;
  if (disc < 0.0f) return std::numeric_limits<float>::infinity();
  const float t = (-b - std::sqrt(disc)) / a;
  return t > 0.0f ? t : std::numeric_limits<float>::infinity();
}

// Where a ray first meets the rod, or infinity.
float hit_rod(const Ray& r) {
  const float ox = r.o.x - kRodX;
  const float oz = r.o.z - kRodZ;
  const float a = r.d.x * r.d.x + r.d.z * r.d.z;
  const float b = ox * r.d.x + oz * r.d.z;
  const float c = ox * ox + oz * oz - kRodR * kRodR;
  const float disc = b * b - a * c;
  if (disc < 0.0f) return std::numeric_limits<float>::infinity();
  const float t = (-b - std::sqrt(disc)) / a;
  return t > 0.0f && std::fabs((r.o + t * r.d).y) <= kRodHalf
             ? t
             : std::numeric_limits<float>::infinity();
}

// The wall, the plate and, with `rod`, the rod, from a camera at the origin.
std::vector<float> render(const sensor::DepthCameraParams& cam,
                          const Plate& plate, bool rod) {
  std::vector<float> depth(std::size_t(kWidth) * kHeight);
  for (std::uint32_t v = 0; v < kHeight; ++v) {
    for (std::uint32_t u = 0; u < kWidth; ++u) {
      const Ray r = ray_of(cam, float(u), float(v));
      float t = std::min((kWall - r.o.z) / r.d.z, plate.hit(r));
      if (rod) t = std::min(t, hit_rod(r));
      depth[std::size_t(v) * kWidth + u] = t;
    }
  }
  return depth;
}

// A voxel of the grid's active blocks: its world position and its element
// in the attributes.
struct Voxel {
  vr::Vec3f world;
  std::size_t index;
};

vkc::Result<std::vector<Voxel>> voxels_of(vol::VoxelBlockGrid& grid) {
  VKC_ASSIGN(const auto blocks, vgrid_test::blocks_of(grid));
  std::vector<Voxel> out;
  for (const auto& [coord, ptr] : blocks) {
    const int b[3] = {std::get<0>(coord), std::get<1>(coord),
                      std::get<2>(coord)};
    for (int l = 0; l < 512; ++l) {
      const int v[3] = {b[0] * 8 + l % 8, b[1] * 8 + (l / 8) % 8,
                        b[2] * 8 + l / 64};
      out.push_back({vr::Vec3f(float(v[0]), float(v[1]), float(v[2])) * kVoxel,
                     std::size_t(ptr) + std::size_t(l)});
    }
  }
  return out;
}

// A grid's voxels with their tsdf and weight.
struct Field {
  std::vector<Voxel> voxels;
  std::vector<float> tsdf;
  std::vector<float> weight;
};

vkc::Result<Field> field_of(vkc::test::GpuContext& gpu,
                            vol::VoxelBlockGrid& grid) {
  Field f;
  VKC_ASSIGN(f.voxels, voxels_of(grid));
  VKC_ASSIGN(f.tsdf, vgrid_test::read_attribute<float>(
                         gpu.device, gpu.allocator, grid, vol::kTsdfAttribute));
  VKC_ASSIGN(f.weight,
             vgrid_test::read_attribute<float>(gpu.device, gpu.allocator, grid,
                                               vol::kWeightAttribute));
  return f;
}

// Beside the plate's edge on the layer in front of its face: the voxels
// with weight within `reach` of the plate, outside its square by at least
// 3 mm in its plane and clear of its corners by 2 cm, on its sides `sides`
// (bit 0 +a, 1 -a, 2 +b, 3 -b); and the worst |tsdf - distance| among them.
struct Beside {
  std::size_t weighted = 0;
  float worst = 0.0f;
};

Beside beside(const Field& f, const Plate& plate, float reach, int sides) {
  Beside out;
  for (const Voxel& x : f.voxels) {
    if (std::fabs(x.world.z - kFront) > 1e-4f || !(f.weight[x.index] > 0.0f)) {
      continue;
    }
    const vr::Vec2f q = plate.uv(x.world);
    const float truth = plate.distance(x.world);
    const bool on_a = std::fabs(q.x) >= kPlateHalf + 0.003f &&
                      std::fabs(q.y) <= kPlateHalf - 0.02f;
    const bool on_b = std::fabs(q.y) >= kPlateHalf + 0.003f &&
                      std::fabs(q.x) <= kPlateHalf - 0.02f;
    const int side = on_a   ? (q.x > 0.0f ? 0 : 1)
                     : on_b ? (q.y > 0.0f ? 2 : 3)
                            : -1;
    if (side < 0 || (sides & (1 << side)) == 0 || truth > reach) continue;
    ++out.weighted;
    out.worst = std::max(out.worst, std::fabs(f.tsdf[x.index] - truth));
  }
  return out;
}

// The plate's outline in a mesh: at points every 5 mm along each side, 2 cm
// clear of the corners, how far outward of the true edge the mesh's front
// reaches -- its triangles within 12 mm of the face, seen along the optical
// axis, met by the line through the point square to the side. A side no
// triangle meets within 3 cm of a point is open there.
struct Outline {
  std::size_t open = 0;
  std::size_t points = 0;
  float mean = 0.0f;
  float worst = 0.0f;
};

Outline outline(const mesh::Mesh& m, const Plate& plate) {
  std::vector<std::array<vr::Vec2f, 3>> front;
  for (std::size_t t = 0; t < m.triangle_count(); ++t) {
    std::array<vr::Vec2f, 3> tri;
    bool near_face = true;
    for (std::size_t c = 0; c < 3; ++c) {
      const vr::Vec3f& p = m.vertices[m.indices[3 * t + c]].position;
      near_face = near_face && std::fabs(p.z - kPlateZ) <= 0.012f;
      tri[c] = plate.uv(p);
    }
    if (near_face) front.push_back(tri);
  }
  Outline out;
  double sum = 0.0;
  for (int side = 0; side < 4; ++side) {
    // Side 0 is +a, 1 -a, 2 +b, 3 -b: `out` past the edge, `along` it.
    const auto out_of = [side](const vr::Vec2f& q) {
      return side == 0   ? q.x - kPlateHalf
             : side == 1 ? -q.x - kPlateHalf
             : side == 2 ? q.y - kPlateHalf
                         : -q.y - kPlateHalf;
    };
    const auto along_of = [side](const vr::Vec2f& q) {
      return side < 2 ? q.y : q.x;
    };
    for (float along = -kPlateHalf + 0.02f; along <= kPlateHalf - 0.02f;
         along += 0.005f) {
      float extent = -std::numeric_limits<float>::infinity();
      for (const auto& tri : front) {
        for (int e = 0; e < 3; ++e) {
          const vr::Vec2f& a = tri[e];
          const vr::Vec2f& b = tri[(e + 1) % 3];
          const float ta = along_of(a) - along;
          const float tb = along_of(b) - along;
          if (ta * tb > 0.0f || ta == tb) continue;
          const float f = ta / (ta - tb);
          const float o = out_of(a) + f * (out_of(b) - out_of(a));
          if (std::fabs(o) < 0.03f) extent = std::max(extent, o);
        }
      }
      ++out.points;
      if (!std::isfinite(extent)) {
        ++out.open;
        continue;
      }
      out.worst = std::max(out.worst, std::fabs(extent));
      sum += std::fabs(extent);
    }
  }
  out.mean = float(sum / double(std::max<std::size_t>(out.points, 1)));
  return out;
}

// The still scene, ten sets of one camera in `mode`.
int still_scene(vkc::test::GpuContext& gpu, tsdf::IntegrationMode mode) {
  const char* name =
      mode == tsdf::IntegrationMode::Dynamic ? "Dynamic" : "Classic";
  auto fuser = tsdf::Fuser::create(gpu.device, gpu.allocator);
  CHECK(fuser.ok());
  auto mc = mesh::MarchingCubes::create(gpu.device, gpu.allocator);
  CHECK(mc.ok());
  auto made =
      tsdf::create_fusion_grid(gpu.device, gpu.allocator, kVoxel, kTrunc, 4096);
  CHECK(made.ok());
  vol::VoxelBlockGrid& grid = made.value();
  const sensor::DepthCameraParams cam = camera(vr::Mat4f(1.0f));
  const Plate plate{{-0.12f, 0.0f}, kTurn};
  const std::vector<float> depth = render(cam, plate, true);
  for (int set = 0; set < 10; ++set) {
    CHECK(fuser
              ->fuse(grid, {{vkc::StorageInput(depth.data()), cam, nullptr, 0}},
                     kMaxWeight, mode)
              .ok());
  }
  auto m = mc->extract_host(grid);
  CHECK(m.ok());

  const Outline o = outline(*m, plate);
  std::printf(
      "%s plate outline: %zu of %zu points open, off by %.2f mm on "
      "average, %.2f at worst\n",
      name, o.open, o.points, 1e3 * o.mean, 1e3 * o.worst);
  CHECK(o.open == 0 && o.mean < 0.002f && o.worst < 0.006f);

  // Within 6 mm of the plate: inside the footprint of a voxel there.
  auto f = field_of(gpu, grid);
  CHECK(f.ok());
  const Beside b = beside(*f, plate, 0.006f, 15);
  std::printf(
      "%s beside the plate: %zu voxels, |tsdf - distance| %.2f mm at "
      "worst\n",
      name, b.weighted, 1e3 * b.worst);
  CHECK(b.weighted >= 10 && b.worst <= kPitch);

  // The rod: every centimetre of its length holds a vertex on it.
  std::size_t gaps = 0;
  std::size_t slices = 0;
  for (float y = -kRodHalf + 0.02f; y <= kRodHalf - 0.02f; y += 0.01f) {
    bool any = false;
    for (const mesh::Vertex& vtx : m->vertices) {
      const vr::Vec3f& p = vtx.position;
      if (std::fabs(p.y - y) <= 0.005f &&
          std::hypot(p.x - kRodX, p.z - kRodZ) <= kRodR + 0.006f) {
        any = true;
        break;
      }
    }
    ++slices;
    gaps += any ? 0 : 1;
  }
  std::printf("%s rod: %zu of %zu centimetres with no vertex\n", name, gaps,
              slices);
  CHECK(gaps == 0);
  return 0;
}

// The moving plate: one camera, Dynamic, the plate unturned, still for sets
// 0-9, 3 cm further along x each set 10-14, still again from 15. Its sides
// lie 4.5 and 5.5 mm inside a voxel column, so a voxel beside each is within
// its footprint of it.
constexpr int kPlateMoves = 10;
constexpr int kPlateStops = 15;
constexpr int kPlateSets = 17;

Plate plate_at(int s) {
  const int k = std::clamp(s - kPlateMoves + 1, 0, kPlateStops - kPlateMoves);
  return Plate{{-0.1545f + 0.03f * float(k), 0.0f}, 0.0f};
}

int moving_plate(vkc::test::GpuContext& gpu) {
  tsdf::FuserConfig config;
  config.change.warm_up = 5;
  auto fuser = tsdf::Fuser::create(gpu.device, gpu.allocator, config);
  CHECK(fuser.ok());
  auto made =
      tsdf::create_fusion_grid(gpu.device, gpu.allocator, kVoxel, kTrunc, 4096);
  CHECK(made.ok());
  vol::VoxelBlockGrid& grid = made.value();
  const sensor::DepthCameraParams cam = camera(vr::Mat4f(1.0f));
  for (int s = 0; s < kPlateSets; ++s) {
    const Plate plate = plate_at(s);
    const std::vector<float> depth = render(cam, plate, false);
    CHECK(fuser
              ->fuse(grid, {{vkc::StorageInput(depth.data()), cam, nullptr, 0}},
                     kMaxWeight, tsdf::IntegrationMode::Dynamic)
              .ok());
    if (s < kPlateMoves - 1) continue;
    auto f = field_of(gpu, grid);
    CHECK(f.ok());
    // Its two side outlines, across the motion: within 3 cm, where the
    // plate's last set left distances too.
    const Beside b = beside(*f, plate, 0.03f, 3);
    const bool moved = s >= kPlateMoves && s < kPlateStops;
    std::printf(
        "moving plate, set %d (%s): %zu voxels beside its sides hold "
        "weight, |tsdf - distance| %.2f mm at worst\n",
        s, moved ? "moved" : "still", b.weighted, 1e3 * b.worst);
    if (moved) {
      CHECK(b.weighted == 0);
    } else {
      CHECK(b.weighted >= 20);
      // Within the footprint, the Euclidean distance.
      const Beside near = beside(*f, plate, 0.006f, 3);
      CHECK(near.weighted >= 10 && near.worst <= kPitch);
    }
  }
  return 0;
}

// The moving sphere: three cameras, Dynamic.
constexpr float kSphereR = 0.2f;
constexpr float kSphereZ = 2.0f;
constexpr float kMoveWall = 3.0f;
constexpr float kStep = 1.0f / 30.0f;  // 1 m/s at 30 sets a second
constexpr int kMoveFrom = 10;          // still before, at x = -0.6
constexpr int kMoveSets = 36;          // to x = +0.6
constexpr int kLeaveSet = 56;          // gone from here
constexpr int kSets = 58;

// The sphere's centre in set `s`, and whether it is there.
bool sphere_at(int s, vr::Vec3f* centre) {
  const int k = std::clamp(s - kMoveFrom + 1, 0, kMoveSets);
  *centre = vr::Vec3f(-0.6f + kStep * float(k), 0.0f, kSphereZ);
  return s < kLeaveSet;
}

std::vector<float> render_moving(const sensor::DepthCameraParams& cam, int s) {
  vr::Vec3f c;
  const bool there = sphere_at(s, &c);
  std::vector<float> depth(std::size_t(kWidth) * kHeight);
  for (std::uint32_t v = 0; v < kHeight; ++v) {
    for (std::uint32_t u = 0; u < kWidth; ++u) {
      const Ray r = ray_of(cam, float(u), float(v));
      float t = (kMoveWall - r.o.z) / r.d.z;
      if (there) t = std::min(t, hit_sphere(r, c, kSphereR));
      depth[std::size_t(v) * kWidth + u] = t;
    }
  }
  return depth;
}

// The share of the sphere's surface the cameras see -- every 4th pixel's --
// with a vertex within 1 cm: the vertices binned by 1 cm cell, so a point
// looks in the 27 cells around its own. Only a vertex within 1 cm of the
// sphere's surface can be within 1 cm of a point on it, so only those are
// binned, not the wall's.
float sphere_cover(const std::vector<sensor::DepthCameraParams>& cams,
                   const vr::Vec3f& c, const mesh::Mesh& m) {
  const auto cell = [](const vr::Vec3f& p) {
    return std::make_tuple(int(std::floor(p.x / 0.01f)),
                           int(std::floor(p.y / 0.01f)),
                           int(std::floor(p.z / 0.01f)));
  };
  std::map<std::tuple<int, int, int>, std::vector<vr::Vec3f>> bins;
  for (const mesh::Vertex& vtx : m.vertices) {
    if (std::abs(glm::length(vtx.position - c) - kSphereR) > 0.011f) continue;
    bins[cell(vtx.position)].push_back(vtx.position);
  }
  std::size_t seen = 0;
  std::size_t covered = 0;
  for (const sensor::DepthCameraParams& cam : cams) {
    for (std::uint32_t v = 0; v < kHeight; v += 4) {
      for (std::uint32_t u = 0; u < kWidth; u += 4) {
        const Ray r = ray_of(cam, float(u), float(v));
        const float t = hit_sphere(r, c, kSphereR);
        if (!std::isfinite(t)) continue;
        const vr::Vec3f p = r.o + t * r.d;
        ++seen;
        const auto [x, y, z] = cell(p);
        bool near = false;
        for (int dz = -1; dz <= 1 && !near; ++dz) {
          for (int dy = -1; dy <= 1 && !near; ++dy) {
            for (int dx = -1; dx <= 1 && !near; ++dx) {
              const auto it = bins.find({x + dx, y + dy, z + dz});
              if (it == bins.end()) continue;
              for (const vr::Vec3f& q : it->second) {
                if (glm::length(q - p) <= 0.01f) {
                  near = true;
                  break;
                }
              }
            }
          }
        }
        covered += near ? 1 : 0;
      }
    }
  }
  return seen > 0 ? float(covered) / float(seen) : 0.0f;
}

int moving_sphere(vkc::test::GpuContext& gpu) {
  tsdf::FuserConfig config;
  config.change.warm_up = 5;
  auto fuser = tsdf::Fuser::create(gpu.device, gpu.allocator, config);
  CHECK(fuser.ok());
  // Shared vertices: the 34 square metres of wall the three cameras see mesh
  // into 0.71 M triangles, whose 2.1 M private vertices, at 64 B each, pass
  // the 128 MiB storage binding every Vulkan device guarantees (lavapipe's),
  // so extract_host refuses that arena. Shared, they are 0.61 M.
  mesh::MarchingCubesConfig mc_config;
  mc_config.share_vertices = true;
  auto mc = mesh::MarchingCubes::create(gpu.device, gpu.allocator, mc_config);
  CHECK(mc.ok());
  auto made =
      tsdf::create_fusion_grid(gpu.device, gpu.allocator, kVoxel, kTrunc, 4096);
  CHECK(made.ok());
  vol::VoxelBlockGrid& grid = made.value();
  const std::vector<sensor::DepthCameraParams> cams = {
      look_at({0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, kSphereZ}),
      look_at({1.2f, 0.1f, 0.4f}, {0.0f, 0.0f, kSphereZ}),
      look_at({-0.7f, -1.0f, 0.5f}, {0.0f, 0.0f, kSphereZ})};
  float worst_moving = 1.0f;
  float worst_still = 1.0f;
  for (int s = 0; s < kSets; ++s) {
    std::vector<std::vector<float>> depths;
    std::vector<tsdf::FrameInput> frames;
    for (std::size_t k = 0; k < cams.size(); ++k) {
      depths.push_back(render_moving(cams[k], s));
    }
    for (std::size_t k = 0; k < cams.size(); ++k) {
      frames.push_back({vkc::StorageInput(depths[k].data()), cams[k], nullptr,
                        static_cast<std::uint32_t>(k)});
    }
    CHECK(fuser->fuse(grid, frames, kMaxWeight, tsdf::IntegrationMode::Dynamic)
              .ok());
    if (s < kMoveFrom) continue;
    auto m = mc->extract_host(grid);
    CHECK(m.ok());
    vr::Vec3f c;
    if (sphere_at(s, &c)) {
      const float cover = sphere_cover(cams, c, *m);
      if (s < kMoveFrom + kMoveSets) {
        worst_moving = std::min(worst_moving, cover);
      } else if (s > kMoveFrom + kMoveSets) {
        worst_still = std::min(worst_still, cover);
      }
      continue;
    }
    // Gone: no vertex and no weighted voxel off the wall's band.
    std::size_t off_wall = 0;
    for (const mesh::Vertex& vtx : m->vertices) {
      off_wall += vtx.position.z < kMoveWall - 0.05f ? 1 : 0;
    }
    auto f = field_of(gpu, grid);
    CHECK(f.ok());
    std::size_t held = 0;
    for (const Voxel& x : f->voxels) {
      held +=
          (x.world.z < kMoveWall - 0.05f && f->weight[x.index] > 0.0f) ? 1 : 0;
    }
    std::printf(
        "moving sphere, set %d (gone): %zu vertices and %zu weighted "
        "voxels off the wall\n",
        s, off_wall, held);
    CHECK(off_wall == 0 && held == 0);
  }
  std::printf(
      "moving sphere: covered %.3f at worst while it moves, %.3f once "
      "it stops\n",
      worst_moving, worst_still);
  CHECK(worst_moving >= 0.97f && worst_still >= 0.995f);
  return 0;
}

}  // namespace

int gpu_main(vkc::test::GpuContext& gpu) {
  if (still_scene(gpu, tsdf::IntegrationMode::Classic) != 0) return 1;
  if (still_scene(gpu, tsdf::IntegrationMode::Dynamic) != 0) return 1;
  if (moving_plate(gpu) != 0) return 1;
  if (moving_sphere(gpu) != 0) return 1;
  std::puts("tsdf_rim_distance: OK");
  return 0;
}

int main() {
  return vkc::test::run_on_gpu(vr::device_requirements(), gpu_main);
}
