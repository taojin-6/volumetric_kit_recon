// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// GPU test: the codec's round trip on a volume with ground truth. A mesh goes
// in through tsdf::MeshIntegrator, through the forward DCT, into v1 frame
// bytes, back out of them, through the inverse DCT into a fresh grid, and out
// through marching cubes -- and the decoded surface is measured against the
// SOURCE MESH, in both directions:
//
//   accuracy -- every decoded vertex's distance to the source surface;
//   coverage -- every source sample's distance to the nearest decoded vertex,
//               which is what a hole shows up in.
//
// A fused scan cannot be measured this way: its only reference is the volume
// before encoding, which is itself an estimate, and there is no ground-truth
// mesh beside room0. The same numbers for the UNCOMPRESSED volume's surface are
// the floor -- marching cubes' own discretization -- so what the codec adds is
// the difference. The test prints both, with the frame's size, and holds the
// default parameters to bounds a regression would cross. Exits 0 (skip) where
// no device is present.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <map>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "bitstream.hpp"
#include "dct_blocks.hpp"
#include "dct_transform.hpp"
#include "volumetric_kit/recon/codec/codec_params.hpp"
#include "volumetric_kit/recon/core/allocator.hpp"
#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/instance.hpp"
#include "volumetric_kit/recon/core/math/vector_types.hpp"
#include "volumetric_kit/recon/mesh/marching_cubes.hpp"
#include "volumetric_kit/recon/mesh/mesh.hpp"
#include "volumetric_kit/recon/tsdf/mesh_integrator.hpp"
#include "volumetric_kit/recon/volume/hash_types.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"
#include "volumetric_kit/recon/volume/voxel_grid.hpp"
#include "volumetric_kit/recon/volume/voxel_hash_map.hpp"

namespace vr = volumetric_kit::recon;
namespace vol = volumetric_kit::recon::volume;
namespace ts = volumetric_kit::recon::tsdf;
namespace codec = volumetric_kit::recon::codec;
namespace cd = volumetric_kit::recon::codec::detail;

#define CHECK(cond)                                                        \
  do {                                                                     \
    if (!(cond)) {                                                         \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      return 1;                                                            \
    }                                                                      \
  } while (0)

namespace {

struct Mesh {
  std::vector<vr::Vec3f> v;
  std::vector<std::uint32_t> i;
  std::uint32_t vertex_count() const {
    return static_cast<std::uint32_t>(v.size());
  }
  std::uint32_t triangle_count() const {
    return static_cast<std::uint32_t>(i.size() / 3);
  }
};

vr::Vec3f corner(const Mesh& m, std::size_t t, int k) {
  return m.v[m.i[3 * t + k]];
}

// Closest point on a triangle (barycentric projection with an edge clamp).
vr::Vec3f closest_point(vr::Vec3f p, vr::Vec3f a, vr::Vec3f b, vr::Vec3f c) {
  const vr::Vec3f ab = b - a;
  const vr::Vec3f ac = c - a;
  const vr::Vec3f n = vr::cross(ab, ac);
  const float nn = vr::dot(n, n);
  const vr::Vec3f q = p - n * (vr::dot(n, p - a) / nn);
  const float u = vr::dot(n, vr::cross(c - b, q - b)) / nn;
  const float v = vr::dot(n, vr::cross(a - c, q - c)) / nn;
  const float w = vr::dot(n, vr::cross(ab, q - a)) / nn;
  if (u >= 0.0f && v >= 0.0f && w >= 0.0f) return q;
  auto on_segment = [](vr::Vec3f pt, vr::Vec3f s0, vr::Vec3f s1) {
    const vr::Vec3f d = s1 - s0;
    float t = vr::dot(pt - s0, d) / vr::dot(d, d);
    t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
    return s0 + d * t;
  };
  const vr::Vec3f e[3] = {on_segment(p, a, b), on_segment(p, b, c),
                          on_segment(p, c, a)};
  vr::Vec3f best = e[0];
  for (const vr::Vec3f& q2 : e) {
    if (vr::dot(p - q2, p - q2) < vr::dot(p - best, p - best)) best = q2;
  }
  return best;
}

// Point-to-mesh distance up to `reach`, through a hash of `cell`-sized cells
// each holding the triangles whose bounding box overlaps it. A query walks
// outward ring by ring and stops as soon as no cell further out could hold
// anything nearer -- after ring r, every unvisited cell is at least r cells
// away -- so the usual sub-voxel answer costs the 27 cells around the point.
// Past `reach` the distance reads as `reach`, which every bound below is under,
// so a miss can only fail a check, never pass one.
class TriangleHash {
 public:
  TriangleHash(std::vector<std::array<vr::Vec3f, 3>> tris, float cell,
               float reach)
      : tris_(std::move(tris)), cell_(cell), reach_(reach) {
    for (std::size_t t = 0; t < tris_.size(); ++t) {
      const auto& [a, b, c] = tris_[t];
      const Key lo = key(glm::min(a, glm::min(b, c)));
      const Key hi = key(glm::max(a, glm::max(b, c)));
      for (int z = lo[2]; z <= hi[2]; ++z) {
        for (int y = lo[1]; y <= hi[1]; ++y) {
          for (int x = lo[0]; x <= hi[0]; ++x) {
            cells_[pack({x, y, z})].push_back(static_cast<std::uint32_t>(t));
          }
        }
      }
    }
  }
  float distance(vr::Vec3f p) const {
    float best = reach_;
    const Key k = key(p);
    const int rings = int(std::ceil(reach_ / cell_));
    for (int r = 0; r <= rings; ++r) {
      for (int dz = -r; dz <= r; ++dz) {
        for (int dy = -r; dy <= r; ++dy) {
          for (int dx = -r; dx <= r; ++dx) {
            if (std::max({std::abs(dx), std::abs(dy), std::abs(dz)}) != r) {
              continue;  // inside the shell: an earlier ring visited it
            }
            auto it = cells_.find(pack({k[0] + dx, k[1] + dy, k[2] + dz}));
            if (it == cells_.end()) continue;
            for (std::uint32_t t : it->second) {
              const auto& [a, b, c] = tris_[t];
              best = std::fmin(best, vr::length(p - closest_point(p, a, b, c)));
            }
          }
        }
      }
      if (best <= float(r) * cell_) break;
    }
    return best;
  }

 private:
  using Key = std::array<int, 3>;
  Key key(vr::Vec3f p) const {
    return {int(std::floor(p.x / cell_)), int(std::floor(p.y / cell_)),
            int(std::floor(p.z / cell_))};
  }
  // 21 bits a component, offset to unsigned, for cell indices well inside
  // +-2^20.
  static std::uint64_t pack(const Key& k) {
    auto u = [](int v) { return std::uint64_t(std::uint32_t(v + (1 << 20))); };
    return (u(k[0]) << 42) | (u(k[1]) << 21) | u(k[2]);
  }
  std::vector<std::array<vr::Vec3f, 3>> tris_;
  float cell_;
  float reach_;
  std::unordered_map<std::uint64_t, std::vector<std::uint32_t>> cells_;
};

std::vector<std::array<vr::Vec3f, 3>> triangles_of(const Mesh& m) {
  std::vector<std::array<vr::Vec3f, 3>> out;
  for (std::size_t t = 0; t < m.triangle_count(); ++t) {
    out.push_back({corner(m, t, 0), corner(m, t, 1), corner(m, t, 2)});
  }
  return out;
}

std::vector<std::array<vr::Vec3f, 3>> triangles_of(const vr::mesh::Mesh& m) {
  std::vector<std::array<vr::Vec3f, 3>> out;
  for (std::size_t t = 0; t < m.triangle_count(); ++t) {
    out.push_back({m.vertices[m.indices[3 * t]].position,
                   m.vertices[m.indices[3 * t + 1]].position,
                   m.vertices[m.indices[3 * t + 2]].position});
  }
  return out;
}

void orient_outward(Mesh& m, vr::Vec3f centre) {
  for (std::size_t t = 0; t < m.triangle_count(); ++t) {
    const vr::Vec3f a = corner(m, t, 0), b = corner(m, t, 1),
                    c = corner(m, t, 2);
    if (vr::dot(vr::cross(b - a, c - a), (a + b + c) / 3.0f - centre) < 0.0f) {
      std::swap(m.i[3 * t + 1], m.i[3 * t + 2]);
    }
  }
}

// An icosahedron subdivided `levels` times and pushed onto the sphere: smooth,
// closed, and made of triangles several voxels across.
Mesh icosphere(vr::Vec3f c, float r, int levels) {
  const float g = (1.0f + std::sqrt(5.0f)) / 2.0f;
  Mesh m;
  m.v = {{-1, g, 0}, {1, g, 0}, {-1, -g, 0}, {1, -g, 0},
         {0, -1, g}, {0, 1, g}, {0, -1, -g}, {0, 1, -g},
         {g, 0, -1}, {g, 0, 1}, {-g, 0, -1}, {-g, 0, 1}};
  m.i = {0, 11, 5,  0, 5,  1, 0, 1, 7, 0, 7,  10, 0, 10, 11, 1, 5, 9, 5, 11,
         4, 11, 10, 2, 10, 7, 6, 7, 1, 8, 3,  9,  4, 3,  4,  2, 3, 2, 6, 3,
         6, 8,  3,  8, 9,  4, 9, 5, 2, 4, 11, 6,  2, 10, 8,  6, 7, 9, 8, 1};
  for (vr::Vec3f& p : m.v) p = vr::normalize(p);
  for (int level = 0; level < levels; ++level) {
    std::map<std::pair<std::uint32_t, std::uint32_t>, std::uint32_t> mid;
    auto midpoint = [&](std::uint32_t a, std::uint32_t b) {
      const auto key = std::minmax(a, b);
      auto it = mid.find(key);
      if (it != mid.end()) return it->second;
      m.v.push_back(vr::normalize(m.v[a] + m.v[b]));
      const auto idx = static_cast<std::uint32_t>(m.v.size() - 1);
      mid.emplace(key, idx);
      return idx;
    };
    std::vector<std::uint32_t> next;
    for (std::size_t t = 0; t < m.i.size() / 3; ++t) {
      const std::uint32_t a = m.i[3 * t], b = m.i[3 * t + 1],
                          d = m.i[3 * t + 2];
      const std::uint32_t ab = midpoint(a, b), bd = midpoint(b, d),
                          da = midpoint(d, a);
      next.insert(next.end(), {a, ab, da, b, bd, ab, d, da, bd, ab, bd, da});
    }
    m.i = std::move(next);
  }
  for (vr::Vec3f& p : m.v) p = c + r * p;
  orient_outward(m, c);
  return m;
}

// A cube with one corner pushed in: flat faces, 90-degree edges, and a concave
// dent -- the sharp features a K-coefficient block transform smooths.
Mesh dented_cube(vr::Vec3f origin, float side) {
  Mesh m;
  for (int k = 0; k < 8; ++k) {
    const vr::Vec3f unit(float((k == 1 || k == 2 || k == 5 || k == 6)),
                         float((k == 2 || k == 3 || k == 6 || k == 7)),
                         float(k >= 4));
    m.v.push_back(origin + side * unit);
  }
  const std::uint32_t quads[6][4] = {{0, 1, 2, 3}, {4, 5, 6, 7}, {0, 1, 5, 4},
                                     {3, 2, 6, 7}, {0, 3, 7, 4}, {1, 2, 6, 5}};
  for (const auto& q : quads) {
    m.i.insert(m.i.end(), {q[0], q[1], q[2], q[0], q[2], q[3]});
  }
  orient_outward(m, origin + vr::Vec3f(0.5f * side));
  m.v[6] = origin + vr::Vec3f(0.6f * side);
  return m;
}

struct SurfaceError {
  double mean = 0.0, rms = 0.0, max = 0.0;
};

SurfaceError summarize(const std::vector<float>& d) {
  SurfaceError e;
  for (float x : d) {
    e.mean += x;
    e.rms += double(x) * x;
    e.max = std::max(e.max, double(x));
  }
  if (!d.empty()) {
    e.mean /= double(d.size());
    e.rms = std::sqrt(e.rms / double(d.size()));
  }
  return e;
}

// How far a distance is measured before it reads as "too far": 2 cm, several
// times any bound checked below.
constexpr float kReach = 0.02f;

// Accuracy: each extracted vertex position's distance to the source surface.
// Unique positions, since the mesher emits three vertices per triangle and so
// every position about six times over.
SurfaceError accuracy(const vr::mesh::Mesh& got, const Mesh& src, float voxel) {
  const TriangleHash source(triangles_of(src), voxel, kReach);
  std::vector<std::array<float, 3>> positions;
  positions.reserve(got.vertices.size());
  for (const vr::mesh::Vertex& v : got.vertices) {
    positions.push_back({v.position.x, v.position.y, v.position.z});
  }
  std::sort(positions.begin(), positions.end());
  positions.erase(std::unique(positions.begin(), positions.end()),
                  positions.end());
  std::vector<float> d;
  d.reserve(positions.size());
  for (const auto& q : positions) {
    d.push_back(source.distance(vr::Vec3f(q[0], q[1], q[2])));
  }
  return summarize(d);
}

// Coverage: how far each point of the source surface is from the extracted
// surface, sampled on a barycentric lattice over every source triangle fine
// enough that no sample is more than ~2 voxels from the next. Measured to the
// extracted TRIANGLES, so a surface without holes reads a fraction of a voxel
// and a hole reads its own radius.
SurfaceError coverage(const Mesh& src, const vr::mesh::Mesh& got, float voxel) {
  const TriangleHash extracted(triangles_of(got), voxel, kReach);
  std::vector<float> d;
  for (std::size_t t = 0; t < src.triangle_count(); ++t) {
    const vr::Vec3f a = corner(src, t, 0), b = corner(src, t, 1),
                    c = corner(src, t, 2);
    const float longest =
        std::max({vr::length(b - a), vr::length(c - b), vr::length(a - c)});
    const int n = std::max(1, int(std::ceil(longest / (2.0f * voxel))));
    for (int i = 0; i <= n; ++i) {
      for (int j = 0; i + j <= n; ++j) {
        const vr::Vec3f p =
            a + (float(i) / n) * (b - a) + (float(j) / n) * (c - a);
        d.push_back(extracted.distance(p));
      }
    }
  }
  return summarize(d);
}

struct RoundTrip {
  std::uint32_t blocks = 0;
  std::size_t frame_bytes = 0;
  std::size_t triangles_ref = 0, triangles_dec = 0;
  SurfaceError ref_accuracy, ref_coverage, dec_accuracy, dec_coverage;
};

vol::VoxelGridParams grid_params() {
  // Production resolution -- 5 mm voxels, 8-voxel blocks (the codec's only
  // block size), a 40 mm band -- over a heap sized for these meshes.
  vol::VoxelGridParams gp{};
  gp.voxel_size = 0.005f;
  gp.block_size = 8;
  gp.voxels_per_block = 512;
  gp.trunc_dist = 0.04f;
  gp.bucket_size = 8;
  gp.num_buckets = 2048;
  gp.num_blocks = 16384;
  gp.max_chain = 128;
  return gp;
}

struct Tools {
  vr::Device& device;
  vr::Allocator& allocator;
  ts::MeshIntegrator& integrator;
  cd::DctTransform& dct;
  vr::mesh::MarchingCubes& mc;
};

// With `measure_floor` false the uncompressed surface is not measured: a second
// run over the same mesh has the same one, so its figures are copied instead.
int round_trip(Tools& tools, const Mesh& src, const codec::CodecParams& params,
               bool measure_floor, RoundTrip& out) {
  const vol::VoxelGridParams gp = grid_params();
  const vol::AttributeSpec attrs[] = {{"tsdf", sizeof(float)},
                                      {"weight", sizeof(float)}};

  // Mesh -> TSDF.
  vr::Result<vol::VoxelBlockGrid> a_r =
      vol::VoxelBlockGrid::create(tools.device, tools.allocator, gp, attrs, 2);
  CHECK(a_r.ok());
  vol::VoxelBlockGrid a = std::move(a_r).value();
  vr::Result<std::uint32_t> failed = a.map().allocate_from_triangles(
      src.v.data(), src.vertex_count(), src.i.data(), src.triangle_count());
  CHECK(failed.ok() && failed.value() == 0);
  CHECK(tools.integrator
            .integrate(a, src.v.data(), src.vertex_count(), src.i.data(),
                       src.triangle_count(), {ts::MeshSdfMode::Signed, 1.5f})
            .ok());

  vr::Result<vr::mesh::Mesh> ref = tools.mc.extract_host(a);
  CHECK(ref.ok());
  CHECK(!ref.value().empty());

  // TSDF -> frame bytes, blocks in the frame's order.
  vr::Result<std::vector<vol::BlockIndex>> active_r =
      a.map().compact_active_blocks();
  CHECK(active_r.ok());
  std::vector<vol::BlockIndex> active = std::move(active_r).value();
  std::sort(active.begin(), active.end(),
            [](const vol::BlockIndex& x, const vol::BlockIndex& y) {
              return cd::coord_less(x.coord, y.coord);
            });
  cd::IntraFrame frame;
  frame.voxel_size = gp.voxel_size;
  for (const vol::BlockIndex& b : active) frame.coords.push_back(b.coord);
  CHECK(tools.dct.forward(a, a.block_list(active), params, frame.blocks).ok());
  vr::Result<std::vector<std::uint8_t>> bytes = cd::write_intra_frame(frame);
  CHECK(bytes.ok());

  // Frame bytes -> a fresh grid.
  vr::Result<cd::IntraFrame> decoded =
      cd::read_intra_frame(bytes.value().data(), bytes.value().size(),
                           static_cast<std::uint32_t>(gp.num_blocks));
  CHECK(decoded.ok());
  CHECK(decoded.value().coords == frame.coords);

  vr::Result<vol::VoxelBlockGrid> b_r =
      vol::VoxelBlockGrid::create(tools.device, tools.allocator, gp, attrs, 2);
  CHECK(b_r.ok());
  vol::VoxelBlockGrid b = std::move(b_r).value();
  std::vector<vol::BlockIndex> want;
  for (const vr::Vec3i& c : decoded.value().coords) want.push_back({c, 0});
  vr::Result<std::uint32_t> b_failed =
      b.map().allocate(want.data(), static_cast<std::uint32_t>(want.size()));
  CHECK(b_failed.ok() && b_failed.value() == 0);
  vr::Result<std::vector<vol::BlockIndex>> b_active =
      b.map().compact_active_blocks();
  CHECK(b_active.ok());
  std::map<std::tuple<int, int, int>, std::int32_t> ptr_of;
  for (const vol::BlockIndex& blk : b_active.value()) {
    ptr_of[{blk.coord.x, blk.coord.y, blk.coord.z}] = blk.ptr;
  }
  std::vector<vol::BlockIndex> in_order;
  for (const vr::Vec3i& c : decoded.value().coords) {
    in_order.push_back({c, ptr_of.at({c.x, c.y, c.z})});
  }
  CHECK(tools.dct.inverse(b, b.block_list(in_order), decoded.value().blocks)
            .ok());

  vr::Result<vr::mesh::Mesh> dec = tools.mc.extract_host(b);
  CHECK(dec.ok());
  CHECK(!dec.value().empty());

  out.blocks = static_cast<std::uint32_t>(active.size());
  out.frame_bytes = bytes.value().size();
  out.triangles_ref = ref.value().triangle_count();
  out.triangles_dec = dec.value().triangle_count();
  if (measure_floor) {
    out.ref_accuracy = accuracy(ref.value(), src, gp.voxel_size);
    out.ref_coverage = coverage(src, ref.value(), gp.voxel_size);
  }
  out.dec_accuracy = accuracy(dec.value(), src, gp.voxel_size);
  out.dec_coverage = coverage(src, dec.value(), gp.voxel_size);
  return 0;
}

void print(const char* what, const codec::CodecParams& params,
           const RoundTrip& r, float voxel) {
  // Millimetres, and the raw tsdf + weight the frame replaces.
  const double raw = double(r.blocks) * 512.0 * 8.0;
  std::printf(
      "%-12s K=%3u  %5u blocks  %8zu B (%.1f%% of raw)  tris %zu -> %zu\n"
      "             uncompressed: accuracy mean %.3f rms %.3f max %.3f | "
      "coverage max %.3f mm\n"
      "             decoded:      accuracy mean %.3f rms %.3f max %.3f | "
      "coverage max %.3f mm  (max %.2f voxels)\n",
      what, params.coefficient_count, r.blocks, r.frame_bytes,
      100.0 * double(r.frame_bytes) / raw, r.triangles_ref, r.triangles_dec,
      1e3 * r.ref_accuracy.mean, 1e3 * r.ref_accuracy.rms,
      1e3 * r.ref_accuracy.max, 1e3 * r.ref_coverage.max,
      1e3 * r.dec_accuracy.mean, 1e3 * r.dec_accuracy.rms,
      1e3 * r.dec_accuracy.max, 1e3 * r.dec_coverage.max,
      r.dec_accuracy.max / voxel);
}

}  // namespace

int main() {
  vr::Result<vr::Instance> instance = vr::Instance::create({});
  if (!instance) {
    std::fprintf(stderr, "no Vulkan instance (%s); skipping\n",
                 instance.status().message().c_str());
    return 0;
  }
  vr::Result<VkPhysicalDevice> gpu = instance.value().select_physical_device();
  if (!gpu) {
    std::fprintf(stderr, "no compute-capable device (%s); skipping\n",
                 gpu.status().message().c_str());
    return 0;
  }
  vr::Result<vr::Device> device =
      vr::Device::create(instance.value(), gpu.value(), {});
  CHECK(device.ok());
  vr::Result<vr::Allocator> allocator =
      vr::Allocator::create(instance.value().handle(), device.value());
  CHECK(allocator.ok());
  vr::Result<ts::MeshIntegrator> integrator =
      ts::MeshIntegrator::create(device.value(), allocator.value());
  CHECK(integrator.ok());
  vr::Result<cd::DctTransform> dct =
      cd::DctTransform::create(device.value(), allocator.value());
  CHECK(dct.ok());
  vr::Result<vr::mesh::MarchingCubes> mc =
      vr::mesh::MarchingCubes::create(device.value(), allocator.value());
  CHECK(mc.ok());
  Tools tools{device.value(), allocator.value(), integrator.value(),
              dct.value(), mc.value()};
  const float voxel = grid_params().voxel_size;

  const Mesh sphere =
      icosphere(vr::Vec3f(0.0131f, -0.0217f, 0.0093f), 0.15f, 3);
  const Mesh cube = dented_cube(vr::Vec3f(-0.1127f, -0.0893f, -0.1011f), 0.22f);
  const codec::CodecParams defaults{};
  codec::CodecParams full = defaults;
  full.coefficient_count = 512;  // every coefficient: quantization alone

  RoundTrip sphere_default, sphere_full, cube_default;
  if (round_trip(tools, sphere, defaults, true, sphere_default) != 0) return 1;
  if (round_trip(tools, sphere, full, false, sphere_full) != 0) return 1;
  sphere_full.ref_accuracy = sphere_default.ref_accuracy;
  sphere_full.ref_coverage = sphere_default.ref_coverage;
  if (round_trip(tools, cube, defaults, true, cube_default) != 0) return 1;
  print("sphere", defaults, sphere_default, voxel);
  print("sphere", full, sphere_full, voxel);
  print("dented cube", defaults, cube_default, voxel);

  // Bounds sit at about twice what an M5 Max measured (Release, 5 mm voxels),
  // so they hold across devices and catch a regression, not rounding. The
  // figures themselves print above.
  //
  // The floor, before any compression: marching cubes over the exact field.
  // On the sphere it sits on the source to a hundredth of a voxel and covers
  // it to a tenth (measured 0.16 mm and 0.35 mm).
  CHECK(sphere_default.ref_accuracy.max < 0.1 * voxel);
  CHECK(sphere_default.ref_coverage.max < 0.2 * voxel);
  // On the dented cube the dent tilts the six triangles around it, which closes
  // six of the cube's edges to about 56 degrees, and marching cubes cuts a
  // wedge that thin back by up to a voxel (measured 1.54 mm and 7.09 mm).
  CHECK(cube_default.ref_accuracy.max < 0.5 * voxel);
  CHECK(cube_default.ref_coverage.max < 2.0 * voxel);

  // What the default parameters add, on a smooth surface (measured rms
  // 0.39 mm, max 1.42 mm, coverage 1.30 mm): under a sixth of a voxel rms,
  // half a voxel at worst, and no holes.
  CHECK(sphere_default.dec_accuracy.rms < 0.15 * voxel);
  CHECK(sphere_default.dec_accuracy.max < 0.5 * voxel);
  CHECK(sphere_default.dec_coverage.max < 0.5 * voxel);
  // Keeping every coefficient cannot be worse than keeping 32.
  CHECK(sphere_full.dec_accuracy.rms <= sphere_default.dec_accuracy.rms);
  // And on the sharp features (measured 3.11 mm and 9.86 mm).
  CHECK(cube_default.dec_accuracy.max < 1.0 * voxel);
  CHECK(cube_default.dec_coverage.max < 3.0 * voxel);
  // The frame is a small fraction of the raw tsdf + weight it replaces
  // (measured 0.8%).
  CHECK(double(sphere_default.frame_bytes) <
        0.02 * double(sphere_default.blocks) * 512.0 * 8.0);

  std::printf("codec_mesh_roundtrip: OK\n");
  return 0;
}
