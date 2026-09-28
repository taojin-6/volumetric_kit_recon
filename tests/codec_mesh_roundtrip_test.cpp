// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// GPU test: the codec's round trip on a volume with ground truth. A mesh goes
// in through tsdf::MeshIntegrator, through the forward DCT, into v1 frame
// bytes, back out of them, through the inverse DCT into a fresh grid, and out
// through marching cubes -- and the decoded surface is measured against the
// SOURCE MESH, in both directions:
//
//   accuracy -- every decoded vertex's distance to the source surface;
//   coverage -- every source sample's distance to the decoded triangles,
//               which is what a hole shows up in.
//
// A fused scan cannot be measured this way: its only reference is the volume
// before encoding, which is itself an estimate, and there is no ground-truth
// mesh beside room0. The same numbers for the UNCOMPRESSED volume's surface are
// the floor -- marching cubes' own discretization -- so what the codec adds is
// the difference. The test prints both, with the frame's size, and holds the
// default parameters to bounds a regression would cross. The frame itself must
// decode to exactly what was written: a surface is blind to a field scaled as a
// whole, so the distances alone would pass an entropy layer that scaled every
// coefficient. Exits 0 (skip) where no device is present.

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
#include "test_meshes.hpp"
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

using test_meshes::closest_point;
using test_meshes::corner;
using test_meshes::dented_cube;
using test_meshes::Mesh;
using test_meshes::orient_outward;

// Point-to-mesh distance up to `reach`, through a hash of `cell`-sized cells
// each holding the triangles whose bounding box overlaps it. A query walks
// outward ring by ring and stops as soon as no cell further out could hold
// anything nearer. A triangle listed in no visited cell lies outside all of
// them, so after ring r it is at least r cells away, plus the point's distance
// to the nearest face of its own cell. A near answer often costs one cell, and
// the usual sub-voxel one at most the 27 around the point.
// Past `reach` the distance reads as `reach`, which every bound below is under,
// so a miss can only fail a check, never pass one. Every point it sees must be
// finite: a NaN has no cell.
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
    // How far p is from the nearest face of its own cell.
    float margin = cell_;
    for (int a = 0; a < 3; ++a) {
      const float f = p[a] / cell_ - float(k[a]);
      margin = std::min(margin, std::min(f, 1.0f - f) * cell_);
    }
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
      if (best <= float(r) * cell_ + margin) break;
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

// How far a distance is measured before it reads as "too far": 4 cm, twice the
// widest bound checked below.
constexpr float kReach = 0.04f;
// The coverage lattice's spacing, in voxels. One unobserved voxel drops the
// eight cells around it, a hole about two voxels across. At half a voxel some
// sample lands inside it: clearing one surface voxel's mask bit on the sphere
// read at least 4.27 mm in each of 12 placements, against the 2.5 mm bound. At
// two voxels the samples straddled it in 3 of the 12, and the check passed.
constexpr float kCoverageSpacing = 0.5f;

bool finite(const vr::mesh::Mesh& m) {
  return std::all_of(
      m.vertices.begin(), m.vertices.end(), [](const vr::mesh::Vertex& v) {
        return std::isfinite(v.position.x) && std::isfinite(v.position.y) &&
               std::isfinite(v.position.z);
      });
}

// Accuracy: each extracted vertex position's distance to the source surface.
// Unique positions, since the mesher emits three vertices per triangle and so
// every position about six times over.
SurfaceError accuracy(const vr::mesh::Mesh& got, const TriangleHash& source) {
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
// enough that no sample is more than `kCoverageSpacing` voxels from the next.
// Measured to the extracted TRIANGLES, so a surface without holes reads a
// fraction of a voxel and a hole reads its own radius.
SurfaceError coverage(const Mesh& src, const vr::mesh::Mesh& got, float voxel) {
  const TriangleHash extracted(triangles_of(got), voxel, kReach);
  std::vector<float> d;
  for (std::size_t t = 0; t < src.triangle_count(); ++t) {
    const vr::Vec3f a = corner(src, t, 0), b = corner(src, t, 1),
                    c = corner(src, t, 2);
    const float longest =
        std::max({vr::length(b - a), vr::length(c - b), vr::length(a - c)});
    const int n =
        std::max(1, int(std::ceil(longest / (kCoverageSpacing * voxel))));
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

// One extracted surface, measured against its source.
struct Surface {
  std::size_t triangles = 0;
  SurfaceError accuracy, coverage;
};

// One source mesh at one set of params.
struct Coded {
  codec::CodecParams params;
  std::size_t frame_bytes = 0;
  Surface decoded;
};

// One source mesh, fused once: the uncompressed surface, then a decoded one
// per params.
struct RoundTrip {
  std::uint32_t blocks = 0;
  Surface uncompressed;
  std::vector<Coded> coded;
};

vol::VoxelGridParams grid_params() {
  // Production resolution -- the defaults' voxels, blocks (8, the codec's only
  // block size) and band -- over a heap sized for these meshes.
  vol::VoxelGridParams gp = vol::VoxelGridParams::defaults();
  gp.bucket_size = 8;
  gp.num_buckets = 2048;
  gp.num_blocks = gp.bucket_size * gp.num_buckets;
  return gp;
}

// The raw tsdf + weight a frame of `blocks` blocks replaces.
double raw_bytes(std::uint32_t blocks) {
  return double(blocks) * codec::kVoxelsPerBlock * 2.0 * sizeof(float);
}

struct Tools {
  vr::Device& device;
  vr::Allocator& allocator;
  ts::MeshIntegrator& integrator;
  cd::DctTransform& dct;
  vr::mesh::MarchingCubes& mc;
};

int measure(const Mesh& src, const TriangleHash& source,
            const vr::mesh::Mesh& got, float voxel, Surface& out) {
  CHECK(!got.empty());
  // A NaN position has no cell to hash into and no order to sort by.
  CHECK(finite(got));
  out.triangles = got.triangle_count();
  out.accuracy = accuracy(got, source);
  out.coverage = coverage(src, got, voxel);
  return 0;
}

// `a`'s blocks, in the frame's order, through the forward DCT, frame bytes, a
// fresh grid and the inverse DCT, to a mesh measured against `src`.
//
// TODO(codec): drive this through `Encoder` / `Decoder` once they land (the
// 2026-09-26 entry's third PR). Until then the test sorts, allocates and
// remaps the blocks itself, so it checks its own copy of what they will do.
int code(Tools& tools, vol::VoxelBlockGrid& a,
         const std::vector<vol::BlockIndex>& sorted, const Mesh& src,
         const TriangleHash& source, Coded& out) {
  const vol::VoxelGridParams gp = grid_params();

  // TSDF -> frame bytes.
  cd::IntraFrame frame;
  frame.voxel_size = gp.voxel_size;
  for (const vol::BlockIndex& b : sorted) frame.coords.push_back(b.coord);
  CHECK(tools.dct.forward(a, a.block_list(sorted), out.params, frame.blocks)
            .ok());
  vr::Result<std::vector<std::uint8_t>> bytes = cd::write_intra_frame(frame);
  CHECK(bytes.ok());

  // Frame bytes -> exactly the frame written: the entropy layer is lossless.
  vr::Result<cd::IntraFrame> decoded_r =
      cd::read_intra_frame(bytes.value().data(), bytes.value().size(),
                           static_cast<std::uint32_t>(gp.num_blocks));
  CHECK(decoded_r.ok());
  const cd::IntraFrame& decoded = decoded_r.value();
  CHECK(decoded.voxel_size == frame.voxel_size);
  CHECK(decoded.coords == frame.coords);
  CHECK(decoded.blocks.trunc_dist == frame.blocks.trunc_dist);
  CHECK(decoded.blocks.params.coefficient_count ==
        frame.blocks.params.coefficient_count);
  CHECK(decoded.blocks.params.dc_step == frame.blocks.params.dc_step);
  CHECK(decoded.blocks.params.ac_step == frame.blocks.params.ac_step);
  CHECK(decoded.blocks.coefficients == frame.blocks.coefficients);
  CHECK(decoded.blocks.masks == frame.blocks.masks);

  // -> a fresh grid.
  const vol::AttributeSpec attrs[] = {{"tsdf", sizeof(float)},
                                      {"weight", sizeof(float)}};
  vr::Result<vol::VoxelBlockGrid> b_r =
      vol::VoxelBlockGrid::create(tools.device, tools.allocator, gp, attrs, 2);
  CHECK(b_r.ok());
  vol::VoxelBlockGrid b = std::move(b_r).value();
  std::vector<vol::BlockIndex> want;
  for (const vr::Vec3i& c : decoded.coords) want.push_back({c, 0});
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
  for (const vr::Vec3i& c : decoded.coords) {
    auto it = ptr_of.find({c.x, c.y, c.z});
    CHECK(it != ptr_of.end());
    in_order.push_back({c, it->second});
  }
  CHECK(tools.dct.inverse(b, b.block_list(in_order), decoded.blocks).ok());

  vr::Result<vr::mesh::Mesh> dec = tools.mc.extract_host(b);
  CHECK(dec.ok());
  out.frame_bytes = bytes.value().size();
  return measure(src, source, dec.value(), gp.voxel_size, out.decoded);
}

// `src` through the mesh integrator once, its uncompressed surface measured,
// then coded at each of `out.coded`'s params.
int round_trip(Tools& tools, const Mesh& src, RoundTrip& out) {
  const vol::VoxelGridParams gp = grid_params();
  const vol::AttributeSpec attrs[] = {{"tsdf", sizeof(float)},
                                      {"weight", sizeof(float)}};

  // Mesh -> TSDF: every triangle takes part.
  vr::Result<vol::VoxelBlockGrid> a_r =
      vol::VoxelBlockGrid::create(tools.device, tools.allocator, gp, attrs, 2);
  CHECK(a_r.ok());
  vol::VoxelBlockGrid a = std::move(a_r).value();
  vr::Result<std::uint32_t> failed = a.map().allocate_from_triangles(
      src.v.data(), src.vertex_count(), src.i.data(), src.triangle_count());
  CHECK(failed.ok() && failed.value() == 0);
  vr::Result<ts::MeshIntegrateStats> stats = tools.integrator.integrate(
      a, src.v.data(), src.vertex_count(), src.i.data(), src.triangle_count(),
      {ts::MeshSdfMode::Signed, 1.5f});
  CHECK(stats.ok());
  CHECK(stats.value().triangles == src.triangle_count());

  const TriangleHash source(triangles_of(src), gp.voxel_size, kReach);
  vr::Result<vr::mesh::Mesh> ref = tools.mc.extract_host(a);
  CHECK(ref.ok());
  if (measure(src, source, ref.value(), gp.voxel_size, out.uncompressed) != 0) {
    return 1;
  }

  // The blocks the integrator wrote, in the frame's order.
  vr::Result<std::vector<vol::BlockIndex>> active_r =
      a.map().compact_active_blocks();
  CHECK(active_r.ok());
  std::vector<vol::BlockIndex> sorted = std::move(active_r).value();
  CHECK(sorted.size() == stats.value().blocks);
  std::sort(sorted.begin(), sorted.end(),
            [](const vol::BlockIndex& x, const vol::BlockIndex& y) {
              return cd::coord_less(x.coord, y.coord);
            });
  out.blocks = static_cast<std::uint32_t>(sorted.size());

  for (Coded& c : out.coded) {
    if (code(tools, a, sorted, src, source, c) != 0) return 1;
  }
  return 0;
}

void print(const char* what, const RoundTrip& r, float voxel) {
  auto line = [voxel](const char* label, const Surface& s) {
    std::printf(
        "             %-13s accuracy mean %.3f rms %.3f max %.3f | "
        "coverage max %.3f mm = %.2f voxels\n",
        label, 1e3 * s.accuracy.mean, 1e3 * s.accuracy.rms,
        1e3 * s.accuracy.max, 1e3 * s.coverage.max, s.coverage.max / voxel);
  };
  std::printf("%-12s %5u blocks  tris %zu\n", what, r.blocks,
              r.uncompressed.triangles);
  line("uncompressed:", r.uncompressed);
  for (const Coded& c : r.coded) {
    std::printf("             K=%3u  %8zu B (%.1f%% of raw)  tris %zu\n",
                c.params.coefficient_count, c.frame_bytes,
                100.0 * double(c.frame_bytes) / raw_bytes(r.blocks),
                c.decoded.triangles);
    line("decoded:", c.decoded);
  }
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

  const Mesh sphere_mesh =
      icosphere(vr::Vec3f(0.0131f, -0.0217f, 0.0093f), 0.15f, 3);
  const Mesh cube_mesh =
      dented_cube(vr::Vec3f(-0.1127f, -0.0893f, -0.1011f), 0.22f);
  const codec::CodecParams defaults{};
  codec::CodecParams full = defaults;
  full.coefficient_count = codec::kVoxelsPerBlock;  // quantization alone

  RoundTrip sphere, cube;
  sphere.coded.push_back({defaults, 0, {}});
  sphere.coded.push_back({full, 0, {}});
  cube.coded.push_back({defaults, 0, {}});
  if (round_trip(tools, sphere_mesh, sphere) != 0) return 1;
  if (round_trip(tools, cube_mesh, cube) != 0) return 1;
  print("sphere", sphere, voxel);
  print("dented cube", cube, voxel);
  const Surface& sphere_floor = sphere.uncompressed;
  const Surface& sphere_default = sphere.coded[0].decoded;
  const Surface& sphere_full = sphere.coded[1].decoded;
  const Surface& cube_floor = cube.uncompressed;
  const Surface& cube_default = cube.coded[0].decoded;

  // Bounds sit at about twice what an M5 Max measured (Release, 5 mm voxels),
  // 1.8x at the tightest and rounded to a fraction of a voxel, so they hold
  // across devices and catch a regression, not rounding. The figures themselves
  // print above.
  //
  // The floor, before any compression: marching cubes over the exact field.
  // On the sphere it sits on the source to a thirtieth of a voxel and covers
  // it to a fourteenth (measured 0.155 mm and 0.353 mm).
  CHECK(sphere_floor.accuracy.max < 0.1 * voxel);
  CHECK(sphere_floor.coverage.max < 0.2 * voxel);
  // On the dented cube the dent tilts the six triangles around it, which closes
  // six of the cube's edges to about 56 degrees, and marching cubes cuts a
  // wedge that thin back by 1.4 voxels (measured 1.54 mm and 7.09 mm).
  CHECK(cube_floor.accuracy.max < 0.6 * voxel);
  CHECK(cube_floor.coverage.max < 3.0 * voxel);

  // What the default parameters add, on a smooth surface (measured rms
  // 0.39 mm, max 1.42 mm, coverage 1.30 mm): under a sixth of a voxel rms,
  // half a voxel at worst, and no holes.
  CHECK(sphere_default.accuracy.rms < 0.15 * voxel);
  CHECK(sphere_default.accuracy.max < 0.5 * voxel);
  CHECK(sphere_default.coverage.max < 0.5 * voxel);
  // Every coefficient, which leaves the quantization's error alone (measured
  // rms 0.22 mm). Bounded on its own rather than against K = 32: Parseval
  // bounds the field's error, not a vertex's after the clamp and marching
  // cubes, so fewer coefficients measuring better here would not be a bug.
  CHECK(sphere_full.accuracy.rms < 0.09 * voxel);
  // And on the sharp features (measured 3.11 mm and 9.86 mm).
  CHECK(cube_default.accuracy.max < 1.25 * voxel);
  CHECK(cube_default.coverage.max < 4.0 * voxel);
  // The frame is a small fraction of the raw tsdf + weight it replaces
  // (measured 0.8%).
  CHECK(double(sphere.coded[0].frame_bytes) < 0.02 * raw_bytes(sphere.blocks));

  std::printf("codec_mesh_roundtrip: OK\n");
  return 0;
}
