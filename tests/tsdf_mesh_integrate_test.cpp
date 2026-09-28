// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// GPU test for mesh -> TSDF. Every voxel of every block the band reaches is
// checked against a brute-force HOST reference that shares nothing with the
// kernel but the contract:
//
//   - distance: an independent closest point (barycentric projection with an
//     edge clamp, not the shader's region test), minimised over EVERY triangle
//     of the mesh -- so a bin that dropped a triangle diverges too;
//   - inside/outside: the generalized winding number (the solid angles the mesh
//     subtends, summed), not pseudonormals -- so the sign is checked against a
//     different definition of "inside" rather than against itself.
//
// Fixtures are chosen for what breaks sign tests. A regular tetrahedron's faces
// meet at 70.5 degrees, sharp enough that signing by the nearest FACE's normal
// -- the prior engine's rule -- is wrong past its edges; the test proves the
// fixture has such voxels before relying on it. A cube with one corner pushed
// in has concave edges and a concave vertex. An open quad has a rim. The soup
// (unwelded) tetrahedron must write the same bytes as the welded one, and the
// same mesh twice must write the same bytes. Then every refusal, each checked
// to leave the grid untouched. Exits 0 (skip) where no device is present.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "volumetric_kit/recon/core/allocator.hpp"
#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/instance.hpp"
#include "volumetric_kit/recon/core/math/vector_types.hpp"
#include "volumetric_kit/recon/core/vulkan.hpp"
#include "volumetric_kit/recon/tsdf/mesh_integrator.hpp"
#include "volumetric_kit/recon/volume/hash_types.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"
#include "volumetric_kit/recon/volume/voxel_coords.hpp"
#include "volumetric_kit/recon/volume/voxel_grid.hpp"
#include "volumetric_kit/recon/volume/voxel_hash_map.hpp"

namespace vr = volumetric_kit::recon;
namespace vol = volumetric_kit::recon::volume;
namespace ts = volumetric_kit::recon::tsdf;

#define CHECK(cond)                                                        \
  do {                                                                     \
    if (!(cond)) {                                                         \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      return 1;                                                            \
    }                                                                      \
  } while (0)

namespace {

using Coord = std::tuple<int, int, int>;

// Kernel and reference compute the same float distance two different ways; they
// agree to well under a micrometre at these scales.
constexpr float kTol = 2e-6f;
// Voxels this close to a decision boundary (the surface, or the band's edge)
// are not compared: which side a rounding error lands on is not the contract.
constexpr float kEdge = 1e-5f;

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

// ---- The independent reference --------------------------------------------

// Closest point on a triangle: barycentric projection with a clamp to the
// edges, a different formulation from the shader's region test.
vr::Vec3f closest_point(vr::Vec3f p, vr::Vec3f a, vr::Vec3f b, vr::Vec3f c) {
  const vr::Vec3f ab = b - a;
  const vr::Vec3f ac = c - a;
  const vr::Vec3f n = vr::cross(ab, ac);
  const float nn = vr::dot(n, n);
  const vr::Vec3f q = p - n * (vr::dot(n, p - a) / nn);
  const float u = vr::dot(n, vr::cross(c - b, q - b)) / nn;
  const float v = vr::dot(n, vr::cross(a - c, q - c)) / nn;
  const float w = vr::dot(n, vr::cross(ab, q - a)) / nn;
  if (u >= 0.0f && v >= 0.0f && w >= 0.0f) {
    return q;
  }
  auto on_segment = [](vr::Vec3f pt, vr::Vec3f s0, vr::Vec3f s1) {
    const vr::Vec3f d = s1 - s0;
    float t = vr::dot(pt - s0, d) / vr::dot(d, d);
    t = t < 0.0f ? 0.0f : (t > 1.0f ? 1.0f : t);
    return s0 + d * t;
  };
  const vr::Vec3f e0 = on_segment(p, a, b);
  const vr::Vec3f e1 = on_segment(p, b, c);
  const vr::Vec3f e2 = on_segment(p, c, a);
  const float d0 = vr::dot(p - e0, p - e0);
  const float d1 = vr::dot(p - e1, p - e1);
  const float d2 = vr::dot(p - e2, p - e2);
  if (d0 <= d1 && d0 <= d2) return e0;
  return (d1 <= d2) ? e1 : e2;
}

vr::Vec3f corner(const Mesh& m, std::size_t t, int k) {
  return m.v[m.i[3 * t + k]];
}

float nearest(vr::Vec3f p, const Mesh& m) {
  float best = 1e30f;
  for (std::size_t t = 0; t < m.triangle_count(); ++t) {
    const vr::Vec3f q =
        closest_point(p, corner(m, t, 0), corner(m, t, 1), corner(m, t, 2));
    best = std::fmin(best, vr::length(p - q));
  }
  return best;
}

// The generalized winding number: 1 inside a closed outward-wound mesh, 0
// outside (van Oosterom & Strackee's solid angle per triangle, in double).
double winding(vr::Vec3f p, const Mesh& m) {
  double total = 0.0;
  for (std::size_t t = 0; t < m.triangle_count(); ++t) {
    double A[3], B[3], C[3];
    for (int k = 0; k < 3; ++k) {
      A[k] = double(corner(m, t, 0)[k]) - p[k];
      B[k] = double(corner(m, t, 1)[k]) - p[k];
      C[k] = double(corner(m, t, 2)[k]) - p[k];
    }
    auto len = [](const double* x) {
      return std::sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
    };
    auto dot3 = [](const double* x, const double* y) {
      return x[0] * y[0] + x[1] * y[1] + x[2] * y[2];
    };
    const double bc[3] = {B[1] * C[2] - B[2] * C[1], B[2] * C[0] - B[0] * C[2],
                          B[0] * C[1] - B[1] * C[0]};
    const double la = len(A), lb = len(B), lc = len(C);
    const double num = dot3(A, bc);
    const double den =
        la * lb * lc + dot3(A, B) * lc + dot3(A, C) * lb + dot3(B, C) * la;
    total += 2.0 * std::atan2(num, den);
  }
  return total / (4.0 * 3.14159265358979323846);
}

float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

// ---- Fixtures
// ----------------------------------------------------------------

// Flip every face whose normal points toward `centre` -- correct for a convex
// solid, which is all it is used on (the dented cube is oriented before it is
// dented; moving a vertex does not change a winding).
void orient_outward(Mesh& m, vr::Vec3f centre) {
  for (std::size_t t = 0; t < m.triangle_count(); ++t) {
    const vr::Vec3f a = corner(m, t, 0), b = corner(m, t, 1),
                    c = corner(m, t, 2);
    if (vr::dot(vr::cross(b - a, c - a), (a + b + c) / 3.0f - centre) < 0.0f) {
      std::swap(m.i[3 * t + 1], m.i[3 * t + 2]);
    }
  }
}

// A regular tetrahedron, edge 2*sqrt(2)*s, centred at c.
Mesh tetrahedron(vr::Vec3f c, float s) {
  Mesh m;
  m.v = {c + s * vr::Vec3f(1, 1, 1), c + s * vr::Vec3f(1, -1, -1),
         c + s * vr::Vec3f(-1, 1, -1), c + s * vr::Vec3f(-1, -1, 1)};
  m.i = {0, 1, 2, 0, 1, 3, 0, 2, 3, 1, 2, 3};
  orient_outward(m, c);
  return m;
}

// A cube of side `side` at `origin`, 12 triangles, with its (1,1,1) corner
// pushed in to 0.6 of the way along the diagonal: concave edges and a concave
// vertex, the case a convex fixture cannot reach.
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

// The tetrahedron with one face re-tessellated so that two of its corners each
// sit in a fan of many thin triangles from that face, and in a single triangle
// from each of the other two. The face's ANGLE at each corner is
// unchanged, so the angle-weighted pseudonormal is too -- but an unweighted sum
// counts the fanned face `fan + 1` times over and tips the corner's normal far
// enough to sign part of its region wrongly. Every new vertex is interior to
// the face, so no edge of the tetrahedron is split.
Mesh fanned_tetrahedron(vr::Vec3f c, float s, int fan) {
  Mesh m = tetrahedron(c, s);
  // Keep the other three faces; rebuild the one that uses corners 0, 1 and 2.
  std::vector<std::uint32_t> kept;
  std::uint32_t a = 0, b = 0, d = 0;
  for (std::size_t t = 0; t < m.triangle_count(); ++t) {
    const std::uint32_t* f = &m.i[3 * t];
    bool has[3] = {false, false, false};
    for (int k = 0; k < 3; ++k) {
      if (f[k] < 3) has[f[k]] = true;
    }
    if (has[0] && has[1] && has[2]) {
      a = f[0];
      b = f[1];
      d = f[2];  // this face's own winding, which the rebuild keeps
    } else {
      kept.insert(kept.end(), f, f + 3);
    }
  }
  m.i = kept;
  const vr::Vec3f pa = m.v[a], pb = m.v[b], pd = m.v[d];
  // A chain of interior points on a segment parallel to b-d, about a third of
  // the way from a; a fan from a over it, and a fan from b over the rest.
  std::vector<std::uint32_t> chain;
  for (int j = 1; j < fan; ++j) {
    const float u = float(j) / float(fan);
    m.v.push_back(pa + 0.35f * ((1.0f - u) * (pb - pa) + u * (pd - pa)));
    chain.push_back(static_cast<std::uint32_t>(m.v.size() - 1));
  }
  std::vector<std::uint32_t> rim = {b};
  rim.insert(rim.end(), chain.begin(), chain.end());
  rim.push_back(d);
  for (std::size_t j = 0; j + 1 < rim.size(); ++j) {
    m.i.insert(m.i.end(), {a, rim[j], rim[j + 1]});
  }
  // The quadrilateral strip between the chain and edge b-d, fanned from b.
  std::vector<std::uint32_t> back = chain;
  back.push_back(d);
  for (std::size_t j = 0; j + 1 < back.size(); ++j) {
    m.i.insert(m.i.end(), {b, back[j + 1], back[j]});
  }
  orient_outward(m, c);
  return m;
}

// The same triangles with no shared vertex -- how an STL stores a mesh.
Mesh soup(const Mesh& m) {
  Mesh s;
  for (std::size_t t = 0; t < m.triangle_count(); ++t) {
    for (int k = 0; k < 3; ++k) {
      s.v.push_back(corner(m, t, k));
      s.i.push_back(static_cast<std::uint32_t>(s.v.size() - 1));
    }
  }
  return s;
}

// ---- Grid plumbing ---------------------------------------------------------

std::vector<vol::BlockIndex> active_blocks(vol::VoxelBlockGrid& grid) {
  vr::Result<std::vector<vol::BlockIndex>> a =
      grid.map().compact_active_blocks();
  return a.ok() ? std::move(a).value() : std::vector<vol::BlockIndex>{};
}

float* attr(vol::VoxelBlockGrid& grid, const char* name) {
  return static_cast<float*>(
      const_cast<void*>(grid.attribute(name).value().buffer->mapped()));
}

std::size_t attr_count(vol::VoxelBlockGrid& grid, const char* name) {
  return static_cast<std::size_t>(grid.attribute(name).value().element_count);
}

// Every tsdf + weight value, so a refusal can be shown to have written none.
std::vector<float> snapshot(vol::VoxelBlockGrid& grid) {
  std::vector<float> out;
  for (const char* name : {"tsdf", "weight"}) {
    const float* p = attr(grid, name);
    out.insert(out.end(), p, p + attr_count(grid, name));
  }
  return out;
}

// Each written block's voxels, keyed by coordinate: the heap hands a block a
// different slot in each grid, so bytes are compared block by block.
std::map<Coord, std::vector<float>> by_coord(vol::VoxelBlockGrid& grid) {
  std::map<Coord, std::vector<float>> out;
  const float* tsdf = attr(grid, "tsdf");
  const float* weight = attr(grid, "weight");
  const int vpb = grid.grid().voxels_per_block;
  for (const vol::BlockIndex& b : active_blocks(grid)) {
    std::vector<float>& vals = out[{b.coord.x, b.coord.y, b.coord.z}];
    vals.assign(tsdf + b.ptr, tsdf + b.ptr + vpb);
    vals.insert(vals.end(), weight + b.ptr, weight + b.ptr + vpb);
  }
  return out;
}

struct Expect {
  bool skip = false;
  bool observed = false;
  float value = 0.0f;
};

// Compare every voxel of every allocated block with `expect(p)`.
template <class F>
int verify(vol::VoxelBlockGrid& grid, const char* what, F expect,
           int* observed_count = nullptr) {
  const vol::VoxelGridParams& g = grid.grid();
  const float* tsdf = attr(grid, "tsdf");
  const float* weight = attr(grid, "weight");
  int observed = 0;
  for (const vol::BlockIndex& b : active_blocks(grid)) {
    const vr::Vec3i base = vol::block_to_voxel(b.coord, g);
    for (int local = 0; local < g.voxels_per_block; ++local) {
      const vr::Vec3i lc(local % g.block_size,
                         (local / g.block_size) % g.block_size,
                         local / (g.block_size * g.block_size));
      const vr::Vec3f p = vol::voxel_to_world(base + lc, g);
      const Expect e = expect(p);
      if (e.skip) continue;
      const float w = weight[b.ptr + local];
      const float t = tsdf[b.ptr + local];
      const bool ok = e.observed ? (w == 1.0f && std::fabs(t - e.value) <= kTol)
                                 : (w == 0.0f && t == 0.0f);
      if (!ok) {
        std::fprintf(stderr,
                     "FAIL %s: voxel (%.4f, %.4f, %.4f): got tsdf %.7f weight "
                     "%.1f, want %s %.7f\n",
                     what, p.x, p.y, p.z, t, w,
                     e.observed ? "observed" : "unobserved", e.value);
        return 1;
      }
      observed += e.observed ? 1 : 0;
    }
  }
  if (observed_count != nullptr) *observed_count = observed;
  return 0;
}

// The signed contract for a closed mesh: +-distance within the band, the sign
// from the winding number.
Expect signed_closed(vr::Vec3f p, const Mesh& m, float trunc) {
  Expect e;
  const float d = nearest(p, m);
  if (d < kEdge || std::fabs(d - trunc) < kEdge) {
    e.skip = true;
    return e;
  }
  e.observed = d <= trunc;
  if (e.observed) {
    const bool inside = winding(p, m) > 0.5;
    e.value = inside ? -d : d;
  }
  return e;
}

// Could the nearest-FACE rule get this voxel's sign wrong? It can wherever a
// face tied for nearest -- every face meeting at the closest edge or vertex --
// puts the voxel on the wrong side of its own plane, since which of the tied
// faces a loop keeps is arbitrary.
bool face_rule_can_fail(vr::Vec3f p, const Mesh& m, bool inside) {
  const float d = nearest(p, m);
  for (std::size_t t = 0; t < m.triangle_count(); ++t) {
    const vr::Vec3f a = corner(m, t, 0), b = corner(m, t, 1),
                    c = corner(m, t, 2);
    const vr::Vec3f q = closest_point(p, a, b, c);
    if (vr::length(p - q) > d + 1e-6f) continue;
    const bool face_says_inside =
        vr::dot(p - q, vr::cross(b - a, c - a)) < 0.0f;
    if (face_says_inside != inside) return true;
  }
  return false;
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
  vr::Result<vr::Device> device_r =
      vr::Device::create(instance.value(), gpu.value(), {});
  CHECK(device_r.ok());
  vr::Device& device = device_r.value();
  vr::Result<vr::Allocator> allocator_r =
      vr::Allocator::create(instance.value().handle(), device);
  CHECK(allocator_r.ok());
  vr::Allocator& allocator = allocator_r.value();

  // 10 mm voxels in 8-voxel blocks, a 40 mm (4-voxel) band: coarse enough that
  // the brute-force reference stays quick, with a band several voxels deep.
  vol::VoxelGridParams gp{};
  gp.voxel_size = 0.01f;
  gp.block_size = 8;
  gp.voxels_per_block = 512;
  gp.trunc_dist = 0.04f;
  gp.bucket_size = 8;
  gp.num_buckets = 1024;
  gp.num_blocks = 8192;
  gp.max_chain = 128;
  const float trunc = gp.trunc_dist;

  const vol::AttributeSpec attrs[] = {{"tsdf", sizeof(float)},
                                      {"weight", sizeof(float)}};
  vr::Result<vol::VoxelBlockGrid> grid_r =
      vol::VoxelBlockGrid::create(device, allocator, gp, attrs, 2);
  CHECK(grid_r.ok());
  vol::VoxelBlockGrid grid = std::move(grid_r).value();

  vr::Result<ts::MeshIntegrator> integ_r =
      ts::MeshIntegrator::create(device, allocator);
  CHECK(integ_r.ok());
  ts::MeshIntegrator integ = std::move(integ_r).value();
  CHECK(integ.valid());

  // Allocate `m`'s band into a cleared grid, then write it.
  auto convert = [&](const Mesh& m, const ts::MeshSdfParams& params)
      -> vr::Result<ts::MeshIntegrateStats> {
    VR_TRY(grid.clear());
    VR_ASSIGN(std::uint32_t failed, grid.map().allocate_from_triangles(
                                        m.v.data(), m.vertex_count(),
                                        m.i.data(), m.triangle_count()));
    if (failed != 0) {
      return vr::Status::out_of_memory("allocate_from_triangles failed");
    }
    return integ.integrate(grid, m.v.data(), m.vertex_count(), m.i.data(),
                           m.triangle_count(), params);
  };
  const ts::MeshSdfParams kSigned{ts::MeshSdfMode::Signed, 1.5f};
  const ts::MeshSdfParams kShell{ts::MeshSdfMode::Shell, 1.5f};
  const float shell = 1.5f * gp.voxel_size;

  // ---- 1. Signed tetrahedron: the sharp-edge case -------------------------
  const Mesh tet = tetrahedron(vr::Vec3f(0.203f, 0.117f, -0.089f), 0.1f);
  vr::Result<ts::MeshIntegrateStats> tet_stats = convert(tet, kSigned);
  if (!tet_stats.ok()) {
    std::fprintf(stderr, "tetrahedron: %s\n",
                 tet_stats.status().message().c_str());
    return 1;
  }
  CHECK(tet_stats.value().triangles == 4);
  CHECK(tet_stats.value().boundary_edges == 0);
  // Every block the allocation made is written.
  CHECK(tet_stats.value().blocks == active_blocks(grid).size());
  CHECK(tet_stats.value().bin_entries >= tet_stats.value().blocks);

  int tet_observed = 0;
  if (verify(
          grid, "signed tetrahedron",
          [&](vr::Vec3f p) { return signed_closed(p, tet, trunc); },
          &tet_observed) != 0) {
    return 1;
  }
  CHECK(tet_observed > 1000);
  // The fixture earns its place: there are observed voxels the nearest-face
  // rule can sign wrongly, and every one of them passed above.
  int face_rule_hazards = 0;
  if (verify(grid, "tetrahedron hazard census", [&](vr::Vec3f p) {
        Expect e = signed_closed(p, tet, trunc);
        if (!e.skip && e.observed &&
            face_rule_can_fail(p, tet, winding(p, tet) > 0.5)) {
          ++face_rule_hazards;
        }
        return e;
      }) != 0) {
    return 1;
  }
  std::printf(
      "tetrahedron: %d observed voxels, %d the face rule can sign "
      "wrongly\n",
      tet_observed, face_rule_hazards);
  CHECK(face_rule_hazards > 0);

  // ---- 2. Same mesh, same bytes ------------------------------------------
  const std::map<Coord, std::vector<float>> tet_bytes = by_coord(grid);
  CHECK(integ
            .integrate(grid, tet.v.data(), tet.vertex_count(), tet.i.data(),
                       tet.triangle_count(), kSigned)
            .ok());
  CHECK(by_coord(grid) == tet_bytes);

  // ---- 3. A soup welds to the same mesh ------------------------------------
  // Without welding every edge of a soup is a rim, and a voxel nearest one is
  // left unobserved: the bytes would differ wherever an edge or vertex is
  // closest.
  const Mesh tet_soup = soup(tet);
  vr::Result<ts::MeshIntegrateStats> soup_stats = convert(tet_soup, kSigned);
  CHECK(soup_stats.ok());
  CHECK(soup_stats.value().boundary_edges == 0);
  CHECK(by_coord(grid) == tet_bytes);

  // Welding folds -0.0 onto +0.0: the two zeros are the same position, and an
  // exporter that computes one corner as 0 - 0 and another as 0 * -1 writes
  // both. A tetrahedron with vertices on the coordinate planes, whose soup has
  // one triangle's zeros negated.
  const Mesh zeros = tetrahedron(vr::Vec3f(0.1f, 0.1f, 0.1f), 0.1f);
  CHECK(convert(zeros, kSigned).ok());
  const std::map<Coord, std::vector<float>> zeros_bytes = by_coord(grid);
  Mesh zeros_soup = soup(zeros);
  int negated = 0;
  for (int k = 0; k < 3; ++k) {
    for (int axis = 0; axis < 3; ++axis) {
      if (zeros_soup.v[k][axis] == 0.0f) {
        zeros_soup.v[k][axis] = -0.0f;
        ++negated;
      }
    }
  }
  CHECK(negated > 0);
  vr::Result<ts::MeshIntegrateStats> zeros_stats = convert(zeros_soup, kSigned);
  if (!zeros_stats.ok()) {
    std::fprintf(stderr, "signed-zero soup: %s\n",
                 zeros_stats.status().message().c_str());
  }
  CHECK(zeros_stats.ok());
  CHECK(zeros_stats.value().boundary_edges == 0);
  CHECK(by_coord(grid) == zeros_bytes);

  // ---- 4. Signed dented cube: concave edges and a concave vertex ---------
  const Mesh cube = dented_cube(vr::Vec3f(0.013f, -0.021f, 0.017f), 0.3f);
  vr::Result<ts::MeshIntegrateStats> cube_stats = convert(cube, kSigned);
  CHECK(cube_stats.ok());
  CHECK(cube_stats.value().triangles == 12);
  CHECK(cube_stats.value().boundary_edges == 0);
  if (verify(grid, "signed dented cube",
             [&](vr::Vec3f p) { return signed_closed(p, cube, trunc); }) != 0) {
    return 1;
  }

  // A sharp corner in a fan of thin triangles from one face: only the ANGLE
  // weighting keeps its pseudonormal correct.
  const Mesh fanned =
      fanned_tetrahedron(vr::Vec3f(-0.117f, 0.091f, 0.143f), 0.1f, 12);
  vr::Result<ts::MeshIntegrateStats> fanned_stats = convert(fanned, kSigned);
  CHECK(fanned_stats.ok());
  CHECK(fanned_stats.value().boundary_edges == 0);
  if (verify(grid, "signed fanned tetrahedron", [&](vr::Vec3f p) {
        return signed_closed(p, fanned, trunc);
      }) != 0) {
    return 1;
  }

  // ---- 5. Signed open quad: the rim is unobserved, not a skirt ----------
  // A +z-facing quad at z = z0. A voxel over the quad's interior is +-its
  // height; a voxel past the rim has no side and must be left unobserved. The
  // bounds sit off the voxel lattice so no voxel projects onto the rim itself.
  const float z0 = 0.052f;
  const float x0 = 0.013f, x1 = 0.297f, y0 = 0.027f, y1 = 0.243f;
  Mesh quad;
  quad.v = {{x0, y0, z0}, {x1, y0, z0}, {x1, y1, z0}, {x0, y1, z0}};
  quad.i = {0, 1, 2, 0, 2, 3};
  vr::Result<ts::MeshIntegrateStats> quad_stats = convert(quad, kSigned);
  CHECK(quad_stats.ok());
  CHECK(quad_stats.value().boundary_edges == 4);
  int quad_observed = 0;
  if (verify(
          grid, "signed open quad",
          [&](vr::Vec3f p) {
            Expect e;
            const float h = p.z - z0;
            if (std::fabs(h) < kEdge ||
                std::fabs(std::fabs(h) - trunc) < kEdge) {
              e.skip = true;
              return e;
            }
            const bool over = p.x > x0 && p.x < x1 && p.y > y0 && p.y < y1;
            e.observed = over && std::fabs(h) <= trunc;
            e.value = e.observed ? h : 0.0f;
            return e;
          },
          &quad_observed) != 0) {
    return 1;
  }
  CHECK(quad_observed > 0);

  // ---- 6. Shell: any mesh, distance minus the half-thickness ------------
  // The same open quad, now observed past its rim too.
  CHECK(convert(quad, kShell).ok());
  auto shell_expect = [&](const Mesh& m) {
    return [&m, trunc, shell](vr::Vec3f p) {
      Expect e;
      const float d = nearest(p, m);
      if (std::fabs(d - trunc) < kEdge) {
        e.skip = true;
        return e;
      }
      e.observed = d <= trunc;
      e.value = e.observed ? clampf(d - shell, -trunc, trunc) : 0.0f;
      return e;
    };
  };
  if (verify(grid, "shell open quad", shell_expect(quad)) != 0) return 1;

  // Around a closed solid the shell has two walls: an outer one, and an inner
  // one the same distance inside, with positive voxels deeper still.
  CHECK(convert(tet, kShell).ok());
  if (verify(grid, "shell tetrahedron", shell_expect(tet)) != 0) return 1;
  {
    int deep_inside_positive = 0;
    if (verify(grid, "shell inner wall", [&](vr::Vec3f p) {
          Expect e = shell_expect(tet)(p);
          if (!e.skip && e.observed && e.value > 0.0f &&
              winding(p, tet) > 0.5) {
            ++deep_inside_positive;
          }
          return e;
        }) != 0) {
      return 1;
    }
    CHECK(deep_inside_positive > 0);
  }

  // ---- 7. Blocks the band does not reach keep what they held ------------
  CHECK(grid.clear().ok());
  const vol::BlockIndex far_block{vr::Vec3i(40, 40, 40), 0};
  CHECK(grid.map().allocate(&far_block, 1).ok());
  std::int32_t far_ptr = -1;
  for (const vol::BlockIndex& b : active_blocks(grid)) far_ptr = b.ptr;
  CHECK(far_ptr >= 0);
  for (int k = 0; k < gp.voxels_per_block; ++k) {
    attr(grid, "tsdf")[far_ptr + k] = 7.0f;
    attr(grid, "weight")[far_ptr + k] = 7.0f;
  }
  CHECK(grid.map()
            .allocate_from_triangles(tet.v.data(), tet.vertex_count(),
                                     tet.i.data(), tet.triangle_count())
            .ok());
  vr::Result<ts::MeshIntegrateStats> beside =
      integ.integrate(grid, tet.v.data(), tet.vertex_count(), tet.i.data(),
                      tet.triangle_count(), kSigned);
  CHECK(beside.ok());
  CHECK(beside.value().blocks == active_blocks(grid).size() - 1);
  for (int k = 0; k < gp.voxels_per_block; ++k) {
    CHECK(attr(grid, "tsdf")[far_ptr + k] == 7.0f);
    CHECK(attr(grid, "weight")[far_ptr + k] == 7.0f);
  }

  // ---- 8. Refusals, each before the grid is written ---------------------
  auto refused = [&](const Mesh& m, const ts::MeshSdfParams& params,
                     const char* needle) -> bool {
    const std::vector<float> before = snapshot(grid);
    vr::Result<ts::MeshIntegrateStats> r =
        integ.integrate(grid, m.v.data(), m.vertex_count(), m.i.data(),
                        m.triangle_count(), params);
    if (r.ok()) {
      std::fprintf(stderr, "expected a refusal containing \"%s\"\n", needle);
      return false;
    }
    if (r.status().message().find(needle) == std::string::npos) {
      std::fprintf(stderr, "refusal \"%s\" does not mention \"%s\"\n",
                   r.status().message().c_str(), needle);
      return false;
    }
    return snapshot(grid) == before;
  };

  // A band that was never allocated.
  CHECK(grid.clear().ok());
  CHECK(refused(tet, kSigned, "not allocated"));
  // One block of it missing.
  CHECK(grid.map()
            .allocate_from_triangles(tet.v.data(), tet.vertex_count(),
                                     tet.i.data(), tet.triangle_count())
            .ok());
  {
    const std::vector<vol::BlockIndex> band = active_blocks(grid);
    CHECK(!band.empty());
    CHECK(grid.remove(&band[band.size() / 2], 1).ok());
  }
  CHECK(refused(tet, kSigned, "not allocated"));

  // Winding that flips across an edge: refused signed, fine as a shell.
  CHECK(convert(tet, kSigned).ok());
  Mesh flipped = tet;
  std::swap(flipped.i[1], flipped.i[2]);
  // Flipping one face flips all three of its edges.
  CHECK(refused(flipped, kSigned, "3 edge(s) whose winding flips"));
  CHECK(integ
            .integrate(grid, flipped.v.data(), flipped.vertex_count(),
                       flipped.i.data(), flipped.triangle_count(), kShell)
            .ok());

  // Three triangles on one edge.
  Mesh book;
  book.v = {{0.0f, 0.0f, 0.0f},
            {0.2f, 0.0f, 0.0f},
            {0.1f, 0.15f, 0.0f},
            {0.1f, -0.1f, 0.1f},
            {0.1f, -0.1f, -0.1f}};
  book.i = {0, 1, 2, 1, 0, 3, 0, 1, 4};
  CHECK(convert(book, kShell).ok());
  CHECK(refused(book, kSigned, "1 non-manifold edge(s), 0 non-manifold vert"));

  // Two tetrahedra touching at one vertex: every edge manifold, the vertex not.
  Mesh pinch = tet;
  {
    const vr::Vec3f apex = tet.v[0];
    const std::uint32_t base = pinch.vertex_count();
    for (int k = 1; k < 4; ++k) pinch.v.push_back(2.0f * apex - tet.v[k]);
    Mesh other;
    other.v = {apex, pinch.v[base], pinch.v[base + 1], pinch.v[base + 2]};
    other.i = {0, 1, 2, 0, 1, 3, 0, 2, 3, 1, 2, 3};
    orient_outward(other,
                   (other.v[0] + other.v[1] + other.v[2] + other.v[3]) / 4.0f);
    const std::uint32_t remap[4] = {0, base, base + 1, base + 2};
    for (std::uint32_t idx : other.i) pinch.i.push_back(remap[idx]);
  }
  CHECK(convert(pinch, kShell).ok());
  CHECK(refused(pinch, kSigned, "0 non-manifold edge(s), 1 non-manifold vert"));

  // Parameters and inputs.
  CHECK(convert(tet, kSigned).ok());
  CHECK(refused(tet, {ts::MeshSdfMode::Shell, 0.5f}, "shell_voxels"));
  CHECK(refused(tet, {ts::MeshSdfMode::Shell, 4.0f}, "shell_voxels"));
  CHECK(refused(tet, {static_cast<ts::MeshSdfMode>(7), 1.5f}, "unknown mode"));
  {
    const std::vector<float> before = snapshot(grid);
    CHECK(!integ.integrate(grid, nullptr, 4, tet.i.data(), 4, kSigned).ok());
    CHECK(!integ.integrate(grid, tet.v.data(), 4, nullptr, 4, kSigned).ok());
    const std::uint32_t bad[3] = {0, 1, 4};
    CHECK(!integ.integrate(grid, tet.v.data(), 4, bad, 1, kSigned).ok());
    // No triangles: nothing to write, and not an error.
    vr::Result<ts::MeshIntegrateStats> none =
        integ.integrate(grid, tet.v.data(), 4, tet.i.data(), 0, kSigned);
    CHECK(none.ok());
    CHECK(none.value().blocks == 0);
    CHECK(snapshot(grid) == before);
  }
  // A grid without a weight attribute.
  {
    const vol::AttributeSpec tsdf_only[] = {{"tsdf", sizeof(float)}};
    vr::Result<vol::VoxelBlockGrid> bare =
        vol::VoxelBlockGrid::create(device, allocator, gp, tsdf_only, 1);
    CHECK(bare.ok());
    CHECK(!integ
               .integrate(bare.value(), tet.v.data(), tet.vertex_count(),
                          tet.i.data(), tet.triangle_count(), kSigned)
               .ok());
  }

  // ---- 9. Move semantics -------------------------------------------------
  {
    ts::MeshIntegrator moved(std::move(integ));
    CHECK(moved.valid());
    CHECK(!integ.valid());  // NOLINT(bugprone-use-after-move): the point
    CHECK(!integ
               .integrate(grid, tet.v.data(), tet.vertex_count(), tet.i.data(),
                          tet.triangle_count(), kSigned)
               .ok());
    vr::Result<ts::MeshIntegrator> other_r =
        ts::MeshIntegrator::create(device, allocator);
    CHECK(other_r.ok());
    ts::MeshIntegrator other = std::move(other_r).value();
    other = std::move(moved);  // over a live object
    CHECK(other.valid());
    CHECK(!moved.valid());  // NOLINT(bugprone-use-after-move)
    ts::MeshIntegrator* self = &other;
    other = std::move(*self);  // self-move, laundered past -Wself-move
    CHECK(other.valid());
    CHECK(other
              .integrate(grid, tet.v.data(), tet.vertex_count(), tet.i.data(),
                         tet.triangle_count(), kSigned)
              .ok());
  }

  std::printf("tsdf_mesh_integrate: OK\n");
  return 0;
}
