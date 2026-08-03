// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// GPU test for marching-cubes extraction: write an analytic sphere signed-
// distance field into a dense voxel grid, extract the iso-surface on the real
// driver (MoltenVK on Apple, the NVIDIA ICD on the Linux CI box), and verify
// the mesh is a sphere -- every vertex sits on the radius, every normal points
// outward, and the triangle winding agrees with the gradient normal (so the
// ported tri_table convention is right). Also checks the surface-misses-grid
// (empty) case, argument validation, and move semantics. Exits 0 (skip) where
// no device is present. Extraction runs against a synthetic SDF because the
// tsdf tier that fills real voxel blocks does not exist yet -- the per-cell
// kernel is identical either way.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <utility>
#include <vector>

#include "volumetric_kit/recon/core/allocator.hpp"
#include "volumetric_kit/recon/core/color_space.hpp"
#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/instance.hpp"
#include "volumetric_kit/recon/core/math/vector_types.hpp"
#include "volumetric_kit/recon/core/vulkan.hpp"
#include "volumetric_kit/recon/mesh/marching_cubes.hpp"
#include "volumetric_kit/recon/mesh/mesh.hpp"
#include "volumetric_kit/recon/volume/hash_types.hpp"

namespace vr = volumetric_kit::recon;
namespace vol = volumetric_kit::recon::volume;
namespace mesh = volumetric_kit::recon::mesh;

#define CHECK(cond)                                                        \
  do {                                                                     \
    if (!(cond)) {                                                         \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      return 1;                                                            \
    }                                                                      \
  } while (0)

namespace {

// A dense sphere SDF: sample (x,y,z) at world origin + (x,y,z)*h carries the
// signed distance to a sphere of radius R centred in the grid, weight 1.
constexpr int kN = 48;           // samples per axis
constexpr float kH = 0.05f;      // metres between samples
constexpr float kRadius = 0.7f;  // sphere radius (well inside the grid)

int sample_index(int x, int y, int z) { return x + kN * (y + kN * z); }

vr::Vec3f sphere_center() {
  const float c = static_cast<float>(kN - 1) * 0.5f * kH;
  return vr::Vec3f(c, c, c);
}

std::vector<vol::Voxel> make_sphere_field() {
  const vr::Vec3f center = sphere_center();
  std::vector<vol::Voxel> samples(static_cast<std::size_t>(kN) * kN * kN);
  for (int z = 0; z < kN; ++z) {
    for (int y = 0; y < kN; ++y) {
      for (int x = 0; x < kN; ++x) {
        const vr::Vec3f p(static_cast<float>(x) * kH,
                          static_cast<float>(y) * kH,
                          static_cast<float>(z) * kH);
        vol::Voxel& v =
            samples[static_cast<std::size_t>(sample_index(x, y, z))];
        v.sdf = vr::length(p - center) - kRadius;
        v.weight = 1.0f;
      }
    }
  }
  return samples;
}

// Grid span in world units (origin is 0), used to normalize a position to
// [0,1] per axis for the gradient color.
constexpr float kSpan = static_cast<float>(kN - 1) * kH;

// A linear gradient color: world position normalized to [0,1] per axis.
vr::Vec3f grad_color(vr::Vec3f p) {
  auto clamp01 = [](float v) {
    return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
  };
  return vr::Vec3f(clamp01(p.x / kSpan), clamp01(p.y / kSpan),
                   clamp01(p.z / kSpan));
}

// Per-sample color = grad_color(sample world position), quantized to u8 RGB
// (the volume tier's Vec3u8 color payload).
std::vector<vr::Vec3u8> make_gradient_colors() {
  std::vector<vr::Vec3u8> colors(static_cast<std::size_t>(kN) * kN * kN);
  for (int z = 0; z < kN; ++z) {
    for (int y = 0; y < kN; ++y) {
      for (int x = 0; x < kN; ++x) {
        const vr::Vec3f p(static_cast<float>(x) * kH,
                          static_cast<float>(y) * kH,
                          static_cast<float>(z) * kH);
        const vr::Vec3f g = grad_color(p);
        colors[static_cast<std::size_t>(sample_index(x, y, z))] =
            vr::Vec3u8(static_cast<std::uint8_t>(g.x * 255.0f + 0.5f),
                       static_cast<std::uint8_t>(g.y * 255.0f + 0.5f),
                       static_cast<std::uint8_t>(g.z * 255.0f + 0.5f));
      }
    }
  }
  return colors;
}

// An n^3 sphere SDF at the same voxel pitch, radius scaled to fit -- a second,
// smaller grid so a test can drive the extractor's vertex arena up and back
// down.
std::vector<vol::Voxel> make_sphere_field_n(int n) {
  const float c = static_cast<float>(n - 1) * 0.5f * kH;
  const vr::Vec3f center(c, c, c);
  const float radius = c * 0.6f;
  std::vector<vol::Voxel> samples(static_cast<std::size_t>(n) * n * n);
  for (int z = 0; z < n; ++z) {
    for (int y = 0; y < n; ++y) {
      for (int x = 0; x < n; ++x) {
        const vr::Vec3f p(static_cast<float>(x) * kH,
                          static_cast<float>(y) * kH,
                          static_cast<float>(z) * kH);
        vol::Voxel& v = samples[static_cast<std::size_t>(x) +
                                static_cast<std::size_t>(n) *
                                    (y + static_cast<std::size_t>(n) * z)];
        v.sdf = vr::length(p - center) - radius;
        v.weight = 1.0f;
      }
    }
  }
  return samples;
}

// A canonical, order-independent form of a mesh: each triangle's nine position
// floats, sorted. Marching cubes appends through an atomic, so triangle order
// varies run to run -- only the multiset of triangles is stable, which is what
// an equality check between two extracts can rest on.
std::vector<std::array<float, 9>> canonical_triangles(const mesh::Mesh& m) {
  std::vector<std::array<float, 9>> tris(m.vertices.size() / 3);
  for (std::size_t t = 0; t < tris.size(); ++t) {
    for (std::size_t k = 0; k < 3; ++k) {
      const vr::Vec3f& p = m.vertices[t * 3 + k].position;
      tris[t][k * 3 + 0] = p.x;
      tris[t][k * 3 + 1] = p.y;
      tris[t][k * 3 + 2] = p.z;
    }
  }
  std::sort(tris.begin(), tris.end());
  return tris;
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
      vr::Device::create(instance.value().handle(), gpu.value(), {});
  if (!device) {
    std::fprintf(stderr, "device create failed: %s\n",
                 device.status().message().c_str());
    return 1;
  }
  vr::Result<vr::Allocator> allocator =
      vr::Allocator::create(instance.value().handle(), device.value());
  if (!allocator) {
    std::fprintf(stderr, "allocator create failed: %s\n",
                 allocator.status().message().c_str());
    return 1;
  }

  vr::Result<mesh::MarchingCubes> mc_result =
      mesh::MarchingCubes::create(device.value(), allocator.value());
  if (!mc_result) {
    std::fprintf(stderr, "MarchingCubes::create failed: %s\n",
                 mc_result.status().message().c_str());
    return 1;
  }
  mesh::MarchingCubes extractor = std::move(mc_result).value();

  const std::vector<vol::Voxel> samples = make_sphere_field();
  mesh::DenseGrid grid;
  grid.dims = vr::Vec3i(kN, kN, kN);
  grid.voxel_size = kH;
  grid.origin = vr::Vec3f(0.0f, 0.0f, 0.0f);

  // --- Extract the sphere ----------------------------------------------------
  vr::Result<mesh::Mesh> mesh_result =
      extractor.extract(samples.data(), samples.size(), grid, 0.0f);
  CHECK(mesh_result.ok());
  mesh::Mesh sphere = std::move(mesh_result).value();

  CHECK(!sphere.empty());
  // Independent triangles: three vertices per triangle, indices the trivial
  // run.
  CHECK(sphere.indices.size() == sphere.vertices.size());
  CHECK(sphere.indices.size() % 3 == 0);
  CHECK(sphere.triangle_count() > 500);  // 4*pi*R^2/h^2 is ~O(1e3)

  const vr::Vec3f center = sphere_center();

  // Every vertex lies on the sphere; every normal is unit and points outward.
  double radius_sum = 0.0;
  double outward_sum = 0.0;
  for (const mesh::Vertex& v : sphere.vertices) {
    // The renderer's tangent slot. Meshing cannot derive a real tangent, so the
    // kernel writes this placeholder -- and it MUST write it: the vertex arena
    // is grow-only and reused in place, so an unwritten slot carries whatever
    // the previous, larger extraction left there. gfx does not bind `tangent`
    // today, so nothing downstream would notice; this is the only guard.
    CHECK(v.tangent.x == 1.0f && v.tangent.y == 0.0f && v.tangent.z == 0.0f &&
          v.tangent.w == 1.0f);
    const vr::Vec3f d = v.position - center;
    const float r = vr::length(d);
    CHECK(std::fabs(r - kRadius) < 1.5f * kH);  // MC accuracy ~ one voxel
    CHECK(std::fabs(vr::length(v.normal) - 1.0f) < 1e-3f);
    const float outward = vr::dot(vr::normalize(d), v.normal);
    CHECK(outward > 0.5f);  // never inward-facing
    radius_sum += r;
    outward_sum += outward;
  }
  const auto n = static_cast<double>(sphere.vertices.size());
  CHECK(std::fabs(radius_sum / n - kRadius) < 0.25 * kH);  // no radial bias
  CHECK(outward_sum / n > 0.9);                            // mostly radial

  // Winding agrees with the gradient normal: a flipped tri_table would make the
  // face normal (from vertex order) oppose the per-cell gradient normal.
  double face_sum = 0.0;
  int face_count = 0;
  int face_outward = 0;  // faces whose winding normal points along the gradient
  for (std::size_t t = 0; t + 2 < sphere.vertices.size(); t += 3) {
    const vr::Vec3f& a = sphere.vertices[t].position;
    const vr::Vec3f& b = sphere.vertices[t + 1].position;
    const vr::Vec3f& c = sphere.vertices[t + 2].position;
    const vr::Vec3f face = vr::cross(b - a, c - a);
    if (vr::length(face) > 1e-8f) {
      const float fd = vr::dot(vr::normalize(face), sphere.vertices[t].normal);
      face_sum += fd;
      if (fd > 0.0f) ++face_outward;
      ++face_count;
    }
  }
  CHECK(face_count > 0);
  CHECK(face_sum / static_cast<double>(face_count) > 0.5);
  // Per-face, not just on average: a partial winding regression that inverted
  // even a small fraction of faces would clear the mean check above yet drop
  // this ratio. An analytic sphere winds essentially every face outward.
  const double outward_ratio =
      static_cast<double>(face_outward) / static_cast<double>(face_count);
  CHECK(outward_ratio > 0.95);

  // --- Per-vertex color: default (no color input) ----------------------------
  // With no color supplied every vertex is opaque white, and uv0 is the "use
  // vertex color" sentinel (projective texturing has not run).
  for (const mesh::Vertex& v : sphere.vertices) {
    CHECK(v.color.x == 1.0f && v.color.y == 1.0f && v.color.z == 1.0f &&
          v.color.w == 1.0f);
    CHECK(v.uv0.x < 0.0f && v.uv0.y < 0.0f);
  }

  // --- Per-vertex color: interpolated from a color input ---------------------
  // Color each sample by a linear gradient of its world position. Linear interp
  // of a linear field is exact, so each vertex's color must match the gradient
  // at that vertex's *own* position to within u8 quantization (the corner
  // colors round-trip through RGBA8, so ~0.5/255 ~= 0.002). The tolerance is
  // kept deliberately tight -- well below one cell's color span (kH/kSpan ~=
  // 0.021) -- so the two subtle ways this path can break each trip a CHECK
  // instead of hiding under a loose bound: a dropped edge interpolation (corner
  // passthrough) and a color/position winding-reversal mismatch both perturb a
  // vertex's color by up to that per-cell span. uv0 stays the sentinel (no
  // atlas yet).
  //
  // The comparison happens in ENCODED space -- `linear_to_srgb(v.color)`
  // against the gradient -- because `Vertex::color` is now linear working
  // values while the gradient was written as canonical-encoded 8-bit (the
  // 2026-08-02 color-space decision). Inverting the vertex rather than
  // forward-converting the expectation keeps the tolerance meaningful: one u8
  // code spans ~0.0089 in *linear* near white, which alone would blow a 0.005
  // bound, while in encoded space a code is a flat 1/255 everywhere and the
  // bound still reads as "~2.5x the u8 floor". It also stays discriminating
  // rather than vacuous: a kernel that skipped the decode would leave `v.color`
  // encoded, and `linear_to_srgb` of an already-encoded 0.5 is 0.735, nowhere
  // near it.
  const std::vector<vr::Vec3u8> colors = make_gradient_colors();
  vr::Result<mesh::Mesh> colored_result = extractor.extract(
      samples.data(), samples.size(), grid, 0.0f, colors.data());
  CHECK(colored_result.ok());
  mesh::Mesh colored = std::move(colored_result).value();
  CHECK(!colored.empty());
  for (const mesh::Vertex& v : colored.vertices) {
    const vr::Vec3f expected = grad_color(v.position);
    const vr::Vec3f encoded = vr::linear_to_srgb(vr::Vec3f(v.color));
    CHECK(std::fabs(encoded.x - expected.x) < 0.005f);  // ~2.5x the u8 floor
    CHECK(std::fabs(encoded.y - expected.y) < 0.005f);
    CHECK(std::fabs(encoded.z - expected.z) < 0.005f);
    CHECK(v.color.w == 1.0f);
    CHECK(v.uv0.x < 0.0f &&
          v.uv0.y < 0.0f);  // still the sentinel; no atlas yet
  }

  // --- Surface misses the grid -> empty mesh ---------------------------------
  // A constant positive field has no sign change anywhere, so no cell meshes.
  // Reuses the same extractor, proving it is re-entrant across extracts.
  std::vector<vol::Voxel> outside(samples.size());
  for (vol::Voxel& v : outside) {
    v.sdf = 1.0f;
    v.weight = 1.0f;
  }
  vr::Result<mesh::Mesh> empty_result =
      extractor.extract(outside.data(), outside.size(), grid, 0.0f);
  CHECK(empty_result.ok());
  CHECK(std::move(empty_result).value().empty());

  // --- Vertex-arena reuse across extracts ------------------------------------
  // The arena is retained between extracts, so an extract's OUTPUT must not
  // depend on what the extractor meshed before it. Drive the capacity up
  // (small -> large) and back down (large -> small), and require each result to
  // match one from a fresh extractor that meshed nothing else. This pins output
  // stability across a reuse sequence; the sizing policy itself is not
  // observable from the mesh and is pinned in the sparse test, which can read
  // the arena's actual size through ExtractTimings.
  constexpr int kSmallN = 16;
  const std::vector<vol::Voxel> small_samples = make_sphere_field_n(kSmallN);
  mesh::DenseGrid small_grid = grid;
  small_grid.dims = vr::Vec3i(kSmallN, kSmallN, kSmallN);

  auto fresh_triangles =
      [&](const std::vector<vol::Voxel>& s,
          const mesh::DenseGrid& g) -> std::vector<std::array<float, 9>> {
    vr::Result<mesh::MarchingCubes> fresh =
        mesh::MarchingCubes::create(device.value(), allocator.value());
    if (!fresh) return {};
    vr::Result<mesh::Mesh> r =
        fresh.value().extract(s.data(), s.size(), g, 0.0f);
    if (!r) return {};
    return canonical_triangles(std::move(r).value());
  };

  const std::vector<std::array<float, 9>> small_baseline =
      fresh_triangles(small_samples, small_grid);
  const std::vector<std::array<float, 9>> big_baseline =
      fresh_triangles(samples, grid);
  CHECK(!small_baseline.empty());
  CHECK(!big_baseline.empty());
  // The second extract really does need a bigger arena than the first, so the
  // sequence below exercises the grow path rather than reuse throughout.
  CHECK(big_baseline.size() > small_baseline.size());

  vr::Result<mesh::MarchingCubes> reuse_result =
      mesh::MarchingCubes::create(device.value(), allocator.value());
  CHECK(reuse_result.ok());
  mesh::MarchingCubes reuser = std::move(reuse_result).value();

  // Sizes the arena from nothing.
  vr::Result<mesh::Mesh> grow_a = reuser.extract(
      small_samples.data(), small_samples.size(), small_grid, 0.0f);
  CHECK(grow_a.ok());
  CHECK(canonical_triangles(std::move(grow_a).value()) == small_baseline);

  // Needs more than the arena holds: the grow path, including its 1.5x headroom
  // and the fallback when that headroom would not fit.
  vr::Result<mesh::Mesh> grow_b =
      reuser.extract(samples.data(), samples.size(), grid, 0.0f);
  CHECK(grow_b.ok());
  CHECK(canonical_triangles(std::move(grow_b).value()) == big_baseline);

  // Needs less: reuse of an oversized arena. Sizing the dispatch from the arena
  // instead of the request, or reading back past the request, would show here.
  vr::Result<mesh::Mesh> shrink = reuser.extract(
      small_samples.data(), small_samples.size(), small_grid, 0.0f);
  CHECK(shrink.ok());
  CHECK(canonical_triangles(std::move(shrink).value()) == small_baseline);

  // Repeating a call is idempotent: the atomic counter is reset per extract, so
  // a second identical extract emits the same triangles, not twice as many.
  vr::Result<mesh::Mesh> again = reuser.extract(
      small_samples.data(), small_samples.size(), small_grid, 0.0f);
  CHECK(again.ok());
  CHECK(canonical_triangles(std::move(again).value()) == small_baseline);

  // --- Argument validation ---------------------------------------------------
  mesh::DenseGrid too_small = grid;
  too_small.dims = vr::Vec3i(1, 1, 1);
  CHECK(!extractor.extract(samples.data(), 1, too_small, 0.0f).ok());
  CHECK(
      !extractor.extract(samples.data(), samples.size() - 1, grid, 0.0f).ok());
  CHECK(!extractor.extract(nullptr, samples.size(), grid, 0.0f).ok());

  // --- Move-only -------------------------------------------------------------
  // Move-construct: the source empties, the destination lives.
  mesh::MarchingCubes moved = std::move(extractor);
  CHECK(!extractor.valid());
  CHECK(moved.valid());

  // Move-assign over a live object: its prior pipeline/buffers are released by
  // the move-assign (ASan turns a leak/double-free here into a failure).
  vr::Result<mesh::MarchingCubes> other_result =
      mesh::MarchingCubes::create(device.value(), allocator.value());
  CHECK(other_result.ok());
  mesh::MarchingCubes other = std::move(other_result).value();
  moved = std::move(other);
  CHECK(!other.valid());
  CHECK(moved.valid());

  // Self-move (laundered through a pointer to dodge -Wself-move under -Werror):
  // the guarded move leaves the object intact.
  mesh::MarchingCubes* alias = &moved;
  moved = std::move(*alias);
  CHECK(moved.valid());

  // Re-extract through the moved-into object to prove it is fully live.
  vr::Result<mesh::Mesh> reextract =
      moved.extract(samples.data(), samples.size(), grid, 0.0f);
  CHECK(reextract.ok());
  CHECK(!std::move(reextract).value().empty());

  std::printf(
      "recon mesh marching-cubes test passed: extracted %zu triangles from a "
      "sphere SDF; radius, outward normals, and winding (%.4f of faces "
      "outward) verified on-device\n",
      sphere.triangle_count(), outward_ratio);
  return 0;
}
