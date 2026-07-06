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

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <utility>
#include <vector>

#include "volumetric_kit/recon/core/allocator.hpp"
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
  for (std::size_t t = 0; t + 2 < sphere.vertices.size(); t += 3) {
    const vr::Vec3f& a = sphere.vertices[t].position;
    const vr::Vec3f& b = sphere.vertices[t + 1].position;
    const vr::Vec3f& c = sphere.vertices[t + 2].position;
    const vr::Vec3f face = vr::cross(b - a, c - a);
    if (vr::length(face) > 1e-8f) {
      face_sum += vr::dot(vr::normalize(face), sphere.vertices[t].normal);
      ++face_count;
    }
  }
  CHECK(face_count > 0);
  CHECK(face_sum / static_cast<double>(face_count) > 0.5);

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
      "sphere SDF; radius, outward normals, and winding verified on-device\n",
      sphere.triangle_count());
  return 0;
}
