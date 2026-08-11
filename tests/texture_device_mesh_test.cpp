// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The device-resident mesh handoff: extract_device -> texture -> download must
// produce exactly what the host path produces.
//
// Routing a mesh from the `mesh` tier to the `texture` tier through a host
// Mesh costs a full readback and a full re-upload of the same bytes;
// MarchingCubes::extract_device hands the texturer the buffers directly
// instead. That is only worth anything if it is *identical*, so this compares
// the two paths on one extraction:
//
//   extract_device      -> a device mesh
//   download            -> an untextured host copy (uv0 = sentinel)
//   texture(host copy)  -> the reference result (upload + readback)
//   texture(device mesh)-> the same pass, in place, no transfer
//   download            -> the result under test
//
// Both start from the same geometry in the same order, so the comparison is
// exact and index-by-index -- no need to work around the nondeterministic
// triangle order marching cubes' atomic append produces between two extracts.
//
// Needs a device, so the whole test skips (exit 0) where none is present.

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <utility>
#include <vector>

#include "volumetric_kit/recon/core/allocator.hpp"
#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/gpu_timer.hpp"
#include "volumetric_kit/recon/core/instance.hpp"
#include "volumetric_kit/recon/core/stage_metrics.hpp"
#include "volumetric_kit/recon/mesh/marching_cubes.hpp"
#include "volumetric_kit/recon/mesh/mesh.hpp"
#include "volumetric_kit/recon/texture/projective_texturer.hpp"
#include "volumetric_kit/recon/volume/hash_types.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"

namespace vr = volumetric_kit::recon;
namespace vol = volumetric_kit::recon::volume;
namespace mesh = volumetric_kit::recon::mesh;
namespace rtex = volumetric_kit::recon::texture;

#define CHECK(cond)                                                        \
  do {                                                                     \
    if (!(cond)) {                                                         \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      return 1;                                                            \
    }                                                                      \
  } while (0)

namespace {

const vr::StageRow* find_row(const vr::StageMetrics& m, const char* name) {
  for (const vr::StageRow& row : m.rows()) {
    if (std::strcmp(row.name, name) == 0) return &row;
  }
  return nullptr;
}

constexpr int kBlock = 8;             // voxels per block edge
constexpr int kBlocks = 4;            // blocks per axis
constexpr int kN = kBlock * kBlocks;  // voxels per axis
constexpr float kH = 0.05f;           // metres between voxels
constexpr float kRadius = 0.5f;

vr::Vec3f sphere_center() {
  const float c = static_cast<float>(kN - 1) * 0.5f * kH;
  return vr::Vec3f(c, c, c);
}

vol::VoxelGridParams sphere_grid_params() {
  vol::VoxelGridParams grid{};
  grid.voxel_size = kH;
  grid.block_size = kBlock;
  grid.voxels_per_block = kBlock * kBlock * kBlock;
  grid.trunc_dist = 0.04f;  // unused by meshing; must pass validate()
  grid.bucket_size = 8;
  grid.num_buckets = 128;
  grid.num_blocks = 1024;
  grid.max_chain = 128;
  return grid;
}

// Allocate every block of the cube and write the analytic sphere SDF (weight 1)
// into each voxel, addressed by the compacted BlockIndex::ptr + local index.
bool fill_sphere_grid(vol::VoxelBlockGrid& grid) {
  std::vector<vol::BlockIndex> blocks;
  for (int cz = 0; cz < kBlocks; ++cz) {
    for (int cy = 0; cy < kBlocks; ++cy) {
      for (int cx = 0; cx < kBlocks; ++cx) {
        vol::BlockIndex block{};
        block.coord = vr::Vec3i(cx, cy, cz);
        blocks.push_back(block);
      }
    }
  }
  vr::Result<std::uint32_t> failed = grid.map().allocate(
      blocks.data(), static_cast<std::uint32_t>(blocks.size()));
  if (!failed || failed.value() != 0) return false;
  vr::Result<std::vector<vol::BlockIndex>> active =
      grid.map().compact_active_blocks();
  if (!active) return false;

  vr::Result<vol::AttributeView> tsdf = grid.attribute("tsdf");
  vr::Result<vol::AttributeView> weight = grid.attribute("weight");
  if (!tsdf || !weight) return false;
  auto* tsdf_data = static_cast<float*>(tsdf.value().buffer->mapped());
  auto* weight_data = static_cast<float*>(weight.value().buffer->mapped());

  for (const vol::BlockIndex& block : active.value()) {
    for (int lz = 0; lz < kBlock; ++lz) {
      for (int ly = 0; ly < kBlock; ++ly) {
        for (int lx = 0; lx < kBlock; ++lx) {
          const int local = lx + kBlock * (ly + kBlock * lz);
          const vr::Vec3i voxel = block.coord * kBlock + vr::Vec3i(lx, ly, lz);
          const vr::Vec3f world(static_cast<float>(voxel.x) * kH,
                                static_cast<float>(voxel.y) * kH,
                                static_cast<float>(voxel.z) * kH);
          // BlockIndex::ptr is already the block's base offset into the flat
          // per-voxel array, so the local index adds straight onto it.
          const std::size_t index = static_cast<std::size_t>(block.ptr) +
                                    static_cast<std::size_t>(local);
          tsdf_data[index] = vr::length(world - sphere_center()) - kRadius;
          weight_data[index] = 1.0f;
        }
      }
    }
  }
  return true;
}

}  // namespace

int main() {
  vr::Result<vr::Instance> instance = vr::Instance::create({});
  if (!instance) {
    std::fprintf(stderr, "no Vulkan instance; skipping\n");
    return 0;
  }
  vr::Result<VkPhysicalDevice> gpu = instance.value().select_physical_device();
  if (!gpu) {
    std::fprintf(stderr, "no compute-capable device; skipping\n");
    return 0;
  }
  vr::Result<vr::Device> device =
      vr::Device::create(instance.value().handle(), gpu.value(), {});
  CHECK(device.ok());
  vr::Result<vr::Allocator> allocator =
      vr::Allocator::create(instance.value().handle(), device.value());
  CHECK(allocator.ok());
  vr::Result<mesh::MarchingCubes> extractor_result =
      mesh::MarchingCubes::create(device.value(), allocator.value());
  CHECK(extractor_result.ok());
  mesh::MarchingCubes extractor = std::move(extractor_result).value();
  vr::Result<rtex::ProjectiveTexturer> texturer_result =
      rtex::ProjectiveTexturer::create(device.value(), allocator.value());
  CHECK(texturer_result.ok());
  rtex::ProjectiveTexturer texturer = std::move(texturer_result).value();

  const vol::AttributeSpec attrs[] = {{"tsdf", sizeof(float)},
                                      {"weight", sizeof(float)}};
  vr::Result<vol::VoxelBlockGrid> grid_result = vol::VoxelBlockGrid::create(
      device.value(), allocator.value(), sphere_grid_params(), attrs, 2);
  CHECK(grid_result.ok());
  vol::VoxelBlockGrid grid = std::move(grid_result).value();
  CHECK(fill_sphere_grid(grid));

  // A camera in front of the sphere looking down +Z (recon's OpenCV
  // convention), with a constant depth at the sphere's near surface: the
  // front-facing triangles pass the line-of-sight test and the far side fails
  // it, so the comparison covers both the textured and the sentinel branch.
  constexpr std::uint32_t kWidth = 128;
  constexpr std::uint32_t kHeight = 128;
  constexpr float kCameraDistance = 2.0f;
  vr::DepthCameraParams cam{};
  cam.fx = 120.0f;
  cam.fy = 120.0f;
  cam.cx = static_cast<float>(kWidth) * 0.5f;
  cam.cy = static_cast<float>(kHeight) * 0.5f;
  cam.width = kWidth;
  cam.height = kHeight;
  cam.min_depth = 0.1f;
  cam.max_depth = 10.0f;
  cam.cam_to_world = vr::Mat4f(1.0f);
  const vr::Vec3f eye =
      sphere_center() - vr::Vec3f(0.0f, 0.0f, kCameraDistance);
  cam.cam_to_world[3] = vr::Vec4f(eye.x, eye.y, eye.z, 1.0f);
  const std::vector<float> depth(static_cast<std::size_t>(kWidth) * kHeight,
                                 kCameraDistance - kRadius);

  // One extraction feeds both paths, so the geometry -- and its order -- is
  // identical for the index-by-index comparison below.
  vr::Result<mesh::DeviceMesh> device_mesh_result =
      extractor.extract_device(grid, 0.0f);
  CHECK(device_mesh_result.ok());
  const mesh::DeviceMesh device_mesh = device_mesh_result.value();
  CHECK(!device_mesh.empty());
  CHECK(device_mesh.valid());
  CHECK(device_mesh.vertex_count == device_mesh.triangle_count * 3);

  // Reference: the host path, on a copy taken before any texturing.
  vr::Result<mesh::Mesh> host_result = extractor.download(device_mesh);
  CHECK(host_result.ok());
  mesh::Mesh host_mesh = std::move(host_result).value();
  CHECK(host_mesh.vertices.size() == device_mesh.vertex_count);
  CHECK(texturer.texture(host_mesh, depth.data(), cam).ok());

  // Under test: the same pass over the device buffers, then one copy out.
  //
  // Timed, because this overload is the one seam B wires and its reporting path
  // is otherwise covered nowhere -- deleting its timer argument or its publish
  // left the whole suite green. It has five returns between the stage scope and
  // the dispatch, each a chance to skip the publish.
  vr::StageMetrics metrics;
  CHECK(texturer.texture(device_mesh, depth.data(), cam, 0.02f, &metrics).ok());
  const vr::StageRow* row = find_row(metrics, "texture");
  CHECK(row != nullptr);
  CHECK(row->cpu_ms > 0.0);
  // A device that reports timestamps must produce the device half here; one
  // that does not is a supported configuration, and the probe -- not the tier
  // under test -- is what tells the two apart.
  vr::Result<vr::GpuTimer> probe = vr::GpuTimer::create(device.value());
  CHECK(probe.ok());
  if (probe.value().available()) {
    CHECK(row->has_gpu);
    CHECK(row->gpu_ms < row->cpu_ms);
  }
  vr::Result<mesh::Mesh> device_result = extractor.download(device_mesh);
  CHECK(device_result.ok());
  const mesh::Mesh device_out = std::move(device_result).value();

  CHECK(device_out.vertices.size() == host_mesh.vertices.size());
  CHECK(device_out.indices.size() == host_mesh.indices.size());

  // Every vertex must match: uv0 is what the pass writes, and the rest proves
  // the in-place write did not disturb the geometry around it.
  std::size_t textured = 0;
  for (std::size_t i = 0; i < host_mesh.vertices.size(); ++i) {
    const mesh::Vertex& expected = host_mesh.vertices[i];
    const mesh::Vertex& actual = device_out.vertices[i];
    CHECK(actual.uv0 == expected.uv0);
    CHECK(actual.position == expected.position);
    CHECK(actual.normal == expected.normal);
    CHECK(actual.color == expected.color);
    // Also pins that the in-place uv0 rewrite leaves the neighbouring tangent
    // slot alone -- the two are adjacent in the layout gfx dictates.
    CHECK(actual.tangent == expected.tangent);
    if (expected.uv0.x >= 0.0f) ++textured;
  }
  CHECK(device_out.indices.size() == host_mesh.indices.size());
  for (std::size_t i = 0; i < host_mesh.indices.size(); ++i) {
    CHECK(device_out.indices[i] == host_mesh.indices[i]);
  }

  // Guard against a vacuous pass: if nothing was textured, the two paths would
  // agree trivially on an all-sentinel mesh and prove nothing.
  CHECK(textured > 0);
  CHECK(textured < host_mesh.vertices.size());

  // Without MarchingCubesConfig::share_vertices -- this extractor's default --
  // every triangle owns three private vertices written at `tri * 3`, so the run
  // IS the identity 0,1,2,..., the host fills it once per grow and download()
  // regenerates rather than reading back. Asserted rather than assumed, because
  // both the fill and the regeneration are conditional on that flag now and a
  // mismatch between them is silent: the mesh stays the right SIZE and its
  // triangles are drawn from the wrong vertices.
  //
  // Restating `indices.size() == triangle_count * 3` would prove nothing --
  // download() resizes to exactly that -- so the content is what is checked,
  // plus that every index addresses a live vertex (an out-of-range one is an
  // out-of-bounds read in the texturing kernel and an undefined fetch in the
  // renderer; `robustBufferAccess` covers neither).
  CHECK(device_out.indices.size() % 3 == 0);
  for (std::size_t i = 0; i < device_out.indices.size(); ++i) {
    CHECK(device_out.indices[i] == static_cast<std::uint32_t>(i));
    CHECK(device_out.indices[i] < device_out.vertices.size());
  }

  // --- A superseded DeviceMesh is rejected -----------------------------------
  // The arena is grow-only and reused in place, so a later extract leaves an
  // earlier view naming the *same* VkBuffer while the contents have been
  // replaced. Handle comparison cannot see that; the generation stamp can, and
  // must -- downloading a superseded view would silently return the newer
  // geometry under the older counts.
  {
    vr::Result<mesh::DeviceMesh> first = extractor.extract_device(grid, 0.0f);
    CHECK(first.ok());
    const mesh::DeviceMesh superseded = first.value();
    CHECK(!superseded.empty());

    vr::Result<mesh::DeviceMesh> second = extractor.extract_device(grid, 0.0f);
    CHECK(second.ok());
    const mesh::DeviceMesh live = second.value();

    // Same capacity -> no grow -> the buffers really are reused, which is what
    // makes a handle check insufficient. If this ever stops holding the test
    // below still passes, but it stops testing the case that matters.
    CHECK(live.vertices == superseded.vertices);
    CHECK(live.indices == superseded.indices);
    CHECK(live.generation != superseded.generation);

    CHECK(!extractor.download(superseded).ok());
    // ...while the current one still downloads.
    CHECK(extractor.download(live).ok());
  }

  // The DENSE extract shares that same arena, so it invalidates an outstanding
  // view too. It is the sharper case: now that the sparse path fits its arena
  // to the surface rather than to the 5-tri/cell worst case, a dense grid
  // routinely needs MORE than the sparse call left held, so this call
  // reallocates the buffers rather than merely overwriting them -- and a view
  // still accepted here would name freed VkBuffers, not just stale contents.
  {
    vr::Result<mesh::DeviceMesh> before = extractor.extract_device(grid, 0.0f);
    CHECK(before.ok());
    const mesh::DeviceMesh stale = before.value();
    CHECK(!stale.empty());

    std::vector<vol::Voxel> samples(static_cast<std::size_t>(kN) * kN * kN);
    for (int z = 0; z < kN; ++z) {
      for (int y = 0; y < kN; ++y) {
        for (int x = 0; x < kN; ++x) {
          const vr::Vec3f p(static_cast<float>(x) * kH,
                            static_cast<float>(y) * kH,
                            static_cast<float>(z) * kH);
          vol::Voxel& v =
              samples[static_cast<std::size_t>(x + kN * (y + kN * z))];
          v.sdf = vr::length(p - sphere_center()) - kRadius;
          v.weight = 1.0f;
        }
      }
    }
    mesh::DenseGrid dense_grid;
    dense_grid.dims = vr::Vec3i(kN, kN, kN);
    dense_grid.voxel_size = kH;
    dense_grid.origin = vr::Vec3f(0.0f, 0.0f, 0.0f);
    CHECK(extractor.extract(samples.data(), samples.size(), dense_grid, 0.0f)
              .ok());

    CHECK(!extractor.download(stale).ok());
  }

  // A DeviceMesh from another extractor is rejected too: generations are
  // per-object, so one extractor's stamp never authorises another's buffers.
  {
    vr::Result<mesh::MarchingCubes> other_result =
        mesh::MarchingCubes::create(device.value(), allocator.value());
    CHECK(other_result.ok());
    mesh::MarchingCubes other = std::move(other_result).value();
    vr::Result<mesh::DeviceMesh> foreign = other.extract_device(grid, 0.0f);
    CHECK(foreign.ok());
    CHECK(!foreign.value().empty());
    CHECK(!extractor.download(foreign.value()).ok());
  }

  // The *writing* path is guarded too, not only download(). texture() binds
  // these buffers and dispatches over them, so a superseded view is worse than
  // a stale read: if the later extract grew the arena, the old VkBuffer was
  // destroyed synchronously and binding it is a use-after-free -- undefined
  // with validation layers off, the shipping configuration. valid() cannot see
  // it (the handles are non-null, and a grow-only arena reused in place even
  // names the same VkBuffer), which is what DeviceMesh::is_current is for.
  {
    vr::Result<mesh::DeviceMesh> live = extractor.extract_device(grid, 0.0f);
    CHECK(live.ok() && !live.value().empty());
    const mesh::DeviceMesh held = live.value();
    CHECK(held.is_current());
    // Texturing it now is fine.
    CHECK(texturer.texture(held, depth.data(), cam).ok());

    // Extract again on the same extractor; `held` is now superseded.
    vr::Result<mesh::DeviceMesh> next = extractor.extract_device(grid, 0.0f);
    CHECK(next.ok());
    CHECK(!held.is_current());
    CHECK(next.value().is_current());
    // valid() still says yes -- the handles are non-null -- which is exactly
    // why it is not the check that matters here.
    CHECK(held.valid());
    vr::Status stale_texture = texturer.texture(held, depth.data(), cam);
    CHECK(!stale_texture.ok());
    CHECK(stale_texture.domain() == vr::Status::Code::InvalidArgument);
    // The live view from the same extractor still textures.
    CHECK(texturer.texture(next.value(), depth.data(), cam).ok());
  }

  // A mesh whose vertices are SHARED is textured, not refused.
  //
  // It used to be refused, and the refusal was the whole reason DeviceMesh
  // publishes the flag: the pass decided visibility per TRIANGLE and wrote uv0
  // per VERTEX, so a vertex referenced by up to six triangles that disagreed
  // was written by whichever thread ran last -- nondeterministically, visible
  // only as flicker along every silhouette. The dispatch is per vertex now, so
  // there is exactly one writer per vertex and nothing to disagree.
  //
  // The flag has not become useless; it has stopped being an incompatibility.
  // A packed multi-camera atlas will still need a per-PRIMITIVE camera id, and
  // a consumer sizing a vertex arena still needs to know whether `v = 3t`.
  {
    mesh::MarchingCubesConfig share_config;
    share_config.share_vertices = true;
    vr::Result<mesh::MarchingCubes> share_result = mesh::MarchingCubes::create(
        device.value(), allocator.value(), share_config);
    CHECK(share_result.ok());
    mesh::MarchingCubes share_mc = std::move(share_result).value();
    vr::Result<mesh::DeviceMesh> shared = share_mc.extract_device(grid, 0.0f);
    CHECK(shared.ok());
    CHECK(!shared.value().empty());
    CHECK(shared.value().shares_vertices);
    CHECK(shared.value().is_current());
    CHECK(shared.value().valid());
    // Sharing genuinely reduces the vertex count, so this is a mesh the old
    // path could not have produced a result for at all -- not merely the same
    // mesh relabelled.
    CHECK(shared.value().vertex_count < 3 * shared.value().triangle_count);
    vr::Status shared_texture =
        texturer.texture(shared.value(), depth.data(), cam);
    CHECK(shared_texture.ok());
  }

  std::printf(
      "texture device-mesh: OK (%zu triangles, %zu/%zu vertices textured)\n",
      host_mesh.triangle_count(), textured, host_mesh.vertices.size());
  return 0;
}
