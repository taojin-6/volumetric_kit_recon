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

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <utility>
#include <vector>

#include "volumetric_kit/core/base/stage_metrics.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/compute_util.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/core/vulkan/gpu_timer.hpp"
#include "volumetric_kit/core/vulkan/instance.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"
#include "volumetric_kit/recon/mesh/marching_cubes.hpp"
#include "volumetric_kit/recon/mesh/mesh.hpp"
#include "volumetric_kit/recon/texture/projective_texturer.hpp"
#include "volumetric_kit/recon/texture/texture_atlas.hpp"
#include "volumetric_kit/recon/volume/hash_types.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"

#include "grid_readback.hpp"
#include "no_device.hpp"

namespace vr = volumetric_kit::recon;
namespace vkc = volumetric_kit::core;
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

// The coordinate the single-camera pass gives `world` in colour camera `k`:
// the kernel's arithmetic -- the pixel centre as a fraction of the image,
// clamped half a texel inside it.
vr::Vec2f color_uv(const vr::ColorCameraParams& k, vr::Vec3f world) {
  const vr::Vec3f d = world - vr::Vec3f(k.cam_to_world[3]);
  const vr::Vec3f p(vr::dot(vr::Vec3f(k.cam_to_world[0]), d),
                    vr::dot(vr::Vec3f(k.cam_to_world[1]), d),
                    vr::dot(vr::Vec3f(k.cam_to_world[2]), d));
  const float w = static_cast<float>(k.width);
  const float h = static_cast<float>(k.height);
  const float u = (k.fx * (p.x / p.z) + k.cx + 0.5f) / w;
  const float v = (k.fy * (p.y / p.z) + k.cy + 0.5f) / h;
  return vr::Vec2f(std::fmin(std::fmax(u, 0.5f / w), 1.0f - 0.5f / w),
                   std::fmin(std::fmax(v, 0.5f / h), 1.0f - 0.5f / h));
}

bool near(vr::Vec2f a, vr::Vec2f b, float tolerance) {
  return std::fabs(a.x - b.x) <= tolerance && std::fabs(a.y - b.y) <= tolerance;
}

const vkc::StageRow* find_row(const vkc::StageMetrics& m, const char* name) {
  for (const vkc::StageRow& row : m.rows()) {
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

// Which field to write. kSphere is the fixture every case below meshes; kDense
// crosses the iso in EVERY cell -- alternating signs on voxel parity, marching
// cubes' ~5-triangles-per-cell worst case against a shell's ~1 -- and exists
// for one case only, which needs an extract that cannot fit in the arena a
// sphere extract left held.
enum class Field { kSphere, kDense };

// The device the grids live on, for fill_grid.
// Allocate every block of the cube and write @p field (weight 1) into each
// voxel, addressed by the compacted BlockIndex::ptr + local index.
bool fill_grid(const vr_test::Gpu& ctx, vol::VoxelBlockGrid& grid,
               Field field = Field::kSphere) {
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
  vkc::Result<std::uint32_t> failed = grid.map().allocate(
      blocks.data(), static_cast<std::uint32_t>(blocks.size()));
  if (!failed || failed.value() != 0) return false;
  vkc::Result<std::vector<vol::BlockIndex>> active =
      grid.map().compact_active_blocks();
  if (!active) return false;

  auto tsdf =
      vr_test::read_attribute<float>(ctx.device, ctx.allocator, grid, "tsdf");
  auto weight =
      vr_test::read_attribute<float>(ctx.device, ctx.allocator, grid, "weight");
  if (!tsdf || !weight) return false;
  float* tsdf_data = tsdf.value().data();
  float* weight_data = weight.value().data();

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
          tsdf_data[index] =
              field == Field::kSphere
                  ? vr::length(world - sphere_center()) - kRadius
                  : (((voxel.x + voxel.y + voxel.z) & 1) ? kH : -kH);
          weight_data[index] = 1.0f;
        }
      }
    }
  }
  return vr_test::write_attribute(ctx.device, ctx.allocator, grid, "tsdf",
                                  tsdf.value())
             .ok() &&
         vr_test::write_attribute(ctx.device, ctx.allocator, grid, "weight",
                                  weight.value())
             .ok();
}

}  // namespace

int main() {
  vkc::Result<vkc::Instance> instance = vkc::Instance::create({});
  if (!instance) {
    return vr_test::no_device("no Vulkan instance",
                              instance.status().message());
  }
  vkc::Result<vkc::PhysicalDeviceInfo> gpu =
      instance.value().select_physical_device(vr::device_requirements());
  if (!gpu) {
    return vr_test::no_device("no compute-capable device",
                              gpu.status().message());
  }
  vkc::Result<vkc::Device> device = vkc::Device::create(
      instance.value(), gpu.value(), vr::device_requirements());
  CHECK(device.ok());
  vkc::Result<vkc::Allocator> allocator =
      vkc::Allocator::create(instance.value().handle(), device.value());
  CHECK(allocator.ok());
  const vr_test::Gpu ctx{device.value(), allocator.value()};
  vkc::Result<mesh::MarchingCubes> extractor_result =
      mesh::MarchingCubes::create(device.value(), allocator.value());
  CHECK(extractor_result.ok());
  mesh::MarchingCubes extractor = std::move(extractor_result).value();
  vkc::Result<rtex::ProjectiveTexturer> texturer_result =
      rtex::ProjectiveTexturer::create(device.value(), allocator.value());
  CHECK(texturer_result.ok());
  rtex::ProjectiveTexturer texturer = std::move(texturer_result).value();

  const vol::AttributeSpec attrs[] = {{"tsdf", sizeof(float)},
                                      {"weight", sizeof(float)}};
  vkc::Result<vol::VoxelBlockGrid> grid_result = vol::VoxelBlockGrid::create(
      device.value(), allocator.value(), sphere_grid_params(), attrs, 2);
  CHECK(grid_result.ok());
  vol::VoxelBlockGrid grid = std::move(grid_result).value();
  CHECK(fill_grid(ctx, grid));

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
  vkc::Result<mesh::DeviceMesh> device_mesh_result =
      extractor.extract_device(grid, 0.0f);
  CHECK(device_mesh_result.ok());
  const mesh::DeviceMesh device_mesh = device_mesh_result.value();
  CHECK(!device_mesh.empty());
  CHECK(device_mesh.valid());
  CHECK(device_mesh.vertex_count == device_mesh.triangle_count * 3);

  // Reference: the host path, on a copy taken before any texturing.
  vkc::Result<mesh::Mesh> host_result = extractor.download(device_mesh);
  CHECK(host_result.ok());
  mesh::Mesh host_mesh = std::move(host_result).value();
  CHECK(host_mesh.vertices.size() == device_mesh.vertex_count);
  CHECK(texturer.texture(host_mesh, depth.data(), cam).ok());

  // Under test: the same pass over the device buffers, then one copy out.
  //
  // Timed, because this overload is the one seam B wires and its reporting path
  // is otherwise covered nowhere -- deleting its timer argument or its publish
  // left the whole suite green. Every return between the stage scope and the
  // dispatch is a chance to skip the publish.
  vkc::StageMetrics metrics;
  CHECK(texturer.texture(device_mesh, depth.data(), cam, 0.02f, &metrics).ok());
  const vkc::StageRow* row = find_row(metrics, "texture");
  CHECK(row != nullptr);
  CHECK(row->cpu_ms > 0.0);
  // A device that reports timestamps must produce the device half here; one
  // that does not is a supported configuration, and the probe -- not the tier
  // under test -- is what tells the two apart.
  vkc::Result<vkc::GpuTimer> probe = vkc::GpuTimer::create(device.value());
  CHECK(probe.ok());
  if (probe.value().available()) {
    CHECK(row->has_gpu);
    CHECK(row->gpu_ms < row->cpu_ms);
  }
  vkc::Result<mesh::Mesh> device_result = extractor.download(device_mesh);
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

  // A colour camera of its own, as a GpuFramePrep frame has: the depth on the
  // device, and the camera the atlas was taken with, in one TextureView.
  //
  // One identical to the depth camera gives the registered coordinates, since
  // it is the same arithmetic on the same pixel; the mesh is first textured
  // against a frame that sees nothing, so a call that wrote nothing cannot
  // pass. One 5 cm aside, at 256 x 256 with a focal length and principal
  // point of its own, gives every vertex its pixel in THAT camera: textured
  // exactly where the registered pass textured it (it is wider, so it records
  // all the depth camera sees on the sphere, and its line of sight to the
  // near cap is clear), carried everywhere else. Its image blank over its
  // right half carries the vertices there too. One too narrow for what the
  // depth camera sees carries the vertices outside its image. One facing away
  // gives every vertex the sentinel, though the depth camera sees half of
  // them.
  {
    vkc::Result<vkc::Buffer> depth_result = vkc::device_storage_buffer(
        allocator.value(), depth.size() * sizeof(float));
    CHECK(depth_result.ok());
    CHECK(vr_test::write_back(device.value(), allocator.value(),
                              depth_result.value(), depth)
              .ok());
    const auto device_depth =
        std::make_shared<const vkc::Buffer>(std::move(depth_result).value());
    const auto view_from = [&](const vr::ColorCameraParams& k) {
      rtex::TextureView view;
      view.cam = cam;
      view.depth_buffer = device_depth;
      view.color_camera = k;
      return view;
    };

    const std::vector<float> nothing(depth.size(), 0.0f);
    CHECK(texturer.texture(device_mesh, nothing.data(), cam).ok());
    vkc::Result<mesh::Mesh> blind = extractor.download(device_mesh);
    CHECK(blind.ok());
    for (const mesh::Vertex& v : blind.value().vertices) {
      CHECK(v.uv0.x < 0.0f);
    }
    const vr::ColorCameraParams same{cam.fx,          cam.fy,    cam.cx,
                                     cam.cy,          cam.width, cam.height,
                                     cam.cam_to_world};
    CHECK(texturer.texture(device_mesh, view_from(same)).ok());
    vkc::Result<mesh::Mesh> as_same = extractor.download(device_mesh);
    CHECK(as_same.ok());
    for (std::size_t i = 0; i < host_mesh.vertices.size(); ++i) {
      const vr::Vec2f got = as_same.value().vertices[i].uv0;
      const vr::Vec2f want = host_mesh.vertices[i].uv0;
      CHECK((got.x < 0.0f) == (want.x < 0.0f));
      CHECK(near(got, want, 1e-6f));
    }

    vr::ColorCameraParams aside{};
    aside.fx = 200.0f;
    aside.fy = 200.0f;
    aside.cx = 129.5f;
    aside.cy = 126.0f;
    aside.width = 256;
    aside.height = 256;
    aside.cam_to_world = cam.cam_to_world;
    aside.cam_to_world[3] += vr::Vec4f(0.05f, 0.0f, 0.0f, 0.0f);
    // Timed, as the registered overloads are: this is the one a device frame
    // takes, and nothing else would notice it dropping its row.
    vkc::StageMetrics color_metrics;
    CHECK(texturer.texture(device_mesh, view_from(aside), 0.02f, &color_metrics)
              .ok());
    const vkc::StageRow* color_row = find_row(color_metrics, "texture");
    CHECK(color_row != nullptr);
    CHECK(color_row->cpu_ms > 0.0);
    vkc::Result<mesh::Mesh> as_aside = extractor.download(device_mesh);
    CHECK(as_aside.ok());
    std::size_t textured_aside = 0;
    for (std::size_t i = 0; i < host_mesh.vertices.size(); ++i) {
      const vr::Vec2f got = as_aside.value().vertices[i].uv0;
      const vr::Vec2f uv = color_uv(aside, host_mesh.vertices[i].position);
      if (host_mesh.vertices[i].uv0.x >= 0.0f) {
        CHECK(near(got, uv, 1e-5f));
        ++textured_aside;
      } else {
        CHECK(near(got, -uv - vr::Vec2f(1.0f), 1e-5f));
      }
    }
    CHECK(textured_aside > 0);

    // The same camera's image blank from column 128 on, as the corner of an
    // undistorted image is where the lens saw nothing: the vertices there are
    // carried, the rest keep their coordinates. Refused, a coverage a word
    // short of the image, and one that is not a storage buffer, since this
    // pass binds it in place.
    {
      std::vector<std::uint32_t> half(256 * 256, 0u);
      for (std::size_t y = 0; y < 256; ++y) {
        for (std::size_t x = 0; x < 128; ++x) half[y * 256 + x] = 0xFF404040u;
      }
      vkc::Result<vkc::Buffer> coverage = vkc::device_storage_buffer(
          allocator.value(), half.size() * sizeof(std::uint32_t));
      CHECK(coverage.ok());
      CHECK(vr_test::write_back(device.value(), allocator.value(),
                                coverage.value(), half)
                .ok());
      rtex::TextureView covered = view_from(aside);
      covered.coverage =
          std::make_shared<const vkc::Buffer>(std::move(coverage).value());
      CHECK(texturer.texture(device_mesh, covered).ok());
      vkc::Result<mesh::Mesh> as_covered = extractor.download(device_mesh);
      CHECK(as_covered.ok());
      std::size_t left = 0;
      std::size_t right = 0;
      for (std::size_t i = 0; i < host_mesh.vertices.size(); ++i) {
        const vr::Vec2f got = as_covered.value().vertices[i].uv0;
        const vr::Vec2f uv = as_aside.value().vertices[i].uv0;
        if (uv.x < 0.0f) {
          CHECK(got == uv);
          continue;
        }
        // The pixel's column: the coordinate's fraction of the image.
        const float column = uv.x * 256.0f;
        if (column < 127.9f) {
          CHECK(got == uv);
          ++left;
        } else if (column > 128.1f) {
          CHECK(near(got, -uv - vr::Vec2f(1.0f), 1e-6f));
          ++right;
        }
      }
      CHECK(left > 0 && right > 0);

      vkc::Result<vkc::Buffer> short_coverage = vkc::device_storage_buffer(
          allocator.value(), half.size() * sizeof(std::uint32_t) - 4);
      vkc::BufferDesc copy_only;
      copy_only.size = half.size() * sizeof(std::uint32_t);
      copy_only.usage =
          VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
      copy_only.memory = vkc::MemoryUsage::DeviceOnly;
      vkc::Result<vkc::Buffer> unbindable =
          allocator.value().create_buffer(copy_only);
      CHECK(short_coverage.ok() && unbindable.ok());
      covered.coverage = std::make_shared<const vkc::Buffer>(
          std::move(short_coverage).value());
      CHECK(texturer.texture(device_mesh, covered).domain() ==
            vkc::Status::Code::InvalidArgument);
      covered.coverage =
          std::make_shared<const vkc::Buffer>(std::move(unbindable).value());
      CHECK(texturer.texture(device_mesh, covered).domain() ==
            vkc::Status::Code::InvalidArgument);
    }

    // A colour camera too narrow for all the depth camera sees: 16 x 16 at the
    // same focal length, so only the middle of the textured cap lands inside
    // its image. A vertex outside it is carried, though the depth camera
    // proves its line of sight. Within a hundredth of a pixel of the border
    // either answer is the float's to give, so those are not held to one.
    vr::ColorCameraParams narrow = aside;
    narrow.cx = 7.5f;
    narrow.cy = 7.5f;
    narrow.width = 16;
    narrow.height = 16;
    CHECK(texturer.texture(device_mesh, view_from(narrow)).ok());
    vkc::Result<mesh::Mesh> as_narrow = extractor.download(device_mesh);
    CHECK(as_narrow.ok());
    std::size_t inside = 0;
    std::size_t outside = 0;
    for (std::size_t i = 0; i < host_mesh.vertices.size(); ++i) {
      const vr::Vec2f got = as_narrow.value().vertices[i].uv0;
      if (host_mesh.vertices[i].uv0.x < 0.0f) {
        CHECK(got.x < 0.0f);
        continue;
      }
      // The pixel, from the coordinate before its clamp (the camera is
      // unrotated, so camera space is the world shifted).
      const vr::Vec3f d =
          host_mesh.vertices[i].position - vr::Vec3f(narrow.cam_to_world[3]);
      const float px = narrow.fx * (d.x / d.z) + narrow.cx;
      const float py = narrow.fy * (d.y / d.z) + narrow.cy;
      const float edge =
          std::fmin(std::fmin(px, 15.0f - px), std::fmin(py, 15.0f - py));
      if (edge > 0.01f) {
        CHECK(got.x >= 0.0f);
        ++inside;
      } else if (edge < -0.01f) {
        CHECK(got.x < 0.0f);
        ++outside;
      }
    }
    CHECK(inside > 0 && outside > 0);

    // Half a turn about y, so it looks back past the sphere's far side.
    vr::ColorCameraParams away = aside;
    away.cam_to_world[0] = -aside.cam_to_world[0];
    away.cam_to_world[2] = -aside.cam_to_world[2];
    CHECK(texturer.texture(device_mesh, view_from(away)).ok());
    vkc::Result<mesh::Mesh> as_away = extractor.download(device_mesh);
    CHECK(as_away.ok());
    for (const mesh::Vertex& v : as_away.value().vertices) {
      CHECK(v.uv0 == vr::Vec2f(-1.0f, -1.0f));
    }

    // Refused: a colour camera with no image, a view giving its depth on the
    // host and the device both, and one giving it on neither.
    vr::ColorCameraParams blank = aside;
    blank.height = 0;
    CHECK(texturer.texture(device_mesh, view_from(blank)).domain() ==
          vkc::Status::Code::InvalidArgument);
    rtex::TextureView both = view_from(aside);
    both.depth = depth.data();
    CHECK(texturer.texture(device_mesh, both).domain() ==
          vkc::Status::Code::InvalidArgument);
    rtex::TextureView neither = view_from(aside);
    neither.depth_buffer = nullptr;
    CHECK(texturer.texture(device_mesh, neither).domain() ==
          vkc::Status::Code::InvalidArgument);
  }

  // Without MarchingCubesConfig::share_vertices -- this extractor's default --
  // every triangle owns three private vertices written at `tri * 3`, so the run
  // IS the identity 0,1,2,..., the host fills it once per grow and download()
  // regenerates rather than reading back. This checks the regeneration (the
  // fill is checked on the device run in marching_cubes_config_test), because
  // both are conditional on that flag and a mismatch between them is silent:
  // the mesh stays the right SIZE and its triangles are drawn from the wrong
  // vertices.
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

  // --- Several views: the same parity, per triangle -------------------------
  // The front camera and a second looking from +X, each with the constant
  // depth of the sphere's nearest point, so each textures the patch facing it
  // and the two tiles share the mesh. The device pass must equal the host one
  // vertex for vertex, and both views must win triangles.
  vr::DepthCameraParams side = cam;
  {
    const vr::Vec3f side_eye =
        sphere_center() + vr::Vec3f(kCameraDistance, 0.0f, 0.0f);
    side.cam_to_world = vr::Mat4f(1.0f);
    side.cam_to_world[0] = vr::Vec4f(0.0f, 0.0f, 1.0f, 0.0f);   // x
    side.cam_to_world[2] = vr::Vec4f(-1.0f, 0.0f, 0.0f, 0.0f);  // looks -X
    side.cam_to_world[3] = vr::Vec4f(side_eye, 1.0f);
  }
  const std::vector<rtex::TextureView> views = {{depth.data(), cam},
                                                {depth.data(), side}};
  vkc::Result<rtex::AtlasLayout> layout =
      rtex::side_by_side_atlas(views, texturer.max_atlas_extent());
  CHECK(layout.ok());
  {
    mesh::Mesh host_views = host_mesh;
    CHECK(texturer.texture(host_views, views, layout.value()).ok());
    // Timed, for the reason the single-camera call above is: nothing else
    // would notice this overload losing its stage scope or its publish.
    vkc::StageMetrics views_metrics;
    CHECK(
        texturer
            .texture(device_mesh, views, layout.value(), 0.02f, &views_metrics)
            .ok());
    const vkc::StageRow* views_row = find_row(views_metrics, "texture");
    CHECK(views_row != nullptr);
    CHECK(views_row->cpu_ms > 0.0);
    if (probe.value().available()) {
      CHECK(views_row->has_gpu);
      CHECK(views_row->gpu_ms < views_row->cpu_ms);
    }
    vkc::Result<mesh::Mesh> read = extractor.download(device_mesh);
    CHECK(read.ok());
    const mesh::Mesh& device_views = read.value();
    CHECK(device_views.vertices.size() == host_views.vertices.size());
    std::size_t in_tile[2] = {0, 0};
    for (std::size_t i = 0; i < host_views.vertices.size(); ++i) {
      const vr::Vec2f uv = host_views.vertices[i].uv0;
      CHECK(device_views.vertices[i].uv0 == uv);
      CHECK(device_views.vertices[i].position ==
            host_views.vertices[i].position);
      if (uv.x >= 0.0f) ++in_tile[uv.x < 0.5f ? 0 : 1];
    }
    CHECK(in_tile[0] > 0 && in_tile[1] > 0);
  }

  // --- A superseded DeviceMesh is rejected -----------------------------------
  // The arena is grow-only and reused in place, so a later extract leaves an
  // earlier view naming the *same* VkBuffer while the contents have been
  // replaced. Handle comparison cannot see that; the generation stamp can, and
  // must -- downloading a superseded view would silently return the newer
  // geometry under the older counts.
  {
    vkc::Result<mesh::DeviceMesh> first = extractor.extract_device(grid, 0.0f);
    CHECK(first.ok());
    const mesh::DeviceMesh superseded = first.value();
    CHECK(!superseded.empty());

    vkc::Result<mesh::DeviceMesh> second = extractor.extract_device(grid, 0.0f);
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

  // The same refusals over a superseding extract that REALLOCATES the buffers
  // rather than overwriting them. Above, both extracts fit the same arena, so a
  // stale view names a live VkBuffer and only its contents are wrong; here the
  // second extract needs more than the first left held, so the old VkBuffer was
  // destroyed synchronously and the stale view names freed memory.
  //
  // What that buys is not a sharper assertion -- the generation check refuses
  // both, and download() copies out of the LIVE arena either way, so no
  // mutation fails here that survives above. It is the only place in this file
  // where a stale DeviceMesh names a freed buffer at all, which is what makes a
  // future path that binds DeviceMesh::vertices before consulting
  // is_current() a real use-after-free the validation layers can name, rather
  // than a silent read of live-but-stale contents that looks correct
  // everywhere. So the texturing path -- the one that actually BINDS those
  // handles -- is asserted here beside the download.
  //
  // A dedicated extractor, because the arena is grow-only: `extractor` would
  // have to mesh the dense field to reach this, and then it could never grow
  // again for any later case.
  {
    vkc::Result<mesh::MarchingCubes> growing_result =
        mesh::MarchingCubes::create(device.value(), allocator.value());
    CHECK(growing_result.ok());
    mesh::MarchingCubes growing = std::move(growing_result).value();

    vkc::Result<vol::VoxelBlockGrid> dense_result = vol::VoxelBlockGrid::create(
        device.value(), allocator.value(), sphere_grid_params(), attrs, 2);
    CHECK(dense_result.ok());
    vol::VoxelBlockGrid dense = std::move(dense_result).value();
    CHECK(fill_grid(ctx, dense, Field::kDense));

    mesh::ExtractTimings before;
    vkc::Result<mesh::DeviceMesh> first =
        growing.extract_device(grid, 0.0f, &before);
    CHECK(first.ok());
    const mesh::DeviceMesh stale = first.value();
    CHECK(!stale.empty());

    mesh::ExtractTimings after;
    CHECK(growing.extract_device(dense, 0.0f, &after).ok());
    // The grow is what makes this the freed-buffer case rather than a repeat of
    // the one above, so it is asserted rather than assumed.
    CHECK(after.vertex_capacity > before.vertex_capacity);
    CHECK(!stale.is_current());
    CHECK(!growing.download(stale).ok());
    CHECK(!texturer.texture(stale, depth.data(), cam).ok());
    CHECK(!texturer.texture(stale, views, layout.value()).ok());
  }

  // A DeviceMesh from another extractor is rejected too: generations are
  // per-object, so one extractor's stamp never authorises another's buffers.
  {
    vkc::Result<mesh::MarchingCubes> other_result =
        mesh::MarchingCubes::create(device.value(), allocator.value());
    CHECK(other_result.ok());
    mesh::MarchingCubes other = std::move(other_result).value();
    vkc::Result<mesh::DeviceMesh> foreign = other.extract_device(grid, 0.0f);
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
    vkc::Result<mesh::DeviceMesh> live = extractor.extract_device(grid, 0.0f);
    CHECK(live.ok() && !live.value().empty());
    const mesh::DeviceMesh held = live.value();
    CHECK(held.is_current());
    // Texturing it now is fine.
    CHECK(texturer.texture(held, depth.data(), cam).ok());
    CHECK(texturer.texture(held, views, layout.value()).ok());

    // Extract again on the same extractor; `held` is now superseded.
    vkc::Result<mesh::DeviceMesh> next = extractor.extract_device(grid, 0.0f);
    CHECK(next.ok());
    CHECK(!held.is_current());
    CHECK(next.value().is_current());
    // valid() still says yes -- the handles are non-null -- which is exactly
    // why it is not the check that matters here.
    CHECK(held.valid());
    vkc::Status stale_texture = texturer.texture(held, depth.data(), cam);
    CHECK(!stale_texture.ok());
    CHECK(stale_texture.domain() == vkc::Status::Code::InvalidArgument);
    // The several-view overload binds the same buffers, so it asks too.
    CHECK(texturer.texture(held, views, layout.value()).domain() ==
          vkc::Status::Code::InvalidArgument);
    // The live view from the same extractor still textures.
    CHECK(texturer.texture(next.value(), depth.data(), cam).ok());
  }

  // A mesh whose vertices are SHARED is textured by one camera, not refused.
  //
  // It used to be refused, and the refusal was the whole reason DeviceMesh
  // publishes the flag: the pass decided visibility per TRIANGLE and wrote uv0
  // per VERTEX, so a vertex referenced by up to six triangles that disagreed
  // was written by whichever thread ran last -- nondeterministically, visible
  // only as flicker along every silhouette. The dispatch is per vertex now, so
  // there is exactly one writer per vertex and nothing to disagree.
  //
  // The flag has not become useless. The several-view overloads choose per
  // TRIANGLE and refuse a shared mesh (asserted below), and a consumer sizing
  // a vertex arena still needs to know whether `v = 3t`.
  {
    mesh::MarchingCubesConfig share_config;
    share_config.share_vertices = true;
    vkc::Result<mesh::MarchingCubes> share_result = mesh::MarchingCubes::create(
        device.value(), allocator.value(), share_config);
    CHECK(share_result.ok());
    mesh::MarchingCubes share_mc = std::move(share_result).value();
    vkc::Result<mesh::DeviceMesh> shared = share_mc.extract_device(grid, 0.0f);
    CHECK(shared.ok());
    CHECK(!shared.value().empty());
    CHECK(shared.value().shares_vertices);
    CHECK(shared.value().is_current());
    CHECK(shared.value().valid());
    // Sharing genuinely reduces the vertex count, so this is a mesh the old
    // path could not have produced a result for at all -- not merely the same
    // mesh relabelled.
    CHECK(shared.value().vertex_count < 3 * shared.value().triangle_count);
    vkc::Status shared_texture =
        texturer.texture(shared.value(), depth.data(), cam);
    CHECK(shared_texture.ok());
    // Several views choose per triangle, which a shared vertex cannot follow.
    CHECK(texturer.texture(shared.value(), views, layout.value()).domain() ==
          vkc::Status::Code::InvalidArgument);

    // Read the result back. `ok()` alone would pass against a kernel that
    // wrote the sentinel everywhere, inverted its visibility test, sized the
    // dispatch by the TRIANGLE count (leaving two thirds of a shared mesh's
    // vertices untouched), or bound the wrong descriptor slot -- every failure
    // this mesh exists to catch returns OK.
    vkc::Result<mesh::Mesh> shared_host = share_mc.download(shared.value());
    CHECK(shared_host.ok());
    const mesh::Mesh shared_out = std::move(shared_host).value();
    CHECK(shared_out.vertices.size() == shared.value().vertex_count);

    std::size_t shared_textured = 0;
    for (const mesh::Vertex& v : shared_out.vertices) {
      // Every vertex was visited: the dispatch covers the whole arena, so
      // nothing may still hold the (-1,-1) marching-cubes left AND sit inside
      // the image. A vertex the camera cannot place keeps a negative uv0, but
      // the two are told apart by the carried coordinate below.
      if (v.uv0.x >= 0.0f) {
        ++shared_textured;
        CHECK(v.uv0.x > 0.0f && v.uv0.x < 1.0f);
        CHECK(v.uv0.y > 0.0f && v.uv0.y < 1.0f);
      }
    }
    // A shared vertex is referenced by several triangles that can disagree
    // about visibility -- the exact configuration the old refusal existed for.
    // Some of this mesh must land on each side, or the run proves nothing.
    CHECK(shared_textured > 0);
    CHECK(shared_textured < shared_out.vertices.size());

    // And the indices really do share: the identity run cannot describe this
    // mesh, so at least one vertex is referenced more than once. That is what
    // makes the split above a statement about a SHARED mesh rather than about
    // any mesh.
    std::vector<std::uint32_t> uses(shared_out.vertices.size(), 0);
    for (std::uint32_t i : shared_out.indices) {
      CHECK(i < shared_out.vertices.size());
      ++uses[i];
    }
    std::size_t multi = 0;
    for (std::uint32_t u : uses) {
      if (u > 1) ++multi;
    }
    CHECK(multi > 0);
  }

  std::printf(
      "texture device-mesh: OK (%zu triangles, %zu/%zu vertices textured; a "
      "shared-vertex mesh textured in place and read back)\n",
      host_mesh.triangle_count(), textured, host_mesh.vertices.size());
  return 0;
}
