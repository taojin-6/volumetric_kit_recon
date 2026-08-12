// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The progressive patch atlas: addressing, then accumulation.
//
// Two halves, and they fail for different reasons. The addressing half is pure
// host arithmetic -- patch_texel_index has to be a bijection onto
// [0, patch_texel_count), or patches silently overwrite each other's texels
// with no allocation error and no validation message anywhere. The
// accumulation half runs the kernel against a synthetic frame of ONE KNOWN
// COLOUR, which is what makes "did the right colour reach the right texels"
// answerable at all: every texel the camera can see must end up that colour,
// and every texel it cannot must stay untouched.
//
// Needs a device, so the whole test skips (exit 0) where none is present.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <utility>
#include <vector>

#include "volumetric_kit/recon/core/allocator.hpp"
#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/instance.hpp"
#include "volumetric_kit/recon/mesh/marching_cubes.hpp"
#include "volumetric_kit/recon/texture/patch_atlas.hpp"
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

constexpr int kBlock = 8;
constexpr int kBlocks = 4;
constexpr int kN = kBlock * kBlocks;
constexpr float kH = 0.05f;
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
  grid.trunc_dist = 0.04f;
  grid.bucket_size = 8;
  grid.num_buckets = 128;
  grid.num_blocks = 1024;
  grid.max_chain = 128;
  return grid;
}

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

std::uint32_t weight_of(std::uint32_t texel) { return texel >> 24; }

}  // namespace

int main() {
  // --- Addressing, with no device in sight --------------------------------
  //
  // Every (i, j) with i + j < leg must land on a distinct index, and together
  // they must cover [0, count) exactly. Both halves matter: distinctness stops
  // two texels of one patch aliasing, and coverage is what makes the count the
  // stride between patches -- a gap would put patch t+1's texels inside patch
  // t's run.
  for (std::uint32_t leg = 2; leg <= 32; ++leg) {
    const std::uint32_t count = rtex::patch_texel_count(leg);
    std::vector<int> hits(count, 0);
    for (std::uint32_t j = 0; j < leg; ++j) {
      for (std::uint32_t i = 0; i + j < leg; ++i) {
        const std::uint32_t idx = rtex::patch_texel_index(leg, i, j);
        CHECK(idx < count);
        CHECK(hits[idx] == 0);
        hits[idx] = 1;
      }
    }
    for (std::uint32_t k = 0; k < count; ++k) {
      CHECK(hits[k] == 1);
    }
  }
  // A right triangle, not a square: the whole reason the atlas fits at room
  // scale. If this ever reads leg*leg the memory figures in the header are off
  // by two.
  CHECK(rtex::patch_texel_count(8) == 36);

  vr::Result<vr::Instance> instance = vr::Instance::create({});
  if (!instance) {
    std::fprintf(stderr, "no Vulkan instance; skipping device half\n");
    return 0;
  }
  vr::Result<VkPhysicalDevice> gpu = instance.value().select_physical_device();
  if (!gpu) {
    std::fprintf(stderr, "no compute device; skipping device half\n");
    return 0;
  }
  vr::Result<vr::Device> device =
      vr::Device::create(instance.value().handle(), gpu.value(), {});
  CHECK(device.ok());
  vr::Result<vr::Allocator> allocator =
      vr::Allocator::create(instance.value().handle(), device.value());
  CHECK(allocator.ok());

  // A patch of one texel has no barycentric span to interpolate across, so the
  // kernel's divide by leg - 1 would be a divide by zero. Refused at create,
  // where the caller can see it, rather than produced as NaN texels.
  CHECK(!rtex::PatchAtlas::create(device.value(), allocator.value(),
                                  {/*patch_leg=*/1})
             .ok());

  vr::Result<mesh::MarchingCubes> extractor_result =
      mesh::MarchingCubes::create(device.value(), allocator.value(), {});
  CHECK(extractor_result.ok());
  mesh::MarchingCubes extractor = std::move(extractor_result).value();

  const vol::AttributeSpec attrs[] = {{"tsdf", sizeof(float)},
                                      {"weight", sizeof(float)}};
  vr::Result<vol::VoxelBlockGrid> grid_result = vol::VoxelBlockGrid::create(
      device.value(), allocator.value(), sphere_grid_params(), attrs, 2);
  CHECK(grid_result.ok());
  vol::VoxelBlockGrid grid = std::move(grid_result).value();
  CHECK(fill_sphere_grid(grid));

  // A camera in front of the sphere looking down +Z, with a constant depth at
  // the sphere's near surface -- so the front of the sphere passes the
  // line-of-sight test and the far side fails it. Both branches are covered by
  // one frame, which is what lets the assertions below be about *which* texels
  // were written rather than only about how many.
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

  // ONE colour everywhere, and a deliberately awkward one: 0x004080C0 is
  // R=0xC0, G=0x80, B=0x40 under the R-in-the-low-byte packing, so a channel
  // swap anywhere between here and the atlas shows up as a different value
  // rather than as the same grey.
  constexpr std::uint32_t kColor = 0x004080C0u;
  const std::vector<std::uint32_t> color(
      static_cast<std::size_t>(kWidth) * kHeight, kColor);

  vr::Result<mesh::DeviceMesh> mesh_result =
      extractor.extract_device(grid, 0.0f);
  CHECK(mesh_result.ok());
  const mesh::DeviceMesh device_mesh = mesh_result.value();
  CHECK(device_mesh.triangle_count > 0);

  vr::Result<rtex::PatchAtlas> atlas_result =
      rtex::PatchAtlas::create(device.value(), allocator.value(), {});
  CHECK(atlas_result.ok());
  rtex::PatchAtlas atlas = std::move(atlas_result).value();
  CHECK(atlas.patch_leg() == 8);
  CHECK(atlas.texels_per_patch() == 36);
  // Nothing sized until a mesh arrives -- the atlas shadows the arena, so it
  // has no size of its own to have before one.
  CHECK(atlas.triangle_capacity() == 0);
  CHECK(atlas.bytes() == 0);

  CHECK(
      atlas.fuse(device_mesh, depth.data(), color.data(), kWidth, kHeight, cam)
          .ok());
  CHECK(atlas.triangle_capacity() >= device_mesh.triangle_count);
  CHECK(atlas.bytes() == static_cast<std::uint64_t>(atlas.triangle_capacity()) *
                             atlas.texels_per_patch() * sizeof(std::uint32_t));

  // Count what landed. The sphere's front faces the camera and its back does
  // not, so both must be represented: a run where everything was written means
  // the occlusion test never rejected anything, and one where nothing was
  // means the projection never hit.
  std::size_t written = 0;
  std::size_t untouched = 0;
  std::size_t wrong_color = 0;
  {
    const std::uint32_t* patch_data = atlas.mapped();
    CHECK(patch_data != nullptr);
    const std::size_t total =
        static_cast<std::size_t>(device_mesh.triangle_count) *
        atlas.texels_per_patch();
    for (std::size_t k = 0; k < total; ++k) {
      const std::uint32_t t = patch_data[k];
      if (weight_of(t) == 0) {
        // A texel no observation reached must be *entirely* zero, not merely
        // zero-weighted: the colour bits are what a consumer would sample, and
        // leaving them at whatever the allocator handed over would show as
        // noise on exactly the surfaces the camera never saw.
        CHECK(t == 0);
        ++untouched;
        continue;
      }
      ++written;
      // One colour in, one colour out. The blend is a weighted average of a
      // single value, so it must reproduce that value up to the round trip
      // through linear and back -- one 8-bit step per channel, no more.
      const std::uint32_t r = t & 0xFFu;
      const std::uint32_t g = (t >> 8) & 0xFFu;
      const std::uint32_t b = (t >> 16) & 0xFFu;
      if (std::abs(static_cast<int>(r) - 0xC0) > 1 ||
          std::abs(static_cast<int>(g) - 0x80) > 1 ||
          std::abs(static_cast<int>(b) - 0x40) > 1) {
        ++wrong_color;
      }
    }
  }
  CHECK(written > 0);    // the camera saw something
  CHECK(untouched > 0);  // and did not see everything
  CHECK(wrong_color == 0);

  // --- Accumulation -------------------------------------------------------
  //
  // Fusing the same frame again must RAISE the weights (toward saturation) and
  // leave the colours alone, since averaging a value into itself is that
  // value. A weight that did not move would mean the read-modify-write is not
  // reading; a colour that moved would mean the linear round trip is lossy in
  // one direction.
  std::vector<std::uint32_t> after_first(
      atlas.mapped(),
      atlas.mapped() + static_cast<std::size_t>(device_mesh.triangle_count) *
                           atlas.texels_per_patch());
  CHECK(
      atlas.fuse(device_mesh, depth.data(), color.data(), kWidth, kHeight, cam)
          .ok());
  std::size_t grew = 0;
  for (std::size_t k = 0; k < after_first.size(); ++k) {
    const std::uint32_t before = after_first[k];
    const std::uint32_t now = atlas.mapped()[k];
    if (weight_of(before) == 0) {
      CHECK(now == 0);  // still unseen, still exactly zero
      continue;
    }
    CHECK(weight_of(now) >= weight_of(before));
    if (weight_of(now) > weight_of(before)) ++grew;
    CHECK((now & 0x00FFFFFFu) == (before & 0x00FFFFFFu));
  }
  CHECK(grew > 0);

  // --- Reservation --------------------------------------------------------
  //
  // The property is not "the buffer is big enough" -- fuse() would have grown
  // it anyway. It is that the HANDLE DOES NOT MOVE, because growing frees the
  // old buffer with no fence wait, and a renderer drawing the atlas while a
  // scan accumulates into it would be reading freed memory. So reserve past
  // the mesh, fuse, and assert the handle is the one reserve produced.
  {
    vr::Result<rtex::PatchAtlas> reserved_result =
        rtex::PatchAtlas::create(device.value(), allocator.value(), {});
    CHECK(reserved_result.ok());
    rtex::PatchAtlas reserved = std::move(reserved_result).value();

    CHECK(reserved.reserve(0).ok());  // no-op, not an error
    CHECK(reserved.triangle_capacity() == 0);

    const std::uint32_t want = device_mesh.triangle_count * 2;
    CHECK(reserved.reserve(want).ok());
    CHECK(reserved.triangle_capacity() >= want);
    const VkBuffer before = reserved.buffer();
    CHECK(before != VK_NULL_HANDLE);
    const std::uint64_t bytes_before = reserved.bytes();

    CHECK(
        reserved
            .fuse(device_mesh, depth.data(), color.data(), kWidth, kHeight, cam)
            .ok());
    // Same allocation, not merely a big enough one.
    CHECK(reserved.buffer() == before);
    CHECK(reserved.bytes() == bytes_before);
    // And it still fused: a reservation must not make the pass a no-op.
    std::size_t written_after_reserve = 0;
    for (std::size_t k = 0;
         k < static_cast<std::size_t>(device_mesh.triangle_count) *
                 reserved.texels_per_patch();
         ++k) {
      if (weight_of(reserved.mapped()[k]) != 0) ++written_after_reserve;
    }
    CHECK(written_after_reserve > 0);

    // Reserving below what is already held keeps the larger allocation rather
    // than shrinking to fit -- the atlas is grow-only, like the arena it
    // shadows.
    CHECK(reserved.reserve(1).ok());
    CHECK(reserved.buffer() == before);
  }

  // --- Invalidation -------------------------------------------------------
  //
  // The answer to a full extract, where every arena slot is re-reserved and no
  // patch describes the triangle it was fused against any more.
  atlas.invalidate();
  for (std::size_t k = 0; k < after_first.size(); ++k) {
    CHECK(atlas.mapped()[k] == 0);
  }

  // --- The refusals a caller cannot see for itself -------------------------
  CHECK(!atlas.fuse(device_mesh, nullptr, color.data(), kWidth, kHeight, cam)
             .ok());
  CHECK(!atlas.fuse(device_mesh, depth.data(), nullptr, kWidth, kHeight, cam)
             .ok());
  CHECK(!atlas.fuse(device_mesh, depth.data(), color.data(), 0, kHeight, cam)
             .ok());
  // A view the producer has already extracted past: binding it can be a
  // use-after-free of a buffer the extractor destroyed on a grow.
  const mesh::DeviceMesh stale = device_mesh;
  CHECK(extractor.extract_device(grid, 0.0f).ok());
  CHECK(!stale.is_current());
  CHECK(!atlas.fuse(stale, depth.data(), color.data(), kWidth, kHeight, cam)
             .ok());

  std::printf(
      "recon texture patch-atlas test passed: %zu texels fused from one frame, "
      "%zu left unseen, accumulation and invalidation both hold\n",
      written, untouched);
  return 0;
}
