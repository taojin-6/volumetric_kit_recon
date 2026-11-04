// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// The source byte's tests' scenes: small pinhole cameras looking down world
// +z at rectangles square to their view, ray-cast and noise-free; fill
// seeded straight into a grid's attributes; and every voxel of a grid read
// back with its byte.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <tuple>
#include <vector>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/testing/gpu_test.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/command_batch.hpp"
#include "volumetric_kit/core/vulkan/compute_util.hpp"
#include "volumetric_kit/grid/testing/grid_readback.hpp"
#include "volumetric_kit/grid/volume/voxel_block_grid.hpp"
#include "volumetric_kit/recon/core/math/vector_types.hpp"
#include "volumetric_kit/recon/tsdf/source.hpp"
#include "volumetric_kit/recon/tsdf/tsdf_integrator.hpp"

namespace fill_test {

namespace vkc = volumetric_kit::core;
namespace vol = volumetric_kit::grid::volume;
namespace vr = volumetric_kit::recon;
namespace sensor = volumetric_kit::sensor;
namespace tsdf = volumetric_kit::recon::tsdf;

inline constexpr std::uint32_t kWidth = 80;
inline constexpr std::uint32_t kHeight = 60;
inline constexpr float kVoxel = 0.01f;
inline constexpr float kTrunc = 0.04f;
inline constexpr float kMaxWeight = 5.0f;
// The Femto Mega's depth noise, as its driver states it.
inline constexpr sensor::DepthNoise kNoise{0.0f, 0.00095f, 0.0f};

// A camera at `eye` looking down world +z, about 32 x 24 degrees: a pixel is
// about a voxel at 1.5 m.
inline sensor::DepthCameraParams camera(const vr::Vec3f& eye = {}) {
  sensor::DepthCameraParams cam{};
  cam.fx = cam.fy = 140.0f;
  cam.cx = 39.5f;
  cam.cy = 29.5f;
  cam.min_depth = 0.2f;
  cam.max_depth = 5.0f;
  cam.width = kWidth;
  cam.height = kHeight;
  cam.cam_to_world = vr::Mat4f(1.0f);
  cam.cam_to_world[3] = vr::Vec4f(eye, 1.0f);
  return cam;
}

// A rectangle at world z facing the cameras, |x - cx| <= hx, |y - cy| <= hy;
// `dark` returns nothing where it is the nearest surface.
struct Rect {
  float z;
  float hx = 10.0f;
  float hy = 10.0f;
  float cx = 0.0f;
  float cy = 0.0f;
  bool dark = false;
};

// The depth each pixel of `cam` returns: the nearest rectangle its ray meets,
// 0 where that is dark or where it meets none.
inline std::vector<float> render(const sensor::DepthCameraParams& cam,
                                 const std::vector<Rect>& scene) {
  const vr::Vec3f eye(cam.cam_to_world[3]);
  std::vector<float> depth(std::size_t(cam.width) * cam.height, 0.0f);
  for (std::uint32_t v = 0; v < cam.height; ++v) {
    for (std::uint32_t u = 0; u < cam.width; ++u) {
      const float rx = (float(u) - cam.cx) / cam.fx;
      const float ry = (float(v) - cam.cy) / cam.fy;
      float best = 0.0f;
      bool dark = false;
      for (const Rect& r : scene) {
        const float d = r.z - eye.z;
        if (!(d > 0.0f) || (best > 0.0f && d >= best)) continue;
        const float x = eye.x + d * rx;
        const float y = eye.y + d * ry;
        if (std::fabs(x - r.cx) <= r.hx && std::fabs(y - r.cy) <= r.hy) {
          best = d;
          dark = r.dark;
        }
      }
      depth[std::size_t(v) * cam.width + u] = dark ? 0.0f : best;
    }
  }
  return depth;
}

// A fixed frame of a set: camera `index`, stating the Femto's noise.
inline tsdf::FrameInput fixed(const std::vector<float>& depth,
                              const sensor::DepthCameraParams& cam,
                              std::uint32_t index) {
  return {vkc::StorageInput(depth.data()), cam, nullptr, index, kNoise};
}

// A filler's frame.
inline tsdf::FrameInput filler(const std::vector<float>& depth,
                               const sensor::DepthCameraParams& cam,
                               tsdf::FillTier tier, std::uint32_t index = 0) {
  tsdf::FrameInput f{vkc::StorageInput(depth.data()), cam, nullptr, index};
  f.role = tsdf::FrameRole::Filler;
  f.tier = tier;
  return f;
}

// Every voxel of a grid's active blocks: world position, attribute index, and
// the attributes.
struct Voxels {
  std::map<vgrid_test::BlockCoord, std::int32_t> blocks;  // the voxels' order
  std::vector<vr::Vec3f> world;
  std::vector<std::size_t> index;
  std::vector<float> tsdf;
  std::vector<float> weight;
  std::vector<std::uint32_t> color;
  std::vector<std::uint8_t> source;  // empty for a grid without the byte
};

inline vkc::Result<Voxels> read_voxels(vkc::test::GpuContext& gpu,
                                       vol::VoxelBlockGrid& grid) {
  Voxels out;
  VKC_ASSIGN(out.blocks, vgrid_test::blocks_of(grid));
  const int bs = grid.grid().block_size;
  const int vpb = grid.grid().voxels_per_block;
  out.world.reserve(out.blocks.size() * std::size_t(vpb));
  out.index.reserve(out.blocks.size() * std::size_t(vpb));
  for (const auto& [coord, ptr] : out.blocks) {
    for (int l = 0; l < vpb; ++l) {
      const int v[3] = {std::get<0>(coord) * bs + l % bs,
                        std::get<1>(coord) * bs + (l / bs) % bs,
                        std::get<2>(coord) * bs + l / (bs * bs)};
      out.world.push_back(vr::Vec3f(float(v[0]), float(v[1]), float(v[2])) *
                          kVoxel);
      out.index.push_back(std::size_t(ptr) + std::size_t(l));
    }
  }
  VKC_ASSIGN(out.tsdf,
             vgrid_test::read_attribute<float>(gpu.device, gpu.allocator, grid,
                                               vol::kTsdfAttribute));
  VKC_ASSIGN(out.weight,
             vgrid_test::read_attribute<float>(gpu.device, gpu.allocator, grid,
                                               vol::kWeightAttribute));
  VKC_ASSIGN(out.color,
             vgrid_test::read_attribute<std::uint32_t>(
                 gpu.device, gpu.allocator, grid, vol::kColorAttribute));
  if (grid.has_attribute(tsdf::kSourceAttribute)) {
    VKC_ASSIGN(out.source,
               vgrid_test::read_attribute<std::uint8_t>(
                   gpu.device, gpu.allocator, grid, tsdf::kSourceAttribute));
  }
  return out;
}

// Element i of an attribute read earlier: 0, the state of a block not yet
// allocated, past its end (the grid has grown since).
template <typename T>
T at(const std::vector<T>& values, std::size_t i) {
  return i < values.size() ? values[i] : T{};
}

// No voxel: a block the other grid lacks.
inline constexpr std::size_t kNone = ~std::size_t{0};

// For each voxel k of `a`, the attribute index of the voxel at its
// coordinate in `b`, or kNone where `b` holds no block there: two grids
// may store one block at different places.
inline std::vector<std::size_t> match(const Voxels& a, const Voxels& b) {
  std::vector<std::size_t> out;
  out.reserve(a.index.size());
  const std::size_t vpb =
      a.blocks.empty() ? 0 : a.index.size() / a.blocks.size();
  for (const auto& [coord, ptr] : a.blocks) {
    const auto it = b.blocks.find(coord);
    for (std::size_t l = 0; l < vpb; ++l) {
      out.push_back(it == b.blocks.end() ? kNone : std::size_t(it->second) + l);
    }
  }
  return out;
}

// Whether two grids with the same blocks hold the same source byte at every
// voxel.
inline bool same_sources(const Voxels& a, const Voxels& b) {
  if (a.blocks.size() != b.blocks.size()) return false;
  const std::vector<std::size_t> in_b = match(a, b);
  for (std::size_t k = 0; k < a.index.size(); ++k) {
    if (in_b[k] == kNone || a.source[a.index[k]] != b.source[in_b[k]]) {
      return false;
    }
  }
  return true;
}

// What I5's first sentence holds: a nonzero byte exactly where the weight is
// above 0. The count of voxels that break it.
inline std::size_t byte_weight_mismatches(const Voxels& v) {
  std::size_t bad = 0;
  for (const std::size_t i : v.index) {
    bad += (v.source[i] != 0) != (v.weight[i] > 0.0f) ? 1 : 0;
  }
  return bad;
}

// The source words (four voxels each) that hold fill beside a fixed
// camera's byte: where a write that took the whole word would show.
inline std::size_t mixed_words(const Voxels& v) {
  std::size_t mixed = 0;
  for (std::size_t w = 0; w + 4 <= v.source.size(); w += 4) {
    bool fill = false;
    bool rig = false;
    for (std::size_t k = 0; k < 4; ++k) {
      const std::uint8_t b = v.source[w + k];
      fill = fill || (b & tsdf::kMobileBit) != 0;
      rig = rig || (b != 0 && (b & tsdf::kMobileBit) == 0);
    }
    mixed += fill && rig ? 1 : 0;
  }
  return mixed;
}

// Fill seeded straight into `grid`: the blocks of [lo, hi] allocated, then
// each voxel there at weight 0 that `sdf` gives a value takes it at `weight`
// and `byte`. The seeded voxels' attribute indices.
template <typename Sdf>
vkc::Result<std::vector<std::size_t>> seed(
    vkc::test::GpuContext& gpu, vol::VoxelBlockGrid& grid, const vr::Vec3f& lo,
    const vr::Vec3f& hi, Sdf&& sdf, std::uint8_t byte, float weight = 2.0f) {
  const int bs = grid.grid().block_size;
  const float block = float(bs) * kVoxel;
  std::vector<vol::BlockIndex> coords;
  for (int z = int(std::floor(lo.z / block));
       z <= int(std::floor(hi.z / block)); ++z) {
    for (int y = int(std::floor(lo.y / block));
         y <= int(std::floor(hi.y / block)); ++y) {
      for (int x = int(std::floor(lo.x / block));
           x <= int(std::floor(hi.x / block)); ++x) {
        vol::BlockIndex b{};
        b.coord.x = x;
        b.coord.y = y;
        b.coord.z = z;
        coords.push_back(b);
      }
    }
  }
  const VkDeviceSize bytes = VkDeviceSize(coords.size()) * sizeof(coords[0]);
  VKC_ASSIGN(vkc::Buffer buf,
             vkc::device_storage_buffer(gpu.allocator, bytes,
                                        VK_BUFFER_USAGE_TRANSFER_DST_BIT));
  {
    vkc::CommandBatch batch(gpu.device, gpu.allocator);
    VKC_TRY(batch.upload(buf, 0, coords.data(), bytes));
    VKC_TRY(batch.submit());
  }
  VKC_ASSIGN(const std::uint32_t failed,
             grid.map().allocate(buf, std::uint32_t(coords.size())));
  if (failed != 0) return vkc::Status::out_of_memory("seed: allocation failed");
  VKC_ASSIGN(Voxels v, read_voxels(gpu, grid));
  std::vector<std::size_t> seeded;
  for (std::size_t k = 0; k < v.index.size(); ++k) {
    const vr::Vec3f& p = v.world[k];
    const std::size_t i = v.index[k];
    if (p.x < lo.x || p.y < lo.y || p.z < lo.z || p.x > hi.x || p.y > hi.y ||
        p.z > hi.z || v.weight[i] > 0.0f) {
      continue;
    }
    const std::optional<float> s = sdf(p);
    if (!s) continue;
    v.tsdf[i] = *s;
    v.weight[i] = weight;
    v.source[i] = byte;
    seeded.push_back(i);
  }
  VKC_TRY(vgrid_test::write_attribute(gpu.device, gpu.allocator, grid,
                                      vol::kTsdfAttribute, v.tsdf));
  VKC_TRY(vgrid_test::write_attribute(gpu.device, gpu.allocator, grid,
                                      vol::kWeightAttribute, v.weight));
  VKC_TRY(vgrid_test::write_attribute(gpu.device, gpu.allocator, grid,
                                      tsdf::kSourceAttribute, v.source));
  return seeded;
}

// A face of fill at world z = `z` facing the cameras: the band's signed
// distance, positive in front.
inline auto face(float z) {
  return [z](const vr::Vec3f& p) -> std::optional<float> {
    const float s = z - p.z;
    if (std::fabs(s) > kTrunc) return std::nullopt;
    return s;
  };
}

}  // namespace fill_test
