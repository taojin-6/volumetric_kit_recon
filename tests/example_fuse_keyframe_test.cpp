// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// A failed prepare or fuse must leave the viewer's last successful keyframe
// available for texturing, and the frame is fused in the integration mode the
// helper is given, the one fusion.dynamic selects: under Dynamic with the
// depth noise its driver stated, through the prep, and refused when the
// driver states none. Exercise the actual example helper and GPU passes, with
// no window, camera or dataset. The device part skips without a device.

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include "depth_inputs.hpp"
#include "fuse_frame.hpp"
#include "volumetric_kit/core/testing/buffer_readback.hpp"
#include "volumetric_kit/core/testing/gpu_test.hpp"
#include "volumetric_kit/core/testing/test_check.hpp"
#include "volumetric_kit/grid/testing/grid_readback.hpp"
#include "volumetric_kit/grid/volume/voxel_grid.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"
#include "volumetric_kit/recon/core/math/vector_types.hpp"
#include "volumetric_kit/recon/texture/projective_texturer.hpp"
#include "volumetric_kit/recon/tsdf/fusion_grid.hpp"

namespace vr = volumetric_kit::recon;

namespace vgrid = volumetric_kit::grid;
namespace vkc = volumetric_kit::core;
namespace sensor = volumetric_kit::sensor;
constexpr vr::tsdf::IntegrationMode kClassic =
    vr::tsdf::IntegrationMode::Classic;
constexpr vr::tsdf::IntegrationMode kDynamic =
    vr::tsdf::IntegrationMode::Dynamic;

namespace {

constexpr std::uint32_t kWidth = 16, kHeight = 12;
constexpr std::size_t kPixels = kWidth * kHeight;
constexpr std::uint32_t kColor = 0x00563412u;

// A wall facing an identity camera: `depth` (mm) and `color` are its pixels;
// its depth noise the Femto Mega's, as that driver states it.
sensor::RgbdFrame wall(const std::vector<std::uint16_t>& depth,
                       const std::vector<std::uint32_t>& color) {
  sensor::RgbdFrame frame;
  frame.depth = depth.data();
  frame.depth_camera = {{kWidth, kHeight}, {16.0, 16.0, 7.5, 5.5}, {}};
  frame.color_camera = frame.depth_camera;
  frame.color_packed = color.data();
  frame.metres_per_unit = 0.001f;
  frame.min_depth = 0.1f;
  frame.max_depth = 5.0f;
  frame.depth_noise = vr_test::kFemtoNoise;
  frame.timestamp_ns = 1;
  return frame;
}

int check_retained(const vkc::test::Gpu& gpu,
                   vr::texture::ProjectiveTexturer& texturer,
                   const std::optional<sensor::DeviceFrame>& keyframe) {
  CHECK(keyframe && keyframe->has_color());
  CHECK(keyframe->timestamp_ns == 1);
  CHECK(keyframe->depth_noise == vr_test::kFemtoNoise);
  CHECK(keyframe->depth_camera.cam_to_world == vr::Mat4f(1.0f));
  CHECK(keyframe->color_camera.cam_to_world == vr::Mat4f(1.0f));
  auto depth = vkc::test::read_back<float>(gpu.device, gpu.allocator,
                                           *keyframe->depth, kPixels);
  auto color = vkc::test::read_back<std::uint32_t>(gpu.device, gpu.allocator,
                                                   *keyframe->color, kPixels);
  CHECK(depth.ok() && color.ok());
  CHECK(*depth == std::vector<float>(kPixels, 1.0f));
  CHECK(*color == std::vector<std::uint32_t>(kPixels, kColor | 0xFF000000u));

  // The final/remeshed surface can still use this frame's atlas, including
  // coverage and occlusion, after the source or fusion rejected a later one.
  vgrid::mesh::Mesh mesh;
  mesh.vertices.resize(3);
  mesh.indices = {0, 1, 2};
  mesh.vertices[0].position = {-0.05f, -0.05f, 1.0f};
  mesh.vertices[1].position = {0.05f, -0.05f, 1.0f};
  mesh.vertices[2].position = {0.0f, 0.05f, 1.0f};
  for (auto& vertex : mesh.vertices) {
    vertex.normal = {0.0f, 0.0f, -1.0f};
    vertex.uv0 = {-1.0f, -1.0f};
  }
  vr::texture::TextureView view;
  view.cam = keyframe->depth_camera;
  view.depth_buffer = keyframe->depth;
  view.color_camera = keyframe->color_camera;
  view.coverage = keyframe->color;
  CHECK(texturer.texture(mesh, view).ok());
  for (const auto& vertex : mesh.vertices) {
    CHECK(vertex.uv0.x > 0.0f && vertex.uv0.x < 1.0f);
    CHECK(vertex.uv0.y > 0.0f && vertex.uv0.y < 1.0f);
  }
  return 0;
}

// fuse_keyframe fuses in the mode it is given: a wall at 1 m, then the same
// camera seeing it at 2 m, so the first wall's band is free space well ahead
// of the second's. Classic leaves weight there; Dynamic, its change test
// running from a pixel's second sample, resets all of it. Dynamic refuses a
// frame whose driver states no depth noise, keeping no keyframe; Classic
// fuses it.
int check_mode(vkc::test::GpuContext& gpu, vr::tsdf::IntegrationMode mode) {
  auto volume = vr::tsdf::create_fusion_grid(gpu.device, gpu.allocator, 0.05f,
                                             0.15f, 256);
  vr::tsdf::FuserConfig config;
  config.change.warm_up = 1;
  auto fuser = vr::tsdf::Fuser::create(gpu.device, gpu.allocator, config);
  auto prep = sensor::GpuFramePrep::create(gpu.device, gpu.allocator);
  CHECK(volume.ok() && fuser.ok() && prep.ok());
  // The weight in blocks wholly nearer than 1.6 m: a block is 8 voxels of
  // 5 cm, so its z coordinate is at most 3.
  const auto near_weight = [&]() -> double {
    auto blocks = vgrid_test::blocks_of(*volume);
    auto weight = vgrid_test::read_attribute<float>(
        gpu.device, gpu.allocator, *volume, vgrid::volume::kWeightAttribute);
    if (!blocks.ok() || !weight.ok()) return -1.0;
    double sum = 0.0;
    for (const auto& [coord, ptr] : *blocks) {
      if (std::get<2>(coord) > 3) continue;
      for (std::int32_t v = 0; v < 512; ++v) {
        sum += (*weight)[std::size_t(ptr + v)];
      }
    }
    return sum;
  };
  std::vector<std::uint16_t> depth(kPixels, 1000);
  const std::vector<std::uint32_t> color(kPixels, kColor);
  sensor::RgbdFrame frame = wall(depth, color);
  std::optional<sensor::DeviceFrame> keyframe;
  if (mode == kDynamic) {
    sensor::RgbdFrame unstated = frame;
    unstated.depth_noise = {};
    const vkc::Status refused = vr_example::fuse_keyframe(
        *fuser, *volume, *prep, unstated, keyframe, 20.0f, mode, nullptr);
    CHECK(refused.domain() == vkc::Status::Code::InvalidArgument);
    CHECK(refused.message().find("states no depth noise") != std::string::npos);
    CHECK(!keyframe);
  }
  CHECK(vr_example::fuse_keyframe(*fuser, *volume, *prep, frame, keyframe,
                                  20.0f, mode, nullptr)
            .ok());
  CHECK(keyframe && keyframe->depth_noise == vr_test::kFemtoNoise);
  CHECK(near_weight() > 0.0);
  std::fill(depth.begin(), depth.end(), 2000);
  frame.timestamp_ns = 2;
  CHECK(vr_example::fuse_keyframe(*fuser, *volume, *prep, frame, keyframe,
                                  20.0f, mode, nullptr)
            .ok());
  const double left = near_weight();
  CHECK(mode == kDynamic ? left == 0.0 : left > 0.0);
  return 0;
}

int gpu_main(vkc::test::GpuContext& gpu) {
  auto volume = vr::tsdf::create_fusion_grid(gpu.device, gpu.allocator, 0.05f,
                                             0.15f, 256);
  auto fuser = vr::tsdf::Fuser::create(gpu.device, gpu.allocator);
  auto prep = sensor::GpuFramePrep::create(gpu.device, gpu.allocator);
  auto texturer =
      vr::texture::ProjectiveTexturer::create(gpu.device, gpu.allocator);
  CHECK(volume.ok() && fuser.ok() && prep.ok() && texturer.ok());

  std::vector<std::uint16_t> raw_depth(kPixels, 1000);
  std::vector<std::uint32_t> raw_color(kPixels, kColor);
  sensor::RgbdFrame frame = wall(raw_depth, raw_color);
  std::optional<sensor::DeviceFrame> keyframe;

  // A rejected first frame must not invent a keyframe.
  frame.color_to_world[0][0] = 2.0;
  CHECK(!vr_example::fuse_keyframe(*fuser, *volume, *prep, frame, keyframe,
                                   20.0f, kClassic, nullptr)
             .ok());
  CHECK(!keyframe);
  frame.color_to_world = sensor::camera::Mat4d(1.0);
  CHECK(vr_example::fuse_keyframe(*fuser, *volume, *prep, frame, keyframe,
                                  20.0f, kClassic, nullptr)
            .ok());
  CHECK(check_retained(gpu, *texturer, keyframe) == 0);

  // A later bad pose fails preparation without discarding the good frame.
  frame.timestamp_ns = 2;
  frame.color_to_world[0][0] = 2.0;
  const auto refused = vr_example::fuse_keyframe(
      *fuser, *volume, *prep, frame, keyframe, 20.0f, kClassic, nullptr);
  CHECK(!refused.ok());
  CHECK(refused.message().find("GpuFramePrep") != std::string::npos);
  CHECK(check_retained(gpu, *texturer, keyframe) == 0);

  // Preparation and allocation succeed, but fusion refuses a volume without
  // a colour attribute. Different pixels and pose expose an early replacement
  // or reuse of the retained buffers even if the optional stays engaged.
  const vgrid::volume::AttributeSpec attrs[] = {
      {vgrid::volume::kTsdfAttribute, sizeof(float)},
      {vgrid::volume::kWeightAttribute, sizeof(float)}};
  auto depth_only = vgrid::volume::VoxelBlockGrid::create(
      gpu.device, gpu.allocator,
      vgrid::volume::block_grid_params(0.05f, 0.15f, 256), attrs, 2);
  CHECK(depth_only.ok());
  std::fill(raw_depth.begin(), raw_depth.end(), 2000);
  std::fill(raw_color.begin(), raw_color.end(), 0x001122CCu);
  frame.color_to_world = sensor::camera::Mat4d(1.0);
  frame.color_to_world[3][0] = 0.25;
  // A second fuser: the first remembers the grid it fuses.
  auto depth_only_fuser = vr::tsdf::Fuser::create(gpu.device, gpu.allocator);
  CHECK(depth_only_fuser.ok());
  const auto failed =
      vr_example::fuse_keyframe(*depth_only_fuser, *depth_only, *prep, frame,
                                keyframe, 20.0f, kClassic, nullptr);
  CHECK(failed.domain() == vkc::Status::Code::InvalidArgument);
  CHECK(failed.message().find("VoxelBlockGrid::attribute") !=
        std::string::npos);
  CHECK(check_retained(gpu, *texturer, keyframe) == 0);

  // A subsequent successful fuse commits the new frame and its atlas.
  CHECK(vr_example::fuse_keyframe(*fuser, *volume, *prep, frame, keyframe,
                                  20.0f, kClassic, nullptr)
            .ok());
  CHECK(keyframe && keyframe->timestamp_ns == 2);
  CHECK(keyframe->color_camera.cam_to_world[3][0] == 0.25f);
  auto depth = vkc::test::read_back<float>(gpu.device, gpu.allocator,
                                           *keyframe->depth, kPixels);
  auto color = vkc::test::read_back<std::uint32_t>(gpu.device, gpu.allocator,
                                                   *keyframe->color, kPixels);
  CHECK(depth.ok() && *depth == std::vector<float>(kPixels, 2.0f));
  CHECK(color.ok() &&
        *color == std::vector<std::uint32_t>(kPixels, 0xFF1122CCu));

  CHECK(check_mode(gpu, kClassic) == 0);
  CHECK(check_mode(gpu, kDynamic) == 0);
  std::puts("example_fuse_keyframe: OK");
  return 0;
}

}  // namespace

int main() {
  CHECK(vr_example::integration_mode(true) == kDynamic);
  CHECK(vr_example::integration_mode(false) == kClassic);
  return vkc::test::run_on_gpu(vr::device_requirements(), gpu_main);
}
