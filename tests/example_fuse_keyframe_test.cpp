// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// A failed prepare or fuse must leave the viewer's last successful keyframe
// available for texturing. Exercise the actual example helper and GPU passes,
// with no window, camera or dataset. Skips without a device.

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

#include "buffer_readback.hpp"
#include "fuse_frame.hpp"
#include "gpu_test.hpp"
#include "test_check.hpp"
#include "volumetric_kit/recon/texture/projective_texturer.hpp"

namespace vr = volumetric_kit::recon;
namespace vkc = volumetric_kit::core;
namespace sensor = vr::sensor;

namespace {

constexpr std::uint32_t kWidth = 16, kHeight = 12;
constexpr std::size_t kPixels = kWidth * kHeight;
constexpr std::uint32_t kColor = 0x00563412u;

int check_retained(const vr_test::Gpu& gpu,
                   vr::texture::ProjectiveTexturer& texturer,
                   const std::optional<sensor::DeviceFrame>& keyframe) {
  CHECK(keyframe && keyframe->has_color());
  CHECK(keyframe->timestamp_ns == 1);
  CHECK(keyframe->depth_camera.cam_to_world == vr::Mat4f(1.0f));
  CHECK(keyframe->color_camera.cam_to_world == vr::Mat4f(1.0f));
  auto depth = vr_test::read_back<float>(gpu.device, gpu.allocator,
                                         *keyframe->depth, kPixels);
  auto color = vr_test::read_back<std::uint32_t>(gpu.device, gpu.allocator,
                                                 *keyframe->color, kPixels);
  CHECK(depth.ok() && color.ok());
  CHECK(*depth == std::vector<float>(kPixels, 1.0f));
  CHECK(*color == std::vector<std::uint32_t>(kPixels, kColor | 0xFF000000u));

  // The final/remeshed surface can still use this frame's atlas, including
  // coverage and occlusion, after the source or fusion rejected a later one.
  vr::mesh::Mesh mesh;
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

int gpu_main(vr_test::GpuContext& gpu) {
  auto volume = vr_example::create_fusion_grid(gpu.device, gpu.allocator, 0.05f,
                                               0.15f, 256);
  auto fuser = vr::tsdf::Fuser::create(gpu.device, gpu.allocator);
  auto prep = sensor::GpuFramePrep::create(gpu.device, gpu.allocator);
  auto texturer =
      vr::texture::ProjectiveTexturer::create(gpu.device, gpu.allocator);
  CHECK(volume.ok() && fuser.ok() && prep.ok() && texturer.ok());

  std::vector<std::uint16_t> raw_depth(kPixels, 1000);
  std::vector<std::uint32_t> raw_color(kPixels, kColor);
  sensor::RgbdFrame frame;
  frame.depth = raw_depth.data();
  frame.depth_camera = {{kWidth, kHeight}, {16.0, 16.0, 7.5, 5.5}, {}};
  frame.color_camera = frame.depth_camera;
  frame.color_packed = raw_color.data();
  frame.metres_per_unit = 0.001f;
  frame.min_depth = 0.1f;
  frame.max_depth = 5.0f;
  frame.timestamp_ns = 1;
  std::optional<sensor::DeviceFrame> keyframe;

  // A rejected first frame must not invent a keyframe.
  frame.color_to_world[0][0] = 2.0;
  CHECK(!vr_example::fuse_keyframe(*fuser, *volume, *prep, frame, keyframe,
                                   20.0f, nullptr)
             .ok());
  CHECK(!keyframe);
  frame.color_to_world = vr::camera::Mat4d(1.0);
  CHECK(vr_example::fuse_keyframe(*fuser, *volume, *prep, frame, keyframe,
                                  20.0f, nullptr)
            .ok());
  CHECK(check_retained(gpu, *texturer, keyframe) == 0);

  // A later bad pose fails preparation without discarding the good frame.
  frame.timestamp_ns = 2;
  frame.color_to_world[0][0] = 2.0;
  const auto refused = vr_example::fuse_keyframe(*fuser, *volume, *prep, frame,
                                                 keyframe, 20.0f, nullptr);
  CHECK(!refused.ok());
  CHECK(refused.message().find("GpuFramePrep") != std::string::npos);
  CHECK(check_retained(gpu, *texturer, keyframe) == 0);

  // Preparation and allocation succeed, but fusion refuses a volume without
  // a colour attribute. Different pixels and pose expose an early replacement
  // or reuse of the retained buffers even if the optional stays engaged.
  const vr::volume::AttributeSpec attrs[] = {{"tsdf", sizeof(float)},
                                             {"weight", sizeof(float)}};
  auto depth_only = vr::volume::VoxelBlockGrid::create(
      gpu.device, gpu.allocator,
      vr_example::example_grid_params(0.05f, 0.15f, 256), attrs, 2);
  CHECK(depth_only.ok());
  std::fill(raw_depth.begin(), raw_depth.end(), 2000);
  std::fill(raw_color.begin(), raw_color.end(), 0x001122CCu);
  frame.color_to_world = vr::camera::Mat4d(1.0);
  frame.color_to_world[3][0] = 0.25;
  // A second fuser: the first remembers the grid it fuses.
  auto depth_only_fuser = vr::tsdf::Fuser::create(gpu.device, gpu.allocator);
  CHECK(depth_only_fuser.ok());
  const auto failed = vr_example::fuse_keyframe(
      *depth_only_fuser, *depth_only, *prep, frame, keyframe, 20.0f, nullptr);
  CHECK(failed.domain() == vkc::Status::Code::InvalidArgument);
  CHECK(failed.message().find("VoxelBlockGrid::attribute") !=
        std::string::npos);
  CHECK(check_retained(gpu, *texturer, keyframe) == 0);

  // A subsequent successful fuse commits the new frame and its atlas.
  CHECK(vr_example::fuse_keyframe(*fuser, *volume, *prep, frame, keyframe,
                                  20.0f, nullptr)
            .ok());
  CHECK(keyframe && keyframe->timestamp_ns == 2);
  CHECK(keyframe->color_camera.cam_to_world[3][0] == 0.25f);
  auto depth = vr_test::read_back<float>(gpu.device, gpu.allocator,
                                         *keyframe->depth, kPixels);
  auto color = vr_test::read_back<std::uint32_t>(gpu.device, gpu.allocator,
                                                 *keyframe->color, kPixels);
  CHECK(depth.ok() && *depth == std::vector<float>(kPixels, 2.0f));
  CHECK(color.ok() &&
        *color == std::vector<std::uint32_t>(kPixels, 0xFF1122CCu));
  std::puts("example_fuse_keyframe: OK");
  return 0;
}

}  // namespace

int main() { return vr_test::run_on_gpu(gpu_main); }
