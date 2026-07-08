// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// GPU test for ProjectiveTexturer: texture a mesh of three triangles -- one the
// camera sees head-on, one hidden behind nearer geometry (a depth map closer
// than the triangle), one behind the camera -- and verify the visible triangle
// gets the expected projected atlas UVs while the occluded + behind ones keep
// the (-1,-1) sentinel. Then re-texture with a closer depth map and verify the
// once-visible triangle reverts to the sentinel (uv0 is overwritten each call).
// A translated pose confirms the world->camera transform is applied. Runs on
// the real driver (MoltenVK / NVIDIA); exits 0 (skip) where no device is
// present.

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "volumetric_kit/recon/core/allocator.hpp"
#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/instance.hpp"
#include "volumetric_kit/recon/core/math/vector_types.hpp"
#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/core/vulkan.hpp"
#include "volumetric_kit/recon/mesh/mesh.hpp"
#include "volumetric_kit/recon/texture/projective_texturer.hpp"
#include "volumetric_kit/recon/volume/voxel_hash_map.hpp"  // DepthCameraParams

namespace vr = volumetric_kit::recon;
namespace tex = volumetric_kit::recon::texture;
namespace vol = volumetric_kit::recon::volume;
namespace rmesh = volumetric_kit::recon::mesh;

#define CHECK(cond)                                                        \
  do {                                                                     \
    if (!(cond)) {                                                         \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      return 1;                                                            \
    }                                                                      \
  } while (0)

namespace {

bool approx(float a, float b, float eps) { return std::fabs(a - b) <= eps; }

bool is_sentinel(const vr::Vec2f& uv) { return uv.x == -1.0f && uv.y == -1.0f; }

// The shader's projection + atlas-UV math for the identity/translated pose, so
// the test predicts the exact uv0 a visible vertex must receive: world ->
// camera is (world - t) (identity rotation), pinhole u = fx*x/z + cx, then the
// +0.5 half-texel normalized by the image size and clamped half a texel inside.
vr::Vec2f expected_uv(const vr::Vec3f& world, const vr::Vec3f& t,
                      const vol::DepthCameraParams& cam) {
  const vr::Vec3f p = world - t;
  const float u = cam.fx * (p.x / p.z) + cam.cx;
  const float v = cam.fy * (p.y / p.z) + cam.cy;
  const float w = static_cast<float>(cam.width);
  const float h = static_cast<float>(cam.height);
  float au = (u + 0.5f) / w;
  float av = (v + 0.5f) / h;
  au = std::fmin(std::fmax(au, 0.5f / w), 1.0f - 0.5f / w);
  av = std::fmin(std::fmax(av, 0.5f / h), 1.0f - 0.5f / h);
  return vr::Vec2f(au, av);
}

// A vertex with the given world position; normal/color are irrelevant to the
// single-camera view-selection path (occlusion uses only position + depth).
rmesh::Vertex vtx(float x, float y, float z) {
  rmesh::Vertex v{};
  v.position = vr::Vec3f(x, y, z);
  v.normal = vr::Vec3f(0.0f, 0.0f, -1.0f);
  v.color = vr::Vec4f(1.0f, 1.0f, 1.0f, 1.0f);
  v.uv0 = vr::Vec2f(0.25f, 0.25f);  // non-sentinel, must be overwritten
  return v;
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

  vr::Result<tex::ProjectiveTexturer> tex_result =
      tex::ProjectiveTexturer::create(device.value(), allocator.value());
  if (!tex_result) {
    std::fprintf(stderr, "ProjectiveTexturer::create failed: %s\n",
                 tex_result.status().message().c_str());
    return 1;
  }
  tex::ProjectiveTexturer texturer = std::move(tex_result).value();

  // Camera at the world origin looking down +Z (identity pose), 640x480
  // pinhole.
  vol::DepthCameraParams cam{};
  cam.fx = 500.0f;
  cam.fy = 500.0f;
  cam.cx = 320.0f;
  cam.cy = 240.0f;
  cam.min_depth = 0.1f;
  cam.max_depth = 10.0f;
  cam.width = 640;
  cam.height = 480;
  cam.cam_to_world = vr::Mat4f(1.0f);

  // Triangle A at z = 1 m, near the optical axis -> in front, in frame.
  // Triangle B at z = 2 m projects into frame too, but the depth map (below)
  // reads 1 m there, so B is occluded by nearer geometry. Triangle C at z = -1
  // m is behind the camera. Nine independent vertices (the mesh tier's shape),
  // indices 0..8.
  rmesh::Mesh mesh;
  mesh.vertices = {vtx(-0.1f, -0.1f, 1.0f),  vtx(0.1f, -0.1f, 1.0f),
                   vtx(0.0f, 0.1f, 1.0f),    vtx(-0.1f, -0.1f, 2.0f),
                   vtx(0.1f, -0.1f, 2.0f),   vtx(0.0f, 0.1f, 2.0f),
                   vtx(-0.1f, -0.1f, -1.0f), vtx(0.1f, -0.1f, -1.0f),
                   vtx(0.0f, 0.1f, -1.0f)};
  mesh.indices = {0, 1, 2, 3, 4, 5, 6, 7, 8};

  // A depth map at a constant 1 m: triangle A sits on the surface (|d - z| =
  // 0), triangle B is 1 m behind it (|1 - 2| = 1 > threshold -> occluded).
  std::vector<float> depth(static_cast<std::size_t>(cam.width) * cam.height,
                           1.0f);

  CHECK(texturer.texture(mesh, depth.data(), cam).ok());

  // Triangle A: every vertex textured with the exact projected atlas UV.
  const vr::Vec3f no_t(0.0f, 0.0f, 0.0f);
  for (int i = 0; i < 3; ++i) {
    const vr::Vec2f uv = mesh.vertices[i].uv0;
    const vr::Vec2f want = expected_uv(mesh.vertices[i].position, no_t, cam);
    CHECK(!is_sentinel(uv));
    CHECK(uv.x > 0.0f && uv.x < 1.0f && uv.y > 0.0f && uv.y < 1.0f);
    CHECK(approx(uv.x, want.x, 1e-5f) && approx(uv.y, want.y, 1e-5f));
  }
  // Triangle B (occluded) and C (behind the camera): the sentinel, which
  // overwrote the vtx() initial (0.25, 0.25) -- so the renderer falls back to
  // per-vertex color.
  for (int i = 3; i < 9; ++i) {
    CHECK(is_sentinel(mesh.vertices[i].uv0));
  }

  // Re-texture with a closer depth map (0.5 m everywhere): triangle A is now
  // 0.5 m behind the surface (|0.5 - 1| = 0.5 > threshold), so it too is
  // occluded -- and its previously-written UVs are overwritten back to the
  // sentinel.
  std::vector<float> depth_near(depth.size(), 0.5f);
  CHECK(texturer.texture(mesh, depth_near.data(), cam).ok());
  for (int i = 0; i < 9; ++i) {
    CHECK(is_sentinel(mesh.vertices[i].uv0));
  }

  // Translated pose: move the camera +0.2 m in world X (still looking down +Z).
  // Triangle A stays at depth 1 m (X-translation does not change camera Z), so
  // it is visible again, but every vertex projects 0.2 m * fx / z = 100 px to
  // the left -- proving the world->camera transform is applied, not ignored.
  rmesh::Mesh mesh2;
  mesh2.vertices = {vtx(-0.1f, -0.1f, 1.0f), vtx(0.1f, -0.1f, 1.0f),
                    vtx(0.0f, 0.1f, 1.0f)};
  mesh2.indices = {0, 1, 2};
  vol::DepthCameraParams cam_t = cam;
  const vr::Vec3f t(0.2f, 0.0f, 0.0f);
  cam_t.cam_to_world = vr::Mat4f(1.0f);
  cam_t.cam_to_world[3] = vr::Vec4f(t, 1.0f);
  CHECK(texturer.texture(mesh2, depth.data(), cam_t).ok());
  for (int i = 0; i < 3; ++i) {
    const vr::Vec2f uv = mesh2.vertices[i].uv0;
    const vr::Vec2f want = expected_uv(mesh2.vertices[i].position, t, cam_t);
    const vr::Vec2f ident =
        expected_uv(mesh2.vertices[i].position, no_t, cam_t);
    CHECK(!is_sentinel(uv));
    CHECK(approx(uv.x, want.x, 1e-5f) && approx(uv.y, want.y, 1e-5f));
    CHECK(uv.x < ident.x);  // shifted left by the +X camera translation
  }

  std::printf(
      "recon texture projective test passed: 1 triangle textured with exact "
      "projected UVs, occluded + behind-camera triangles kept the sentinel, a "
      "closer depth map reverted the textured triangle, and a translated pose "
      "shifted the projection\n");
  return 0;
}
