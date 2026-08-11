// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// GPU test for ProjectiveTexturer: texture a mesh of three triangles -- one the
// camera sees head-on, one hidden behind nearer geometry (a depth map closer
// than the triangle), one behind the camera -- and verify each of the THREE uv0
// outcomes. The visible one gets the expected projected atlas UVs; the occluded
// one is in frame, so it selects the vertex colour while CARRYING its
// coordinate as -uv-1 (decoded and compared against the projection, which is
// what pins this encoding to gfx's decode in hybrid_mesh.vert); the
// behind-camera one projects nowhere and keeps the bare (-1,-1). Then
// re-texture with a closer depth map and verify the once-visible triangle
// reverts (uv0 is overwritten each call). A translated pose confirms the
// world->camera transform is applied. Then four discriminating cases: the
// occlusion threshold accepts a small within-tolerance offset and rejects an
// out-of-tolerance one (and honours an explicit tighter threshold) -- so the
// head-on triangle's |d - zc| = 0 was not hiding a dropped/zeroed threshold; a
// rotated pose projects through R^T to hand-computed pixels (the identity poses
// above never exercise the rotation); and a depth discontinuity across the
// bilinear taps textures a foreground vertex via the nearest-tap fallback (the
// discontinuity guard). Runs on the real driver (MoltenVK / NVIDIA); exits 0
// (skip) where no device is present.

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

// Does this uv0 select the per-vertex colour? This is the RENDERER's test
// (gfx's hybrid_mesh.vert) -- the sign, not an exact value -- and asserting it
// rather than `uv == (-1,-1)` is what lets the two non-visible outcomes below
// be told apart without either of them looking like a regression.
bool uses_vertex_color(const vr::Vec2f& uv) { return uv.x < 0.0f; }

// The vertex projected nowhere -- behind the camera, or outside the image -- so
// no atlas coordinate exists to carry and uv0 is the bare sentinel.
bool is_offscreen(const vr::Vec2f& uv) {
  return uv.x == -1.0f && uv.y == -1.0f;
}

// Recover the atlas coordinate a non-visible but IN-FRAME vertex carries: the
// inverse of the kernel's `-uv - 1`. Kept here as its own function because it
// must stay identical to hybrid_mesh.vert's decode -- the two are one contract
// written twice, and this test is the only thing that can notice them drifting.
vr::Vec2f decode_carried(const vr::Vec2f& uv) {
  return vr::Vec2f(-uv.x - 1.0f, -uv.y - 1.0f);
}

// The shader's projection + atlas-UV math, so the test predicts the exact uv0 a
// visible vertex must receive. world -> camera is R^T (world - t) with R (the
// camera axes) and t read from cam.cam_to_world -- the rigid inverse the shader
// computes -- so this handles a rotated pose, not only translation; pinhole
// u = fx*x/z + cx; then the +0.5 half-texel normalized by the image size and
// clamped half a texel inside.
vr::Vec2f expected_uv(const vr::Vec3f& world,
                      const vr::DepthCameraParams& cam) {
  const vr::Vec3f d = world - vr::Vec3f(cam.cam_to_world[3]);
  const vr::Vec3f cam_x(cam.cam_to_world[0]);  // camera X axis in world
  const vr::Vec3f cam_y(cam.cam_to_world[1]);  // camera Y axis in world
  const vr::Vec3f cam_z(cam.cam_to_world[2]);  // camera Z axis in world
  // R^T (world - t): the rows of R^T are the columns of R (the camera axes).
  const vr::Vec3f p(vr::dot(cam_x, d), vr::dot(cam_y, d), vr::dot(cam_z, d));
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
  vr::DepthCameraParams cam{};
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
  for (int i = 0; i < 3; ++i) {
    const vr::Vec2f uv = mesh.vertices[i].uv0;
    const vr::Vec2f want = expected_uv(mesh.vertices[i].position, cam);
    CHECK(!uses_vertex_color(uv));
    CHECK(uv.x > 0.0f && uv.x < 1.0f && uv.y > 0.0f && uv.y < 1.0f);
    CHECK(approx(uv.x, want.x, 1e-5f) && approx(uv.y, want.y, 1e-5f));
  }
  // Triangle B: in frame, but occluded by the nearer surface. It selects the
  // per-vertex colour like any non-visible vertex -- and because it HAS an
  // atlas coordinate, uv0 carries that coordinate as `-uv - 1` instead of
  // collapsing to the bare sentinel. Both halves are asserted: the class the
  // renderer reads, and the coordinate it decodes back out.
  //
  // The carried coordinate is the part worth pinning. It is what stops a
  // triangle straddling the visibility boundary from interpolating toward the
  // atlas origin and smearing the corner of the image across its face, which is
  // how a shared-vertex mesh used to render along every silhouette.
  for (int i = 3; i < 6; ++i) {
    const vr::Vec2f uv = mesh.vertices[i].uv0;
    const vr::Vec2f want = expected_uv(mesh.vertices[i].position, cam);
    CHECK(uses_vertex_color(uv));
    CHECK(!is_offscreen(uv));
    const vr::Vec2f carried = decode_carried(uv);
    CHECK(approx(carried.x, want.x, 1e-5f) && approx(carried.y, want.y, 1e-5f));
  }
  // Triangle C: behind the camera, so it projects nowhere and there is nothing
  // to carry -- the bare (-1,-1). Both outcomes overwrote the vtx() initial
  // (0.25, 0.25), so the renderer falls back to per-vertex colour either way.
  for (int i = 6; i < 9; ++i) {
    CHECK(uses_vertex_color(mesh.vertices[i].uv0));
    CHECK(is_offscreen(mesh.vertices[i].uv0));
  }

  // Re-texture with a closer depth map (0.5 m everywhere): triangle A is now
  // 0.5 m behind the surface (|0.5 - 1| = 0.5 > threshold), so it too is
  // occluded -- and its previously-written visible UVs are overwritten. A and B
  // are still in frame, so they carry their coordinates; only C, which projects
  // nowhere, is the bare sentinel. This is the "a vertex that leaves the view
  // reverts rather than keeping a stale coordinate" property.
  std::vector<float> depth_near(depth.size(), 0.5f);
  CHECK(texturer.texture(mesh, depth_near.data(), cam).ok());
  for (int i = 0; i < 6; ++i) {
    const vr::Vec2f uv = mesh.vertices[i].uv0;
    const vr::Vec2f want = expected_uv(mesh.vertices[i].position, cam);
    CHECK(uses_vertex_color(uv));
    const vr::Vec2f carried = decode_carried(uv);
    CHECK(approx(carried.x, want.x, 1e-5f) && approx(carried.y, want.y, 1e-5f));
  }
  for (int i = 6; i < 9; ++i) {
    CHECK(is_offscreen(mesh.vertices[i].uv0));
  }

  // Translated pose: move the camera +0.2 m in world X (still looking down +Z).
  // Triangle A stays at depth 1 m (X-translation does not change camera Z), so
  // it is visible again, but every vertex projects 0.2 m * fx / z = 100 px to
  // the left -- proving the world->camera transform is applied, not ignored.
  rmesh::Mesh mesh2;
  mesh2.vertices = {vtx(-0.1f, -0.1f, 1.0f), vtx(0.1f, -0.1f, 1.0f),
                    vtx(0.0f, 0.1f, 1.0f)};
  mesh2.indices = {0, 1, 2};
  vr::DepthCameraParams cam_t = cam;
  const vr::Vec3f t(0.2f, 0.0f, 0.0f);
  cam_t.cam_to_world = vr::Mat4f(1.0f);
  cam_t.cam_to_world[3] = vr::Vec4f(t, 1.0f);
  CHECK(texturer.texture(mesh2, depth.data(), cam_t).ok());
  for (int i = 0; i < 3; ++i) {
    const vr::Vec2f uv = mesh2.vertices[i].uv0;
    const vr::Vec2f want = expected_uv(mesh2.vertices[i].position, cam_t);
    // The same vertex through the un-translated (identity) camera, for the
    // shifted-left comparison.
    const vr::Vec2f ident = expected_uv(mesh2.vertices[i].position, cam);
    CHECK(!uses_vertex_color(uv));
    CHECK(approx(uv.x, want.x, 1e-5f) && approx(uv.y, want.y, 1e-5f));
    CHECK(uv.x < ident.x);  // shifted left by the +X camera translation
  }

  // Occlusion-threshold discrimination: with the 1 m depth map, a triangle a
  // small distance IN FRONT of the surface is textured iff that distance is
  // within occlusion_threshold. The visible triangle above sat exactly on the
  // surface (|d - zc| = 0), which passes for any threshold >= 0; these cases
  // pin the threshold to a nonzero value and prove it is actually consulted.
  auto near_axis_tri = [](float z) {
    rmesh::Mesh m;
    m.vertices = {vtx(-0.02f, -0.02f, z), vtx(0.02f, -0.02f, z),
                  vtx(0.0f, 0.02f, z)};
    m.indices = {0, 1, 2};
    return m;
  };
  // z = 1.015 m -> |1.0 - 1.015| = 0.015 < the 0.02 default -> textured.
  rmesh::Mesh mesh_in = near_axis_tri(1.015f);
  CHECK(texturer.texture(mesh_in, depth.data(), cam).ok());
  for (int i = 0; i < 3; ++i) {
    CHECK(!uses_vertex_color(mesh_in.vertices[i].uv0));
  }
  // z = 1.03 m -> 0.03 > 0.02 -> occluded (a broken/zeroed threshold would fail
  // here by texturing it). In frame throughout, so these carry their
  // coordinates rather than going offscreen-sentinel -- checked, because
  // `uses_vertex_color` alone would also pass on a kernel that had stopped
  // projecting at all.
  rmesh::Mesh mesh_out = near_axis_tri(1.03f);
  CHECK(texturer.texture(mesh_out, depth.data(), cam).ok());
  for (int i = 0; i < 3; ++i) {
    CHECK(uses_vertex_color(mesh_out.vertices[i].uv0));
    CHECK(!is_offscreen(mesh_out.vertices[i].uv0));
  }
  // z = 1.015 m again, but an explicit tighter 0.01 threshold -> 0.015 > 0.01
  // -> occluded, proving the threshold argument is honoured.
  rmesh::Mesh mesh_tight = near_axis_tri(1.015f);
  CHECK(texturer.texture(mesh_tight, depth.data(), cam, 0.01f).ok());
  for (int i = 0; i < 3; ++i) {
    CHECK(uses_vertex_color(mesh_tight.vertices[i].uv0));
    CHECK(!is_offscreen(mesh_tight.vertices[i].uv0));
  }

  // Rotated pose: rotate the camera 90 deg about world +Y so it looks down
  // world +X (camera +Z axis -> world +X). This exercises the R^T rotation the
  // shader applies, which the identity-rotation poses above leave untested. A
  // point 2 m along +X is straight ahead (projects to the principal point); a
  // +Z world offset maps to -X in camera space (u shifts left). The
  // principal-point assertion is hand-computed, independent of expected_uv's
  // own formula.
  rmesh::Mesh mesh3;
  mesh3.vertices = {vtx(2.0f, 0.0f, 0.0f), vtx(2.0f, 0.1f, 0.0f),
                    vtx(2.0f, 0.0f, 0.1f)};
  mesh3.indices = {0, 1, 2};
  vr::DepthCameraParams cam_r = cam;
  cam_r.cam_to_world = vr::Mat4f(1.0f);
  cam_r.cam_to_world[0] =
      vr::Vec4f(0.0f, 0.0f, -1.0f, 0.0f);  // camX -> world -Z
  cam_r.cam_to_world[1] =
      vr::Vec4f(0.0f, 1.0f, 0.0f, 0.0f);  // camY -> world +Y
  cam_r.cam_to_world[2] =
      vr::Vec4f(1.0f, 0.0f, 0.0f, 0.0f);  // camZ -> world +X
  std::vector<float> depth_2m(depth.size(),
                              2.0f);  // all three vertices at zc=2m
  CHECK(texturer.texture(mesh3, depth_2m.data(), cam_r).ok());
  CHECK(!uses_vertex_color(mesh3.vertices[0].uv0));
  CHECK(approx(mesh3.vertices[0].uv0.x, (320.0f + 0.5f) / 640.0f, 1e-5f));
  CHECK(approx(mesh3.vertices[0].uv0.y, (240.0f + 0.5f) / 480.0f, 1e-5f));
  for (int i = 0; i < 3; ++i) {
    const vr::Vec2f uv = mesh3.vertices[i].uv0;
    const vr::Vec2f want = expected_uv(mesh3.vertices[i].position, cam_r);
    CHECK(!uses_vertex_color(uv));
    CHECK(approx(uv.x, want.x, 1e-5f) && approx(uv.y, want.y, 1e-5f));
  }
  // World +Z maps to camera -X, so the +Z vertex sits left of the on-axis one.
  CHECK(mesh3.vertices[2].uv0.x < mesh3.vertices[0].uv0.x);

  // Depth discontinuity: a foreground vertex projecting onto a depth edge must
  // texture via the nearest-tap fallback, not be rejected by a blended fg/bg
  // depth. Depth is 1 m for columns < 330 and 3 m beyond (a vertical edge at
  // col 330). A foreground (z = 1 m) triangle has one vertex projecting to
  // u ~ 329.3 -- its 2x2 taps straddle the edge (1 m and 3 m) -- and two safely
  // on the fg side. Without the discontinuity guard the straddling vertex
  // samples the ~1.6 m blend (|1.6 - 1| = 0.6 > 0.02) and the whole triangle
  // drops to the sentinel; with it, the sampler returns the nearest 1 m tap and
  // the triangle textures.
  std::vector<float> depth_edge(depth.size(), 1.0f);
  for (std::uint32_t y = 0; y < cam.height; ++y) {
    for (std::uint32_t x = 330; x < cam.width; ++x) {
      depth_edge[static_cast<std::size_t>(y) * cam.width + x] = 3.0f;
    }
  }
  // u = fx*x/z + cx = 500*x + 320 at z = 1: x = 0.0186 -> u ~ 329.3
  // (straddles); x = 0.010 -> u = 325 (fg side). v ~ cy so the taps stay on
  // those columns.
  rmesh::Mesh mesh_edge;
  mesh_edge.vertices = {vtx(0.0186f, 0.0f, 1.0f), vtx(0.010f, 0.004f, 1.0f),
                        vtx(0.010f, -0.004f, 1.0f)};
  mesh_edge.indices = {0, 1, 2};
  CHECK(texturer.texture(mesh_edge, depth_edge.data(), cam).ok());
  for (int i = 0; i < 3; ++i) {
    CHECK(!uses_vertex_color(mesh_edge.vertices[i].uv0));
  }

  std::printf(
      "recon texture projective test passed: 1 triangle textured with exact "
      "projected UVs, an occluded triangle carried its coordinate as -uv-1 "
      "while a behind-camera one kept the bare sentinel, a "
      "closer depth map reverted the textured triangle, a translated pose "
      "shifted the projection, the occlusion threshold discriminated a "
      "within-tolerance offset (and honoured an explicit tighter threshold), a "
      "rotated pose projected through R^T to hand-computed pixels, and a depth "
      "discontinuity textured a foreground vertex via the nearest-tap "
      "fallback\n");
  return 0;
}
