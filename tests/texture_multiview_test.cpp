// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// Projective texturing from several views into an atlas, on a synthetic
// scene: a wall at z = 3 seen by three cameras at x = 0, +2 and -2, the two
// outer ones turned toward the middle, each with the wall's depth ray-cast
// into its own map. Each triangle must take the view that faces it most
// squarely among those that see all three of its vertices, and its vertices
// must land inside that view's tile at the projected pixel:
//
//   centre   (x = 0)    -> camera 0, head on
//   right    (x = 1.5)  -> camera 1, which faces it more squarely than 0 does
//   left     (x = -1.5) -> camera 2
//   occluded (x = 0.3)  -> camera 1: camera 0 would win, but its depth map
//                          has nearer geometry there
//   behind   (z = -1)   -> no view: (-1, -1)
//   partial  (one vertex off every image) -> no view: (-1, -1)
//   back     (x = -0.3, wound away from the cameras) -> no view: (-1, -1),
//                          though camera 0's depth agrees with all three
//
// Then the same with the atlas wrapped into two rows; with the depth maps on
// the device, all of them or one; with colour cameras of their own beside the
// depth cameras, and a colour camera that does not see a triangle refusing
// it; a rig of GpuFramePrep-shaped views, whose images mark their coverage;
// a colour camera whose line of sight an occluder in the depth map blocks,
// and one across the surface's plane from its depth camera, in both passes;
// one view against the single-camera pass; a half-resolution depth map under
// a full-resolution tile against the full-resolution one; the atlas layout
// and packing on their own; the refusals; and a moved-from texturer. Skips
// (exit 0) where no device is present.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <utility>
#include <vector>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/buffer.hpp"
#include "volumetric_kit/core/vulkan/compute_util.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/core/vulkan/instance.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"
#include "volumetric_kit/recon/core/math/vector_types.hpp"
#include "volumetric_kit/recon/mesh/mesh.hpp"
#include "volumetric_kit/recon/texture/projective_texturer.hpp"
#include "volumetric_kit/recon/texture/texture_atlas.hpp"

#include "buffer_readback.hpp"
#include "no_device.hpp"

namespace vr = volumetric_kit::recon;
namespace vkc = volumetric_kit::core;
namespace tex = volumetric_kit::recon::texture;
namespace rmesh = volumetric_kit::recon::mesh;

#define CHECK(cond)                                                        \
  do {                                                                     \
    if (!(cond)) {                                                         \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      return 1;                                                            \
    }                                                                      \
  } while (0)

namespace {

constexpr std::uint32_t kW = 320;
constexpr std::uint32_t kH = 240;
constexpr float kWallZ = 3.0f;

// A camera at `eye` looking along `forward` (in the x-z plane), y down.
vr::DepthCameraParams camera(vr::Vec3f eye, vr::Vec3f forward) {
  const vr::Vec3f z = glm::normalize(forward);
  const vr::Vec3f y(0.0f, 1.0f, 0.0f);
  const vr::Vec3f x = vr::cross(y, z);  // so that x cross y = z
  vr::DepthCameraParams c{};
  c.fx = 250.0f;
  c.fy = 250.0f;
  c.cx = 160.0f;
  c.cy = 120.0f;
  c.min_depth = 0.1f;
  c.max_depth = 10.0f;
  c.width = kW;
  c.height = kH;
  c.cam_to_world = vr::Mat4f(1.0f);
  c.cam_to_world[0] = vr::Vec4f(x, 0.0f);
  c.cam_to_world[1] = vr::Vec4f(y, 0.0f);
  c.cam_to_world[2] = vr::Vec4f(z, 0.0f);
  c.cam_to_world[3] = vr::Vec4f(eye, 1.0f);
  return c;
}

vr::Vec3f to_camera(const vr::DepthCameraParams& c, vr::Vec3f world) {
  const vr::Vec3f d = world - vr::Vec3f(c.cam_to_world[3]);
  return vr::Vec3f(vr::dot(vr::Vec3f(c.cam_to_world[0]), d),
                   vr::dot(vr::Vec3f(c.cam_to_world[1]), d),
                   vr::dot(vr::Vec3f(c.cam_to_world[2]), d));
}

// `c` at `scale` of its resolution, its intrinsics rescaled about pixel
// centres as sensor::depth_from_registered_color does, so a pixel centre is
// the same fraction of either image.
vr::DepthCameraParams rescaled(vr::DepthCameraParams c, float scale) {
  c.fx *= scale;
  c.fy *= scale;
  c.cx = (c.cx + 0.5f) * scale - 0.5f;
  c.cy = (c.cy + 0.5f) * scale - 0.5f;
  c.width = static_cast<std::uint32_t>(static_cast<float>(c.width) * scale);
  c.height = static_cast<std::uint32_t>(static_cast<float>(c.height) * scale);
  return c;
}

// The wall's depth in each pixel of `c`: the camera-space z of the ray's hit.
std::vector<float> wall_depth(const vr::DepthCameraParams& c) {
  std::vector<float> depth(static_cast<std::size_t>(c.width) * c.height, 0.0f);
  const vr::Vec3f eye(c.cam_to_world[3]);
  for (std::uint32_t v = 0; v < c.height; ++v) {
    for (std::uint32_t u = 0; u < c.width; ++u) {
      // Camera-space ray with z = 1, so the hit's t is its depth.
      const vr::Vec3f ray((u - c.cx) / c.fx, (v - c.cy) / c.fy, 1.0f);
      const vr::Vec3f world = vr::Vec3f(c.cam_to_world[0]) * ray.x +
                              vr::Vec3f(c.cam_to_world[1]) * ray.y +
                              vr::Vec3f(c.cam_to_world[2]) * ray.z;
      if (world.z > 1e-6f) {
        depth[v * c.width + u] = (kWallZ - eye.z) / world.z;
      }
    }
  }
  return depth;
}

// A colour camera beside depth camera `c`: `baseline` metres along its x axis,
// facing the same way, at 640 x 480 with a focal length and principal point
// of its own. Not a rescale of `c`, so a coordinate taken from the depth
// camera instead lands elsewhere; a little wider, so it records everything
// `c` sees on the wall.
vr::ColorCameraParams color_beside(const vr::DepthCameraParams& c,
                                   float baseline) {
  vr::ColorCameraParams k{};
  k.fx = 480.0f;
  k.fy = 480.0f;
  k.cx = 321.5f;
  k.cy = 238.0f;
  k.width = 640;
  k.height = 480;
  k.cam_to_world = c.cam_to_world;
  k.cam_to_world[3] += c.cam_to_world[0] * baseline;
  return k;
}

// A host view of `depth` through `cam`, its image taken by `color`.
tex::TextureView color_view(const float* depth,
                            const vr::DepthCameraParams& cam,
                            const vr::ColorCameraParams& color) {
  tex::TextureView view{depth, cam};
  view.color_camera = color;
  return view;
}

// color_beside's intrinsics, at `eye` looking along `forward`.
vr::ColorCameraParams color_camera(vr::Vec3f eye, vr::Vec3f forward) {
  return color_beside(camera(eye, forward), 0.0f);
}

// The atlas coordinate a vertex seen by view `v` must get: the kernel's
// arithmetic, from the projected pixel -- its centre as a fraction of the
// image (the colour camera's when the view has one, else the depth map),
// scaled to the tile and clamped half a texel inside it.
vr::Vec2f expected_uv(vr::Vec3f world, const tex::TextureView& view,
                      const tex::AtlasTile& tile,
                      const tex::AtlasLayout& layout) {
  vr::DepthCameraParams c = view.cam;
  if (view.color_camera) {
    const vr::ColorCameraParams& k = *view.color_camera;
    c.fx = k.fx;
    c.fy = k.fy;
    c.cx = k.cx;
    c.cy = k.cy;
    c.width = k.width;
    c.height = k.height;
    c.cam_to_world = k.cam_to_world;
  }
  const vr::Vec3f p = to_camera(c, world);
  const float u = c.fx * (p.x / p.z) + c.cx;
  const float v = c.fy * (p.y / p.z) + c.cy;
  const float tw = static_cast<float>(tile.width);
  const float th = static_cast<float>(tile.height);
  const float x =
      std::fmin(std::fmax((u + 0.5f) / c.width * tw, 0.5f), tw - 0.5f);
  const float y =
      std::fmin(std::fmax((v + 0.5f) / c.height * th, 0.5f), th - 0.5f);
  return vr::Vec2f((tile.x + x) / layout.width, (tile.y + y) / layout.height);
}

rmesh::Vertex vtx(vr::Vec3f p) {
  rmesh::Vertex v{};
  v.position = p;
  v.normal = vr::Vec3f(0.0f, 0.0f, -1.0f);
  v.color = vr::Vec4f(1.0f);
  v.uv0 = vr::Vec2f(0.25f, 0.25f);  // must be overwritten
  return v;
}

// A small triangle on the wall around (x, y), its front -- the
// counter-clockwise side, which marching cubes' outward normal leaves by --
// toward the cameras at z = 0, or away from them when `facing` is false.
void add_wall_triangle(rmesh::Mesh& m, float x, float y, bool facing = true) {
  const vr::Vec3f a(x - 0.1f, y - 0.05f, kWallZ);
  const vr::Vec3f b(x + 0.1f, y - 0.05f, kWallZ);
  const vr::Vec3f c(x, y + 0.1f, kWallZ);
  m.vertices.push_back(vtx(a));
  m.vertices.push_back(vtx(facing ? c : b));
  m.vertices.push_back(vtx(facing ? b : c));
}

// A triangle `size` metres across on the wall around (x, y), facing the
// cameras, with every vertex's normal `normal`.
void add_small_triangle(rmesh::Mesh& m, float x, float y, float size,
                        vr::Vec3f normal = vr::Vec3f(0.0f, 0.0f, -1.0f)) {
  const float h = size * 0.5f;
  for (const vr::Vec3f p :
       {vr::Vec3f(x - h, y - h, kWallZ), vr::Vec3f(x, y + h, kWallZ),
        vr::Vec3f(x + h, y - h, kWallZ)}) {
    rmesh::Vertex v = vtx(p);
    v.normal = normal;
    m.vertices.push_back(v);
  }
}

// `data` in a new device-local buffer, held as a view holds one; `usage`
// defaults to device_storage_buffer's.
template <typename T>
std::shared_ptr<const vkc::Buffer> to_device(
    const vkc::Device& device, vkc::Allocator& allocator,
    const std::vector<T>& data,
    VkBufferUsageFlags usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                               VK_BUFFER_USAGE_TRANSFER_SRC_BIT |
                               VK_BUFFER_USAGE_TRANSFER_DST_BIT) {
  vkc::BufferDesc desc;
  desc.size = VkDeviceSize(data.size()) * sizeof(T);
  desc.usage = usage;
  desc.memory = vkc::MemoryUsage::DeviceOnly;
  vkc::Result<vkc::Buffer> buffer = allocator.create_buffer(desc);
  if (!buffer ||
      !vr_test::write_back(device, allocator, buffer.value(), data).ok()) {
    return nullptr;
  }
  return std::make_shared<const vkc::Buffer>(std::move(buffer).value());
}

// Colour camera `k`'s image marking its coverage as GpuFramePrep does: grey
// where `seen(x, y)`, and a 0 word -- black, recording nothing -- elsewhere.
template <typename Seen>
std::vector<std::uint32_t> coverage_image(const vr::ColorCameraParams& k,
                                          Seen seen) {
  std::vector<std::uint32_t> image(static_cast<std::size_t>(k.width) *
                                   k.height);
  for (std::uint32_t y = 0; y < k.height; ++y) {
    for (std::uint32_t x = 0; x < k.width; ++x) {
      image[y * k.width + x] = seen(x, y) ? 0xFF808080u : 0u;
    }
  }
  return image;
}

void identity_indices(rmesh::Mesh& m) {
  m.indices.resize(m.vertices.size());
  for (std::size_t i = 0; i < m.indices.size(); ++i) {
    m.indices[i] = static_cast<std::uint32_t>(i);
  }
}

bool near(vr::Vec2f a, vr::Vec2f b) {
  return std::fabs(a.x - b.x) <= 1e-5f && std::fabs(a.y - b.y) <= 1e-5f;
}

// Triangle t's vertices must all point into view `want`'s tile at their
// projection, or all be (-1, -1) for want < 0.
int check_triangle(const rmesh::Mesh& m, std::size_t t, int want,
                   const std::vector<tex::TextureView>& views,
                   const tex::AtlasLayout& layout) {
  for (std::size_t k = 0; k < 3; ++k) {
    const rmesh::Vertex& v = m.vertices[3 * t + k];
    if (want < 0) {
      CHECK(v.uv0.x == -1.0f && v.uv0.y == -1.0f);
      continue;
    }
    const tex::AtlasTile& tile = layout.tiles[static_cast<std::size_t>(want)];
    const vr::Vec2f uv = expected_uv(
        v.position, views[static_cast<std::size_t>(want)], tile, layout);
    if (!near(v.uv0, uv)) {
      std::fprintf(stderr,
                   "triangle %zu vertex %zu: uv (%f, %f), want (%f, %f)\n", t,
                   k, v.uv0.x, v.uv0.y, uv.x, uv.y);
      CHECK(false);
    }
    // Inside the tile, half a texel in.
    CHECK(v.uv0.x * layout.width > tile.x &&
          v.uv0.x * layout.width < tile.x + tile.width);
    CHECK(v.uv0.y * layout.height > tile.y &&
          v.uv0.y * layout.height < tile.y + tile.height);
  }
  return 0;
}

// Texture `m` from `view` alone, its tile the whole atlas, and check that its
// first triangle took the view (`want` 0) or the vertex colour (`want` -1).
int texture_alone(tex::ProjectiveTexturer& texturer,
                  const tex::TextureView& view, int want, rmesh::Mesh m) {
  const std::vector<tex::TextureView> one = {view};
  vkc::Result<tex::AtlasLayout> layout =
      tex::side_by_side_atlas(one, texturer.max_atlas_extent());
  CHECK(layout.ok());
  CHECK(texturer.texture(m, one, layout.value()).ok());
  return check_triangle(m, 0, want, one, layout.value());
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
  vkc::Result<tex::ProjectiveTexturer> created =
      tex::ProjectiveTexturer::create(device.value(), allocator.value());
  CHECK(created.ok());
  tex::ProjectiveTexturer texturer = std::move(created).value();
  CHECK(texturer.max_atlas_extent() >= 4096);  // the Vulkan minimum

  const vr::Vec3f target(0.0f, 0.0f, kWallZ);
  const vr::DepthCameraParams cam0 =
      camera(vr::Vec3f(0.0f, 0.0f, 0.0f), vr::Vec3f(0.0f, 0.0f, 1.0f));
  const vr::DepthCameraParams cam1 =
      camera(vr::Vec3f(2.0f, 0.0f, 0.0f), target - vr::Vec3f(2.0f, 0.0f, 0.0f));
  const vr::DepthCameraParams cam2 = camera(
      vr::Vec3f(-2.0f, 0.0f, 0.0f), target - vr::Vec3f(-2.0f, 0.0f, 0.0f));
  const std::vector<float> depth1 = wall_depth(cam1);
  const std::vector<float> depth2 = wall_depth(cam2);
  // Camera 0 sees nearer geometry (1.5 m) over the occluded triangle.
  std::vector<float> depth0 = wall_depth(cam0);
  {
    const vr::Vec3f p = to_camera(cam0, vr::Vec3f(0.3f, 0.0f, kWallZ));
    const int cu = static_cast<int>(cam0.fx * p.x / p.z + cam0.cx);
    const int cv = static_cast<int>(cam0.fy * p.y / p.z + cam0.cy);
    for (int v = cv - 25; v <= cv + 25; ++v) {
      for (int u = cu - 25; u <= cu + 25; ++u) {
        depth0[static_cast<std::size_t>(v) * kW + static_cast<std::size_t>(u)] =
            1.5f;
      }
    }
  }
  const std::vector<tex::TextureView> views = {
      {depth0.data(), cam0}, {depth1.data(), cam1}, {depth2.data(), cam2}};

  rmesh::Mesh mesh;
  add_wall_triangle(mesh, 0.0f, -0.6f);  // 0 centre
  add_wall_triangle(mesh, 1.5f, 0.0f);   // 1 right
  add_wall_triangle(mesh, -1.5f, 0.0f);  // 2 left
  add_wall_triangle(mesh, 0.3f, 0.0f);   // 3 occluded for camera 0
  mesh.vertices.push_back(vtx(vr::Vec3f(-0.1f, 0.0f, -1.0f)));  // 4 behind
  mesh.vertices.push_back(vtx(vr::Vec3f(0.1f, 0.0f, -1.0f)));
  mesh.vertices.push_back(vtx(vr::Vec3f(0.0f, 0.1f, -1.0f)));
  mesh.vertices.push_back(vtx(vr::Vec3f(0.0f, 0.5f, kWallZ)));  // 5 partial
  mesh.vertices.push_back(vtx(vr::Vec3f(0.1f, 0.6f, kWallZ)));
  mesh.vertices.push_back(vtx(vr::Vec3f(8.0f, 0.5f, kWallZ)));
  add_wall_triangle(mesh, -0.3f, 0.3f, false);  // 6 back to every camera
  identity_indices(mesh);
  const int want[] = {0, 1, 2, 1, -1, -1, -1};
  constexpr std::size_t kTriangles = sizeof(want) / sizeof(want[0]);
  // The front test is what refuses triangle 6; the depth test would not.
  {
    rmesh::Mesh facing;
    add_wall_triangle(facing, -0.3f, 0.3f);
    identity_indices(facing);
    vkc::Result<tex::AtlasLayout> one =
        tex::side_by_side_atlas({views[0]}, texturer.max_atlas_extent());
    CHECK(one.ok());
    CHECK(texturer.texture(facing, {views[0]}, one.value()).ok());
    if (check_triangle(facing, 0, 0, {views[0]}, one.value()) != 0) return 1;
  }

  // A half-resolution depth map under a full-resolution tile: the colour
  // image is the tile at its own size, and a vertex lands where the
  // full-resolution depth puts it. First, so the calls after it grow the
  // texturer's buffers from one small view to three large ones.
  {
    const vr::DepthCameraParams half = rescaled(cam1, 0.5f);
    const std::vector<float> half_depth = wall_depth(half);
    tex::TextureView low{half_depth.data(), half, kW, kH};
    const std::vector<tex::TextureView> lows = {low};
    const std::vector<tex::TextureView> fulls = {views[1]};
    vkc::Result<tex::AtlasLayout> layout =
        tex::side_by_side_atlas(lows, texturer.max_atlas_extent());
    CHECK(layout.ok());
    CHECK(layout->width == kW && layout->height == kH);
    rmesh::Mesh from_low = mesh;
    rmesh::Mesh from_full = mesh;
    CHECK(texturer.texture(from_low, lows, layout.value()).ok());
    CHECK(texturer.texture(from_full, fulls, layout.value()).ok());
    std::size_t textured = 0;
    for (std::size_t i = 0; i < mesh.vertices.size(); ++i) {
      const vr::Vec2f a = from_low.vertices[i].uv0;
      const vr::Vec2f b = from_full.vertices[i].uv0;
      CHECK((a.x < 0.0f) == (b.x < 0.0f));
      if (a.x >= 0.0f) {
        CHECK(near(a, b));
        ++textured;
      }
    }
    CHECK(textured >= 9);  // centre, right and occluded, at least
    // One side of the image and not the other is refused.
    tex::TextureView lopsided = low;
    lopsided.image_height = 0;
    CHECK(tex::side_by_side_atlas({lopsided}, texturer.max_atlas_extent())
              .status()
              .domain() == vkc::Status::Code::InvalidArgument);
  }

  // One row: three views stay side by side, 960 x 240, tiles at x = 0, 320,
  // 640.
  {
    vkc::Result<tex::AtlasLayout> layout =
        tex::side_by_side_atlas(views, texturer.max_atlas_extent());
    CHECK(layout.ok());
    CHECK(layout->width == 3 * kW && layout->height == kH);
    CHECK(layout->tiles[1].x == kW && layout->tiles[2].x == 2 * kW);
    rmesh::Mesh m = mesh;
    CHECK(texturer.texture(m, views, layout.value()).ok());
    for (std::size_t t = 0; t < kTriangles; ++t) {
      if (check_triangle(m, t, want[t], views, layout.value()) != 0) return 1;
    }
  }

  // A fallback view takes only what no other view qualifies for. Views 1 and
  // 0, view 1 first: unmarked, view 1 takes the right triangle, which it faces
  // more squarely, and the occluded one; marked, it keeps only the occluded
  // one, which view 0 cannot take, and the right goes to view 0.
  {
    std::vector<tex::TextureView> two = {views[1], views[0]};
    vkc::Result<tex::AtlasLayout> layout =
        tex::side_by_side_atlas(two, texturer.max_atlas_extent());
    CHECK(layout.ok());
    for (const bool fallback : {false, true}) {
      two[0].fallback = fallback;
      rmesh::Mesh m = mesh;
      CHECK(texturer.texture(m, two, layout.value()).ok());
      const int want_two[] = {1, fallback ? 1 : 0, 1, 0, -1, -1, -1};
      for (std::size_t t = 0; t < kTriangles; ++t) {
        if (check_triangle(m, t, want_two[t], two, layout.value()) != 0) {
          return 1;
        }
      }
    }
  }

  // Two rows: an extent of 700 fits two tiles a row, so view 2 wraps to the
  // second, at (0, 240) of a 640 x 480 atlas.
  {
    vkc::Result<tex::AtlasLayout> layout = tex::side_by_side_atlas(views, 700);
    CHECK(layout.ok());
    CHECK(layout->width == 2 * kW && layout->height == 2 * kH);
    CHECK(layout->tiles[2].x == 0 && layout->tiles[2].y == kH);
    rmesh::Mesh m = mesh;
    CHECK(texturer.texture(m, views, layout.value()).ok());
    for (std::size_t t = 0; t < kTriangles; ++t) {
      if (check_triangle(m, t, want[t], views, layout.value()) != 0) return 1;
    }
    CHECK(m.vertices[6].uv0.y > 0.5f);  // the left triangle, in row two
  }

  // The depth maps on the device, as GpuFramePrep leaves them: every view's,
  // then only view 1's beside two on the host. The kernel reads the same
  // bytes either way, so the coordinates are the host run's exactly -- and
  // the scene's. Each run follows one against depth maps of nothing, the same
  // sizes, so the pass's own depth buffer holds zeros going in: a copy that
  // moved nothing would read those, not the host run's identical bytes.
  {
    vkc::Result<tex::AtlasLayout> layout =
        tex::side_by_side_atlas(views, texturer.max_atlas_extent());
    CHECK(layout.ok());
    rmesh::Mesh from_host = mesh;
    CHECK(texturer.texture(from_host, views, layout.value()).ok());

    const std::vector<float>* maps[] = {&depth0, &depth1, &depth2};
    std::vector<tex::TextureView> on_device = views;
    for (std::size_t i = 0; i < on_device.size(); ++i) {
      on_device[i].depth = nullptr;
      on_device[i].depth_buffer =
          to_device(device.value(), allocator.value(), *maps[i]);
      CHECK(on_device[i].depth_buffer != nullptr);
    }
    // The one beside the host views in a buffer the pass can copy from but
    // not bind, which is all it needs of one.
    std::vector<tex::TextureView> mixed = views;
    mixed[1].depth = nullptr;
    mixed[1].depth_buffer = to_device(
        device.value(), allocator.value(), depth1,
        VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
    CHECK(mixed[1].depth_buffer != nullptr);
    const std::vector<float> nothing(depth0.size(), 0.0f);
    std::vector<tex::TextureView> blind = views;
    for (tex::TextureView& v : blind) {
      v.depth = nothing.data();
    }
    for (const std::vector<tex::TextureView>* run : {&on_device, &mixed}) {
      rmesh::Mesh unseen = mesh;
      CHECK(texturer.texture(unseen, blind, layout.value()).ok());
      CHECK(unseen.vertices[0].uv0 == vr::Vec2f(-1.0f, -1.0f));
      rmesh::Mesh m = mesh;
      CHECK(texturer.texture(m, *run, layout.value()).ok());
      for (std::size_t i = 0; i < m.vertices.size(); ++i) {
        CHECK(m.vertices[i].uv0 == from_host.vertices[i].uv0);
      }
      for (std::size_t t = 0; t < kTriangles; ++t) {
        if (check_triangle(m, t, want[t], views, layout.value()) != 0) return 1;
      }
    }
  }

  // Colour cameras of their own, 5 cm beside the depth cameras: the same
  // views win -- the depth cameras still decide what is visible, and the
  // score is theirs -- but the tiles are the 640 x 480 colour images and each
  // vertex lands at its pixel in the winner's colour camera.
  {
    std::vector<tex::TextureView> colored = views;
    for (tex::TextureView& v : colored) {
      v.color_camera = color_beside(v.cam, 0.05f);
    }
    vkc::Result<tex::AtlasLayout> layout =
        tex::side_by_side_atlas(colored, texturer.max_atlas_extent());
    CHECK(layout.ok());
    CHECK(layout->width == 3 * 640 && layout->height == 480);
    rmesh::Mesh m = mesh;
    CHECK(texturer.texture(m, colored, layout.value()).ok());
    for (std::size_t t = 0; t < kTriangles; ++t) {
      if (check_triangle(m, t, want[t], colored, layout.value()) != 0) {
        return 1;
      }
    }
    // A size its colour camera does not have is refused, by the layout and
    // the pass alike.
    tex::TextureView wrong = colored[0];
    wrong.image_width = kW;
    wrong.image_height = kH;
    CHECK(tex::side_by_side_atlas({wrong}, texturer.max_atlas_extent())
              .status()
              .domain() == vkc::Status::Code::InvalidArgument);
    std::vector<tex::TextureView> wrong_views = colored;
    wrong_views[0] = wrong;
    rmesh::Mesh untouched = mesh;
    CHECK(texturer.texture(untouched, wrong_views, layout.value()).domain() ==
          vkc::Status::Code::InvalidArgument);
    CHECK(untouched.vertices[0].uv0 == vr::Vec2f(0.25f, 0.25f));
    // So is a colour camera with no image.
    tex::TextureView blank = colored[0];
    blank.color_camera->width = 0;
    CHECK(tex::side_by_side_atlas({blank}, texturer.max_atlas_extent())
              .status()
              .domain() == vkc::Status::Code::InvalidArgument);
  }

  // The colour camera must see the triangle as well: the centre triangle,
  // which view 0 textures with its colour camera beside it, falls to the
  // vertex colour when that camera sees only the triangle's back (from behind
  // the wall), sees nothing of it (looking away), or sees it past the edge of
  // its image (moved 3 m aside). Each time the depth camera alone would
  // texture it.
  {
    rmesh::Mesh centre;
    add_wall_triangle(centre, 0.0f, -0.6f);
    identity_indices(centre);
    tex::TextureView view = views[0];
    view.color_camera = color_beside(view.cam, 0.05f);
    if (texture_alone(texturer, view, 0, centre) != 0) return 1;
    view.color_camera = color_camera(vr::Vec3f(0.0f, 0.0f, 2.0f * kWallZ),
                                     vr::Vec3f(0.0f, 0.0f, -1.0f));
    if (texture_alone(texturer, view, -1, centre) != 0) return 1;
    view.color_camera =
        color_camera(vr::Vec3f(0.0f, 0.0f, 0.0f), vr::Vec3f(0.0f, 0.0f, -1.0f));
    if (texture_alone(texturer, view, -1, centre) != 0) return 1;
    view.color_camera =
        color_camera(vr::Vec3f(3.0f, 0.0f, 0.0f), vr::Vec3f(0.0f, 0.0f, 1.0f));
    if (texture_alone(texturer, view, -1, centre) != 0) return 1;
    view.color_camera.reset();
    if (texture_alone(texturer, view, 0, centre) != 0) return 1;
  }

  // A rig of GpuFramePrep frames: the depth on the device, colour cameras of
  // their own, and each image marking its coverage. Wholly covered, it gives
  // the host run's coordinates exactly, after a run whose images recorded
  // nothing, so coverage the copy did not move cannot pass. With view 0's
  // image blank over the centre triangle -- the black corner an undistorted
  // image has where its lens saw nothing -- view 0 no longer takes it, though
  // its depth camera faces it most squarely, and a view that recorded it does.
  {
    std::vector<tex::TextureView> colored = views;
    for (tex::TextureView& v : colored) {
      v.color_camera = color_beside(v.cam, 0.05f);
    }
    vkc::Result<tex::AtlasLayout> layout =
        tex::side_by_side_atlas(colored, texturer.max_atlas_extent());
    CHECK(layout.ok());
    rmesh::Mesh from_host = mesh;
    CHECK(texturer.texture(from_host, colored, layout.value()).ok());

    const std::vector<float>* maps[] = {&depth0, &depth1, &depth2};
    std::vector<tex::TextureView> rig = colored;
    std::vector<tex::TextureView> blind = colored;
    const auto all = [](std::uint32_t, std::uint32_t) { return true; };
    const auto none = [](std::uint32_t, std::uint32_t) { return false; };
    for (std::size_t i = 0; i < rig.size(); ++i) {
      rig[i].depth = nullptr;
      rig[i].depth_buffer =
          to_device(device.value(), allocator.value(), *maps[i]);
      rig[i].coverage = to_device(device.value(), allocator.value(),
                                  coverage_image(*rig[i].color_camera, all));
      blind[i].coverage = to_device(device.value(), allocator.value(),
                                    coverage_image(*rig[i].color_camera, none));
      CHECK(rig[i].depth_buffer && rig[i].coverage && blind[i].coverage);
    }
    rmesh::Mesh unseen = mesh;
    CHECK(texturer.texture(unseen, blind, layout.value()).ok());
    for (const rmesh::Vertex& v : unseen.vertices) {
      CHECK(v.uv0 == vr::Vec2f(-1.0f, -1.0f));
    }
    rmesh::Mesh m = mesh;
    CHECK(texturer.texture(m, rig, layout.value()).ok());
    for (std::size_t i = 0; i < m.vertices.size(); ++i) {
      CHECK(m.vertices[i].uv0 == from_host.vertices[i].uv0);
    }

    // The centre triangle lies in rows 134 to 158 of view 0's image, and
    // nothing else view 0 takes lies above row 170.
    rig[0].coverage = to_device(
        device.value(), allocator.value(),
        coverage_image(*rig[0].color_camera,
                       [](std::uint32_t, std::uint32_t y) { return y > 170; }));
    CHECK(rig[0].coverage != nullptr);
    m = mesh;
    CHECK(texturer.texture(m, rig, layout.value()).ok());
    const int took =
        m.vertices[0].uv0.x * layout->width >= layout->tiles[2].x ? 2 : 1;
    if (check_triangle(m, 0, took, rig, layout.value()) != 0) return 1;
    for (std::size_t t = 1; t < kTriangles; ++t) {
      if (check_triangle(m, t, want[t], rig, layout.value()) != 0) return 1;
    }
  }

  // The colour camera's line of sight, walked through the depth map. Camera
  // 0's map holds an occluder 1.5 m out over pixels 160 to 200 of rows 100 to
  // 140, and a small triangle on the wall just left of it, at pixels 153 to
  // 157. The depth camera sees the triangle past the occluder's edge; a colour
  // camera 20 cm to its right looks at it through the occluder, whose fringe
  // there is 250 * 0.2 * (1/1.5 - 1/3) = 17 pixels wide. So that view gives the
  // triangle up to camera 2, which sees it clearly, though camera 0 faces it
  // more squarely -- and takes it back with its colour camera 20 cm to the
  // left instead, looking away from the occluder, or with none. The
  // single-camera pass carries the triangle's vertices under the right-hand
  // colour camera and textures them under the others, while a triangle far
  // from the occluder is textured under all three.
  for (float min_depth : {0.1f, 1e-8f, 0.0f}) {
    vr::DepthCameraParams sight_cam0 = cam0;
    vr::DepthCameraParams sight_cam2 = cam2;
    sight_cam0.min_depth = min_depth;
    sight_cam2.min_depth = min_depth;
    std::vector<float> occluded = wall_depth(sight_cam0);
    for (std::uint32_t v = 100; v <= 140; ++v) {
      for (std::uint32_t u = 160; u <= 200; ++u) {
        occluded[v * kW + u] = 1.5f;
      }
    }
    rmesh::Mesh fringe;
    add_small_triangle(fringe, -0.06f, 0.0f, 0.048f);  // pixels 153 to 157
    add_small_triangle(fringe, -0.7f, 0.0f, 0.048f);   // pixel 76, far off
    identity_indices(fringe);
    const tex::TextureView right =
        color_view(occluded.data(), sight_cam0, color_beside(sight_cam0, 0.2f));
    const tex::TextureView left = color_view(occluded.data(), sight_cam0,
                                             color_beside(sight_cam0, -0.2f));
    const tex::TextureView registered{occluded.data(), sight_cam0};
    const tex::TextureView clear =
        color_view(depth2.data(), sight_cam2, color_beside(sight_cam2, 0.05f));

    for (const tex::TextureView* first : {&right, &left, &registered}) {
      const std::vector<tex::TextureView> two = {*first, clear};
      vkc::Result<tex::AtlasLayout> layout =
          tex::side_by_side_atlas(two, texturer.max_atlas_extent());
      CHECK(layout.ok());
      rmesh::Mesh m = fringe;
      CHECK(texturer.texture(m, two, layout.value()).ok());
      const int want_fringe = first == &right ? 1 : 0;
      if (check_triangle(m, 0, want_fringe, two, layout.value()) != 0) {
        return 1;
      }
      if (check_triangle(m, 1, 0, two, layout.value()) != 0) return 1;

      rmesh::Mesh single = fringe;
      CHECK(texturer.texture(single, *first).ok());
      for (std::size_t i = 0; i < single.vertices.size(); ++i) {
        const bool carried = first == &right && i < 3;
        CHECK((single.vertices[i].uv0.x < 0.0f) == carried);
      }
    }

    // Replace the occluder with missing measurements. A zero-depth hole
    // cannot block the colour camera, even when zero is inside the accepted
    // depth range. The right-hand view must keep both triangles in either
    // pass, and win over the less squarely facing clear view.
    for (std::uint32_t v = 100; v <= 140; ++v) {
      for (std::uint32_t u = 160; u <= 200; ++u) {
        occluded[v * kW + u] = 0.0f;
      }
    }
    const std::vector<tex::TextureView> two = {right, clear};
    auto layout = tex::side_by_side_atlas(two, texturer.max_atlas_extent());
    CHECK(layout.ok());
    rmesh::Mesh multi = fringe;
    CHECK(texturer.texture(multi, two, layout.value()).ok());
    rmesh::Mesh single = fringe;
    CHECK(texturer.texture(single, right).ok());
    for (const rmesh::Vertex& vertex : single.vertices) {
      CHECK(vertex.uv0.x >= 0.0f && vertex.uv0.y >= 0.0f);
    }
    for (std::size_t t = 0; t < 2; ++t) {
      if (check_triangle(multi, t, 0, two, layout.value()) != 0) return 1;
    }
  }

  // Both cameras must see the same side of the surface, in the single-camera
  // pass, which has only the vertex's normal to tell. Normals tilted nearly
  // edge-on to the cameras, toward -x, put a colour camera 20 cm to the right
  // behind the surface's plane while the depth camera is in front of it: the
  // vertices are carried, though the depth map agrees with every one. A
  // colour camera 20 cm to the left is on the depth camera's side, and it and
  // the registered image texture them.
  {
    const std::vector<float> wall = wall_depth(cam0);
    rmesh::Mesh tilted;
    add_small_triangle(tilted, 0.0f, 0.3f, 0.04f,
                       glm::normalize(vr::Vec3f(-1.0f, 0.0f, -0.03f)));
    identity_indices(tilted);
    const tex::TextureView right =
        color_view(wall.data(), cam0, color_beside(cam0, 0.2f));
    const tex::TextureView left =
        color_view(wall.data(), cam0, color_beside(cam0, -0.2f));
    const tex::TextureView registered{wall.data(), cam0};
    for (const tex::TextureView* view : {&right, &left, &registered}) {
      rmesh::Mesh m = tilted;
      CHECK(texturer.texture(m, *view).ok());
      for (const rmesh::Vertex& v : m.vertices) {
        CHECK((v.uv0.x < 0.0f) == (view == &right));
      }
    }
  }

  // One view: where the single-camera pass textures all three vertices, the
  // same coordinates, within rounding (the tiled form clamps before it
  // divides).
  {
    const std::vector<tex::TextureView> one = {views[0]};
    vkc::Result<tex::AtlasLayout> layout =
        tex::side_by_side_atlas(one, texturer.max_atlas_extent());
    CHECK(layout.ok());
    rmesh::Mesh multi = mesh;
    rmesh::Mesh single = mesh;
    CHECK(texturer.texture(multi, one, layout.value()).ok());
    CHECK(texturer.texture(single, depth0.data(), cam0).ok());
    for (std::size_t k = 0; k < 3; ++k) {  // the centre triangle
      CHECK(single.vertices[k].uv0.x >= 0.0f);
      CHECK(std::fabs(multi.vertices[k].uv0.x - single.vertices[k].uv0.x) <=
            1e-6f);
      CHECK(std::fabs(multi.vertices[k].uv0.y - single.vertices[k].uv0.y) <=
            1e-6f);
    }
  }

  // The atlas on its own: tiles in order, in floor(sqrt(n)) rows, a row
  // ended early at the extent, and the refusals.
  {
    vr::DepthCameraParams small = cam0;
    small.width = 4;
    small.height = 2;
    const std::vector<tex::TextureView> two = {{nullptr, small},
                                               {nullptr, small}};
    vkc::Result<tex::AtlasLayout> layout = tex::side_by_side_atlas(two, 16);
    CHECK(layout.ok());
    CHECK(layout->width == 8 && layout->height == 2);
    CHECK(layout->tiles[1].x == 4 && layout->tiles[1].y == 0);
    // Three 4-wide in an extent of 8: two rows, the second half empty.
    const std::vector<tex::TextureView> three = {
        {nullptr, small}, {nullptr, small}, {nullptr, small}};
    vkc::Result<tex::AtlasLayout> wrapped = tex::side_by_side_atlas(three, 8);
    CHECK(wrapped.ok());
    CHECK(wrapped->width == 8 && wrapped->height == 4);
    CHECK(wrapped->tiles[2].x == 0 && wrapped->tiles[2].y == 2);
    // In a wide extent, three stay in one row, four make two rows of two, and
    // five three columns.
    vkc::Result<tex::AtlasLayout> row = tex::side_by_side_atlas(three, 64);
    CHECK(row.ok() && row->width == 12 && row->height == 2);
    const std::vector<tex::TextureView> four(4, {nullptr, small});
    vkc::Result<tex::AtlasLayout> square = tex::side_by_side_atlas(four, 64);
    CHECK(square.ok() && square->width == 8 && square->height == 4);
    CHECK(square->tiles[3].x == 4 && square->tiles[3].y == 2);
    const std::vector<tex::TextureView> five(5, {nullptr, small});
    vkc::Result<tex::AtlasLayout> wide = tex::side_by_side_atlas(five, 64);
    CHECK(wide.ok() && wide->width == 12 && wide->height == 4);
    CHECK(wide->tiles[3].x == 0 && wide->tiles[3].y == 2);
    // Two views a row, unless the second would pass the extent: 4 + 6 > 8.
    vr::DepthCameraParams wider = small;
    wider.width = 6;
    const std::vector<tex::TextureView> uneven = {{nullptr, small},
                                                  {nullptr, wider}};
    vkc::Result<tex::AtlasLayout> early = tex::side_by_side_atlas(uneven, 8);
    CHECK(early.ok() && early->width == 6 && early->height == 4);
    CHECK(early->tiles[1].x == 0 && early->tiles[1].y == 2);
    CHECK(tex::side_by_side_atlas({}, 16).status().domain() ==
          vkc::Status::Code::InvalidArgument);
    // Past the extent is InvalidArgument, as texture() says of a layout past
    // the device's.
    CHECK(tex::side_by_side_atlas(two, 3).status().domain() ==
          vkc::Status::Code::InvalidArgument);  // wider than the extent
    CHECK(tex::side_by_side_atlas(three, 4).status().domain() ==
          vkc::Status::Code::InvalidArgument);  // three rows of 2 > 4 tall
    vr::DepthCameraParams empty = small;
    empty.width = 0;
    CHECK(tex::side_by_side_atlas({{nullptr, empty}}, 16).status().domain() ==
          vkc::Status::Code::InvalidArgument);
  }

  // Refusals, each before anything is written.
  {
    vkc::Result<tex::AtlasLayout> layout =
        tex::side_by_side_atlas(views, texturer.max_atlas_extent());
    CHECK(layout.ok());
    const auto refused = [&](rmesh::Mesh m,
                             const std::vector<tex::TextureView>& v,
                             const tex::AtlasLayout& l) {
      const vkc::Status s = texturer.texture(m, v, l);
      return s.domain() == vkc::Status::Code::InvalidArgument &&
             m.vertices[0].uv0 == vr::Vec2f(0.25f, 0.25f);
    };
    CHECK(refused(mesh, {}, layout.value()));  // no views
    tex::AtlasLayout short_layout = layout.value();
    short_layout.tiles.pop_back();
    CHECK(refused(mesh, views, short_layout));  // a view without a tile
    tex::AtlasLayout wrong_size = layout.value();
    wrong_size.tiles[1].width = kW - 1;
    CHECK(refused(mesh, views, wrong_size));  // a tile not its image's size
    tex::AtlasLayout outside = layout.value();
    outside.tiles[2].x = 2 * kW + 1;
    CHECK(refused(mesh, views, outside));  // a tile past the atlas
    tex::AtlasLayout huge = layout.value();
    huge.width = texturer.max_atlas_extent() + 1;
    CHECK(refused(mesh, views, huge));  // past the device's extent
    std::vector<tex::TextureView> no_depth = views;
    no_depth[1].depth = nullptr;
    CHECK(refused(mesh, no_depth, layout.value()));
    // A camera with no depth range, as one converted from ColorCameraParams
    // arrives: no sample would count, and every triangle would silently take
    // the vertex colour.
    std::vector<tex::TextureView> no_range = views;
    no_range[1].cam.min_depth = 0.0f;
    no_range[1].cam.max_depth = 0.0f;
    CHECK(refused(mesh, no_range, layout.value()));
    // An image larger than the depth map, in a tile the depth map's size.
    std::vector<tex::TextureView> larger = views;
    larger[1].image_width = 2 * kW;
    larger[1].image_height = 2 * kH;
    CHECK(refused(mesh, larger, layout.value()));
    // Tile 1 over tile 0, both inside the atlas.
    tex::AtlasLayout overlapping = layout.value();
    overlapping.tiles[1].x = kW / 2;
    CHECK(refused(mesh, views, overlapping));
    rmesh::Mesh shared = mesh;  // triangle 1 reuses a vertex of triangle 0
    shared.indices[3] = 0;
    CHECK(refused(shared, views, layout.value()));
    // Device depth: given beside the host one, a float short of its map,
    // empty, and in a buffer a batch cannot copy from.
    const VkDeviceSize map_bytes = VkDeviceSize(kW) * kH * sizeof(float);
    vkc::Result<vkc::Buffer> whole =
        vkc::device_storage_buffer(allocator.value(), map_bytes);
    vkc::Result<vkc::Buffer> short_map = vkc::device_storage_buffer(
        allocator.value(), map_bytes - sizeof(float));
    vkc::Result<vkc::Buffer> no_copy = allocator.value().create_buffer(
        {map_bytes, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT});
    CHECK(whole.ok() && short_map.ok() && no_copy.ok());
    CHECK((no_copy->usage() & VK_BUFFER_USAGE_TRANSFER_SRC_BIT) == 0);
    std::vector<tex::TextureView> both = views;
    both[1].depth_buffer =
        std::make_shared<const vkc::Buffer>(std::move(whole).value());
    CHECK(refused(mesh, both, layout.value()));
    const std::shared_ptr<const vkc::Buffer> bad_buffers[] = {
        std::make_shared<const vkc::Buffer>(std::move(short_map).value()),
        std::make_shared<const vkc::Buffer>(),
        std::make_shared<const vkc::Buffer>(std::move(no_copy).value())};
    for (const std::shared_ptr<const vkc::Buffer>& bad : bad_buffers) {
      std::vector<tex::TextureView> device_view = views;
      device_view[1].depth = nullptr;
      device_view[1].depth_buffer = bad;
      CHECK(refused(mesh, device_view, layout.value()));
      // So is a coverage that is any of the three, the first a word short of
      // its image.
      std::vector<tex::TextureView> covered_view = views;
      covered_view[1].coverage = bad;
      CHECK(refused(mesh, covered_view, layout.value()));
    }
    // An empty mesh is a no-op.
    rmesh::Mesh none;
    CHECK(texturer.texture(none, views, layout.value()).ok());
  }

  // A moved-from texturer has no device, so no extent to lay an atlas out in.
  {
    const std::uint32_t extent = texturer.max_atlas_extent();
    tex::ProjectiveTexturer moved = std::move(texturer);
    CHECK(!texturer.valid());  // NOLINT(bugprone-use-after-move)
    CHECK(texturer.max_atlas_extent() == 0);
    CHECK(moved.max_atlas_extent() == extent);
  }

  std::printf("texture multiview: OK\n");
  return 0;
}
