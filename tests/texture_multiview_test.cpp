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
// Then the same with the atlas wrapped into two rows; one view against the
// single-camera pass; a half-resolution depth map under a full-resolution
// tile against the full-resolution one; the atlas layout and packing on their
// own; the refusals; and a moved-from texturer. Skips (exit 0) where no device
// is present.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <utility>
#include <vector>

#include "volumetric_kit/recon/core/allocator.hpp"
#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/instance.hpp"
#include "volumetric_kit/recon/core/math/vector_types.hpp"
#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/mesh/mesh.hpp"
#include "volumetric_kit/recon/texture/projective_texturer.hpp"
#include "volumetric_kit/recon/texture/texture_atlas.hpp"

namespace vr = volumetric_kit::recon;
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
  const vr::Vec3f z = vr::normalize(forward);
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

// The atlas coordinate a vertex seen by view `i` must get: the kernel's
// arithmetic, from the projected pixel -- its centre as a fraction of the
// depth map, scaled to the tile and clamped half a texel inside it.
vr::Vec2f expected_uv(vr::Vec3f world, const vr::DepthCameraParams& c,
                      const tex::AtlasTile& tile,
                      const tex::AtlasLayout& layout) {
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
        v.position, views[static_cast<std::size_t>(want)].cam, tile, layout);
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
      vr::Device::create(instance.value(), gpu.value(), {});
  CHECK(device.ok());
  vr::Result<vr::Allocator> allocator =
      vr::Allocator::create(instance.value().handle(), device.value());
  CHECK(allocator.ok());
  vr::Result<tex::ProjectiveTexturer> created =
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
    vr::Result<tex::AtlasLayout> one =
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
    vr::Result<tex::AtlasLayout> layout =
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
              .domain() == vr::Status::Code::InvalidArgument);
  }

  // One row: 960 x 240, tiles at x = 0, 320, 640.
  {
    vr::Result<tex::AtlasLayout> layout =
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

  // Two rows: an extent of 700 fits two tiles a row, so view 2 wraps to the
  // second, at (0, 240) of a 640 x 480 atlas.
  {
    vr::Result<tex::AtlasLayout> layout = tex::side_by_side_atlas(views, 700);
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

  // One view: where the single-camera pass textures all three vertices, the
  // same coordinates, within rounding (the tiled form clamps before it
  // divides).
  {
    const std::vector<tex::TextureView> one = {views[0]};
    vr::Result<tex::AtlasLayout> layout =
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

  // The atlas on its own: tiles in order, wrapping, and the refusals.
  {
    vr::DepthCameraParams small = cam0;
    small.width = 4;
    small.height = 2;
    const std::vector<tex::TextureView> two = {{nullptr, small},
                                               {nullptr, small}};
    vr::Result<tex::AtlasLayout> layout = tex::side_by_side_atlas(two, 16);
    CHECK(layout.ok());
    CHECK(layout->width == 8 && layout->height == 2);
    const std::vector<std::uint32_t> a(8, 0xaaaaaau);
    const std::vector<std::uint32_t> b(8, 0xbbbbbbu);
    std::vector<std::uint32_t> atlas;
    CHECK(tex::pack_atlas({a.data(), b.data()}, layout.value(), &atlas).ok());
    CHECK(atlas.size() == 16);
    for (std::size_t row = 0; row < 2; ++row) {
      for (std::size_t x = 0; x < 8; ++x) {
        CHECK(atlas[row * 8 + x] == (x < 4 ? 0xaaaaaau : 0xbbbbbbu));
      }
    }
    CHECK(!tex::pack_atlas({a.data()}, layout.value(), &atlas).ok());
    CHECK(!tex::pack_atlas({a.data(), nullptr}, layout.value(), &atlas).ok());
    CHECK(tex::pack_atlas({a.data(), b.data()}, layout.value(), nullptr)
              .domain() == vr::Status::Code::InvalidArgument);
    // Two tiles over one another: the second would overwrite the first.
    tex::AtlasLayout overlapping = layout.value();
    overlapping.tiles[1].x = 2;
    CHECK(tex::pack_atlas({a.data(), b.data()}, overlapping, &atlas).domain() ==
          vr::Status::Code::InvalidArgument);
    // Three 4-wide in an extent of 8: two rows, the second half empty.
    const std::vector<tex::TextureView> three = {
        {nullptr, small}, {nullptr, small}, {nullptr, small}};
    vr::Result<tex::AtlasLayout> wrapped = tex::side_by_side_atlas(three, 8);
    CHECK(wrapped.ok());
    CHECK(wrapped->width == 8 && wrapped->height == 4);
    CHECK(wrapped->tiles[2].x == 0 && wrapped->tiles[2].y == 2);
    std::vector<std::uint32_t> packed;
    CHECK(tex::pack_atlas({a.data(), b.data(), a.data()}, wrapped.value(),
                          &packed)
              .ok());
    CHECK(packed[3 * 8 + 5] == 0u);  // nothing covers it
    CHECK(tex::side_by_side_atlas({}, 16).status().domain() ==
          vr::Status::Code::InvalidArgument);
    // Past the extent is InvalidArgument, as texture() says of a layout past
    // the device's.
    CHECK(tex::side_by_side_atlas(two, 3).status().domain() ==
          vr::Status::Code::InvalidArgument);  // wider than the extent
    CHECK(tex::side_by_side_atlas(three, 4).status().domain() ==
          vr::Status::Code::InvalidArgument);  // three rows of 2 > 4 tall
    vr::DepthCameraParams empty = small;
    empty.width = 0;
    CHECK(tex::side_by_side_atlas({{nullptr, empty}}, 16).status().domain() ==
          vr::Status::Code::InvalidArgument);
  }

  // Refusals, each before anything is written.
  {
    vr::Result<tex::AtlasLayout> layout =
        tex::side_by_side_atlas(views, texturer.max_atlas_extent());
    CHECK(layout.ok());
    const auto refused = [&](rmesh::Mesh m,
                             const std::vector<tex::TextureView>& v,
                             const tex::AtlasLayout& l) {
      const vr::Status s = texturer.texture(m, v, l);
      return s.domain() == vr::Status::Code::InvalidArgument &&
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
