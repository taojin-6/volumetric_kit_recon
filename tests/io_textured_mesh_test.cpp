// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// io::load_textured_mesh on a procedural unit cube, OBJ + MTL + PNG written
// here, its material library and diffuse map in a directory of their own: the
// coordinates each corner names, one vertex per distinct position and UV, the
// triangles of load_mesh in its order, and the map resolved beside the library
// and decoded by load_color_packed. With no texture coordinates it is
// NotFound; an asset load_mesh refuses gets load_mesh's error; each of its
// own refusals; and map paths that Assimp 5 and 6 report differently.

#include <array>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "volumetric_kit/core/testing/scratch_dir.hpp"
#include "volumetric_kit/core/testing/test_check.hpp"
#include "volumetric_kit/recon/io/image_io.hpp"
#include "volumetric_kit/recon/io/mesh_io.hpp"
#include "volumetric_kit/recon/io/textured_mesh.hpp"

namespace vkc = volumetric_kit::core;
namespace io = volumetric_kit::recon::io;
namespace fs = std::filesystem;
using Code = vkc::Status::Code;

namespace {

struct Corner {
  int position;  // Cube corner x + 2y + 4z.
  float u, v;
};
using Face = std::array<Corner, 4>;

// Six outward quads. The sides run in a strip around +y, v in [0, 0.5], its
// last edge the first again at u = 1: a seam. Top and bottom have squares of
// their own. So the corners of the first edge carry three UVs, the others two:
// 18 vertices for 8 positions.
std::vector<Face> cube_faces() {
  const int strip[5][2] = {{0, 0}, {0, 1}, {1, 1}, {1, 0}, {0, 0}};  // x, z
  const auto side = [&strip](int column, int y) {
    return Corner{strip[column][0] + 2 * y + 4 * strip[column][1],
                  0.25f * float(column), 0.5f * float(y)};
  };
  const auto cap = [](int y, float u0, int x, int z) {
    return Corner{x + 2 * y + 4 * z, u0 + 0.25f * float(x),
                  0.625f + 0.25f * float(z)};
  };
  std::vector<Face> faces;
  for (int k = 0; k < 4; ++k) {
    faces.push_back({side(k, 0), side(k + 1, 0), side(k + 1, 1), side(k, 1)});
  }
  faces.push_back({cap(1, 0.125f, 0, 0), cap(1, 0.125f, 0, 1),
                   cap(1, 0.125f, 1, 1), cap(1, 0.125f, 1, 0)});
  faces.push_back({cap(0, 0.625f, 0, 0), cap(0, 0.625f, 1, 0),
                   cap(0, 0.625f, 1, 1), cap(0, 0.625f, 0, 1)});
  return faces;
}

// The cube as OBJ text after `header`, with one `vt` per face corner (so equal
// coordinates join by value, not by index), or none.
std::string cube_obj(const std::vector<Face>& faces, bool uvs,
                     const std::string& header) {
  std::ostringstream obj;
  obj << header;
  for (int i = 0; i < 8; ++i) {
    obj << "v " << (i & 1) << ' ' << ((i >> 1) & 1) << ' ' << (i >> 2) << '\n';
  }
  for (const Face& face : faces) {
    for (const Corner& c : face) {
      if (uvs) obj << "vt " << c.u << ' ' << c.v << '\n';
    }
  }
  int vt = 1;
  for (const Face& face : faces) {
    obj << 'f';
    for (const Corner& c : face) {
      obj << ' ' << c.position + 1;
      if (uvs) obj << '/' << vt++;
    }
    obj << '\n';
  }
  return obj.str();
}

void write(const fs::path& path, const std::string& text) {
  std::ofstream(path) << text;
}

bool same_error(const vkc::Status& a, const vkc::Status& b) {
  return !a.ok() && a.domain() == b.domain() && a.message() == b.message();
}

}  // namespace

int main() {
  vkc::test::Scratch scratch("recon_io_textured_mesh_");
  const fs::path dir = scratch.path;
  CHECK(fs::create_directory(dir / "materials"));

  // An 8x4 diffuse map beside the material library, not the OBJ.
  constexpr std::uint32_t kWidth = 8;
  constexpr std::uint32_t kHeight = 4;
  std::vector<std::uint8_t> rgba;
  std::vector<std::uint32_t> packed;
  for (std::uint32_t y = 0; y < kHeight; ++y) {
    for (std::uint32_t x = 0; x < kWidth; ++x) {
      const std::uint32_t r = 32 * x;
      const std::uint32_t g = 64 * y;
      const std::uint32_t b = 255 - 16 * (x + y);
      rgba.insert(rgba.end(), {std::uint8_t(r), std::uint8_t(g),
                               std::uint8_t(b), std::uint8_t(255)});
      packed.push_back(r | g << 8 | b << 16);
    }
  }
  const fs::path map = dir / "materials" / "cube.png";
  CHECK(io::write_png_rgba8(map.string(), rgba.data(), rgba.size(), kWidth,
                            kHeight)
            .ok());
  write(dir / "materials" / "cube.mtl", "newmtl cube\nmap_Kd cube.png\n");

  const std::vector<Face> faces = cube_faces();
  const std::string cube = (dir / "cube.obj").string();
  write(cube,
        cube_obj(faces, true, "mtllib materials/cube.mtl\nusemtl cube\n"));

  // Positions join across the seams exactly as before.
  const vkc::Result<io::TriangleMesh> plain = io::load_mesh(cube);
  CHECK(plain.ok());
  CHECK(plain->positions.size() == 8);
  CHECK(plain->indices.size() == 36);

  const vkc::Result<io::TexturedMesh> textured = io::load_textured_mesh(cube);
  CHECK(textured.ok());
  const io::TriangleMesh& geometry = textured->geometry;
  std::set<std::pair<int, std::array<float, 2>>> distinct;
  for (const Face& face : faces) {
    for (const Corner& c : face) distinct.insert({c.position, {c.u, c.v}});
  }
  CHECK(distinct.size() == 18);
  CHECK(geometry.positions.size() == distinct.size());
  CHECK(textured->uvs.size() == geometry.positions.size());
  CHECK(geometry.indices.size() == plain->indices.size());
  for (std::size_t i = 0; i < geometry.indices.size(); ++i) {
    CHECK(geometry.indices[i] < geometry.positions.size());
    CHECK(geometry.positions[geometry.indices[i]] ==
          plain->positions[plain->indices[i]]);
  }
  // Each corner's coordinates are those its face names: the face is the one
  // holding all three of the triangle's positions.
  const auto corner_of = [](const io::TriangleMesh& mesh, std::uint32_t index) {
    const auto& p = mesh.positions[index];
    return int(p.x) + 2 * int(p.y) + 4 * int(p.z);
  };
  for (std::size_t t = 0; t < geometry.indices.size(); t += 3) {
    std::array<int, 3> corners{};
    for (std::size_t k = 0; k < 3; ++k) {
      corners[k] = corner_of(geometry, geometry.indices[t + k]);
    }
    int matches = 0;
    for (const Face& face : faces) {
      int held = 0;
      for (const int corner : corners) {
        for (const Corner& c : face) held += c.position == corner;
      }
      if (held != 3) continue;
      ++matches;
      for (std::size_t k = 0; k < 3; ++k) {
        for (const Corner& c : face) {
          if (c.position != corners[k]) continue;
          const auto& uv = textured->uvs[geometry.indices[t + k]];
          CHECK(uv.x == c.u && uv.y == c.v);
        }
      }
    }
    CHECK(matches == 1);
  }
  // Corners join on equal coordinates, -0 as 0, and split on v alone: the
  // square's two corners at (1, 0, 0) name v = 0 and 0.5, and its two at
  // (0, 1, 0) name u = 0 and -0, so 5 vertices.
  const std::string seams = (dir / "seams.obj").string();
  write(seams,
        "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 1 1 0\n"
        "vt 0 0\nvt 1 0\nvt 0 1\nvt 1 1\nvt 1 0.5\nvt -0 1\n"
        "f 1/1 2/2 3/3\nf 2/5 4/4 3/6\n");
  const vkc::Result<io::TexturedMesh> seamed = io::load_textured_mesh(seams);
  CHECK(seamed.ok());
  CHECK(seamed->geometry.positions.size() == 5);

  // The map resolves against the material library, and decodes.
  CHECK(fs::path(textured->diffuse_texture) == map);
  const vkc::Result<std::vector<std::uint32_t>> pixels =
      io::load_color_packed(textured->diffuse_texture, kWidth, kHeight);
  CHECK(pixels.ok());
  CHECK(*pixels == packed);

  // With no material the map is empty; a file with no texture coordinates is
  // NotFound.
  const std::string bare = (dir / "bare.obj").string();
  write(bare, cube_obj(faces, true, ""));
  const vkc::Result<io::TexturedMesh> unmapped = io::load_textured_mesh(bare);
  CHECK(unmapped.ok());
  CHECK(unmapped->diffuse_texture.empty());
  CHECK(unmapped->geometry.positions.size() == 18);
  const std::string no_uvs = (dir / "no_uvs.obj").string();
  write(no_uvs, cube_obj(faces, false, ""));
  CHECK(io::load_mesh(no_uvs).ok());
  CHECK(io::load_textured_mesh(no_uvs).status().domain() == Code::NotFound);

  // load_mesh's error for an asset it refuses, here none with UVs.
  const std::string missing = (dir / "missing.obj").string();
  const std::string lines = (dir / "lines.obj").string();
  write(lines, "v 0 0 0\nv 1 0 0\nl 1 2\n");
  const std::string empty = (dir / "empty.obj").string();
  write(empty, "");
  const struct {
    std::string path;
    bool code_known;
    Code code;
  } refused[] = {{missing, true, Code::NotFound},
                 {(dir / "materials").string(), true, Code::InvalidArgument},
                 {"", true, Code::InvalidArgument},
                 {lines, true, Code::Unsupported},
                 {empty, false, Code::Ok}};
  for (const auto& r : refused) {
    const vkc::Status status = io::load_mesh(r.path).status();
    CHECK(!r.code_known || status.domain() == r.code);
    CHECK(same_error(io::load_textured_mesh(r.path).status(), status));
  }

  // Its own refusals, each for an asset load_mesh accepts.
  // A unit square's two triangles, `head` before them and `mid` between.
  const auto square = [&dir](const std::string& name, const std::string& head,
                             const std::string& mid) {
    const std::string path = (dir / name).string();
    write(path, head +
                    "v 0 0 0\nv 1 0 0\nv 0 1 0\nv 1 1 0\n"
                    "vt 0 0\nvt 1 0\nvt 0 1\nvt 1 1\nf 1/1 2/2 3/3\n" +
                    mid + "f 2/2 4/4 3/3\n");
    return path;
  };
  // Assimp's OBJ reads a nonfinite `vt` as 0; its PLY keeps it.
  const std::string nonfinite = (dir / "nonfinite.ply").string();
  write(nonfinite,
        "ply\nformat ascii 1.0\nelement vertex 3\nproperty float x\n"
        "property float y\nproperty float z\nproperty float s\n"
        "property float t\nelement face 1\n"
        "property list uchar int vertex_indices\nend_header\n"
        "0 0 0 nan 0\n1 0 0 1 0\n0 1 0 0 1\n3 0 1 2\n");
  write(dir / "two.mtl", "newmtl a\nmap_Kd a.png\nnewmtl b\nmap_Kd b.png\n");
  write(dir / "held.mtl", "newmtl a\nmap_Kd *0\n");
  CHECK(fs::create_directory(dir / "other"));
  write(dir / "other" / "more.mtl", "newmtl c\nKd 1 1 1\n");
  const struct {
    std::string path;
    Code code;
    const char* named;
  } own[] = {
      {nonfinite, Code::InvalidArgument, "nonfinite"},
      {square("two_maps.obj", "mtllib two.mtl\nusemtl a\n", "usemtl b\n"),
       Code::Unsupported, "different diffuse maps"},
      {square("held.obj", "mtllib held.mtl\nusemtl a\n", ""), Code::Unsupported,
       "embedded"},
      {square("two_dirs.obj",
              "mtllib two.mtl\nmtllib other/more.mtl\nusemtl a\n", ""),
       Code::Unsupported, "more than one directory"}};
  for (const auto& r : own) {
    CHECK(io::load_mesh(r.path).ok());
    const vkc::Status status = io::load_textured_mesh(r.path).status();
    CHECK(status.domain() == r.code);
    CHECK(status.message().find(r.named) != std::string::npos);
  }

  // The same map path on every Assimp: an absolute one is kept, a backslash
  // separates directories in a library's name and in its map's, a library's
  // extension matches in either case, and a map may repeat its library's
  // directory. A missing map is still a path. The last two OBJs are named
  // from their own directory, one with an absolute library path.
  write(dir / "materials" / "abs.MTL",
        "newmtl a\nmap_Kd " + map.string() + "\n");
  write(dir / "materials" / "back.MTL", "newmtl a\nmap_Kd sub\\cube.png\n");
  write(dir / "materials" / "nest.mtl",
        "newmtl a\nmap_Kd materials/cube.png\n");
  square(
      "rooted.obj",
      "mtllib " + (dir / "materials" / "cube.mtl").string() + "\nusemtl cube\n",
      "");
  const struct {
    std::string path;
    fs::path map;
  } maps[] = {
      {square("abs.obj", "mtllib materials/abs.MTL\nusemtl a\n", ""), map},
      {square("back.obj", "mtllib materials\\back.MTL\nusemtl a\n", ""),
       dir / "materials" / "sub" / "cube.png"},
      {square("nest.obj", "mtllib materials/nest.mtl\nusemtl a\n", ""),
       dir / "materials" / "materials" / "cube.png"},
      {"cube.obj", fs::path("materials") / "cube.png"},
      {"rooted.obj", map}};
  const fs::path before = fs::current_path();
  fs::current_path(dir);
  for (const auto& m : maps) {
    const vkc::Result<io::TexturedMesh> mesh = io::load_textured_mesh(m.path);
    CHECK(mesh.ok());
    CHECK(fs::path(mesh->diffuse_texture) == m.map);
  }
  fs::current_path(before);
  std::puts("io_textured_mesh: OK");
  return 0;
}
