# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# A separate project: every include and linked archive must come from the
# installation, never from recon's build-tree targets or private dependencies.
cmake_minimum_required(VERSION 3.21)
project(recon_installed_io_consumer LANGUAGES CXX)

option(VR_EXPECT_ASSIMP "Expect the optional installed mesh loader" OFF)
find_package(volumetric_kit_recon CONFIG REQUIRED)
if(VR_EXPECT_ASSIMP AND NOT TARGET volumetric_kit::recon_io_assimp)
  message(FATAL_ERROR "The installed package is missing recon_io_assimp")
elseif(NOT VR_EXPECT_ASSIMP AND TARGET volumetric_kit::recon_io_assimp)
  message(FATAL_ERROR "The Assimp-disabled package exported recon_io_assimp")
endif()

file(
  WRITE "${CMAKE_CURRENT_BINARY_DIR}/main.cpp"
  [=[
#include <cstdio>
#include <fstream>
#include <string>
#include <volumetric_kit/recon/io/image_io.hpp>
#include <volumetric_kit/recon/io/ply_writer.hpp>
#if VR_EXPECT_ASSIMP
#include <volumetric_kit/recon/io/mesh_io.hpp>
#else
#if __has_include(<volumetric_kit/recon/io/mesh_io.hpp>) || \
    __has_include(<volumetric_kit/recon/io/assimp_export.hpp>)
#error "An Assimp-disabled installation must not ship optional IO headers"
#endif
#endif

// Model an application bringing its own backend copies without including any
// private or third-party header. These strong symbols must coexist with IO.
int consumer_stb_calls = 0;
extern "C" unsigned char* stbi_load(const char*, int*, int*, int*, int) {
  ++consumer_stb_calls;
  return nullptr;
}
extern "C" void stbi_image_free(void*) { ++consumer_stb_calls; }
extern "C" int stbi_write_png_to_func(void (*)(void*, void*, int), void*,
                                      int, int, int, const void*, int) {
  ++consumer_stb_calls;
  return 0;
}
int consumer_ply_constructions = 0;
int consumer_ply_destructions = 0;
namespace tinyply {
class PlyFile {
 public:
  PlyFile();
  ~PlyFile();
};
PlyFile::PlyFile() { ++consumer_ply_constructions; }
PlyFile::~PlyFile() { ++consumer_ply_destructions; }
}

struct RemoveOutputs {
  ~RemoveOutputs() {
    std::remove("consumer.png");
    std::remove("consumer.ply");
  }
};

int main() {
  namespace vr = volumetric_kit::recon;
  RemoveOutputs cleanup;
  // Invalid inputs exercise every exported entry point without a device.
  auto color = vr::io::load_color_packed("", 1, 1);
  auto depth = vr::io::load_depth_metres("", 1, 1, 1.0f);
  auto png = vr::io::write_png_rgba8("", nullptr, 0, 0, 0);
  vr::mesh::Mesh mesh;
  auto ply = vr::io::write_ply("", mesh);
  if (color.ok() || depth.ok() || png.ok() || ply.ok()) return 1;

  const std::uint8_t pixels[] = {12, 34, 56, 78, 90, 123, 210, 255};
  if (!vr::io::write_png_rgba8("consumer.png", pixels, sizeof(pixels), 2, 1)) {
    std::fprintf(stderr, "installed IO PNG write failed\n");
    return 1;
  }
  auto loaded = vr::io::load_color_packed("consumer.png", 2, 1);
  if (!loaded || loaded.value().size() != 2 ||
      loaded.value()[0] != (12u | (34u << 8) | (56u << 16)) ||
      loaded.value()[1] != (90u | (123u << 8) | (210u << 16)) ||
      consumer_stb_calls != 0) {
    std::fprintf(stderr, "installed IO collided with consumer stb symbols\n");
    return 1;
  }
  // A valid image with the wrong depth format must still be rejected.
  if (vr::io::load_depth_metres("consumer.png", 2, 1, 1.0f)) return 1;

  {
    tinyply::PlyFile consumer_ply;
  }
  mesh.vertices.resize(3);
  for (auto& vertex : mesh.vertices) {
    vertex.position = vr::Vec3f(0.0f);
    vertex.normal = vr::Vec3f(0.0f, 0.0f, 1.0f);
    vertex.color = vr::Vec4f(1.0f);
  }
  mesh.vertices[1].position.x = 1.0f;
  mesh.vertices[2].position.y = 1.0f;
  mesh.indices = {0, 1, 2};
  if (!vr::io::write_ply("consumer.ply", mesh) ||
      consumer_ply_constructions != 1 || consumer_ply_destructions != 1) {
    std::fprintf(stderr, "installed IO collided with consumer tinyply symbols\n");
    return 1;
  }
  std::ifstream ply_file("consumer.ply", std::ios::binary);
  std::string magic;
  if (!std::getline(ply_file, magic) || magic != "ply") return 1;
#if VR_EXPECT_ASSIMP
  if (vr::io::load_mesh("").ok()) return 1;
#endif
  return 0;
}
]=])
add_executable(recon_installed_io_consumer
               "${CMAKE_CURRENT_BINARY_DIR}/main.cpp")
target_compile_features(recon_installed_io_consumer PRIVATE cxx_std_17)
target_compile_definitions(recon_installed_io_consumer
                           PRIVATE VR_EXPECT_ASSIMP=$<BOOL:${VR_EXPECT_ASSIMP}>)
# Applications may disable exceptions; the IO implementation catches its own.
target_compile_options(
  recon_installed_io_consumer
  PRIVATE "$<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-fno-exceptions>"
          "$<$<CXX_COMPILER_ID:MSVC>:/EHs-c->")
if(VR_EXPECT_ASSIMP)
  target_link_libraries(recon_installed_io_consumer
                        PRIVATE volumetric_kit::recon_io_assimp)
else()
  target_link_libraries(recon_installed_io_consumer
                        PRIVATE volumetric_kit::recon_io)
endif()
enable_testing()
add_test(NAME recon_installed_io_consumer COMMAND recon_installed_io_consumer)
