# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# Example programs: the end-to-end dataset fuse pipeline (and, later, its live
# gfx viewer). Built when VR_BUILD_EXAMPLES is on. They link the recon tiers
# plus stb_image for RGB/depth decode -- an examples-only dependency; the
# shipped tiers take no image dependency, so this fetch is scoped here (not in
# third_party/, which is compiled into recon_core unconditionally).

include(FetchContent)

# stb: single-header image loaders (JPEG colour + 16-bit PNG depth). No CMake or
# tagged releases upstream, so pin a commit and expose the header directory as
# an INTERFACE target; SYSTEM include so the header stays off the tiers' -Werror
# surface wherever it is pulled in for its declarations.
FetchContent_Declare(
  stb
  GIT_REPOSITORY https://github.com/nothings/stb.git
  GIT_TAG 31c1ad37456438565541f4919958214b6e762fb4
  GIT_SHALLOW FALSE)
FetchContent_MakeAvailable(stb)

add_library(vr_stb_image INTERFACE)
target_include_directories(vr_stb_image SYSTEM INTERFACE "${stb_SOURCE_DIR}")

# tinyply: a small PLY reader/writer for the example's coloured mesh dump. Its
# single header carries the implementation under TINYPLY_IMPLEMENTATION (like
# stb), but it also ships a CMakeLists that unconditionally builds a demo app
# and adds install/export rules -- none of which we want -- so fetch it
# *without* running that CMake (SOURCE_SUBDIR points at a dir with no
# CMakeLists, so FetchContent skips add_subdirectory) and expose the header as
# an INTERFACE target, exactly as stb. The implementation is compiled in one -w
# TU (common/tinyply_impl.cpp). Examples-only: the shipped tiers take no mesh-io
# dependency; the first-class glTF/GLB exporter will live in the mesh tier
# (tinygltf, matching volumetric_kit_gfx).
FetchContent_Declare(
  tinyply
  GIT_REPOSITORY https://github.com/ddiakopoulos/tinyply.git
  GIT_TAG c9bb690dfe5e9105961e9e28120c48c9ae084bc6 # 3.0
  GIT_SHALLOW FALSE
  SOURCE_SUBDIR do-not-add-tinyply-cmake)
FetchContent_MakeAvailable(tinyply)

add_library(vr_tinyply INTERFACE)
target_include_directories(vr_tinyply SYSTEM
                           INTERFACE "${tinyply_SOURCE_DIR}/source")

add_subdirectory(common)
add_subdirectory(fuse_replica)

# The gfx-linked reconstruction viewer examples (fuse_render headless PNG +
# fuse_viewer live window). OFF by default: opting in is the ONLY place the
# recon examples pull in the renderer sibling (volumetric_kit_gfx via
# FetchContent) + system GLFW. The library tiers, the default build, and the
# other examples stay renderer-independent -- see the 2026-07-07 "viewer opts
# into gfx" decision in DECISIONS.md. Added last so it can link
# vr_example_common + vr_stb_image.
option(
  VR_BUILD_VIEWER
  "Build the gfx-linked reconstruction viewer examples (pulls volumetric_kit_gfx + GLFW)"
  OFF)
if(VR_BUILD_VIEWER)
  add_subdirectory(viewer)
endif()
