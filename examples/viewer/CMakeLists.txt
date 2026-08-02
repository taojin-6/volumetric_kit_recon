# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# The recon -> gfx reconstruction viewer examples: fuse_render (headless colour
# PNG) and fuse_viewer (live window). This directory is the ONE place recon's
# examples link the renderer sibling volumetric_kit_gfx -- pulled here (never in
# the tiers) and only under VR_BUILD_VIEWER, so recon's library and default
# build stay renderer-independent (the 2026-07-07 decision amendment in
# CLAUDE.md).

include(FetchContent)

# volumetric_kit_gfx: the sibling renderer. Its HybridMeshPipeline draws the
# per-vertex-coloured reconstruction the mesh tier emits. Pinned, built
# renderer-only (no tests / examples / install). Its core windowing is
# surface-agnostic, so the *app* owns the window (GLFW below); VG_WITH_GLFW here
# only keeps the pin consistent with gfx's own examples.
set(VG_BUILD_TESTS
    OFF
    CACHE BOOL "" FORCE)
set(VG_BUILD_EXAMPLES
    OFF
    CACHE BOOL "" FORCE)
set(VG_INSTALL
    OFF
    CACHE BOOL "" FORCE)
set(VG_WITH_GLFW
    ON
    CACHE BOOL "" FORCE)
FetchContent_Declare(
  volumetric_kit_gfx
  GIT_REPOSITORY https://github.com/taojin-6/volumetric_kit_gfx.git
  GIT_TAG 3516ebb45748285fea38f9aed666d16244e1d4a7 # hybrid mesh pipeline (#86)
  GIT_SHALLOW FALSE)
FetchContent_MakeAvailable(volumetric_kit_gfx)

# GLFW for the live viewer's window (matches how gfx's own examples get it).
find_package(glfw3 REQUIRED)

set(_viewer_recon_libs
    volumetric_kit::recon_tsdf volumetric_kit::recon_mesh
    volumetric_kit::recon_texture volumetric_kit::recon_volume
    vr_example_common)
set(_viewer_gfx_libs volumetric_kit::gfx_app volumetric_kit::gfx_pipelines
                     volumetric_kit::gfx_camera volumetric_kit::gfx_core)
set(_viewer_common_inc "${CMAKE_CURRENT_SOURCE_DIR}"
                       "${PROJECT_SOURCE_DIR}/examples/common")

# fuse_render: fuse a sequence, render the coloured mesh to a PNG headlessly
# (gfx OffscreenTarget + HybridMeshPipeline). No window -- runnable in CI.
add_executable(fuse_render fuse_render.cpp)
target_link_libraries(fuse_render PRIVATE ${_viewer_recon_libs} vr_stb_image
                                          ${_viewer_gfx_libs} Vulkan::Vulkan)
target_include_directories(fuse_render PRIVATE ${_viewer_common_inc})

# fuse_viewer: the live window -- fuses on a background thread, draws the
# growing reconstruction each frame following the capture trajectory.
add_executable(fuse_viewer fuse_viewer.cpp)
target_link_libraries(
  fuse_viewer PRIVATE ${_viewer_recon_libs} ${_viewer_gfx_libs} glfw
                      Vulkan::Vulkan)
target_include_directories(fuse_viewer PRIVATE ${_viewer_common_inc})
