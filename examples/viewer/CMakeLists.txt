# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# The recon -> gfx reconstruction viewer examples: fuse_render (headless colour
# PNG) and fuse_viewer (live window). This directory is the ONE place recon's
# examples link the renderer sibling volumetric_kit_gfx -- pulled here (never in
# the tiers) and only under VR_BUILD_VIEWER, so recon's library and default
# build stay renderer-independent (the 2026-07-07 decision amendment in
# CLAUDE.md).
#
# Because it is that seam, it is also the thing most likely to drift out of
# build unnoticed -- and did, until a dedicated build-only macOS CI leg
# (.github/workflows/viewer.yml) started compiling both targets on every PR.

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
  # Must stay at or past gfx #88: before it, vg_embed_shaders left one glslc
  # rule reachable from two unrelated targets, so `make -j` ran it twice at once
  # and truncated the .spv -- "Invalid SPIR-V magic number",
  # nondeterministically. A warm build tree hides it by skipping shader
  # compilation entirely, which is why it only surfaced once CI started
  # discarding stale trees.
  #
  # And at or past gfx #92, which made AdoptedDevice's enabled_* block the
  # authority `Device::adopt` verifies against. shared_device.hpp's
  # gfx_adopt_payload sets those fields, so it does not compile before it -- and
  # a pin that stopped short would not merely lose a check but fail the adopt,
  # since the fields default to "not enabled".
  GIT_TAG d8949f6c1ddaa7c1011074c2e48a02521e53977b # gfx #92
  GIT_SHALLOW FALSE)
FetchContent_MakeAvailable(volumetric_kit_gfx)

# GLFW for the live viewer's window (matches how gfx's own examples get it).
find_package(glfw3 REQUIRED)

# Name only the tiers that are not already pulled in transitively. recon_texture
# PUBLIC-links recon_mesh + recon_volume and recon_tsdf PUBLIC-links
# recon_volume, so listing those again repeats their archives on the link line
# and Apple's linker warns "ignoring duplicate libraries". Same for gfx_core,
# which gfx_app / gfx_pipelines / gfx_ui all PUBLIC-link -- gfx_camera does not
# link it, which is why that one is named. Resist re-adding the covered tiers
# for readability's sake; the warning comes straight back.
set(_viewer_recon_libs volumetric_kit::recon_tsdf volumetric_kit::recon_texture
                       vr_example_common)
set(_viewer_gfx_libs volumetric_kit::gfx_app volumetric_kit::gfx_pipelines
                     volumetric_kit::gfx_camera)
set(_viewer_common_inc "${CMAKE_CURRENT_SOURCE_DIR}"
                       "${PROJECT_SOURCE_DIR}/examples/common")

# fuse_render: fuse a sequence, render the coloured mesh to a PNG headlessly
# (gfx OffscreenTarget + HybridMeshPipeline). No window -- runnable in CI.
add_executable(fuse_render fuse_render.cpp)
target_link_libraries(fuse_render PRIVATE ${_viewer_recon_libs} vr_stb_image
                                          ${_viewer_gfx_libs} Vulkan::Vulkan)
target_include_directories(fuse_render PRIVATE ${_viewer_common_inc})
vr_target_warnings(fuse_render)

# The Dear ImGui sources gfx fetched for its ui tier. gfx exports the path only
# into its own scope (third_party/CMakeLists.txt sets VG_IMGUI_SOURCE_DIR
# PARENT_SCOPE), so ask FetchContent for the populated dir instead of reaching
# into a variable that does not reach us. Guard it: an empty imgui_SOURCE_DIR
# would otherwise surface as an unreadable "Cannot find source file: /backends/
# imgui_impl_glfw.cpp" far from the cause.
FetchContent_GetProperties(imgui)
if(NOT imgui_SOURCE_DIR)
  message(
    FATAL_ERROR
      "fuse_viewer needs the Dear ImGui sources gfx fetches for its ui tier, "
      "but FetchContent has no populated 'imgui'. Check that the pinned "
      "volumetric_kit_gfx still declares it in third_party/CMakeLists.txt.")
endif()

# fuse_viewer: the live window -- fuses on a background thread, draws the
# growing reconstruction each frame following the capture trajectory, and shows
# the Dear ImGui performance + reconstruction panels (gfx's ui tier).
#
# The ui tier wraps only ImGui's Vulkan renderer backend, deliberately staying
# windowing-free, so the *platform* backend (imgui_impl_glfw) is this example's
# to compile -- exactly as gfx's own examples do it. imgui.h arrives
# transitively from gfx_ui (SYSTEM); backends/ is added here for
# imgui_impl_glfw.h.
add_executable(fuse_viewer fuse_viewer.cpp
                           "${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp")
target_link_libraries(
  fuse_viewer PRIVATE ${_viewer_recon_libs} ${_viewer_gfx_libs}
                      volumetric_kit::gfx_ui glfw Vulkan::Vulkan)
target_include_directories(fuse_viewer PRIVATE ${_viewer_common_inc})
target_include_directories(fuse_viewer SYSTEM
                           PRIVATE "${imgui_SOURCE_DIR}/backends")
# Both viewer examples carry the project's warning policy, like every other
# first-party target -- they are our code, and the overlay/panel logic is where
# a stale format specifier or an unused capture would otherwise go unseen.
#
# These targets are the one place recon's code meets gfx's API, so they are
# covered by their own CI leg (.github/workflows/viewer.yml) -- build-only on
# macOS, since the live viewer needs a window and both examples need a dataset
# CI does not carry. Added after that gap let a real recon/gfx API mismatch
# reach `main` unbuilt.
vr_target_warnings(fuse_viewer)
# ...but ImGui's own TU carries its own warning posture and would trip that
# policy. The source-level opt-out follows the target flags on the command line,
# so it wins; only this third-party TU is exempt, fuse_viewer.cpp stays strict.
# MSVC spells "no warnings" /w, everyone else -w.
set_source_files_properties(
  "${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp"
  PROPERTIES COMPILE_OPTIONS "$<IF:$<CXX_COMPILER_ID:MSVC>,/w,-w>")
