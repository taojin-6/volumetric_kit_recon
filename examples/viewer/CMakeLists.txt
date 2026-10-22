# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# The recon -> gfx reconstruction viewer examples: fuse_render (headless colour
# PNG) and fuse_viewer (live window). This directory is the ONE place recon's
# examples link the renderer sibling volumetric_kit_gfx -- pulled here (never in
# the tiers) and only under VR_BUILD_VIEWER, so recon's library and default
# build stay renderer-independent (the 2026-07-07 decision amendment in
# DECISIONS.md).
#
# Because it is that seam, it is also the thing most likely to drift out of
# build unnoticed -- and did, until a dedicated build-only macOS CI leg
# (.github/workflows/viewer.yml) started compiling both targets on every PR.

include(FetchContent)

# volumetric_kit_gfx: the sibling renderer. Its HybridMeshPipeline draws the
# per-vertex-coloured reconstruction the mesh tier emits. Pinned, built without
# tests, examples or install. VG_WITH_GLFW builds gfx_glfw, the live viewers'
# window glue.
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
  # And at or past gfx #93, which decodes a negative uv0 as the carried
  # coordinate `-uv - 1` rather than substituting (0, 0). This floor is a
  # RENDERING one, not a compile one, and it is the only one here that fails
  # silently. The texture tier now decides visibility per vertex, so a triangle
  # may mix a textured vertex with a carried one -- the all-three-vertices gate
  # that made such a triangle impossible is gone. Against gfx #92 the carried
  # vertices forward (0, 0), so frag_uv interpolates from a real coordinate to
  # the atlas origin and smears the corner of the camera image across the face,
  # along every occlusion silhouette and the whole frustum boundary. Both viewer
  # examples draw that on default flags; nothing in CI catches it, since this
  # leg builds them and does not render.
  #
  # And at or past gfx #133, which adds gfx_glfw (above). A compile floor.
  #
  # And at or past gfx #144, which shows ImGui's colours as authored on the
  # default _SRGB swapchain; before it the panels were encoded twice and drew
  # washed out. A rendering floor, and the newest, so it covers the older ones;
  # recon_example_viewer_pass checks it.
  GIT_TAG b35520f5c6068e93bef0dccadd3ef1fa10fa9c6b # gfx main, after #148
  GIT_SHALLOW FALSE)
FetchContent_MakeAvailable(volumetric_kit_gfx)

# gfx builds gfx_glfw only where its own lookup, which holds the GLFW version
# floor, finds GLFW.
if(NOT TARGET volumetric_kit::gfx_glfw)
  message(
    FATAL_ERROR
      "The live viewers need gfx's gfx_glfw, which gfx builds only where it "
      "finds GLFW: install it (brew install glfw, or libglfw3-dev).")
endif()

# Name only the tiers that are not already pulled in transitively. recon_texture
# PUBLIC-links recon_mesh + recon_volume and recon_tsdf PUBLIC-links
# recon_volume, so listing those again repeats their archives on the link line
# and Apple's linker warns "ignoring duplicate libraries". Same for gfx_core,
# which gfx_app / gfx_pipelines / gfx_ui all PUBLIC-link -- gfx_camera does not
# link it, which is why that one is named. Resist re-adding the covered tiers
# for readability's sake; the warning comes straight back.
set(_viewer_recon_libs volumetric_kit::recon_tsdf volumetric_kit::recon_texture
                       volumetric_kit::recon_sensor_utils vr_example_common)
set(_viewer_gfx_libs volumetric_kit::gfx_app volumetric_kit::gfx_pipelines
                     volumetric_kit::gfx_camera)
set(_viewer_common_inc "${CMAKE_CURRENT_SOURCE_DIR}"
                       "${PROJECT_SOURCE_DIR}/examples/common")

# fuse_render: fuse a sequence, render the coloured mesh to a PNG headlessly
# (gfx OffscreenTarget + HybridMeshPipeline). No window -- runnable in CI.
add_executable(fuse_render fuse_render.cpp)
target_link_libraries(fuse_render PRIVATE ${_viewer_recon_libs}
                                          ${_viewer_gfx_libs} Vulkan::Vulkan)
target_include_directories(fuse_render PRIVATE ${_viewer_common_inc})
vr_target_warnings(fuse_render)

# The atlas both live viewers draw with (viewer_atlas.hpp), on a headless gfx
# device. Defined here, where gfx is fetched. Skips with no device under the
# core policy.
if(VR_BUILD_TESTS)
  add_executable(recon_example_viewer_atlas_test
                 "${PROJECT_SOURCE_DIR}/tests/example_viewer_atlas_test.cpp")
  target_link_libraries(
    recon_example_viewer_atlas_test
    PRIVATE volumetric_kit::core_test_policy volumetric_kit::recon_texture
            volumetric_kit::gfx_app volumetric_kit::gfx_pipelines
            Vulkan::Vulkan)
  target_include_directories(
    recon_example_viewer_atlas_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}"
                                            "${PROJECT_SOURCE_DIR}/tests")
  vr_target_warnings(recon_example_viewer_atlas_test)
  add_test(NAME recon_example_viewer_atlas
           COMMAND recon_example_viewer_atlas_test)
  # Bound a regression that waits for a frame which was never submitted.
  set_tests_properties(recon_example_viewer_atlas
                       PROPERTIES TIMEOUT 30 SKIP_RETURN_CODE 77)

  # The device both live viewers adopt (viewer_common.hpp). Host only.
  add_executable(recon_example_viewer_device_test
                 "${PROJECT_SOURCE_DIR}/tests/example_viewer_device_test.cpp")
  target_link_libraries(
    recon_example_viewer_device_test
    PRIVATE volumetric_kit::recon_mesh volumetric_kit::gfx_glfw Vulkan::Vulkan)
  target_include_directories(
    recon_example_viewer_device_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}"
                                             "${PROJECT_SOURCE_DIR}/tests")
  vr_target_warnings(recon_example_viewer_device_test)
  add_test(NAME recon_example_viewer_device
           COMMAND recon_example_viewer_device_test)

  # The render pass both live viewers record (viewer_common.hpp), on a headless
  # gfx device. Skips with no device under the core policy.
  add_executable(recon_example_viewer_pass_test
                 "${PROJECT_SOURCE_DIR}/tests/example_viewer_pass_test.cpp")
  target_link_libraries(
    recon_example_viewer_pass_test
    PRIVATE volumetric_kit::core_test_policy volumetric_kit::recon_mesh
            volumetric_kit::gfx_glfw Vulkan::Vulkan)
  target_include_directories(
    recon_example_viewer_pass_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}"
                                           "${PROJECT_SOURCE_DIR}/tests")
  vr_target_warnings(recon_example_viewer_pass_test)
  add_test(NAME recon_example_viewer_pass
           COMMAND recon_example_viewer_pass_test)
  set_tests_properties(recon_example_viewer_pass PROPERTIES SKIP_RETURN_CODE 77)
endif()

# fuse_viewer: the live window -- fuses on a background thread, draws the
# growing reconstruction each frame following the capture trajectory, and shows
# the Dear ImGui performance + reconstruction panels (gfx's ui tier). gfx_glfw
# PUBLIC-links gfx_app, gfx_ui (and through it gfx_camera) and GLFW, so those
# are not named again.
add_executable(fuse_viewer fuse_viewer.cpp)
target_link_libraries(
  fuse_viewer PRIVATE ${_viewer_recon_libs} volumetric_kit::gfx_glfw
                      volumetric_kit::gfx_pipelines Vulkan::Vulkan)
target_include_directories(fuse_viewer PRIVATE ${_viewer_common_inc})
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

# rig_viewer: fuse_viewer's sibling for a live, synced rig of Orbbec cameras,
# read as a sensor array, every camera's frame prepared, fused and textured on
# the GPU. Built only where the Orbbec driver is (VR_WITH_ORBBEC); the viewer CI
# leg installs the SDK, so it is compiled on every PR with the other two.
if(TARGET volumetric_kit::recon_sensor_orbbec)
  add_executable(rig_viewer rig_viewer.cpp)
  # recon_sensor_array ahead of the list, which names recon_sensor_utils: a
  # library named before one that links it is named again, as above.
  target_link_libraries(
    rig_viewer
    PRIVATE volumetric_kit::recon_sensor_array ${_viewer_recon_libs}
            volumetric_kit::recon_sensor_orbbec volumetric_kit::gfx_glfw
            volumetric_kit::gfx_pipelines Vulkan::Vulkan)
  target_include_directories(rig_viewer PRIVATE ${_viewer_common_inc})
  vr_target_warnings(rig_viewer)
endif()
