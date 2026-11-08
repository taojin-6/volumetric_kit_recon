# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# The recon -> gfx reconstruction viewer examples: fuse_render (headless colour
# PNG), fuse_viewer (live window) and rig_viewer (a live Orbbec rig). This
# directory is the ONE place recon's examples link the renderer sibling
# volumetric_kit_gfx -- pulled here (never in the tiers) and only under
# VR_BUILD_VIEWER, so recon's library and default build stay
# renderer-independent (the 2026-07-07 decision amendment in DECISIONS.md).
#
# Because it is that seam, it is also the thing most likely to drift out of
# build unnoticed -- and did, until the macOS viewer CI leg
# (.github/workflows/viewer.yml) started building these targets on every PR; it
# also runs their headless tests.

include(FetchContent)

# volumetric_kit_gfx: the sibling renderer. Its HybridMeshPipeline draws the
# per-vertex-coloured reconstruction the mesh tier emits. Pinned, built without
# tests or examples. VG_WITH_GLFW builds gfx_glfw, the live viewers' window
# glue.
set(VG_BUILD_TESTS
    OFF
    CACHE BOOL "" FORCE)
set(VG_BUILD_EXAMPLES
    OFF
    CACHE BOOL "" FORCE)
set(VG_WITH_GLFW
    ON
    CACHE BOOL "" FORCE)
FetchContent_Declare(
  volumetric_kit_gfx
  GIT_REPOSITORY https://github.com/taojin-6/volumetric_kit_gfx.git
  # Floors, newest first (CHANGELOG and this pin's git history record each
  # move). gfx #172: the View options' kHybridMeshCullBackFaces and
  # kHybridMeshWireframe (compile; #173 draws the wireframe as its visible edges
  # alone). gfx #144: ImGui's colours as authored on the _SRGB swapchain, not
  # washed out (rendering; recon_example_viewer_pass checks it). gfx #93: a
  # negative uv0 decodes as the carried coordinate -uv - 1; against #92 a
  # triangle mixing textured and carried vertices smears the atlas origin across
  # its face, along every occlusion silhouette, on default flags. That floor
  # fails silently: recon_fuse_render_run counts the pixels drawn, not their
  # colours (DECISIONS 2026-08-11). gfx refuses a core older than 0.20.0
  # (VG_VKC_MIN_VERSION).
  GIT_TAG ffc1a17a3d14aeb546e5484067dbf0160e76c60c # gfx main, #173's merge
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
# PUBLIC-links grid_mesh, recon_sender (the fuser's inputs, fuse_frame.hpp's)
# PUBLIC-links recon_tsdf (grid_volume) + sensor_frame, and vr_example_common
# the sensor package's prep, so listing those again repeats their archives on
# the link line and Apple's linker warns "ignoring duplicate libraries". Same
# for gfx_core, which gfx_app / gfx_pipelines / gfx_ui all PUBLIC-link --
# gfx_camera does not link it, which is why that one is named. Resist re-adding
# the covered tiers for readability's sake; the warning comes straight back.
set(_viewer_recon_libs volumetric_kit::recon_sender
                       volumetric_kit::recon_texture vr_example_common)
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
# fuse_render on a device: two frames of recon_fuse_replica_run's wall, the
# sensor package's JPEG fixture for colour, rendered to a PNG. A hang fails in
# 300 s, as tests/'s do.
if(VR_BUILD_TESTS)
  add_test(
    NAME recon_fuse_render_run
    COMMAND
      ${CMAKE_COMMAND} -DPROGRAM=$<TARGET_FILE:fuse_render>
      -DCOLOR=${VS_TEST_DATA_DIR}/jpeg/patches_256x144.jpg
      -DDEPTH=${PROJECT_SOURCE_DIR}/tests/data/replica/depth_1m_256x144.png
      -DWORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/fuse-render-run -P
      ${PROJECT_SOURCE_DIR}/tests/example_fuse_render_run.cmake)
  set_tests_properties(
    recon_fuse_render_run PROPERTIES SKIP_REGULAR_EXPRESSION
                                     "skipping: no Vulkan device" TIMEOUT 300)
endif()

# The atlas both live viewers draw with (viewer_atlas.hpp), on a headless gfx
# device. Defined here, where gfx is fetched. Skips with no device under the
# core policy.
if(VR_BUILD_TESTS)
  add_executable(recon_example_viewer_atlas_test
                 "${PROJECT_SOURCE_DIR}/tests/example_viewer_atlas_test.cpp")
  target_link_libraries(
    recon_example_viewer_atlas_test
    PRIVATE volumetric_kit::core_test_base volumetric_kit::core_test_policy
            volumetric_kit::recon_texture volumetric_kit::gfx_app
            volumetric_kit::gfx_pipelines Vulkan::Vulkan)
  target_include_directories(recon_example_viewer_atlas_test
                             PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
  vr_target_warnings(recon_example_viewer_atlas_test)
  add_test(NAME recon_example_viewer_atlas
           COMMAND recon_example_viewer_atlas_test)
  # Bound a regression that waits for a frame which was never submitted.
  set_tests_properties(recon_example_viewer_atlas
                       PROPERTIES TIMEOUT 30 SKIP_RETURN_CODE 77)

  # The device both live viewers adopt (viewer_common.hpp), for recon's and the
  # sensor package's requirements (examples/common/fuse_device.hpp). Host only.
  add_executable(recon_example_viewer_device_test
                 "${PROJECT_SOURCE_DIR}/tests/example_viewer_device_test.cpp")
  target_link_libraries(
    recon_example_viewer_device_test
    PRIVATE volumetric_kit::core_test_base
            volumetric_kit::recon_core
            volumetric_kit::grid_mesh
            volumetric_kit::sensor_frame
            volumetric_kit::gfx_glfw
            volumetric_kit::gfx_pipelines
            Vulkan::Vulkan)
  target_include_directories(recon_example_viewer_device_test
                             PRIVATE ${_viewer_common_inc})
  vr_target_warnings(recon_example_viewer_device_test)
  add_test(NAME recon_example_viewer_device
           COMMAND recon_example_viewer_device_test)

  # What rig_viewer prints at the end (rig_report.hpp): the atlas copies' device
  # time from the profiler's late snapshots, and a count's rate. Host only;
  # gfx_glfw for ImGui's headers, which the report's panel includes,
  # gfx_pipelines for the View options' flags (viewer_panels.hpp), and
  # grid_example_common for its stage table.
  add_executable(
    recon_example_viewer_rig_report_test
    "${PROJECT_SOURCE_DIR}/tests/example_viewer_rig_report_test.cpp")
  target_link_libraries(
    recon_example_viewer_rig_report_test
    PRIVATE volumetric_kit::core_test_base
            volumetric_kit::sensor_array
            volumetric_kit::recon_texture
            volumetric_kit::gfx_glfw
            volumetric_kit::gfx_pipelines
            grid_example_common
            Vulkan::Vulkan)
  target_include_directories(recon_example_viewer_rig_report_test
                             PRIVATE ${_viewer_common_inc})
  vr_target_warnings(recon_example_viewer_rig_report_test)
  add_test(NAME recon_example_viewer_rig_report
           COMMAND recon_example_viewer_rig_report_test)

  # The render pass both live viewers record (viewer_common.hpp), on a headless
  # gfx device. Skips with no device under the core policy.
  add_executable(recon_example_viewer_pass_test
                 "${PROJECT_SOURCE_DIR}/tests/example_viewer_pass_test.cpp")
  target_link_libraries(
    recon_example_viewer_pass_test
    PRIVATE volumetric_kit::core_test_base
            volumetric_kit::core_test_policy
            volumetric_kit::recon_core
            volumetric_kit::grid_mesh
            volumetric_kit::sensor_frame
            volumetric_kit::gfx_glfw
            volumetric_kit::gfx_pipelines
            Vulkan::Vulkan)
  target_include_directories(recon_example_viewer_pass_test
                             PRIVATE ${_viewer_common_inc})
  vr_target_warnings(recon_example_viewer_pass_test)
  add_test(NAME recon_example_viewer_pass
           COMMAND recon_example_viewer_pass_test)
endif()

# fuse_viewer: the live window -- fuses on a background thread, draws the
# growing reconstruction each frame following the capture trajectory, and shows
# the Dear ImGui performance, reconstruction and View panels (gfx's ui tier).
# gfx_glfw PUBLIC-links gfx_app, gfx_ui (and through it gfx_camera) and GLFW, so
# those are not named again.
add_executable(fuse_viewer fuse_viewer.cpp)
target_link_libraries(
  fuse_viewer PRIVATE ${_viewer_recon_libs} volumetric_kit::gfx_glfw
                      volumetric_kit::gfx_pipelines Vulkan::Vulkan)
target_include_directories(fuse_viewer PRIVATE ${_viewer_common_inc})
# The viewer examples carry the project's warning policy, like every other
# first-party target -- they are our code, and the overlay/panel logic is where
# a stale format specifier or an unused capture would otherwise go unseen.
#
# These targets are the one place recon's code meets gfx's API, so they have
# their own CI leg (.github/workflows/viewer.yml), added after a real recon/gfx
# API mismatch reached `main` unbuilt: it builds them on macOS and runs the
# headless tests above, fuse_render on a two-frame wall among them, and their
# config entries; the live viewers need a window and a dataset or a rig CI does
# not carry.
vr_target_warnings(fuse_viewer)

# rig_viewer: fuse_viewer's sibling for a live, synced rig of Orbbec cameras,
# read as a sensor array, every camera's frame prepared, fused and textured on
# the GPU. Built only where the Orbbec driver is (VS_WITH_ORBBEC); the viewer CI
# leg installs the SDK, so it is compiled on every PR with the other two.
if(TARGET volumetric_kit::sensor_orbbec)
  add_executable(rig_viewer rig_viewer.cpp)
  # sensor_array ahead of the list, whose vr_example_common links sensor_prep: a
  # library named before one that links it is named again, as above.
  target_link_libraries(
    rig_viewer
    PRIVATE volumetric_kit::sensor_array ${_viewer_recon_libs}
            volumetric_kit::sensor_orbbec volumetric_kit::gfx_glfw
            volumetric_kit::gfx_pipelines Vulkan::Vulkan)
  target_include_directories(rig_viewer PRIVATE ${_viewer_common_inc})
  vr_target_warnings(rig_viewer)
endif()

# Last, so every test above takes examples/'s skip code and timeout.
if(VR_BUILD_TESTS)
  vr_example_test_limits()
endif()
