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
  # And at or past gfx #98, whose kHybridMeshNormals is rig_viewer's normals
  # shading mode. A compile floor: the flag does not exist before it.
  #
  # And at or past gfx #106, the end of gfx's move onto volumetric_kit_core
  # (#100-#106): gfx takes the core's DeviceRequirements and AdoptedDevice, so
  # shared_device.hpp hands it the shared device's graphics payload unconverted,
  # its textures are the core's Image, and FrameMetrics holds the core's
  # StageRow, so recon's stage rows reach the overlay as they are. A compile
  # floor: before it gfx has a Device, Status and Texture of its own.
  #
  # And at or past gfx #123 (on #120 and #118): frames numbered on a timeline,
  # with a RetireQueue keyed on those numbers; StreamedAtlas, the ring of atlas
  # images updated inside the frame, from several buffers at once (each camera's
  # tile from its own colour buffer); and a HybridMeshPipeline that owns the
  # fallback for a frame with no atlas. A compile floor: the viewers hold no
  # atlas ring, fallback or frame-slot keep-alive of their own.
  #
  # gfx builds against recon's core pin (third_party/CMakeLists.txt declares the
  # core first, so it wins over gfx's VG_VKC_PIN), which may run ahead of gfx's
  # own -- its check takes that commit or newer -- only by core changes that add
  # API. They agree at 5913731 (core #18).
  #
  # TODO: this pin is a placeholder. gfx's ubuntu-24.04 GPU and sanitizer legs
  # fail at it: synchronization validation reports a write-after-read hazard in
  # StreamedAtlas's update. Re-pin at gfx's fix, and at gfx main once #118, #120
  # and #123 merge.
  GIT_TAG 2ab9e86e5d0109af3da91f6dc7981fefdea90491 # gfx #123
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
# device. Defined here, where gfx is fetched. Skips (exit 0) with no device.
if(VR_BUILD_TESTS)
  add_executable(recon_example_viewer_atlas_test
                 "${PROJECT_SOURCE_DIR}/tests/example_viewer_atlas_test.cpp")
  target_link_libraries(
    recon_example_viewer_atlas_test
    PRIVATE volumetric_kit::recon_texture volumetric_kit::gfx_app
            volumetric_kit::gfx_pipelines Vulkan::Vulkan)
  target_include_directories(
    recon_example_viewer_atlas_test PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}"
                                            "${PROJECT_SOURCE_DIR}/tests")
  vr_target_warnings(recon_example_viewer_atlas_test)
  add_test(NAME recon_example_viewer_atlas
           COMMAND recon_example_viewer_atlas_test)
endif()

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
set_source_files_properties("${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp"
                            PROPERTIES COMPILE_OPTIONS -w)

# rig_viewer: fuse_viewer's sibling for a live, synced rig of Orbbec cameras,
# read as a sensor array, every camera's frame prepared, fused and textured on
# the GPU. Built only where the Orbbec driver is (VR_WITH_ORBBEC); the viewer CI
# leg installs the SDK, so it is compiled on every PR with the other two.
if(TARGET volumetric_kit::recon_sensor_orbbec)
  add_executable(rig_viewer rig_viewer.cpp
                            "${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp")
  # recon_sensor_array ahead of the list, which names recon_sensor_utils: a
  # library named before one that links it is named again, as above.
  target_link_libraries(
    rig_viewer
    PRIVATE volumetric_kit::recon_sensor_array
            ${_viewer_recon_libs}
            volumetric_kit::recon_sensor_orbbec
            ${_viewer_gfx_libs}
            volumetric_kit::gfx_ui
            glfw
            Vulkan::Vulkan)
  target_include_directories(rig_viewer PRIVATE ${_viewer_common_inc})
  target_include_directories(rig_viewer SYSTEM
                             PRIVATE "${imgui_SOURCE_DIR}/backends")
  vr_target_warnings(rig_viewer)
endif()
