# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# Example programs use the production IO tiers for image and mesh assets.
# Replica sequence parsing remains an example utility. grid's examples
# (codec_mesh, serve_stream) and the scaffolding and codec helpers recon's reach
# are grid's (its examples/).

add_subdirectory(common)
add_subdirectory(fuse_replica)
add_subdirectory(codec_replica)
# The live counterparts, each when its driver is built.
if(TARGET volumetric_kit::sensor_orbbec)
  add_subdirectory(fuse_orbbec)
endif()
if(TARGET volumetric_kit::sensor_realsense)
  add_subdirectory(fuse_realsense)
endif()
# fuse_decklink on every build: one without sensor's DeckLink driver or its FFS
# depth refuses the scenario before anything opens.
add_subdirectory(fuse_decklink)

# The gfx-linked reconstruction viewer examples (fuse_render headless PNG +
# fuse_viewer live window). OFF by default: opting in is the ONLY place the
# recon examples pull in the renderer sibling (volumetric_kit_gfx via
# FetchContent) + system GLFW. The library tiers, the default build, and the
# other examples stay renderer-independent -- see the 2026-07-07 "viewer opts
# into gfx" decision in DECISIONS.md. Added last so it can link
# vr_example_common.
option(
  VR_BUILD_VIEWER
  "Build the gfx-linked reconstruction viewer examples (pulls volumetric_kit_gfx + GLFW)"
  OFF)
if(VR_BUILD_VIEWER)
  add_subdirectory(viewer)
endif()

# Exercise each real executable's startup contract without creating resources,
# with the check grid's examples run (its tests/example_config_entry.cmake).
if(VR_BUILD_TESTS)
  foreach(
    _vr_example
    fuse_replica
    codec_replica
    fuse_orbbec
    fuse_realsense
    fuse_decklink
    fuse_render
    fuse_viewer
    rig_viewer)
    if(TARGET ${_vr_example})
      add_test(
        NAME recon_${_vr_example}_config_entry
        COMMAND
          ${CMAKE_COMMAND} -DPROGRAM=$<TARGET_FILE:${_vr_example}>
          -DWORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/config-entry/${_vr_example} -P
          ${volumetric_kit_grid_SOURCE_DIR}/tests/example_config_entry.cmake)
    endif()
  endforeach()
  # fuse_realsense's depth against its build: FFS refused before the GPU where
  # sensor_ffs is not built, and let through where it is.
  if(TARGET fuse_realsense)
    if(TARGET volumetric_kit::sensor_ffs)
      set(_vr_ffs ON)
    else()
      set(_vr_ffs OFF)
    endif()
    add_test(
      NAME recon_fuse_realsense_depth
      COMMAND
        ${CMAKE_COMMAND} -DPROGRAM=$<TARGET_FILE:fuse_realsense>
        -DFFS=${_vr_ffs}
        -DWORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/fuse-realsense-depth -P
        ${PROJECT_SOURCE_DIR}/tests/example_fuse_realsense_depth.cmake)
  endif()
  # fuse_decklink against its build: refused before the GPU, naming what is
  # missing, without the DeckLink driver or FFS; let through with both.
  foreach(_vr_tier decklink ffs)
    if(TARGET volumetric_kit::sensor_${_vr_tier})
      set(_vr_has_${_vr_tier} ON)
    else()
      set(_vr_has_${_vr_tier} OFF)
    endif()
  endforeach()
  add_test(
    NAME recon_fuse_decklink_build
    COMMAND
      ${CMAKE_COMMAND} -DPROGRAM=$<TARGET_FILE:fuse_decklink>
      -DDECKLINK=${_vr_has_decklink} -DFFS=${_vr_has_ffs}
      -DWORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/fuse-decklink-build -P
      ${PROJECT_SOURCE_DIR}/tests/example_fuse_decklink_build.cmake)
  # The colour frame of the Replica runs below: the sensor package's JPEG
  # fixture (VS_TEST_DATA_DIR, which the package sets).
  set(_vr_color_fixture "${VS_TEST_DATA_DIR}/jpeg/patches_256x144.jpg")
  # fuse_replica on a device: two frames of a wall, each meshed on the device,
  # then the PLY.
  add_test(
    NAME recon_fuse_replica_run
    COMMAND
      ${CMAKE_COMMAND} -DPROGRAM=$<TARGET_FILE:fuse_replica>
      -DCOLOR=${_vr_color_fixture}
      -DDEPTH=${PROJECT_SOURCE_DIR}/tests/data/replica/depth_1m_256x144.png
      -DWORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/fuse-replica-run -P
      ${PROJECT_SOURCE_DIR}/tests/example_fuse_replica_run.cmake)
  set_tests_properties(
    recon_fuse_replica_run PROPERTIES SKIP_REGULAR_EXPRESSION
                                      "skipping: no Vulkan device")
  # codec_replica on a device: four frames of the wall, each coded, and the
  # per-set table they print.
  add_test(
    NAME recon_codec_replica_run
    COMMAND
      ${CMAKE_COMMAND} -DPROGRAM=$<TARGET_FILE:codec_replica>
      -DCOLOR=${_vr_color_fixture}
      -DDEPTH=${PROJECT_SOURCE_DIR}/tests/data/replica/depth_1m_256x144.png
      -DWORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/codec-replica-run -P
      ${PROJECT_SOURCE_DIR}/tests/example_codec_replica_run.cmake)
  set_tests_properties(
    recon_codec_replica_run PROPERTIES SKIP_REGULAR_EXPRESSION
                                       "skipping: no Vulkan device")
endif()
