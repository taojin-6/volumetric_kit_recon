# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# Example programs use the production IO tiers for image and mesh assets.
# Replica sequence parsing remains an example utility. grid's examples
# (codec_mesh, serve_stream) and the scaffolding and codec helpers recon's reach
# are grid's (grid/examples).

add_subdirectory(common)
add_subdirectory(fuse_replica)
add_subdirectory(codec_replica)
# The live counterpart, when the Orbbec driver is built.
if(TARGET volumetric_kit::recon_sensor_orbbec)
  add_subdirectory(fuse_orbbec)
endif()

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
# with the check grid's examples run (grid/tests/example_config_entry.cmake).
if(VR_BUILD_TESTS)
  foreach(_vr_example fuse_replica codec_replica fuse_orbbec fuse_render
                      fuse_viewer rig_viewer)
    if(TARGET ${_vr_example})
      add_test(
        NAME recon_${_vr_example}_config_entry
        COMMAND
          ${CMAKE_COMMAND} -DPROGRAM=$<TARGET_FILE:${_vr_example}>
          -DWORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/config-entry/${_vr_example} -P
          ${volumetric_kit_grid_SOURCE_DIR}/tests/example_config_entry.cmake)
    endif()
  endforeach()
  # fuse_replica on a device: two frames of a wall, each meshed on the device,
  # then the PLY.
  add_test(
    NAME recon_fuse_replica_run
    COMMAND
      ${CMAKE_COMMAND} -DPROGRAM=$<TARGET_FILE:fuse_replica>
      -DCOLOR=${PROJECT_SOURCE_DIR}/tests/data/jpeg/patches_256x144.jpg
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
      -DCOLOR=${PROJECT_SOURCE_DIR}/tests/data/jpeg/patches_256x144.jpg
      -DDEPTH=${PROJECT_SOURCE_DIR}/tests/data/replica/depth_1m_256x144.png
      -DWORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/codec-replica-run -P
      ${PROJECT_SOURCE_DIR}/tests/example_codec_replica_run.cmake)
  set_tests_properties(
    recon_codec_replica_run PROPERTIES SKIP_REGULAR_EXPRESSION
                                       "skipping: no Vulkan device")
endif()
