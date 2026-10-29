# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# Example programs use the production IO tier for image and mesh assets. Replica
# sequence parsing remains an example utility.

add_subdirectory(common)
add_subdirectory(fuse_replica)
add_subdirectory(codec_replica)
if(TARGET recon_mesh_io_assimp)
  add_subdirectory(codec_mesh)
endif()
# The live counterpart, when the Orbbec driver is built.
if(TARGET volumetric_kit::recon_sensor_orbbec)
  add_subdirectory(fuse_orbbec)
endif()
# A recording served live, with no camera or GPU.
add_subdirectory(serve_stream)

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

# Exercise each real executable's startup contract without creating resources.
if(VR_BUILD_TESTS)
  foreach(
    _vr_example
    fuse_replica
    codec_replica
    codec_mesh
    fuse_orbbec
    fuse_render
    fuse_viewer
    rig_viewer)
    if(TARGET ${_vr_example})
      add_test(
        NAME recon_${_vr_example}_config_entry
        COMMAND
          ${CMAKE_COMMAND} -DPROGRAM=$<TARGET_FILE:${_vr_example}>
          -DWORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/config-entry/${_vr_example} -P
          ${PROJECT_SOURCE_DIR}/tests/example_config_entry.cmake)
    endif()
  endforeach()
  # serve_stream's arguments: usage and exit 2 for bad ones, 1 for a missing
  # file, before it listens; it is never run to serve, which would listen on
  # every interface.
  add_test(
    NAME recon_serve_stream_args
    COMMAND
      ${CMAKE_COMMAND} -DPROGRAM=$<TARGET_FILE:serve_stream>
      -DWORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/serve-stream-args -P
      ${PROJECT_SOURCE_DIR}/tests/example_serve_stream_args.cmake)
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
  # codec_mesh's order on a device: an existing stream refused before any frame
  # is converted or written, then a two-frame run.
  if(TARGET codec_mesh)
    add_test(
      NAME recon_codec_mesh_run
      COMMAND
        ${CMAKE_COMMAND} -DPROGRAM=$<TARGET_FILE:codec_mesh>
        -DWORK_DIR=${CMAKE_CURRENT_BINARY_DIR}/codec-mesh-run -P
        ${PROJECT_SOURCE_DIR}/tests/example_codec_mesh_run.cmake)
    set_tests_properties(
      recon_codec_mesh_run PROPERTIES SKIP_REGULAR_EXPRESSION
                                      "skipping: no Vulkan device")
  endif()
endif()
