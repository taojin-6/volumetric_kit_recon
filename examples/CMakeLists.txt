# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# Example programs use the production IO tier for image and mesh assets. Replica
# sequence parsing remains an example utility.

add_subdirectory(common)
add_subdirectory(fuse_replica)
add_subdirectory(codec_replica)
if(TARGET recon_io_assimp)
  add_subdirectory(codec_mesh)
endif()
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
