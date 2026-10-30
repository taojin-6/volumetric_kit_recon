# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# Every test is a main() of its own on the core's test support: CHECK,
# VKC_TEST_SANITIZED and the scratch directory (core_test_base). A GPU test also
# links core_test_policy, and one that fails or measures allocations
# core_test_allocation_failure.
link_libraries(volumetric_kit::core_test_base)

# recon's libraries link the sensor package's frame contract alone
# (sensor_frame): the frames they read, not the camera stack that made them. No
# library target under src/ links another sensor tier or the package's umbrella,
# as this configure links them (library_sensor_links.cmake reads the list
# written here); examples and tests may link any tier.
set(_vr_library_links "")
set(_vr_dirs "${PROJECT_SOURCE_DIR}/src")
while(_vr_dirs)
  list(POP_FRONT _vr_dirs _vr_dir)
  get_property(
    _vr_subdirs
    DIRECTORY "${_vr_dir}"
    PROPERTY SUBDIRECTORIES)
  list(APPEND _vr_dirs ${_vr_subdirs})
  get_property(
    _vr_targets
    DIRECTORY "${_vr_dir}"
    PROPERTY BUILDSYSTEM_TARGETS)
  foreach(_vr_target IN LISTS _vr_targets)
    get_target_property(_vr_type ${_vr_target} TYPE)
    if(NOT _vr_type MATCHES "LIBRARY$")
      continue()
    endif()
    set(_vr_links "")
    foreach(_vr_property LINK_LIBRARIES INTERFACE_LINK_LIBRARIES)
      if(_vr_type STREQUAL "INTERFACE_LIBRARY" AND _vr_property STREQUAL
                                                   "LINK_LIBRARIES")
        continue()
      endif()
      get_target_property(_vr_value ${_vr_target} ${_vr_property})
      if(_vr_value)
        list(APPEND _vr_links ${_vr_value})
      endif()
    endforeach()
    string(REPLACE ";" " " _vr_links "${_vr_links}")
    string(APPEND _vr_library_links "${_vr_target}: ${_vr_links}\n")
  endforeach()
endwhile()
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/library_links.txt"
     "${_vr_library_links}")
add_test(
  NAME recon_library_sensor_links
  COMMAND
    "${CMAKE_COMMAND}" "-DLINKS=${CMAKE_CURRENT_BINARY_DIR}/library_links.txt"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/library_sensor_links.cmake")

# A sibling that fetches recon (subproject/) takes each dependency this build
# fetched as a local source, so it needs no network.
file(
  GLOB _vr_fetched
  LIST_DIRECTORIES true
  "${FETCHCONTENT_BASE_DIR}/*-src")
set(_vr_fetched_sources)
foreach(_vr_source IN LISTS _vr_fetched)
  get_filename_component(_vr_name "${_vr_source}" NAME)
  string(REGEX REPLACE "-src$" "" _vr_name "${_vr_name}")
  string(TOUPPER "${_vr_name}" _vr_name)
  list(APPEND _vr_fetched_sources
       "-DFETCHCONTENT_SOURCE_DIR_${_vr_name}=${_vr_source}")
endforeach()
# A dependency this build took from a local source instead, as given.
get_cmake_property(_vr_cache CACHE_VARIABLES)
foreach(_vr_var IN LISTS _vr_cache)
  if(_vr_var MATCHES "^FETCHCONTENT_SOURCE_DIR_")
    if(NOT "${${_vr_var}}" STREQUAL "")
      list(APPEND _vr_fetched_sources "-D${_vr_var}=${${_vr_var}}")
    endif()
  endif()
endforeach()

# The tests also take grid's shared helpers (grid_test_support): a grid's blocks
# made and read on the device, a sphere's field, a codec test's device, a
# stage's rows, a .vrst's bytes and loopback serving.
link_libraries(volumetric_kit::grid_test_support)

# The core tier's own pieces: the version and the log seam's "vr" source.
add_executable(recon_core_smoke_test core_smoke_test.cpp)
target_link_libraries(recon_core_smoke_test PRIVATE volumetric_kit::recon_core)
vr_target_warnings(recon_core_smoke_test)
add_test(NAME recon_core_smoke COMMAND recon_core_smoke_test)

# Every tier reports, and reports the DEVICE half. The load-bearing assertion is
# per row: gpu_ms < cpu_ms, since the host span wraps the submit the device span
# sits inside -- equal numbers mean the wall clock was relabelled. Also pins
# that a breakdown sits inside its stage, device time included, that a null
# out-param measures nothing, and that a call keeps no queries.
add_executable(recon_tier_stage_metrics_test tier_stage_metrics_test.cpp)
target_link_libraries(
  recon_tier_stage_metrics_test
  PRIVATE volumetric_kit::recon_texture volumetric_kit::grid_mesh
          volumetric_kit::recon_tsdf volumetric_kit::grid_volume
          volumetric_kit::recon_core volumetric_kit::core_test_policy)
vr_target_warnings(recon_tier_stage_metrics_test)
target_link_libraries(recon_tier_stage_metrics_test PRIVATE ${CMAKE_DL_LIBS})
add_test(NAME recon_tier_stage_metrics COMMAND recon_tier_stage_metrics_test)

# grid's depth unprojection (volume/shaders/depth_pinhole.glsl, its copy of the
# published pinhole) on the device against sensor/frame/shaders/pinhole.glsl's,
# bit for bit, and the host's pinhole in double
# (shaders/depth_unproject_cases.comp). Here in recon, which has both: the
# kernel reads grid's src/, whose copy is internal, and the GLSL sensor_frame
# exports. Skips where no device is present.
add_executable(recon_volume_depth_unproject_test
               volume_depth_unproject_test.cpp)
vkc_embed_shaders(
  recon_volume_depth_unproject_test
  SYMBOL_PREFIX
  vr_
  INCLUDE_DIRS
  "${volumetric_kit_grid_SOURCE_DIR}/src"
  INCLUDE_TARGETS
  volumetric_kit::sensor_frame
  SPIRV_VAL_ARGS
  --scalar-block-layout
  SHADERS
  shaders/depth_unproject_cases.comp)
target_link_libraries(
  recon_volume_depth_unproject_test
  PRIVATE volumetric_kit::recon_core volumetric_kit::sensor_frame
          Vulkan::Vulkan volumetric_kit::core_test_policy)
vr_target_warnings(recon_volume_depth_unproject_test)
add_test(NAME recon_volume_depth_unproject
         COMMAND recon_volume_depth_unproject_test)

# tsdf::to_depth_pinhole: fusion's DepthCameraParams becomes volume's
# DepthPinhole field for field, byte for byte, the two layouts asserted equal.
# Host-only, so it always runs.
add_executable(recon_tsdf_depth_pinhole_test tsdf_depth_pinhole_test.cpp)
target_link_libraries(recon_tsdf_depth_pinhole_test
                      PRIVATE volumetric_kit::recon_tsdf)
vr_target_warnings(recon_tsdf_depth_pinhole_test)
add_test(NAME recon_tsdf_depth_pinhole COMMAND recon_tsdf_depth_pinhole_test)

# The sensor boundary's curve (sensor/frame/color_space.hpp) and the voxel
# colour format's (volume/voxel_color.hpp), held to each other bit for bit. Here
# in tsdf, which links both, so the volume tier takes no colour vocabulary from
# core. Host-only, so it always runs.
add_executable(recon_tsdf_voxel_color_space_test
               tsdf_voxel_color_space_test.cpp)
target_link_libraries(recon_tsdf_voxel_color_space_test
                      PRIVATE volumetric_kit::recon_tsdf)
vr_target_warnings(recon_tsdf_voxel_color_space_test)
add_test(NAME recon_tsdf_voxel_color_space
         COMMAND recon_tsdf_voxel_color_space_test)

# tsdf integrate: fuse a synthetic constant-depth plane into a VoxelBlockGrid
# and verify the classic projective TSDF math per voxel (sdf, truncation, weight
# + cap), and a split dispatch (grid's base/dispatch.hpp test seam). It also
# runs the kernel's projection alone (shaders/project_pinhole) on inputs that
# make its depth or pixel NaN, through recon's src/ and the GLSL grid_volume and
# sensor_frame export, as the tsdf kernels read them. Skips with no device.
add_executable(recon_tsdf_integrate_test tsdf_integrate_test.cpp)
target_link_libraries(
  recon_tsdf_integrate_test PRIVATE volumetric_kit::recon_tsdf Vulkan::Vulkan
                                    volumetric_kit::core_test_policy)
include(vr_embed)
vr_embed_shaders(
  recon_tsdf_integrate_test
  INCLUDE_TARGETS
  volumetric_kit::grid_volume
  volumetric_kit::sensor_frame
  WITH_SRC
  SHADERS
  "${CMAKE_CURRENT_SOURCE_DIR}/shaders/project_pinhole.comp")
vr_target_warnings(recon_tsdf_integrate_test)
add_test(NAME recon_tsdf_integrate COMMAND recon_tsdf_integrate_test)

# The sensor package's GpuFramePrep output, the frame contract, fused through
# the device-input overloads: a lensed depth camera beside a larger colour one,
# and a slanted plane's field crossing zero within a tenth of a voxel of the
# plane. The pass itself is the package's test (sensor_gpu_frame_prep). Skips
# where no device is present.
add_executable(recon_tsdf_prepared_frame_test tsdf_prepared_frame_test.cpp)
target_link_libraries(
  recon_tsdf_prepared_frame_test
  PRIVATE volumetric_kit::sensor_prep volumetric_kit::recon_tsdf
          volumetric_kit::core_test_policy)
vr_target_warnings(recon_tsdf_prepared_frame_test)
add_test(NAME recon_tsdf_prepared_frame COMMAND recon_tsdf_prepared_frame_test)

# The viewer's retained keyframe survives preparation and integration errors,
# and still supplies depth, coverage and atlas pixels for projective texturing;
# the helper fuses in the integration mode it is given, the one fusion.dynamic
# selects. Headless: uses the example's fusion helper, without gfx or a dataset.
add_executable(recon_example_fuse_keyframe_test example_fuse_keyframe_test.cpp)
target_include_directories(recon_example_fuse_keyframe_test
                           PRIVATE ${PROJECT_SOURCE_DIR}/examples/common)
target_link_libraries(
  recon_example_fuse_keyframe_test
  PRIVATE volumetric_kit::recon_sender volumetric_kit::recon_texture
          volumetric_kit::sensor_prep volumetric_kit::core_test_policy)
vr_target_warnings(recon_example_fuse_keyframe_test)
add_test(NAME recon_example_fuse_keyframe
         COMMAND recon_example_fuse_keyframe_test)

# The core parses and validates scenarios. This test pins the fusing examples'
# templates, their codec options through grid's codec_config.hpp among them, and
# their sender and sensor options, without opening a camera or GPU.
add_executable(recon_example_fuse_config_test example_fuse_config_test.cpp)
target_include_directories(recon_example_fuse_config_test
                           PRIVATE ${PROJECT_SOURCE_DIR}/examples/common)
if(TARGET volumetric_kit::sensor_orbbec)
  target_link_libraries(recon_example_fuse_config_test
                        PRIVATE volumetric_kit::sensor_orbbec)
  target_compile_definitions(recon_example_fuse_config_test
                             PRIVATE VR_TEST_ORBBEC_CONFIG=1)
endif()
if(TARGET volumetric_kit::sensor_realsense)
  # realsense_camera.hpp opens the camera as an array.
  target_link_libraries(
    recon_example_fuse_config_test PRIVATE volumetric_kit::sensor_realsense
                                           volumetric_kit::sensor_array)
  target_compile_definitions(recon_example_fuse_config_test
                             PRIVATE VR_TEST_REALSENSE_CONFIG=1)
  # And, where sensor builds FFS, its depth's options (VR_EXAMPLE_FFS, as
  # fuse_realsense is built).
  if(TARGET volumetric_kit::sensor_ffs)
    target_link_libraries(recon_example_fuse_config_test
                          PRIVATE volumetric_kit::sensor_ffs)
    target_compile_definitions(recon_example_fuse_config_test
                               PRIVATE VR_EXAMPLE_FFS=1)
  endif()
endif()
# fuse_sender.hpp reports through codec_report.hpp, which compares with eval's,
# fuses with the frame prep's frames, and codec_config.hpp maps a mesh source
# into voxelize's MeshSdfParams. recon_sender brings the codec stream: naming it
# too would repeat its archives on the link line.
target_link_libraries(
  recon_example_fuse_config_test
  PRIVATE grid_example_common volumetric_kit::core_config
          volumetric_kit::grid_eval volumetric_kit::grid_voxelize
          volumetric_kit::recon_sender volumetric_kit::sensor_prep)
target_compile_definitions(
  recon_example_fuse_config_test
  PRIVATE VR_CONFIG_FIXTURES="${PROJECT_SOURCE_DIR}/config/examples")
vr_target_warnings(recon_example_fuse_config_test)
add_test(NAME recon_example_fuse_config COMMAND recon_example_fuse_config_test)

# fuse_orbbec's cameras (examples/common/fuse_orbbec_sets.hpp), where the driver
# is built: the one camera a calibration file names, and a camera's report. Host
# only.
if(TARGET volumetric_kit::sensor_orbbec)
  add_executable(recon_example_fuse_orbbec_sets_test
                 example_fuse_orbbec_sets_test.cpp)
  target_include_directories(recon_example_fuse_orbbec_sets_test
                             PRIVATE ${PROJECT_SOURCE_DIR}/examples/common)
  target_link_libraries(
    recon_example_fuse_orbbec_sets_test
    PRIVATE grid_example_common volumetric_kit::sensor_orbbec
            volumetric_kit::sensor_array volumetric_kit::core_config)
  vr_target_warnings(recon_example_fuse_orbbec_sets_test)
  add_test(NAME recon_example_fuse_orbbec_sets
           COMMAND recon_example_fuse_orbbec_sets_test)
endif()

# The live examples' array report (examples/common/live_sets.hpp), and, where
# the RealSense driver is built, fuse_realsense's camera report
# (examples/common/realsense_camera.hpp). Host only: fake sensors.
add_executable(recon_example_live_sets_test example_live_sets_test.cpp)
target_include_directories(recon_example_live_sets_test
                           PRIVATE ${PROJECT_SOURCE_DIR}/examples/common)
target_link_libraries(recon_example_live_sets_test
                      PRIVATE volumetric_kit::sensor_array)
if(TARGET volumetric_kit::sensor_realsense)
  target_link_libraries(
    recon_example_live_sets_test
    PRIVATE grid_example_common volumetric_kit::core_config
            volumetric_kit::sensor_realsense)
  target_compile_definitions(recon_example_live_sets_test
                             PRIVATE VR_TEST_REALSENSE_CAMERA=1)
endif()
vr_target_warnings(recon_example_live_sets_test)
add_test(NAME recon_example_live_sets COMMAND recon_example_live_sets_test)

# The Replica example's sensor (examples/common/replica_sensor.hpp), where the
# examples are built: its intrinsics read through the core's read_file, a
# missing file NotFound. Host only. examples/ defines vr_example_common after
# this directory; the name resolves when the build is generated.
if(VR_BUILD_EXAMPLES)
  add_executable(recon_example_replica_sensor_test
                 example_replica_sensor_test.cpp)
  target_link_libraries(recon_example_replica_sensor_test
                        PRIVATE vr_example_common)
  vr_target_warnings(recon_example_replica_sensor_test)
  add_test(NAME recon_example_replica_sensor
           COMMAND recon_example_replica_sensor_test)
endif()

# fuse_orbbec's dump of its first sets (examples/common/set_dump.hpp): the JPEGs
# byte for byte, the luma as R = G = B PNGs, and the refusals. Host only.
add_executable(recon_example_set_dump_test example_set_dump_test.cpp)
target_include_directories(recon_example_set_dump_test
                           PRIVATE ${PROJECT_SOURCE_DIR}/examples/common)
target_link_libraries(
  recon_example_set_dump_test PRIVATE volumetric_kit::sensor_array
                                      volumetric_kit::sensor_image_io)
vr_target_warnings(recon_example_set_dump_test)
add_test(NAME recon_example_set_dump COMMAND recon_example_set_dump_test)

# A sender's per-set log (examples/common/set_log.hpp): counters across each
# set, P-frame, I-frame and uncoded sets apart, nearest-rank percentiles over
# the sets that ran a row, n/t, n/a and dash cells, latencies and redo and tail
# counts; a viewer's remesh rows in their set, its sets by kind, the newest
# kept. Host only.
add_executable(recon_example_set_log_test example_set_log_test.cpp)
target_include_directories(recon_example_set_log_test
                           PRIVATE ${PROJECT_SOURCE_DIR}/examples/common)
target_link_libraries(recon_example_set_log_test
                      PRIVATE grid_example_common volumetric_kit::grid_codec)
vr_target_warnings(recon_example_set_log_test)
add_test(NAME recon_example_set_log COMMAND recon_example_set_log_test)

# A sender's set into the run's figures (examples/common/fuse_sender.hpp): a set
# stamped from what the set step returned, its frame's row kept. Host only.
add_executable(recon_example_fuse_sender_test example_fuse_sender_test.cpp)
target_include_directories(recon_example_fuse_sender_test
                           PRIVATE ${PROJECT_SOURCE_DIR}/examples/common)
target_link_libraries(
  recon_example_fuse_sender_test
  PRIVATE grid_example_common volumetric_kit::core_config
          volumetric_kit::grid_eval volumetric_kit::recon_sender
          volumetric_kit::sensor_prep)
vr_target_warnings(recon_example_fuse_sender_test)
add_test(NAME recon_example_fuse_sender COMMAND recon_example_fuse_sender_test)

# fuse_replica's --dirty-every survey (examples/common/remesh_report.hpp): the
# changed blocks after a tick, the blocks they put back to marching cubes, and
# the survey's windows. Skips (exit 0) with no device.
add_executable(recon_example_remesh_report_test example_remesh_report_test.cpp)
target_include_directories(recon_example_remesh_report_test
                           PRIVATE ${PROJECT_SOURCE_DIR}/examples/common)
target_link_libraries(
  recon_example_remesh_report_test
  PRIVATE volumetric_kit::core_test_policy volumetric_kit::recon_core
          volumetric_kit::grid_mesh)
vr_target_warnings(recon_example_remesh_report_test)
add_test(NAME recon_example_remesh_report
         COMMAND recon_example_remesh_report_test)

# tsdf fuser: a set fused through tsdf::Fuser, growing on the way, matches the
# tier calls on a grid that never needed to; it grows ahead before any failure,
# refuses incomplete bands by default, and keeps fusing only when the caller
# explicitly allows partial coverage, whose one-submit sets match the tier calls
# too and grow a set late. Skips (exit 0) with no device.
add_executable(recon_tsdf_fuser_test tsdf_fuser_test.cpp)
target_link_libraries(
  recon_tsdf_fuser_test PRIVATE volumetric_kit::core_test_policy
                                volumetric_kit::recon_tsdf Vulkan::Vulkan)
vr_target_warnings(recon_tsdf_fuser_test)
add_test(NAME recon_tsdf_fuser COMMAND recon_tsdf_fuser_test)

# Host OOM stays in Status, resize rolls back, and private live backoff expires.
add_executable(recon_tsdf_fuser_oom_test tsdf_fuser_oom_test.cpp)
target_link_libraries(
  recon_tsdf_fuser_oom_test
  PRIVATE volumetric_kit::core_test_policy volumetric_kit::recon_tsdf
          Vulkan::Vulkan volumetric_kit::core_test_allocation_failure)
vr_target_warnings(recon_tsdf_fuser_oom_test)
add_test(NAME recon_tsdf_fuser_oom COMMAND recon_tsdf_fuser_oom_test)

# tsdf fusion grid: the start grows for a finer grid, capped to fit the device's
# maxStorageBufferRange (lavapipe's 128 MiB and a 4 GiB range, and a capped
# caller's warning, on the host); on a device, the grid's layout and attributes
# and the cap against its own ceiling. Skips the device half with no device.
add_executable(recon_tsdf_fusion_grid_test tsdf_fusion_grid_test.cpp)
target_link_libraries(
  recon_tsdf_fusion_grid_test PRIVATE volumetric_kit::core_test_policy
                                      volumetric_kit::recon_tsdf)
vr_target_warnings(recon_tsdf_fusion_grid_test)
add_test(NAME recon_tsdf_fusion_grid COMMAND recon_tsdf_fusion_grid_test)

# The device-input overloads of allocate_from_depth and integrate: the same
# frame fused from host arrays and from storage buffers must give the same grid.
# Skips where no device is present.
add_executable(recon_tsdf_device_input_test tsdf_device_input_test.cpp)
target_link_libraries(
  recon_tsdf_device_input_test
  PRIVATE volumetric_kit::recon_tsdf Vulkan::Vulkan
          volumetric_kit::core_test_policy)
vr_target_warnings(recon_tsdf_device_input_test)
add_test(NAME recon_tsdf_device_input COMMAND recon_tsdf_device_input_test)

# Borrowed gates are checked before recording or advancing the fusion tick.
add_executable(recon_tsdf_recorded_integrate_test
               tsdf_recorded_integrate_test.cpp)
target_link_libraries(
  recon_tsdf_recorded_integrate_test PRIVATE volumetric_kit::recon_tsdf
                                             volumetric_kit::core_test_policy)
vr_target_warnings(recon_tsdf_recorded_integrate_test)
add_test(NAME recon_tsdf_recorded_integrate
         COMMAND recon_tsdf_recorded_integrate_test)

# tsdf integrate set: several cameras allocated and fused in one call each must
# give, under classic, the grid fusing them one at a time gives, bit for bit,
# and under dynamic the grid the same set in the other order gives, each camera
# keeping its slot, bit for bit too. Skips where no device is present.
add_executable(recon_tsdf_integrate_set_test tsdf_integrate_set_test.cpp)
target_link_libraries(
  recon_tsdf_integrate_set_test
  PRIVATE volumetric_kit::recon_tsdf Vulkan::Vulkan
          volumetric_kit::core_test_policy)
vr_target_warnings(recon_tsdf_integrate_set_test)
add_test(NAME recon_tsdf_integrate_set COMMAND recon_tsdf_integrate_set_test)

# tsdf change detector: synthetic depth sequences, a camera slot each -- static
# ToF-like and stereo-like noise, steps, ramps, a foreground sweeping in,
# dropouts, a camera that leaves and returns, a resized one -- against the rates
# and delays measured on the rig, and the refusals. Skips (exit 0) where no
# device is present.
add_executable(recon_tsdf_change_detector_test tsdf_change_detector_test.cpp)
target_link_libraries(
  recon_tsdf_change_detector_test PRIVATE volumetric_kit::core_test_policy
                                          volumetric_kit::recon_tsdf)
vr_target_warnings(recon_tsdf_change_detector_test)
add_test(NAME recon_tsdf_change_detector
         COMMAND recon_tsdf_change_detector_test)

# tsdf dynamic change: two fixed cameras that disagree by 8 mm on a wall, a card
# inside the wall's band that leaves, a box that arrives, stays and leaves, a
# set one camera misses, through the Fuser under Dynamic: no weight falls while
# the scene is still, each change clears in the set it is seen, and the hidden
# wall keeps its history. Skips (exit 0) where no device is present.
add_executable(recon_tsdf_dynamic_change_test tsdf_dynamic_change_test.cpp)
target_link_libraries(
  recon_tsdf_dynamic_change_test
  PRIVATE volumetric_kit::core_test_policy volumetric_kit::recon_tsdf
          volumetric_kit::grid_mesh)
vr_target_warnings(recon_tsdf_dynamic_change_test)
add_test(NAME recon_tsdf_dynamic_change COMMAND recon_tsdf_dynamic_change_test)

# tsdf rim distance: one camera's plate outline at its true edge in either mode,
# the voxels beside it at their Euclidean distance, a thin rod solid; a plate
# moving past one Dynamic camera with nothing beside its moving outline; and a
# sphere moving past three Dynamic cameras covered in every set, nothing of it
# left once gone. Skips (exit 0) where no device is present.
add_executable(recon_tsdf_rim_distance_test tsdf_rim_distance_test.cpp)
target_link_libraries(
  recon_tsdf_rim_distance_test
  PRIVATE volumetric_kit::core_test_policy volumetric_kit::recon_tsdf
          volumetric_kit::grid_mesh)
vr_target_warnings(recon_tsdf_rim_distance_test)
add_test(NAME recon_tsdf_rim_distance COMMAND recon_tsdf_rim_distance_test)

# tsdf integrate dynamic set: a frontal camera and a grazing one that sees past
# its band, fused as a Dynamic set in either order, must give the same grid and
# mesh and keep the frontal camera's surface whole; the frontal wall receding is
# reset in the set it is seen. Skips (exit 0) where no device is present.
add_executable(recon_tsdf_integrate_dynamic_set_test
               tsdf_integrate_dynamic_set_test.cpp)
target_link_libraries(
  recon_tsdf_integrate_dynamic_set_test
  PRIVATE volumetric_kit::core_test_policy volumetric_kit::recon_tsdf
          volumetric_kit::grid_mesh Vulkan::Vulkan)
vr_target_warnings(recon_tsdf_integrate_dynamic_set_test)
add_test(NAME recon_tsdf_integrate_dynamic_set
         COMMAND recon_tsdf_integrate_dynamic_set_test)

# tsdf integrate cull: fusing only the blocks the frames' frusta reach must give
# the grid fusing every active block gives, bit for bit, at each edge of that
# reach. Skips where no device is present.
add_executable(recon_tsdf_integrate_cull_test tsdf_integrate_cull_test.cpp)
target_link_libraries(
  recon_tsdf_integrate_cull_test
  PRIVATE volumetric_kit::recon_tsdf Vulkan::Vulkan
          volumetric_kit::core_test_policy)
vr_target_warnings(recon_tsdf_integrate_cull_test)
add_test(NAME recon_tsdf_integrate_cull COMMAND recon_tsdf_integrate_cull_test)

# texture projective: texture a mesh of three triangles (one the camera sees
# head-on, one occluded by a closer depth reading, one behind the camera) and
# verify the visible triangle gets the exact projected atlas UVs while the
# others keep the (-1,-1) sentinel; a closer depth map reverts the textured
# triangle, and a translated pose shifts the projection (the world->camera
# transform). Skips where no device is present.
add_executable(recon_texture_projective_test texture_projective_test.cpp)
target_link_libraries(
  recon_texture_projective_test
  PRIVATE volumetric_kit::recon_texture Vulkan::Vulkan
          volumetric_kit::core_test_policy)
vr_target_warnings(recon_texture_projective_test)
add_test(NAME recon_texture_projective COMMAND recon_texture_projective_test)

# Several views into an atlas, on a synthetic scene: each triangle takes the
# view facing it most squarely among those that see it whole, inside that view's
# tile; the atlas layout and packing; the refusals. Skips where no device is
# present.
add_executable(recon_texture_multiview_test texture_multiview_test.cpp)
target_link_libraries(
  recon_texture_multiview_test
  PRIVATE volumetric_kit::recon_texture Vulkan::Vulkan
          volumetric_kit::core_test_policy)
vr_target_warnings(recon_texture_multiview_test)
add_test(NAME recon_texture_multiview COMMAND recon_texture_multiview_test)

# mesh + texture composed: the device-resident handoff (extract_device ->
# texture -> download) must equal the host path vertex for vertex. Links both
# tiers, since it is the seam between them that is under test. Skips where no
# device is present.
add_executable(recon_texture_device_mesh_test texture_device_mesh_test.cpp)
target_link_libraries(
  recon_texture_device_mesh_test
  PRIVATE volumetric_kit::recon_texture volumetric_kit::grid_mesh
          Vulkan::Vulkan volumetric_kit::core_test_policy)
vr_target_warnings(recon_texture_device_mesh_test)
add_test(NAME recon_texture_device_mesh COMMAND recon_texture_device_mesh_test)

# config/femto_mega_sync.json, the lab rig's sync file the Orbbec examples'
# configurations name, where the driver is built: the sensor package's reader
# takes it, with one primary and every secondary's depth delay staggered. No
# camera.
if(TARGET volumetric_kit::sensor_orbbec)
  add_executable(recon_example_rig_sync_config_test
                 example_rig_sync_config_test.cpp)
  target_link_libraries(recon_example_rig_sync_config_test
                        PRIVATE volumetric_kit::sensor_orbbec)
  target_compile_definitions(
    recon_example_rig_sync_config_test
    PRIVATE
      VR_RIG_SYNC_CONFIG="${PROJECT_SOURCE_DIR}/config/femto_mega_sync.json")
  vr_target_warnings(recon_example_rig_sync_config_test)
  add_test(NAME recon_example_rig_sync_config
           COMMAND recon_example_rig_sync_config_test)
endif()

if(TARGET grid_mesh_io_assimp)
  # grid's mesh_io::load_textured_mesh on a procedural OBJ + MTL + PNG cube it
  # writes, the PNG with the sensor package's image I/O; built with the Assimp
  # backend.
  add_executable(recon_io_textured_mesh_test io_textured_mesh_test.cpp)
  target_link_libraries(
    recon_io_textured_mesh_test PRIVATE volumetric_kit::grid_mesh_io_assimp
                                        volumetric_kit::sensor_image_io)
  vr_target_warnings(recon_io_textured_mesh_test)
  add_test(NAME recon_io_textured_mesh COMMAND recon_io_textured_mesh_test)
endif()

# sender tier: the set step (sender/set_sender.hpp) with no camera driver: fake
# cameras read as a sensor array, each set's prep, fusion, free pass and frame
# recorded by the sender and coded into a .vrst whose records, keys, sets and
# pts are the sets', every frame decoded, the blocking calls' frames. A wall the
# cameras later see past: under Dynamic gone from the frames that see past it,
# and its blocks freed by the free pass due; under Classic its observed blocks
# kept. A coded set is 2 submits and at most 2 waits, a fused one 1 and 1.
# Served with keys on demand, a frame is a key when a viewer joined before it,
# once however many, and otherwise at the intra period. With no frames to fuse,
# a sender's frames are the file CodecStream::code writes. The device part skips
# with no device.
add_executable(recon_sender_set_sender_test sender_set_sender_test.cpp)
target_link_libraries(
  recon_sender_set_sender_test
  PRIVATE volumetric_kit::core_test_policy volumetric_kit::recon_sender
          volumetric_kit::grid_mesh volumetric_kit::sensor_array Vulkan::Vulkan)
vr_target_warnings(recon_sender_set_sender_test)
add_test(NAME recon_sender_set_sender COMMAND recon_sender_set_sender_test)

# What a sibling that fetches recon gets (subproject/): its configure, from a
# clean tree, with each dependency this build fetched as a local source, so it
# needs no network. The defaults hold, the sibling's vkc_require_sibling accepts
# this recon and refuses it under a floor above its version, and recon refuses a
# core the sibling made available first that is older than VR_CORE_MIN_VERSION
# or lacks the vulkan tier. The old core is this one stamped 0.0.1, standing in
# for one below a later floor: a real core older than 0.1.0 has no
# vkc_require_core at all.
#
# TODO: move this core-requirement harness -- the message helper and
# subproject/'s version stamp, which grid, gfx and calib repeat -- into the
# core's test support, as a function each sibling calls with its package and
# floor, and port the siblings onto it in one series.
set(_vr_subproject "${CMAKE_CURRENT_BINARY_DIR}/subproject")
add_test(NAME recon_subproject_clean COMMAND "${CMAKE_COMMAND}" -E rm -rf
                                             "${_vr_subproject}")
set_tests_properties(recon_subproject_clean PROPERTIES FIXTURES_SETUP
                                                       vr_subproject)
# Configures subproject/ into <dir> under _vr_subproject with the given -D args.
function(_vr_add_subproject_test name dir)
  add_test(
    NAME ${name}
    COMMAND
      "${CMAKE_COMMAND}" -S "${CMAKE_CURRENT_SOURCE_DIR}/subproject" -B
      "${_vr_subproject}/${dir}" -G "${CMAKE_GENERATOR}"
      "-DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}"
      "-DVR_SOURCE_DIR=${PROJECT_SOURCE_DIR}" ${_vr_fetched_sources} ${ARGN})
  set_tests_properties(${name} PROPERTIES FIXTURES_REQUIRED vr_subproject)
endfunction()
# Sets <test>'s PASS_REGULAR_EXPRESSION to <message>, a space in which matches
# however CMake wrapped the error's lines.
function(_vr_expect_message test message)
  string(REPLACE " " "[ \n]+" message "${message}")
  set_tests_properties(${test} PROPERTIES PASS_REGULAR_EXPRESSION "${message}")
endfunction()

_vr_add_subproject_test(recon_subproject_defaults defaults
                        "-DVR_REQUIRE_RECON=${PROJECT_VERSION}")
math(EXPR _vr_next_minor "${PROJECT_VERSION_MINOR} + 1")
set(_vr_newer "${PROJECT_VERSION_MAJOR}.${_vr_next_minor}.0")
_vr_add_subproject_test(recon_subproject_old_recon old_recon
                        "-DVR_REQUIRE_RECON=${_vr_newer}")
_vr_expect_message(
  recon_subproject_old_recon
  "vr_subproject_consumer needs volumetric_kit_recon ${_vr_newer} or newer, but the recon at .+ is ${PROJECT_VERSION}: pin v${_vr_newer} or newer"
)
_vr_add_subproject_test(recon_subproject_old_core old_core -DVR_CORE_FIRST=ON
                        -DVKC_WITH_VULKAN=ON -DVR_CONSUMER_CORE_VERSION=0.0.1)
_vr_expect_message(
  recon_subproject_old_core
  "volumetric_kit_recon needs volumetric_kit_core ${VR_CORE_MIN_VERSION} or newer, but the core at .+ is 0.0.1: pin v${VR_CORE_MIN_VERSION} or newer"
)
_vr_add_subproject_test(recon_subproject_core_without_vulkan no_vulkan
                        -DVR_CORE_FIRST=ON -DVKC_WITH_VULKAN=OFF)
_vr_expect_message(
  recon_subproject_core_without_vulkan
  "volumetric_kit_recon needs volumetric_kit_core's vulkan tier, which the core at .+ was built without: set VKC_WITH_VULKAN ON before"
)

# Start the slowest tests first under `ctest --parallel`. Without a COST, CTest
# runs tests in the order they were added, and a fresh CI runner has no timing
# history, so a slow test added late finished last. Seconds under the sanitizers
# job, the longest in CI, in run 39717939263 (before the 2026-10-22 test trims);
# only the order matters.
set_tests_properties(recon_sender_set_sender PROPERTIES COST 29)

# A test skips by exiting with the core's test::kSkipExitCode (gpu_test.hpp). A
# hang (a fence never signalled, a deadlocked thread) fails in 300 s rather than
# holding a shared runner to its job's limit; 300 is well above the largest
# COST.
get_property(
  _vr_tests
  DIRECTORY
  PROPERTY TESTS)
set_tests_properties(${_vr_tests} PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 300)
