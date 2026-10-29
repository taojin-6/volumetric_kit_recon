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
  PRIVATE volumetric_kit::recon_texture volumetric_kit::recon_mesh
          volumetric_kit::recon_tsdf volumetric_kit::recon_volume
          volumetric_kit::recon_core volumetric_kit::core_test_policy)
vr_target_warnings(recon_tier_stage_metrics_test)
target_link_libraries(recon_tier_stage_metrics_test PRIVATE ${CMAKE_DL_LIBS})
add_test(NAME recon_tier_stage_metrics COMMAND recon_tier_stage_metrics_test)

# volume coordinate math: pure host transforms (world <-> voxel <-> block,
# negative-coordinate block indexing, truncation band), the params' validation
# and the block layout (block_grid_params); and the tests' host oracle of the
# spatial hash (volume_hash_oracle.hpp). CPU-only (no device), so these always
# run -- not gated on a Vulkan driver.
add_executable(recon_volume_coords_test volume_coords_test.cpp)
target_link_libraries(
  recon_volume_coords_test PRIVATE volumetric_kit::recon_volume
                                   volumetric_kit::core_test_allocation_failure)
vr_target_warnings(recon_volume_coords_test)
add_test(NAME recon_volume_coords COMMAND recon_volume_coords_test)

add_executable(recon_volume_hash_test volume_hash_test.cpp)
target_link_libraries(recon_volume_hash_test
                      PRIVATE volumetric_kit::recon_volume)
vr_target_warnings(recon_volume_hash_test)
add_test(NAME recon_volume_hash COMMAND recon_volume_hash_test)

# The volume tier is camera-free: no header, source or shader of it includes the
# camera vocabulary, sensor/frame/camera_params.hpp or
# sensor/frame/shaders/pinhole.glsl (volume_camera_free.cmake).
add_test(NAME recon_volume_camera_free
         COMMAND "${CMAKE_COMMAND}" "-DSOURCE_DIR=${PROJECT_SOURCE_DIR}" -P
                 "${CMAKE_CURRENT_SOURCE_DIR}/volume_camera_free.cmake")

# Volume's depth unprojection (volume/shaders/depth_pinhole.glsl) on the device
# against sensor/frame/shaders/pinhole.glsl's, bit for bit, and the host's
# pinhole in double (shaders/depth_unproject_cases.comp). Skips where no device
# is present.
add_executable(recon_volume_depth_unproject_test
               volume_depth_unproject_test.cpp)
vr_embed_shaders(
  recon_volume_depth_unproject_test EXPORTED_FROM volumetric_kit::sensor_frame
  SHADERS shaders/depth_unproject_cases.comp)
target_link_libraries(
  recon_volume_depth_unproject_test
  PRIVATE volumetric_kit::recon_core volumetric_kit::sensor_frame
          Vulkan::Vulkan volumetric_kit::core_test_policy)
vr_target_warnings(recon_volume_depth_unproject_test)
add_test(NAME recon_volume_depth_unproject
         COMMAND recon_volume_depth_unproject_test)

# The grid's exported GLSL, built through INCLUDE_TARGETS
# volumetric_kit::recon_volume as a consumer's kernels are, against volume's
# host types on the device: grid_types.glsl's tick comparison, in-block index
# and struct layouts (shaders/grid_types_cases.comp), and voxel_color.glsl's
# colour format, decode, running-mean encode, first observation and the mesher's
# read (shaders/voxel_color_cases.comp). The colour test also holds
# volume/voxel_color.hpp to the published sRGB curve, its own reference, on the
# host, bit for bit; recon_tsdf_voxel_color_space holds it to
# sensor/frame/color_space.hpp. Skip the device half where no device is present.
add_executable(recon_volume_grid_types_test volume_grid_types_test.cpp)
vr_embed_shaders(
  recon_volume_grid_types_test INCLUDE_TARGETS volumetric_kit::recon_volume
  SHADERS "${CMAKE_CURRENT_SOURCE_DIR}/shaders/grid_types_cases.comp")
target_link_libraries(
  recon_volume_grid_types_test
  PRIVATE volumetric_kit::recon_volume Vulkan::Vulkan
          volumetric_kit::core_test_policy)
vr_target_warnings(recon_volume_grid_types_test)
add_test(NAME recon_volume_grid_types COMMAND recon_volume_grid_types_test)

add_executable(recon_volume_voxel_color_test volume_voxel_color_test.cpp)
vr_embed_shaders(
  recon_volume_voxel_color_test INCLUDE_TARGETS volumetric_kit::recon_volume
  SHADERS "${CMAKE_CURRENT_SOURCE_DIR}/shaders/voxel_color_cases.comp")
target_link_libraries(
  recon_volume_voxel_color_test
  PRIVATE volumetric_kit::recon_volume Vulkan::Vulkan
          volumetric_kit::core_test_policy)
vr_target_warnings(recon_volume_voxel_color_test)
add_test(NAME recon_volume_voxel_color COMMAND recon_volume_voxel_color_test)

# volume GPU hash map: build a VoxelHashMap, allocate a cube of block coords,
# compact, and verify the active set round-trips on the real driver (MoltenVK /
# NVIDIA). The table's entry layout is internal (hash.hpp), reached by include
# path. Skips where no device is present.
add_executable(recon_volume_hash_map_test volume_hash_map_test.cpp)
target_link_libraries(
  recon_volume_hash_map_test
  PRIVATE volumetric_kit::recon_volume Vulkan::Vulkan
          volumetric_kit::core_test_policy)
target_include_directories(
  recon_volume_hash_map_test
  PRIVATE ${PROJECT_SOURCE_DIR}/src/volumetric_kit/recon/volume)
vr_target_warnings(recon_volume_hash_map_test)
add_test(NAME recon_volume_hash_map COMMAND recon_volume_hash_map_test)

# volume block grid: declare independent SoA per-voxel attributes, verify they
# are distinct + correctly-sized buffers keyed by the block pool, the composed
# map allocates, and the error / move paths. Skips with no device.
add_executable(recon_volume_block_grid_test volume_block_grid_test.cpp)
target_link_libraries(
  recon_volume_block_grid_test
  PRIVATE volumetric_kit::recon_volume Vulkan::Vulkan
          volumetric_kit::core_test_policy)
vr_target_warnings(recon_volume_block_grid_test)
add_test(NAME recon_volume_block_grid COMMAND recon_volume_block_grid_test)

# volume block stamps: every allocation path stamps the blocks it asks for with
# the map's tick, the block pass stamps those holding weight, and
# free_stale_blocks frees those neither for more than max_age ticks; resize
# keeps the records, remove and clear zero them; a pass split into several
# dispatches (core/dispatch.hpp's test seam, by src/). Skips with no device.
add_executable(recon_volume_block_stamps_test volume_block_stamps_test.cpp)
target_link_libraries(
  recon_volume_block_stamps_test
  PRIVATE volumetric_kit::recon_volume Vulkan::Vulkan
          volumetric_kit::core_test_policy)
target_include_directories(recon_volume_block_stamps_test
                           PRIVATE ${PROJECT_SOURCE_DIR}/src)
vr_target_warnings(recon_volume_block_stamps_test)
add_test(NAME recon_volume_block_stamps COMMAND recon_volume_block_stamps_test)

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
# + cap), and a split dispatch (core/dispatch.hpp's test seam, by src/). It also
# runs the kernel's projection alone (shaders/project_pinhole) on inputs that
# make its depth or pixel NaN. Skips with no device.
add_executable(recon_tsdf_integrate_test tsdf_integrate_test.cpp)
target_link_libraries(
  recon_tsdf_integrate_test PRIVATE volumetric_kit::recon_tsdf Vulkan::Vulkan
                                    volumetric_kit::core_test_policy)
target_include_directories(recon_tsdf_integrate_test
                           PRIVATE ${PROJECT_SOURCE_DIR}/src)
include(vr_embed)
vr_embed_shaders(
  recon_tsdf_integrate_test EXPORTED_FROM volumetric_kit::sensor_frame SHADERS
  "${CMAKE_CURRENT_SOURCE_DIR}/shaders/project_pinhole.comp")
vr_target_warnings(recon_tsdf_integrate_test)
add_test(NAME recon_tsdf_integrate COMMAND recon_tsdf_integrate_test)

# voxelize mesh integrate: write a mesh's truncated distance field (signed
# through pseudonormals, or as a shell) and check every voxel of every written
# block against a brute-force host reference -- an independent closest point,
# and the winding number for inside/outside -- on a sharp tetrahedron, a dented
# cube, an open quad and a triangle soup, plus determinism (a split binning
# among it) and every refusal. Skips with no device.
add_executable(recon_voxelize_mesh_integrate_test
               voxelize_mesh_integrate_test.cpp)
target_link_libraries(
  recon_voxelize_mesh_integrate_test
  PRIVATE volumetric_kit::recon_voxelize Vulkan::Vulkan
          volumetric_kit::core_test_policy)
target_include_directories(recon_voxelize_mesh_integrate_test
                           PRIVATE ${PROJECT_SOURCE_DIR}/src)
vr_target_warnings(recon_voxelize_mesh_integrate_test)
add_test(NAME recon_voxelize_mesh_integrate
         COMMAND recon_voxelize_mesh_integrate_test)

# voxelize mesh integrate, exactly: the kernel's early exit writes, bit for bit,
# what a brute force (shaders/mesh_integrate_reference.comp) writes measuring
# every triangle of each voxel's bin -- on shared vertices with opposite
# windings, duplicates, near-coincident sheets, slivers, metre-long triangles, a
# far sphere, bins past the kernel's sort, three block sizes, both modes. The
# brute force binds the hash entries (volume/hash_map_internal.hpp, by src/).
# Skips with no device.
add_executable(recon_voxelize_mesh_integrate_exact_test
               voxelize_mesh_integrate_exact_test.cpp)
target_link_libraries(
  recon_voxelize_mesh_integrate_exact_test
  PRIVATE volumetric_kit::recon_voxelize Vulkan::Vulkan
          volumetric_kit::core_test_policy)
target_include_directories(recon_voxelize_mesh_integrate_exact_test
                           PRIVATE ${PROJECT_SOURCE_DIR}/src)
vr_embed_shaders(
  recon_voxelize_mesh_integrate_exact_test SHADERS
  "${CMAKE_CURRENT_SOURCE_DIR}/shaders/mesh_integrate_reference.comp")
vr_target_warnings(recon_voxelize_mesh_integrate_exact_test)
add_test(NAME recon_voxelize_mesh_integrate_exact
         COMMAND recon_voxelize_mesh_integrate_exact_test)
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

# The core parses and validates scenarios. These tests pin the conversion into
# the runtime codec options (the codec's steps checked here, not by core), and
# recon_example_fuse_config the fusing examples' sender and sensor options,
# without opening a camera or GPU.
add_executable(recon_example_config_test example_config_test.cpp)
target_include_directories(recon_example_config_test
                           PRIVATE ${PROJECT_SOURCE_DIR}/examples/common)
# codec_config.hpp maps the outputs into the codec stream's CodecOutputs and a
# mesh source into voxelize's MeshSdfParams, and the sweep's report compares
# with eval's. recon_sender brings the codec stream and the codec tier: naming
# them too would repeat their archives on the link line.
target_link_libraries(
  recon_example_config_test
  PRIVATE volumetric_kit::core_config volumetric_kit::recon_eval
          volumetric_kit::recon_sender volumetric_kit::recon_voxelize)
target_compile_definitions(
  recon_example_config_test
  PRIVATE VR_CONFIG_FIXTURES="${PROJECT_SOURCE_DIR}/config/examples")
vr_target_warnings(recon_example_config_test)
add_test(NAME recon_example_config COMMAND recon_example_config_test)

add_executable(recon_example_fuse_config_test example_fuse_config_test.cpp)
target_include_directories(recon_example_fuse_config_test
                           PRIVATE ${PROJECT_SOURCE_DIR}/examples/common)
if(TARGET volumetric_kit::sensor_orbbec)
  target_link_libraries(recon_example_fuse_config_test
                        PRIVATE volumetric_kit::sensor_orbbec)
  target_compile_definitions(recon_example_fuse_config_test
                             PRIVATE VR_TEST_ORBBEC_CONFIG=1)
endif()
# fuse_sender.hpp reports through codec_report.hpp, which compares with eval's,
# and fuses with the frame prep's frames. recon_sender brings the codec tiers:
# naming them too would repeat their archives on the link line.
target_link_libraries(
  recon_example_fuse_config_test
  PRIVATE volumetric_kit::core_config volumetric_kit::recon_eval
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
    PRIVATE volumetric_kit::sensor_orbbec volumetric_kit::sensor_array
            volumetric_kit::core_config)
  vr_target_warnings(recon_example_fuse_orbbec_sets_test)
  add_test(NAME recon_example_fuse_orbbec_sets
           COMMAND recon_example_fuse_orbbec_sets_test)
endif()

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

# The live examples' Ctrl+C (examples/common/interrupt.hpp): a raised SIGINT
# sets the flag and restores its default handler, SIGTERM's still caught. Host
# only.
add_executable(recon_example_interrupt_test example_interrupt_test.cpp)
target_include_directories(recon_example_interrupt_test
                           PRIVATE ${PROJECT_SOURCE_DIR}/examples/common)
vr_target_warnings(recon_example_interrupt_test)
add_test(NAME recon_example_interrupt COMMAND recon_example_interrupt_test)

# The senders' log (examples/common/info_log.hpp): Info, Warning and Error in
# the default sink's line form, Debug dropped, and concurrent lines whole. Host
# only.
add_executable(recon_example_info_log_test example_info_log_test.cpp)
target_include_directories(recon_example_info_log_test
                           PRIVATE ${PROJECT_SOURCE_DIR}/examples/common)
target_link_libraries(recon_example_info_log_test
                      PRIVATE volumetric_kit::core_base Threads::Threads)
vr_target_warnings(recon_example_info_log_test)
add_test(NAME recon_example_info_log COMMAND recon_example_info_log_test)

# The examples' one stage table (examples/common/stage_table.hpp). Host only.
add_executable(recon_example_stage_table_test example_stage_table_test.cpp)
target_include_directories(recon_example_stage_table_test
                           PRIVATE ${PROJECT_SOURCE_DIR}/examples/common)
target_link_libraries(recon_example_stage_table_test
                      PRIVATE volumetric_kit::recon_core)
vr_target_warnings(recon_example_stage_table_test)
add_test(NAME recon_example_stage_table COMMAND recon_example_stage_table_test)

# A sender's per-set log (examples/common/set_log.hpp): counters across each
# set, P-frame, I-frame and uncoded sets apart, nearest-rank percentiles over
# the sets that ran a row, n/t, n/a and dash cells, latencies and redo and tail
# counts; a viewer's remesh rows in their set, its sets by kind, the newest
# kept. Host only.
add_executable(recon_example_set_log_test example_set_log_test.cpp)
target_include_directories(recon_example_set_log_test
                           PRIVATE ${PROJECT_SOURCE_DIR}/examples/common)
target_link_libraries(recon_example_set_log_test
                      PRIVATE volumetric_kit::recon_codec)
vr_target_warnings(recon_example_set_log_test)
add_test(NAME recon_example_set_log COMMAND recon_example_set_log_test)

# The codec examples' report of a stream (examples/common/codec_report.hpp): on
# the host, a row's type and host and device milliseconds, its CSV line, a frame
# type's sizes, a GOP's bytes and bitrate, a frame's pts, the comparison options
# the codec examples share, and a path that is the stream's or the CSV's file
# refused; on a device, a CSV refused after the stream is made or because it is
# the stream's file, a row per frame matching the frame and each type's stage
# rows its own, the CSV replaced on create and written as rows come, and a CSV
# that stops taking rows refused. The device part skips with no device.
add_executable(recon_example_codec_report_test example_codec_report_test.cpp)
target_include_directories(recon_example_codec_report_test
                           PRIVATE ${PROJECT_SOURCE_DIR}/examples/common)
target_link_libraries(
  recon_example_codec_report_test
  PRIVATE volumetric_kit::core_config volumetric_kit::core_test_policy
          volumetric_kit::recon_eval volumetric_kit::recon_sender
          volumetric_kit::recon_voxelize Vulkan::Vulkan)
vr_target_warnings(recon_example_codec_report_test)
add_test(NAME recon_example_codec_report
         COMMAND recon_example_codec_report_test)

# A sender's set into the run's figures (examples/common/fuse_sender.hpp): a set
# stamped from what the set step returned, its frame's row kept. Host only.
add_executable(recon_example_fuse_sender_test example_fuse_sender_test.cpp)
target_include_directories(recon_example_fuse_sender_test
                           PRIVATE ${PROJECT_SOURCE_DIR}/examples/common)
target_link_libraries(
  recon_example_fuse_sender_test
  PRIVATE volumetric_kit::core_config volumetric_kit::recon_eval
          volumetric_kit::recon_sender volumetric_kit::sensor_prep)
vr_target_warnings(recon_example_fuse_sender_test)
add_test(NAME recon_example_fuse_sender COMMAND recon_example_fuse_sender_test)

# codec_mesh's sequence (examples/codec_mesh/mesh_sequence.hpp), where the mesh
# example is built. On the host: the frame-number pattern, every file checked
# before any load, one transform for the whole range, signed mode's rule and
# source.allow_holes, and inspection. On a device: each frame converted into a
# cleared grid, and code_frame's records and the transform in the stream.
# examples/ defines vr_example_mesh after this directory. The device part skips
# with no device.
if(VR_BUILD_EXAMPLES AND TARGET recon_mesh_io_assimp)
  add_executable(recon_example_mesh_sequence_test
                 example_mesh_sequence_test.cpp)
  target_link_libraries(
    recon_example_mesh_sequence_test
    PRIVATE vr_example_mesh volumetric_kit::core_test_policy Vulkan::Vulkan)
  vr_target_warnings(recon_example_mesh_sequence_test)
  add_test(NAME recon_example_mesh_sequence
           COMMAND recon_example_mesh_sequence_test)
endif()

# fuse_replica's --dirty-every survey (examples/common/remesh_report.hpp): the
# changed blocks after a tick, the blocks they put back to marching cubes, and
# the survey's windows. Skips (exit 0) with no device.
add_executable(recon_example_remesh_report_test example_remesh_report_test.cpp)
target_include_directories(recon_example_remesh_report_test
                           PRIVATE ${PROJECT_SOURCE_DIR}/examples/common)
target_link_libraries(
  recon_example_remesh_report_test PRIVATE volumetric_kit::core_test_policy
                                           volumetric_kit::recon_mesh)
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
# and under dynamic the grid the same set in the other order gives. Skips where
# no device is present.
add_executable(recon_tsdf_integrate_set_test tsdf_integrate_set_test.cpp)
target_link_libraries(
  recon_tsdf_integrate_set_test
  PRIVATE volumetric_kit::recon_tsdf Vulkan::Vulkan
          volumetric_kit::core_test_policy)
vr_target_warnings(recon_tsdf_integrate_set_test)
add_test(NAME recon_tsdf_integrate_set COMMAND recon_tsdf_integrate_set_test)

# tsdf integrate dynamic set: a frontal camera and a grazing one that sees past
# its band, fused as a Dynamic set in either order, must give the same grid and
# mesh, the grid fusing the grazing one and then the frontal one a call each
# gives, and keep the frontal camera's surface whole. Skips (exit 0) where no
# device is present.
add_executable(recon_tsdf_integrate_dynamic_set_test
               tsdf_integrate_dynamic_set_test.cpp)
target_link_libraries(
  recon_tsdf_integrate_dynamic_set_test
  PRIVATE volumetric_kit::core_test_policy volumetric_kit::recon_tsdf
          volumetric_kit::recon_mesh Vulkan::Vulkan)
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

# volume delete: allocate a cube, remove a subset, verify the survivors + heap
# reuse (chain splice / successor pull-up) on the real driver. Skips where no
# device is present.
add_executable(recon_volume_delete_test volume_delete_test.cpp)
target_link_libraries(
  recon_volume_delete_test PRIVATE volumetric_kit::recon_volume Vulkan::Vulkan
                                   volumetric_kit::core_test_policy)
vr_target_warnings(recon_volume_delete_test)
add_test(NAME recon_volume_delete COMMAND recon_volume_delete_test)

# volume recorded rounds: record_allocate, record_remove and settle in a
# caller's batch against the blocking calls, the device count and gate, the
# rounds' bound under planted contention, one batch holding both kinds with the
# grid's zeroing first, the split dispatch (core/dispatch.hpp's test seam, by
# src/), staleness, the steady-state counters and the result slots. Skips with
# no device.
add_executable(recon_volume_recorded_test volume_recorded_test.cpp)
target_link_libraries(
  recon_volume_recorded_test
  PRIVATE volumetric_kit::recon_volume Vulkan::Vulkan
          volumetric_kit::core_test_policy)
target_include_directories(recon_volume_recorded_test
                           PRIVATE ${PROJECT_SOURCE_DIR}/src)
vr_target_warnings(recon_volume_recorded_test)
add_test(NAME recon_volume_recorded COMMAND recon_volume_recorded_test)

# volume resize: allocate, grow the table to more buckets, and verify the active
# set survives + further allocation past the old capacity works. Skips with no
# device.
add_executable(recon_volume_resize_test volume_resize_test.cpp)
target_link_libraries(
  recon_volume_resize_test PRIVATE volumetric_kit::recon_volume Vulkan::Vulkan
                                   volumetric_kit::core_test_policy)
vr_target_warnings(recon_volume_resize_test)
add_test(NAME recon_volume_resize COMMAND recon_volume_resize_test)

# Stateless volume growth: doubling, an explicit minimum, content retention, and
# rejection of invalid or unreachable sizes. The table's entry layout is
# internal (hash.hpp), reached by include path. Skips (exit 0) with no device.
add_executable(recon_volume_grid_growth_test volume_grid_growth_test.cpp)
target_link_libraries(
  recon_volume_grid_growth_test
  PRIVATE volumetric_kit::core_test_policy volumetric_kit::recon_volume
          Vulkan::Vulkan)
target_include_directories(
  recon_volume_grid_growth_test
  PRIVATE ${PROJECT_SOURCE_DIR}/src/volumetric_kit/recon/volume)
vr_target_warnings(recon_volume_grid_growth_test)
add_test(NAME recon_volume_grid_growth COMMAND recon_volume_grid_growth_test)

# volume allocate-from-depth: unproject a synthetic one-pixel depth frame and
# verify exactly the expected (2*tb+1)^3 truncation-band block cube is allocated
# on the real driver -- the camera unprojection + band dilation, plus the
# DepthPinhole scalar-layout upload -- and pin the blocks, heap count and stamps
# three posed frames allocate. Skips where no device is present.
add_executable(recon_volume_allocate_test volume_allocate_test.cpp)
target_link_libraries(
  recon_volume_allocate_test
  PRIVATE volumetric_kit::recon_volume Vulkan::Vulkan
          volumetric_kit::core_test_policy)
vr_target_warnings(recon_volume_allocate_test)
vr_embed_shaders(recon_volume_allocate_test SHADERS
                 shaders/grid_truncation.comp)
add_test(NAME recon_volume_allocate COMMAND recon_volume_allocate_test)

# volume triangle-mesh allocation: allocate from a small mesh, then verify on
# the real driver that every block holding a voxel within trunc_dist is present
# -- brute-forced on the host against an independent closest-point -- that a
# triangle wider than the band leaves no hole (where the point path does), that
# the distance prune keeps the bounding box's far corners out, and that a split
# dispatch (core/dispatch.hpp's test seam, by src/) allocates the same blocks.
# Skips where no device is present.
add_executable(recon_volume_allocate_triangles_test
               volume_allocate_triangles_test.cpp)
target_link_libraries(
  recon_volume_allocate_triangles_test
  PRIVATE volumetric_kit::recon_volume Vulkan::Vulkan
          volumetric_kit::core_test_policy)
target_include_directories(recon_volume_allocate_triangles_test
                           PRIVATE ${PROJECT_SOURCE_DIR}/src)
vr_target_warnings(recon_volume_allocate_triangles_test)
add_test(NAME recon_volume_allocate_triangles
         COMMAND recon_volume_allocate_triangles_test)

# volume frustum-culled compaction: allocate blocks at known world positions
# (one dead-centre, others behind / beyond-far / off to the side), then verify
# compact_active_blocks_in_frusta_on_device keeps only the in-view blocks while
# plain compaction returns all -- proving the plane math + the on-device AABB
# cull -- and that several frusta keep their union. Skips where no device is
# present.
add_executable(recon_volume_frustum_test volume_frustum_test.cpp)
target_link_libraries(
  recon_volume_frustum_test PRIVATE volumetric_kit::recon_volume Vulkan::Vulkan
                                    volumetric_kit::core_test_policy)
vr_target_warnings(recon_volume_frustum_test)
add_test(NAME recon_volume_frustum COMMAND recon_volume_frustum_test)

# Sparse marching cubes: mesh an analytic sphere written into a real multi-block
# VoxelBlockGrid, exercising cross-block neighbour sampling, and prove the same
# field meshes identically however it is divided into blocks or dispatches
# (core/dispatch.hpp's test seam, by src/). Skips where no device is present.
add_executable(recon_mesh_marching_cubes_sparse_test
               marching_cubes_sparse_test.cpp)
target_link_libraries(
  recon_mesh_marching_cubes_sparse_test
  PRIVATE volumetric_kit::recon_mesh Vulkan::Vulkan
          volumetric_kit::core_test_policy)
target_include_directories(recon_mesh_marching_cubes_sparse_test
                           PRIVATE ${PROJECT_SOURCE_DIR}/src)
vr_target_warnings(recon_mesh_marching_cubes_sparse_test)
add_test(NAME recon_mesh_marching_cubes_sparse
         COMMAND recon_mesh_marching_cubes_sparse_test)

# Marching cubes at corners whose distance is exactly the iso-value: lattice
# planes at 1 cm (k = -248, -1, 0; both orientations; per-column slopes, and
# slopes below the near-tangent guard) put every vertex on its corner, bit for
# bit, with that corner's attribute, from both kernels, at iso 0 and 0.0025.
# Planes 1e-8 off the iso-value keep their vertices off the lattice. Skips where
# no device is present.
add_executable(recon_mesh_marching_cubes_corner_test
               marching_cubes_corner_test.cpp)
target_link_libraries(
  recon_mesh_marching_cubes_corner_test
  PRIVATE volumetric_kit::recon_mesh Vulkan::Vulkan
          volumetric_kit::core_test_policy)
vr_target_warnings(recon_mesh_marching_cubes_corner_test)
add_test(NAME recon_mesh_marching_cubes_corner
         COMMAND recon_mesh_marching_cubes_corner_test)

# mesh recorded extract: record_extract, finish and abandon against
# extract_device, sharing on and off; the refit and its later ready value, the
# closed gate, the empty set, abandoned and dropped tokens, the skipped signal,
# the superseded pending, ready across a move, the steady-state counters and the
# sharing refusal. Skips with no device.
add_executable(recon_mesh_recorded_extract_test mesh_recorded_extract_test.cpp)
target_link_libraries(
  recon_mesh_recorded_extract_test
  PRIVATE volumetric_kit::recon_mesh Vulkan::Vulkan
          volumetric_kit::core_test_policy)
vr_target_warnings(recon_mesh_recorded_extract_test)
add_test(NAME recon_mesh_recorded_extract
         COMMAND recon_mesh_recorded_extract_test)

# mesh MarchingCubesConfig: prove a consumer's extra buffer usage reaches the
# vertex arena and index run -- on the first allocation AND across a grow -- and
# that an unsupported bit is rejected at create(). recon itself never sets one,
# so this is the only place the non-zero path runs. Skips with no device.
add_executable(recon_mesh_marching_cubes_config_test
               marching_cubes_config_test.cpp)
target_link_libraries(
  recon_mesh_marching_cubes_config_test
  PRIVATE volumetric_kit::recon_mesh Vulkan::Vulkan
          volumetric_kit::core_test_policy)
vr_target_warnings(recon_mesh_marching_cubes_config_test)
add_test(NAME recon_mesh_marching_cubes_config
         COMMAND recon_mesh_marching_cubes_config_test)

# mesh MeshExchange: the fuse-to-render handoff against a host model of the ring
# -- the mark, the take, a parked take never released, an empty mesh drawing
# nothing, the latch, a seeded run with failures in which no slot the consumer
# holds is reclaimed, and a dropped mesh, with the ring a consumer that drops
# needs. Host only, so it always runs.
add_executable(recon_mesh_exchange_test mesh_exchange_test.cpp)
target_link_libraries(recon_mesh_exchange_test
                      PRIVATE volumetric_kit::recon_mesh)
vr_target_warnings(recon_mesh_exchange_test)
add_test(NAME recon_mesh_exchange COMMAND recon_mesh_exchange_test)

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
  PRIVATE volumetric_kit::recon_texture volumetric_kit::recon_mesh
          Vulkan::Vulkan volumetric_kit::core_test_policy)
vr_target_warnings(recon_texture_device_mesh_test)
add_test(NAME recon_texture_device_mesh COMMAND recon_texture_device_mesh_test)

# codec tier, host half: CodecParams validation, EncoderConfig's sizes and the
# DCT's two tables (basis orthonormality, the zigzag order against the prior
# engine's). CPU-only, so it always runs. The tables are internal
# (non-installed), reached by include path.
add_executable(recon_codec_params_test codec_params_test.cpp)
target_link_libraries(recon_codec_params_test
                      PRIVATE volumetric_kit::recon_codec)
target_include_directories(
  recon_codec_params_test
  PRIVATE ${PROJECT_SOURCE_DIR}/src/volumetric_kit/recon/codec)
vr_target_warnings(recon_codec_params_test)
add_test(NAME recon_codec_params COMMAND recon_codec_params_test)

# codec tier, the CPU oracle (codec_oracle*.hpp): the CPU rANS coder and frame
# coder that pin the format's bytes, which the library codes on the device
# alone, and their ends on the device -- a host frame's entries uploaded, the
# forward read back, the inverse fed from the host, and the frame an Encoder
# must write -- and the decoder-side deblock's filters in double
# (codec_deblock_oracle.hpp). Test-only; the codec tests that hold the device to
# it link it, and reach the internal headers through it (core/dispatch.hpp's by
# src/).
add_library(recon_codec_oracle STATIC codec_oracle.cpp codec_oracle_device.cpp
                                      codec_deblock_oracle.cpp)
target_link_libraries(recon_codec_oracle PUBLIC volumetric_kit::recon_codec)
target_include_directories(
  recon_codec_oracle PUBLIC ${PROJECT_SOURCE_DIR}/src/volumetric_kit/recon/codec
                            ${PROJECT_SOURCE_DIR}/src)
vr_target_warnings(recon_codec_oracle)

# codec tier, GPU half: the internal DctTransform's forward kernel against a
# double-precision reference, the orthonormal reconstruction bounds, the
# observed mask, batching past one dispatch, the refusals and the moves. Skips
# with no device.
add_executable(recon_codec_dct_test codec_dct_test.cpp)
target_link_libraries(
  recon_codec_dct_test PRIVATE recon_codec_oracle Vulkan::Vulkan
                               volumetric_kit::core_test_policy)
vr_target_warnings(recon_codec_dct_test)
add_test(NAME recon_codec_dct COMMAND recon_codec_dct_test)

# codec tier, the decoder-side deblock: detail::Deblock's chains and every rule
# of its filters against the CPU oracle, workgroups one at a time in either
# order and in chunks to the same bits, its stamps, recording without a submit,
# its refusals and the step-keyed rule; and the Decoder's switch -- off exact,
# on the oracle's filter of each frame under the rule, I- and P-frames, toggled
# between frames, the filter changing in one decoder, rebuilt while paused, its
# stamps and rows. Internal headers through the oracle. Skips without a device.
add_executable(recon_codec_deblock_test codec_deblock_test.cpp)
target_link_libraries(
  recon_codec_deblock_test PRIVATE recon_codec_oracle Vulkan::Vulkan
                                   volumetric_kit::core_test_policy)
vr_target_warnings(recon_codec_deblock_test)
add_test(NAME recon_codec_deblock COMMAND recon_codec_deblock_test)

# codec tier, the device frame writer and reader: every frame the writer writes
# is the CPU oracle's, byte for byte, across block counts, K, segment sizes and
# int32 coordinate extremes, the golden frames' pinned bytes among them, intra
# and P, and every frame the reader reads, or corrupt payload it refuses, is the
# oracle's. Skips without a device.
add_executable(recon_codec_device_frame_test codec_device_frame_test.cpp)
target_link_libraries(
  recon_codec_device_frame_test PRIVATE recon_codec_oracle Vulkan::Vulkan
                                        volumetric_kit::core_test_policy)
vr_target_warnings(recon_codec_device_frame_test)
add_test(NAME recon_codec_device_frame COMMAND recon_codec_device_frame_test)

# codec tier, P-frames on the device: the reference store's apply against plain
# uint16 arithmetic, and the Decoder's frames -- the Encoder's, and I-then-P
# chains -- leaving the grid as the CPU oracle's decode of the same content
# does, bit for bit, with NotFound, a segment longer than the format allows, the
# refusals and failures that keep the reference, and moves; and the Encoder's
# P-frames, the CPU oracle's bytes, with the diff's every choice and refusal.
# Internal headers through the oracle (the public decoder test stays
# public-only). Skips without a device.
add_executable(recon_codec_inter_test codec_inter_test.cpp)
target_link_libraries(
  recon_codec_inter_test PRIVATE recon_codec_oracle Vulkan::Vulkan
                                 volumetric_kit::core_test_policy)
vr_target_warnings(recon_codec_inter_test)
add_test(NAME recon_codec_inter COMMAND recon_codec_inter_test)

# codec tier, the block lists on the device: the encoder's sort against
# std::stable_sort, over all of int32 and every kind of tie; the decoder's merge
# against the oracle's map and row 13's first collision against a walk; the grid
# diff against std::set_difference; garbage writing nothing past an output; and
# the refusals. Internal headers through the oracle. Skips without a device.
add_executable(recon_codec_block_lists_test codec_block_lists_test.cpp)
target_link_libraries(
  recon_codec_block_lists_test PRIVATE recon_codec_oracle Vulkan::Vulkan
                                       volumetric_kit::core_test_policy)
vr_target_warnings(recon_codec_block_lists_test)
add_test(NAME recon_codec_block_lists COMMAND recon_codec_block_lists_test)

# codec tier, the device frame writer's step layout: every entry's start and
# segment's slot against the CPU oracle's, for real frames' steps (every K and
# segment size, int32 extremes, the worked example, the P-frame matrix) and any
# steps up to the bound, the step total's cap, and the refusals. Internal
# headers through the oracle. Skips without a device.
add_executable(recon_codec_step_layout_test codec_step_layout_test.cpp)
target_link_libraries(
  recon_codec_step_layout_test PRIVATE recon_codec_oracle Vulkan::Vulkan
                                       volumetric_kit::core_test_policy)
vr_target_warnings(recon_codec_step_layout_test)
add_test(NAME recon_codec_step_layout COMMAND recon_codec_step_layout_test)

# codec tier, entropy coding: the table normalization and the CPU oracle's rANS
# coder (round trips, a probability-one symbol's zero cost, size against the
# ideal code length, truncation and bit flips), the v3 frame through the oracle
# (round trips, every writer and reader refusal, the frame type, the section
# rules, corruption, and an all-zero frame's cost), and its P-frames (the
# reference hash, the worked example, round trips and chains against an
# independent model, the block merge, every refusal in order, corruption). All
# host-only, so they always run.
foreach(_vr_codec_host_test rans bitstream inter_bitstream)
  add_executable(recon_codec_${_vr_codec_host_test}_test
                 codec_${_vr_codec_host_test}_test.cpp)
  target_link_libraries(recon_codec_${_vr_codec_host_test}_test
                        PRIVATE recon_codec_oracle)
  vr_target_warnings(recon_codec_${_vr_codec_host_test}_test)
  add_test(NAME recon_codec_${_vr_codec_host_test}
           COMMAND recon_codec_${_vr_codec_host_test}_test)
endforeach()

# codec tier, public API: the Encoder (an empty grid, the CPU oracle's bytes,
# independent of the hash table's order, unobserved blocks left out, its rows,
# refusals, moves, and the rules for an I-frame or a P-frame) and the Decoder
# with the codec end to end -- a decoded grid within the orthonormal bound of
# its source, meshing to the analytic sphere it came from, frame after frame
# into one grid, untouched on every early refusal, OutOfMemory that a resize
# recovers, and P-frame chains decoding to the all-intra chain's grids. Public
# headers only, plus the shared codec_fixture.hpp and the oracle's public-typed
# codec_oracle_encode.hpp; the decoder test links recon_mesh, which the tier may
# not. Both skip with no device.
add_executable(recon_codec_encoder_test codec_encoder_test.cpp)
target_link_libraries(
  recon_codec_encoder_test PRIVATE recon_codec_oracle Vulkan::Vulkan
                                   volumetric_kit::core_test_policy)
vr_target_warnings(recon_codec_encoder_test)
add_test(NAME recon_codec_encoder COMMAND recon_codec_encoder_test)

# Allocation refusal preserves the predictor, GOP and retryable front results.
add_executable(recon_codec_encoder_oom_test codec_encoder_oom_test.cpp)
target_link_libraries(
  recon_codec_encoder_oom_test
  PRIVATE volumetric_kit::recon_codec Vulkan::Vulkan
          volumetric_kit::core_test_policy
          volumetric_kit::core_test_allocation_failure)
vr_target_warnings(recon_codec_encoder_oom_test)
add_test(NAME recon_codec_encoder_oom COMMAND recon_codec_encoder_oom_test)

add_executable(recon_codec_decoder_test codec_decoder_test.cpp)
target_link_libraries(
  recon_codec_decoder_test
  PRIVATE volumetric_kit::recon_codec volumetric_kit::recon_mesh Vulkan::Vulkan
          volumetric_kit::core_test_policy)
vr_target_warnings(recon_codec_decoder_test)
add_test(NAME recon_codec_decoder COMMAND recon_codec_decoder_test)

# codec tier, a player's grid (decode_grid.hpp): make_decode_grid's geometry,
# layout and attributes off a frame's header, and decode_growing's one grow, its
# second OutOfMemory, its timing of the attempt that succeeds, and the decoder's
# refusals passed through; the caller's ceiling, against frames forged with the
# internal frame writer. Skips with no device.
add_executable(recon_codec_decode_grid_test codec_decode_grid_test.cpp)
target_link_libraries(
  recon_codec_decode_grid_test
  PRIVATE volumetric_kit::recon_codec Vulkan::Vulkan
          volumetric_kit::core_test_policy)
target_include_directories(
  recon_codec_decode_grid_test
  PRIVATE ${PROJECT_SOURCE_DIR}/src/volumetric_kit/recon/codec)
vr_target_warnings(recon_codec_decode_grid_test)
add_test(NAME recon_codec_decode_grid COMMAND recon_codec_decode_grid_test)

# codec tier, a frame decoded and meshed in the caller's batch: the recorded
# decode against decode(), deblock on and off; the host refusals, which record
# nothing; a device refusal gating the extract after it; superseded and
# abandoned tokens and the input slots; the rows; the steady-state counters; the
# pinned dispatches and barriers of a rafa2-like P-frame's batch; and
# record_growing and finish_growing. Links recon_mesh, which the codec tier
# itself may not. Skips with no device.
add_executable(recon_codec_recorded_test codec_recorded_test.cpp)
target_link_libraries(
  recon_codec_recorded_test
  PRIVATE volumetric_kit::recon_codec volumetric_kit::recon_mesh Vulkan::Vulkan
          volumetric_kit::core_test_policy)
vr_target_warnings(recon_codec_recorded_test)
add_test(NAME recon_codec_recorded COMMAND recon_codec_recorded_test)

# Allocation injection stays in its own executable: factory diagnostics and a
# metrics merge after a committed P-frame, followed by the next P-frame.
add_executable(recon_codec_decode_grid_oom_test codec_decode_grid_oom_test.cpp)
target_link_libraries(
  recon_codec_decode_grid_oom_test
  PRIVATE volumetric_kit::recon_codec Vulkan::Vulkan
          volumetric_kit::core_test_policy
          volumetric_kit::core_test_allocation_failure)
vr_target_warnings(recon_codec_decode_grid_oom_test)
add_test(NAME recon_codec_decode_grid_oom
         COMMAND recon_codec_decode_grid_oom_test)

# A pending-frame allocation failure stays inside record and both blocking
# decode APIs, preserves the grid/reference, and permits the same P-frame retry.
add_executable(recon_codec_decoder_oom_test codec_decoder_oom_test.cpp)
target_link_libraries(
  recon_codec_decoder_oom_test
  PRIVATE volumetric_kit::recon_codec Vulkan::Vulkan
          volumetric_kit::core_test_policy
          volumetric_kit::core_test_allocation_failure)
vr_target_warnings(recon_codec_decoder_oom_test)
add_test(NAME recon_codec_decoder_oom COMMAND recon_codec_decoder_oom_test)

# eval tier: closest points, the reach clamp, the summary statistics, accuracy /
# coverage and the F-score between meshes a known distance apart, and every
# refusal. Host-only.
add_executable(recon_eval_mesh_distance_test eval_mesh_distance_test.cpp)
target_link_libraries(recon_eval_mesh_distance_test
                      PRIVATE volumetric_kit::recon_eval)
vr_target_warnings(recon_eval_mesh_distance_test)
add_test(NAME recon_eval_mesh_distance COMMAND recon_eval_mesh_distance_test)

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

# Host mesh I/O remains testable without creating a Vulkan device.
add_executable(recon_io_ply_test io_ply_test.cpp)
target_link_libraries(recon_io_ply_test PRIVATE volumetric_kit::recon_mesh_io)
vr_target_warnings(recon_io_ply_test)
add_test(NAME recon_io_ply COMMAND recon_io_ply_test)

# The opt-in mesh normalization and topology helpers, on synthetic meshes.
add_executable(recon_io_mesh_normalization_test io_mesh_normalization_test.cpp)
target_link_libraries(
  recon_io_mesh_normalization_test
  PRIVATE volumetric_kit::recon_mesh_io
          volumetric_kit::core_test_allocation_failure)
vr_target_warnings(recon_io_mesh_normalization_test)
add_test(NAME recon_io_mesh_normalization
         COMMAND recon_io_mesh_normalization_test)

# io::check_mesh_file, the one home of a mesh path's checks, and load_mesh
# answering through it. Host only; built with the Assimp backend.
if(TARGET recon_mesh_io_assimp)
  add_executable(recon_io_mesh_file_test io_mesh_file_test.cpp)
  target_link_libraries(recon_io_mesh_file_test
                        PRIVATE volumetric_kit::recon_mesh_io_assimp)
  vr_target_warnings(recon_io_mesh_file_test)
  add_test(NAME recon_io_mesh_file COMMAND recon_io_mesh_file_test)

  # io::load_textured_mesh on a procedural OBJ + MTL + PNG cube it writes, the
  # PNG with image I/O.
  add_executable(recon_io_textured_mesh_test io_textured_mesh_test.cpp)
  target_link_libraries(
    recon_io_textured_mesh_test PRIVATE volumetric_kit::recon_mesh_io_assimp
                                        volumetric_kit::sensor_image_io)
  vr_target_warnings(recon_io_textured_mesh_test)
  add_test(NAME recon_io_textured_mesh COMMAND recon_io_textured_mesh_test)
endif()

# stream: the .vrst container through its public API -- golden bytes whose CRCs
# Python's zlib.crc32 computed, round trips, seeking, unknown kinds and chunks,
# the limits, every refusal, two threads on one reader -- its chunks in memory:
# the encoder's bytes and refusals against the writer's, the parser fed a file
# in any pieces against the reader, each refusal at its byte -- what its reader
# recovers: flipped bits, the tail cut at every byte, damaged spans, a killed
# writer and one stopped by RLIMIT_FSIZE -- and its open from the footer: the
# scan's counts and keys, each footer check refusing a forged footer, and
# records past 4 GiB (skipped, 77, without sparse files). Host-only.
find_package(Threads REQUIRED)
add_executable(recon_stream_container_test stream_container_test.cpp)
target_link_libraries(recon_stream_container_test
                      PRIVATE volumetric_kit::recon_stream Threads::Threads)
vr_target_warnings(recon_stream_container_test)
add_test(NAME recon_stream_container COMMAND recon_stream_container_test)

add_executable(recon_stream_chunks_test stream_chunks_test.cpp)
target_link_libraries(recon_stream_chunks_test
                      PRIVATE volumetric_kit::recon_stream)
vr_target_warnings(recon_stream_chunks_test)
add_test(NAME recon_stream_chunks COMMAND recon_stream_chunks_test)

add_executable(recon_stream_recovery_test stream_recovery_test.cpp)
target_link_libraries(recon_stream_recovery_test
                      PRIVATE volumetric_kit::recon_stream)
vr_target_warnings(recon_stream_recovery_test)
add_test(NAME recon_stream_recovery COMMAND recon_stream_recovery_test)

add_executable(recon_stream_footer_test stream_footer_test.cpp)
target_link_libraries(
  recon_stream_footer_test PRIVATE volumetric_kit::recon_stream
                                   volumetric_kit::core_test_allocation_failure)
# A 64-bit off_t, as the library's, for pwrite and truncate past 4 GiB.
target_compile_definitions(recon_stream_footer_test
                           PRIVATE _FILE_OFFSET_BITS=64)
vr_target_warnings(recon_stream_footer_test)
add_test(NAME recon_stream_footer COMMAND recon_stream_footer_test)

# Public stream calls contain allocation failures in validation, diagnostics,
# saved errors, and index/footer growth. Host-only, including retry behavior.
add_executable(recon_stream_oom_test stream_oom_test.cpp)
target_link_libraries(
  recon_stream_oom_test PRIVATE volumetric_kit::recon_stream
                                volumetric_kit::core_test_allocation_failure)
vr_target_warnings(recon_stream_oom_test)
add_test(NAME recon_stream_oom COMMAND recon_stream_oom_test)

# networking: the server's GOP log with no sockets (join, newest key wins at a
# chunk boundary, the GOP limit, a lost copy, no P-frame without its key; keys
# on demand: joins and fall-backs counted, the bytes held bounded) and the
# socket calls (each socket's settings read back, no SIGPIPE, the bytes a peer
# has not acknowledged, a wait a signal interrupts) -- internal and unexported,
# so the tests compile their sources themselves -- then the server and the
# client on loopback, 127.0.0.1 port 0 only: the wire against the writer's file,
# joins, newest key wins, keys on demand, the limits, the rules, teardown,
# signals, the client's errors against raw senders, and a stress run in both
# modes for ThreadSanitizer. Host-only.
set(_vr_networking_src
    ${PROJECT_SOURCE_DIR}/src/volumetric_kit/recon/networking)
add_executable(recon_networking_gop_log_test networking_gop_log_test.cpp
                                             ${_vr_networking_src}/gop_log.cpp)
target_include_directories(recon_networking_gop_log_test
                           PRIVATE ${_vr_networking_src})
vr_target_warnings(recon_networking_gop_log_test)
add_test(NAME recon_networking_gop_log COMMAND recon_networking_gop_log_test)

add_executable(recon_networking_socket_test networking_socket_test.cpp
                                            ${_vr_networking_src}/socket.cpp)
target_include_directories(recon_networking_socket_test
                           PRIVATE ${_vr_networking_src})
target_link_libraries(recon_networking_socket_test
                      PRIVATE volumetric_kit::core_base Threads::Threads)
vr_target_warnings(recon_networking_socket_test)
add_test(NAME recon_networking_socket COMMAND recon_networking_socket_test)

foreach(_vr_networking_test server client stress)
  add_executable(recon_networking_${_vr_networking_test}_test
                 networking_${_vr_networking_test}_test.cpp)
  target_link_libraries(
    recon_networking_${_vr_networking_test}_test
    PRIVATE volumetric_kit::recon_networking Threads::Threads)
  vr_target_warnings(recon_networking_${_vr_networking_test}_test)
  add_test(NAME recon_networking_${_vr_networking_test}
           COMMAND recon_networking_${_vr_networking_test}_test)
endforeach()

# The client also checks its progress output across an allocation failure while
# copying a saved error.
target_link_libraries(recon_networking_client_test
                      PRIVATE volumetric_kit::core_test_allocation_failure)

# The server, keys on demand, when a record's copy fails: the viewers it sends
# back counted before publish returns. Apart from the server's other tests,
# which link no replaced operator new, so they build under ThreadSanitizer on
# Linux too.
add_executable(recon_networking_server_oom_test networking_server_oom_test.cpp)
target_link_libraries(
  recon_networking_server_oom_test
  PRIVATE volumetric_kit::recon_networking Threads::Threads
          volumetric_kit::core_test_allocation_failure)
vr_target_warnings(recon_networking_server_oom_test)
add_test(NAME recon_networking_server_oom
         COMMAND recon_networking_server_oom_test)

# codec_stream tier: the codec stream (codec_stream/codec_stream.hpp) on a grid
# holding a sphere, linking no fuser and no sensor tier -- outputs it cannot
# make and a session that is not one Geometry track refused on create, none
# written without outputs, each frame as it went out, the .vrst's records (every
# encoded frame, byte for byte, a frame the decoder refuses included and
# counted, a key exactly when an I-frame, the same bytes on a rerun), an I-frame
# after a frame not decoded, a frame out of order or any call once finished or
# moved from refused, an unfinished stream read whole and an empty one removed,
# frames in halves around a caller's batch the file code() writes, the batch's
# device rows split between the frame and the caller, the front half again,
# decode off, and the frames served live to a client on 127.0.0.1 port 0 as the
# file records them. The device part skips with no device.
add_executable(recon_codec_stream_test codec_stream_test.cpp)
target_link_libraries(
  recon_codec_stream_test
  PRIVATE volumetric_kit::core_test_policy volumetric_kit::recon_codec_stream
          Vulkan::Vulkan)
vr_target_warnings(recon_codec_stream_test)
add_test(NAME recon_codec_stream COMMAND recon_codec_stream_test)

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
          volumetric_kit::recon_mesh volumetric_kit::sensor_array
          Vulkan::Vulkan)
vr_target_warnings(recon_sender_set_sender_test)
add_test(NAME recon_sender_set_sender COMMAND recon_sender_set_sender_test)

# serve_stream's replay (examples/common/stream_replay.hpp): the records from
# the first key, seq and key pts across passes by the writer's rules, the dues,
# damage left out and an allocation failure past it refused, and a damaged
# payload skipped to the next key. Host only.
add_executable(recon_example_stream_replay_test example_stream_replay_test.cpp)
target_include_directories(recon_example_stream_replay_test
                           PRIVATE ${PROJECT_SOURCE_DIR}/examples/common)
target_link_libraries(
  recon_example_stream_replay_test
  PRIVATE volumetric_kit::recon_stream
          volumetric_kit::core_test_allocation_failure)
vr_target_warnings(recon_example_stream_replay_test)
add_test(NAME recon_example_stream_replay
         COMMAND recon_example_stream_replay_test)

# The tests whose code runs threads -- a reader shared by two, the mesh
# handoff's producer and consumer, the sockets', the senders' log, the Orbbec
# decode workers -- which CI builds and runs again under ThreadSanitizer
# (.github/workflows/_build.yml): one target to build them.
add_custom_target(recon_tsan_tests)
add_dependencies(
  recon_tsan_tests
  recon_stream_container_test
  recon_mesh_exchange_test
  recon_networking_gop_log_test
  recon_networking_socket_test
  recon_networking_server_test
  recon_networking_server_oom_test
  recon_networking_client_test
  recon_networking_stress_test
  recon_example_info_log_test)

# What a sibling that fetches recon gets (subproject/): its configure, from a
# clean tree, with each dependency this build fetched as a local source, so it
# needs no network. The defaults hold, image I/O links the core's base tier
# alone, the sibling's vkc_require_sibling accepts this recon and refuses it
# under a floor above its version, and recon refuses a core the sibling made
# available first that is older than VR_CORE_MIN_VERSION or lacks the vulkan
# tier. The old core is this one stamped 0.0.1, standing in for one below a
# later floor: a real core older than 0.1.0 has no vkc_require_core at all.
#
# TODO: move this core-requirement harness -- the message helper and
# subproject/'s version stamp, which gfx and calib repeat -- into the core's
# test support, as a function each sibling calls with its package and floor, and
# port the siblings onto it in one series.
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
set_tests_properties(recon_subproject_defaults
                     PROPERTIES FIXTURES_SETUP vr_subproject_defaults)
math(EXPR _vr_next_minor "${PROJECT_VERSION_MINOR} + 1")
set(_vr_newer "${PROJECT_VERSION_MAJOR}.${_vr_next_minor}.0")
_vr_add_subproject_test(recon_subproject_old_recon old_recon
                        "-DVR_REQUIRE_RECON=${_vr_newer}")
_vr_expect_message(
  recon_subproject_old_recon
  "vr_subproject_consumer needs volumetric_kit_recon ${_vr_newer} or newer, but the recon at .+ is ${PROJECT_VERSION}: pin v${_vr_newer} or newer"
)
# The sibling compiles kernels that include recon's exported GLSL through the
# roots recon_core and recon_volume name, each holding exactly what its target
# exports (glsl_consumer.cmake).
add_test(NAME recon_subproject_glsl
         COMMAND "${CMAKE_COMMAND}" --build "${_vr_subproject}/defaults"
                 --target vr_glsl_consumer)
set_tests_properties(recon_subproject_glsl PROPERTIES FIXTURES_REQUIRED
                                                      vr_subproject_defaults)
# ... and compiles the public headers that reach the volume tier, which hold
# none of the hash table's internals (subproject/internal_names.cpp).
add_test(NAME recon_subproject_headers
         COMMAND "${CMAKE_COMMAND}" --build "${_vr_subproject}/defaults"
                 --target vr_public_headers)
set_tests_properties(recon_subproject_headers PROPERTIES FIXTURES_REQUIRED
                                                         vr_subproject_defaults)
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
set_tests_properties(recon_codec_inter PROPERTIES COST 115)
set_tests_properties(recon_codec_decoder PROPERTIES COST 96)
set_tests_properties(recon_voxelize_mesh_integrate_exact PROPERTIES COST 56)
set_tests_properties(recon_eval_mesh_distance PROPERTIES COST 55)
set_tests_properties(recon_codec_device_frame PROPERTIES COST 52)
set_tests_properties(recon_codec_stream PROPERTIES COST 49)
set_tests_properties(recon_voxelize_mesh_integrate PROPERTIES COST 36)
set_tests_properties(recon_codec_encoder PROPERTIES COST 30)
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
