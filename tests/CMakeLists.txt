# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# camera tier: the camera model (its checks, the lens template against OpenCV's
# numbers), its geometry (rigid checks, Rodrigues both ways), and the sensor
# array's calibration file (versions 2 and 1 read, the refusals, written and
# read back, its document parsed independently with nlohmann/json). Host-only,
# so they always run.
add_executable(recon_camera_model_test camera_model_test.cpp)
target_link_libraries(recon_camera_model_test
                      PRIVATE volumetric_kit::recon_camera)
vr_target_warnings(recon_camera_model_test)
add_test(NAME recon_camera_model COMMAND recon_camera_model_test)

add_executable(recon_camera_geometry_test camera_geometry_test.cpp)
target_link_libraries(recon_camera_geometry_test
                      PRIVATE volumetric_kit::recon_camera)
vr_target_warnings(recon_camera_geometry_test)
add_test(NAME recon_camera_geometry COMMAND recon_camera_geometry_test)

add_executable(recon_camera_array_calibration_test
               camera_array_calibration_test.cpp)
target_link_libraries(recon_camera_array_calibration_test
                      PRIVATE volumetric_kit::recon_camera)
target_include_directories(recon_camera_array_calibration_test SYSTEM
                           PRIVATE "${VR_JSON_INCLUDE_DIR}")
vr_target_warnings(recon_camera_array_calibration_test)
add_test(NAME recon_camera_array_calibration
         COMMAND recon_camera_array_calibration_test)

# The frame prep's same-K depth crop against a folding lens: a forward scan of
# distort_rational that finds a strong barrel lens's fold, run on a stand-in for
# a Femto Mega's NFOV depth lens (not an SDK report). It needs recon_camera
# alone, so it runs in camera-only builds too.
add_executable(recon_sensor_depth_lens_fold_test
               sensor_depth_lens_fold_test.cpp)
target_link_libraries(recon_sensor_depth_lens_fold_test
                      PRIVATE volumetric_kit::recon_camera)
vr_target_warnings(recon_sensor_depth_lens_fold_test)
add_test(NAME recon_sensor_depth_lens_fold
         COMMAND recon_sensor_depth_lens_fold_test)

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

# The camera tier is all VR_CAMERA_ONLY builds. Only they check recon
# camera-only (subproject/camera_only.cmake): with find_package(Vulkan)
# disabled, the options that need the rest are refused, and a consumer builds
# against the fetched source. The camera-only CI job, which has no Vulkan at
# all, is its home; it is Release whatever the build's type.
if(VR_CAMERA_ONLY)
  add_test(
    NAME recon_subproject_camera_only
    COMMAND
      "${CMAKE_COMMAND}" "-DSOURCE_DIR=${PROJECT_SOURCE_DIR}"
      "-DBINARY_DIR=${CMAKE_CURRENT_BINARY_DIR}/subproject_camera_only"
      "-DGENERATOR=${CMAKE_GENERATOR}" "-DCXX_COMPILER=${CMAKE_CXX_COMPILER}"
      ${_vr_fetched_sources} -P
      "${CMAKE_CURRENT_SOURCE_DIR}/subproject/camera_only.cmake")
  return()
endif()

# The core tier's own pieces: the version and the log seam's "vr" source.
add_executable(recon_core_smoke_test core_smoke_test.cpp)
target_link_libraries(recon_core_smoke_test PRIVATE volumetric_kit::recon_core)
vr_target_warnings(recon_core_smoke_test)
add_test(NAME recon_core_smoke COMMAND recon_core_smoke_test)

# The GPU tests' harness (gpu_test.hpp): a test fails on an error the validation
# layer reports, a leaked object's included, and skips as the core's test policy
# and VR_TEST_HEVC_BACKEND say. Under LeakSanitizer, tools/lsan.supp leaves a
# block leaked where the layer reports an error reported. Skips without the
# layer or a device.
add_executable(recon_gpu_test_harness_test gpu_test_harness_test.cpp)
target_link_libraries(
  recon_gpu_test_harness_test PRIVATE volumetric_kit::recon_core
                                      volumetric_kit::core_test_policy)
vr_target_warnings(recon_gpu_test_harness_test)
add_test(NAME recon_gpu_test_harness COMMAND recon_gpu_test_harness_test)

# Every tier reports, and reports the DEVICE half. The load-bearing assertion is
# per row: gpu_ms < cpu_ms, since the host span wraps the submit the device span
# sits inside -- equal numbers mean the wall clock was relabelled. Also pins
# that a null out-param measures nothing, and that publishing ends the timer
# window.
add_executable(recon_tier_stage_metrics_test tier_stage_metrics_test.cpp)
target_link_libraries(
  recon_tier_stage_metrics_test
  PRIVATE volumetric_kit::recon_texture volumetric_kit::recon_tsdf
          volumetric_kit::recon_volume volumetric_kit::recon_core
          volumetric_kit::core_test_policy)
vr_target_warnings(recon_tier_stage_metrics_test)
add_test(NAME recon_tier_stage_metrics COMMAND recon_tier_stage_metrics_test)

# volume coordinate math: pure host transforms (world <-> voxel <-> block,
# negative-coordinate block indexing, truncation band); and the tests' host
# oracle of the spatial hash (volume_hash_oracle.hpp). CPU-only (no device), so
# these always run -- not gated on a Vulkan driver.
add_executable(recon_volume_coords_test volume_coords_test.cpp)
target_link_libraries(recon_volume_coords_test
                      PRIVATE volumetric_kit::recon_volume)
vr_target_warnings(recon_volume_coords_test)
add_test(NAME recon_volume_coords COMMAND recon_volume_coords_test)

add_executable(recon_volume_hash_test volume_hash_test.cpp)
target_link_libraries(recon_volume_hash_test
                      PRIVATE volumetric_kit::recon_volume)
vr_target_warnings(recon_volume_hash_test)
add_test(NAME recon_volume_hash COMMAND recon_volume_hash_test)

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
# keeps the records, remove and clear zero them. Skips with no device.
add_executable(recon_volume_block_stamps_test volume_block_stamps_test.cpp)
target_link_libraries(
  recon_volume_block_stamps_test
  PRIVATE volumetric_kit::recon_volume Vulkan::Vulkan
          volumetric_kit::core_test_policy)
vr_target_warnings(recon_volume_block_stamps_test)
add_test(NAME recon_volume_block_stamps COMMAND recon_volume_block_stamps_test)

# tsdf integrate: fuse a synthetic constant-depth plane into a VoxelBlockGrid
# and verify the classic projective TSDF math per voxel (sdf, truncation, weight
# + cap). It also runs the kernel's projection alone (shaders/project_pinhole)
# on inputs that make its depth or pixel NaN. Skips with no device.
add_executable(recon_tsdf_integrate_test tsdf_integrate_test.cpp)
target_link_libraries(
  recon_tsdf_integrate_test PRIVATE volumetric_kit::recon_tsdf Vulkan::Vulkan
                                    volumetric_kit::core_test_policy)
include(vr_embed)
vr_embed_shaders(recon_tsdf_integrate_test SHADERS
                 "${CMAKE_CURRENT_SOURCE_DIR}/shaders/project_pinhole.comp")
vr_target_warnings(recon_tsdf_integrate_test)
add_test(NAME recon_tsdf_integrate COMMAND recon_tsdf_integrate_test)

# tsdf mesh integrate: write a mesh's truncated distance field (signed through
# pseudonormals, or as a shell) and check every voxel of every written block
# against a brute-force host reference -- an independent closest point, and the
# winding number for inside/outside -- on a sharp tetrahedron, a dented cube, an
# open quad and a triangle soup, plus determinism and every refusal. Skips with
# no device.
add_executable(recon_tsdf_mesh_integrate_test tsdf_mesh_integrate_test.cpp)
target_link_libraries(
  recon_tsdf_mesh_integrate_test
  PRIVATE volumetric_kit::recon_tsdf Vulkan::Vulkan
          volumetric_kit::core_test_policy)
vr_target_warnings(recon_tsdf_mesh_integrate_test)
add_test(NAME recon_tsdf_mesh_integrate COMMAND recon_tsdf_mesh_integrate_test)

# tsdf mesh integrate, exactly: the kernel's early exit writes, bit for bit,
# what a brute force (shaders/mesh_integrate_reference.comp) writes measuring
# every triangle of each voxel's bin -- on shared vertices with opposite
# windings, duplicates, near-coincident sheets, slivers, metre-long triangles, a
# far sphere, bins past the kernel's sort, three block sizes, both modes. Skips
# with no device.
add_executable(recon_tsdf_mesh_integrate_exact_test
               tsdf_mesh_integrate_exact_test.cpp)
target_link_libraries(
  recon_tsdf_mesh_integrate_exact_test
  PRIVATE volumetric_kit::recon_tsdf Vulkan::Vulkan
          volumetric_kit::core_test_policy)
vr_embed_shaders(
  recon_tsdf_mesh_integrate_exact_test SHADERS
  "${CMAKE_CURRENT_SOURCE_DIR}/shaders/mesh_integrate_reference.comp")
vr_target_warnings(recon_tsdf_mesh_integrate_exact_test)
add_test(NAME recon_tsdf_mesh_integrate_exact
         COMMAND recon_tsdf_mesh_integrate_exact_test)
# GPU pre-processing (recon_sensor_utils) on synthetic frames: undistortion in
# the right direction, the colour matrix, a host reference of the sampling, the
# refusals, the outputs' sharing, read on another queue family, and fusing the
# result. Skips where no device is present.
add_executable(recon_sensor_gpu_frame_prep_test sensor_gpu_frame_prep_test.cpp)
target_link_libraries(
  recon_sensor_gpu_frame_prep_test
  PRIVATE volumetric_kit::recon_sensor_utils volumetric_kit::recon_tsdf
          volumetric_kit::core_test_policy)
vr_target_warnings(recon_sensor_gpu_frame_prep_test)
add_test(NAME recon_sensor_gpu_frame_prep
         COMMAND recon_sensor_gpu_frame_prep_test)

# Allocation failure while staging is an OutOfMemory result, and a retained
# frame survives it. Separate executable for its replacement operator new.
add_executable(recon_sensor_gpu_frame_prep_oom_test
               sensor_gpu_frame_prep_oom_test.cpp test_allocation_failure.cpp)
target_link_libraries(
  recon_sensor_gpu_frame_prep_oom_test
  PRIVATE volumetric_kit::core_test_policy volumetric_kit::recon_sensor_utils)
vr_target_warnings(recon_sensor_gpu_frame_prep_oom_test)
add_test(NAME recon_sensor_gpu_frame_prep_oom
         COMMAND recon_sensor_gpu_frame_prep_oom_test)

# A set's pictures read back to the host (recon_sensor_utils' PictureReadback)
# on synthetic NV12 images and NV12 and I420 buffers: exact bytes at odd sizes,
# offsets and strides, every queue family a buffer comes from, the host memory
# grown and reused, the refusals and the stage row. Skips with no device.
add_executable(recon_sensor_picture_readback_test
               sensor_picture_readback_test.cpp)
target_link_libraries(
  recon_sensor_picture_readback_test PRIVATE volumetric_kit::recon_sensor_utils
                                             volumetric_kit::core_test_policy)
vr_target_warnings(recon_sensor_picture_readback_test)
add_test(NAME recon_sensor_picture_readback
         COMMAND recon_sensor_picture_readback_test)

# Allocation failure during creation (including the timer) or a read is an
# OutOfMemory result, and retry succeeds. Separate allocation overrides.
add_executable(recon_sensor_picture_readback_oom_test
               sensor_picture_readback_oom_test.cpp test_allocation_failure.cpp)
target_link_libraries(
  recon_sensor_picture_readback_oom_test
  PRIVATE volumetric_kit::recon_sensor_utils volumetric_kit::core_test_policy)
vr_target_warnings(recon_sensor_picture_readback_oom_test)
add_test(NAME recon_sensor_picture_readback_oom
         COMMAND recon_sensor_picture_readback_oom_test)

# The viewer's retained keyframe survives preparation and integration errors,
# and still supplies depth, coverage and atlas pixels for projective texturing;
# the helper fuses in the integration mode it is given, the one fusion.dynamic
# selects. Headless: uses the example's fusion helper, without gfx or a dataset.
add_executable(recon_example_fuse_keyframe_test example_fuse_keyframe_test.cpp)
target_include_directories(recon_example_fuse_keyframe_test
                           PRIVATE ${PROJECT_SOURCE_DIR}/examples/common)
target_link_libraries(
  recon_example_fuse_keyframe_test
  PRIVATE volumetric_kit::recon_sensor_utils volumetric_kit::recon_tsdf
          volumetric_kit::recon_texture volumetric_kit::core_test_policy)
vr_target_warnings(recon_example_fuse_keyframe_test)
add_test(NAME recon_example_fuse_keyframe
         COMMAND recon_example_fuse_keyframe_test)

# The core parses and validates scenarios. These tests pin the conversion into
# the runtime sensor and codec options (the codec's steps checked here, not by
# core), without opening a camera or GPU.
add_executable(recon_example_config_test example_config_test.cpp)
target_include_directories(recon_example_config_test
                           PRIVATE ${PROJECT_SOURCE_DIR}/examples/common)
if(TARGET volumetric_kit::recon_sensor_orbbec)
  target_link_libraries(
    recon_example_config_test
    PRIVATE volumetric_kit::recon_codec volumetric_kit::recon_sensor_orbbec
            volumetric_kit::core_config)
  target_compile_definitions(recon_example_config_test
                             PRIVATE VR_TEST_ORBBEC_CONFIG=1)
else()
  target_link_libraries(
    recon_example_config_test
    PRIVATE volumetric_kit::recon_codec volumetric_kit::recon_sensor
            volumetric_kit::core_config)
endif()
# codec_config.hpp maps the outputs into codec_stream.hpp's CodecOutputs.
target_link_libraries(
  recon_example_config_test
  PRIVATE volumetric_kit::recon_eval volumetric_kit::recon_networking
          volumetric_kit::recon_stream volumetric_kit::recon_tsdf)
target_compile_definitions(
  recon_example_config_test
  PRIVATE VR_CONFIG_FIXTURES="${PROJECT_SOURCE_DIR}/config/examples")
vr_target_warnings(recon_example_config_test)
add_test(NAME recon_example_config COMMAND recon_example_config_test)

# fuse_orbbec's sets (examples/common/fuse_orbbec_sets.hpp), where the driver is
# built: a set's first frames kept, the rest let go, and a run over fake cameras
# counted, cut to its length and its skew kept. Host only.
if(TARGET volumetric_kit::recon_sensor_orbbec)
  add_executable(recon_example_fuse_orbbec_sets_test
                 example_fuse_orbbec_sets_test.cpp)
  target_include_directories(recon_example_fuse_orbbec_sets_test
                             PRIVATE ${PROJECT_SOURCE_DIR}/examples/common)
  target_link_libraries(
    recon_example_fuse_orbbec_sets_test
    PRIVATE volumetric_kit::recon_sensor_orbbec
            volumetric_kit::recon_sensor_array volumetric_kit::core_config)
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
  recon_example_set_dump_test PRIVATE volumetric_kit::recon_sensor_array
                                      volumetric_kit::recon_io)
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

# The codec examples' stream (examples/common/codec_stream.hpp): on the host, a
# row's type and host and device milliseconds, its CSV line, a frame type's
# sizes, a GOP's bytes and bitrate, a frame's pts, and the comparison options
# the codec examples share; on a device, outputs it cannot make refused on
# create, none written without outputs, a row per decoded frame matching the
# frame and each type's stage rows its own, the .vrst's records (every encoded
# frame, byte for byte, a frame the decoder refuses included and the stream
# going on after it, a key exactly when an I-frame, and the same bytes on a
# rerun), an I-frame after a frame not decoded or whose CSV row fails, a frame
# out of order or any call once finished refused before anything is encoded, the
# CSV replaced on create and written as frames are coded, and the frames served
# live to a client on 127.0.0.1 port 0 as the file records them. The device part
# skips with no device.
add_executable(recon_example_codec_stream_test example_codec_stream_test.cpp)
target_include_directories(recon_example_codec_stream_test
                           PRIVATE ${PROJECT_SOURCE_DIR}/examples/common)
target_link_libraries(
  recon_example_codec_stream_test
  PRIVATE volumetric_kit::core_test_policy volumetric_kit::recon_codec
          volumetric_kit::recon_eval volumetric_kit::recon_networking
          volumetric_kit::recon_stream Vulkan::Vulkan)
vr_target_warnings(recon_example_codec_stream_test)
add_test(NAME recon_example_codec_stream
         COMMAND recon_example_codec_stream_test)

# fuse_orbbec's coded sets (examples/common/set_codec.hpp), with no camera
# driver: fake cameras read as a sensor array and prepared on the device, each
# set fused and coded into a .vrst whose records, keys, sets and pts are the
# sets', every frame decoded. A wall the cameras later see past: under Dynamic
# gone from the frames that see past it, and its blocks freed by the
# fusion.free_after pass due; under Classic its observed blocks kept. On the
# host, a path that is the stream's or the CSV's file refused. The device part
# skips with no device.
add_executable(recon_example_set_codec_test example_set_codec_test.cpp)
target_include_directories(recon_example_set_codec_test
                           PRIVATE ${PROJECT_SOURCE_DIR}/examples/common)
target_link_libraries(
  recon_example_set_codec_test
  PRIVATE volumetric_kit::core_config
          volumetric_kit::core_test_policy
          volumetric_kit::recon_codec
          volumetric_kit::recon_eval
          volumetric_kit::recon_networking
          volumetric_kit::recon_sensor_array
          volumetric_kit::recon_stream
          volumetric_kit::recon_tsdf
          Vulkan::Vulkan)
vr_target_warnings(recon_example_set_codec_test)
add_test(NAME recon_example_set_codec COMMAND recon_example_set_codec_test)

# codec_mesh's sequence (examples/codec_mesh/mesh_sequence.hpp), where the mesh
# example is built. On the host: the frame-number pattern, every file checked
# before any load, one transform for the whole range, signed mode's rule and
# source.allow_holes, and inspection. On a device: each frame converted into a
# cleared grid, and code_frame's records and the transform in the stream.
# examples/ defines vr_example_mesh after this directory. The device part skips
# with no device.
if(VR_BUILD_EXAMPLES AND TARGET recon_io_assimp)
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
# explicitly allows partial coverage. Skips (exit 0) with no device.
add_executable(recon_tsdf_fuser_test tsdf_fuser_test.cpp)
target_link_libraries(
  recon_tsdf_fuser_test PRIVATE volumetric_kit::core_test_policy
                                volumetric_kit::recon_tsdf Vulkan::Vulkan)
vr_target_warnings(recon_tsdf_fuser_test)
add_test(NAME recon_tsdf_fuser COMMAND recon_tsdf_fuser_test)

# Host OOM stays in Status, resize rolls back, and private live backoff expires.
add_executable(recon_tsdf_fuser_oom_test tsdf_fuser_oom_test.cpp
                                         test_allocation_failure.cpp)
target_link_libraries(
  recon_tsdf_fuser_oom_test PRIVATE volumetric_kit::core_test_policy
                                    volumetric_kit::recon_tsdf Vulkan::Vulkan)
vr_target_warnings(recon_tsdf_fuser_oom_test)
add_test(NAME recon_tsdf_fuser_oom COMMAND recon_tsdf_fuser_oom_test)

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
# DepthCameraParams scalar-layout upload. Skips where no device is present.
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
# triangle wider than the band leaves no hole (where the point path does), and
# that the distance prune keeps the bounding box's far corners out. Skips where
# no device is present.
add_executable(recon_volume_allocate_triangles_test
               volume_allocate_triangles_test.cpp)
target_link_libraries(
  recon_volume_allocate_triangles_test
  PRIVATE volumetric_kit::recon_volume Vulkan::Vulkan
          volumetric_kit::core_test_policy)
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
# field meshes identically however it is divided into blocks. Skips where no
# device is present.
add_executable(recon_mesh_marching_cubes_sparse_test
               marching_cubes_sparse_test.cpp)
target_link_libraries(
  recon_mesh_marching_cubes_sparse_test
  PRIVATE volumetric_kit::recon_mesh Vulkan::Vulkan
          volumetric_kit::core_test_policy)
vr_target_warnings(recon_mesh_marching_cubes_sparse_test)
add_test(NAME recon_mesh_marching_cubes_sparse
         COMMAND recon_mesh_marching_cubes_sparse_test)

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
# nothing, the latch, and a seeded run with failures in which no slot the
# consumer holds is reclaimed. Host only, so it always runs.
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

include(vr_shaders)

# Color-space vocabulary: the exact piecewise sRGB curve, the packed helpers the
# fusion kernels mirror, and the primaries matrices. Pure host math over core,
# so it always runs -- no device, no platform. That portability is the reason
# the curve lives in `core` rather than in whichever tier decodes with it.
add_executable(recon_core_color_space_test core_color_space_test.cpp)
target_link_libraries(recon_core_color_space_test
                      PRIVATE volumetric_kit::recon_core)
vr_target_warnings(recon_core_color_space_test)
add_test(NAME recon_core_color_space COMMAND recon_core_color_space_test)

# ...and the device half of the same claim: run core/shaders/color_common.glsl
# over all 256 codes on the real driver and compare against the host curve. Two
# implementations of one transfer function is the silent drift the 2026-08-02
# decision exists to prevent, so it is pinned rather than trusted. Skips where
# no device is present.
add_executable(recon_core_color_space_gpu_test core_color_space_gpu_test.cpp)
vr_compile_shaders(recon_core_color_space_gpu_test SHADERS
                   "${CMAKE_CURRENT_SOURCE_DIR}/shaders/color_parity.comp")
target_compile_definitions(
  recon_core_color_space_gpu_test
  PRIVATE
    VR_COLOR_PARITY_SPV="${CMAKE_CURRENT_BINARY_DIR}/shaders/color_parity.comp.spv"
)
target_link_libraries(
  recon_core_color_space_gpu_test
  PRIVATE volumetric_kit::recon_core Vulkan::Vulkan
          volumetric_kit::core_test_policy)
vr_target_warnings(recon_core_color_space_gpu_test)
add_test(NAME recon_core_color_space_gpu
         COMMAND recon_core_color_space_gpu_test)

# core/shaders/pinhole.glsl, the GLSL pinhole recon exports, against the host's
# arithmetic in double (shaders/pinhole_cases.comp, std430 as a consumer's
# kernel): written-out edges in exact arithmetic, and a grid around five
# cameras. Skips where no device is present.
add_executable(recon_core_pinhole_gpu_test core_pinhole_gpu_test.cpp)
vr_embed_shaders(recon_core_pinhole_gpu_test SHADERS
                 "${CMAKE_CURRENT_SOURCE_DIR}/shaders/pinhole_cases.comp")
target_link_libraries(
  recon_core_pinhole_gpu_test
  PRIVATE volumetric_kit::recon_core volumetric_kit::recon_camera
          Vulkan::Vulkan volumetric_kit::core_test_policy)
vr_target_warnings(recon_core_pinhole_gpu_test)
add_test(NAME recon_core_pinhole_gpu COMMAND recon_core_pinhole_gpu_test)

# sensor tier: the camera-convention conversions -- the OpenGL/ARKit -> CV pose
# change of basis and the registered-depth intrinsics rescale. These are the two
# things a capture integration gets *silently* wrong (a flipped axis or a
# half-pixel bias misaligns the reconstruction without ever erroring), and they
# are pure host math, so they always run rather than being gated on a device or
# on the platform whose driver produced the numbers.
add_executable(recon_sensor_conventions_test sensor_conventions_test.cpp)
target_link_libraries(recon_sensor_conventions_test
                      PRIVATE volumetric_kit::recon_sensor)
vr_target_warnings(recon_sensor_conventions_test)
add_test(NAME recon_sensor_conventions COMMAND recon_sensor_conventions_test)

# sensor tier: the sensor array's trigger grouper, driven by synthetic streams
# -- drops, a silent sensor, a slow poll, a clock off by more than the
# tolerance, a late frame, the start-up frames. Host-only. Internal and
# unexported, so the test compiles its source itself, as the Orbbec tests do.
set(_vr_sensor_array_src
    ${PROJECT_SOURCE_DIR}/src/volumetric_kit/recon/sensor/array)
add_executable(
  recon_sensor_trigger_grouper_test sensor_trigger_grouper_test.cpp
                                    ${_vr_sensor_array_src}/trigger_grouper.cpp)
target_link_libraries(recon_sensor_trigger_grouper_test
                      PRIVATE volumetric_kit::recon_sensor)
target_include_directories(recon_sensor_trigger_grouper_test
                           PRIVATE ${_vr_sensor_array_src})
vr_target_warnings(recon_sensor_trigger_grouper_test)
add_test(NAME recon_sensor_trigger_grouper
         COMMAND recon_sensor_trigger_grouper_test)

# sensor tier: the sensor array over scripted fake sensors -- the arrays it
# refuses, its start order, trigger grouping, poses from the calibration.
# Host-only.
add_executable(recon_sensor_array_test sensor_array_test.cpp)
target_link_libraries(recon_sensor_array_test
                      PRIVATE volumetric_kit::recon_sensor_array)
vr_target_warnings(recon_sensor_array_test)
add_test(NAME recon_sensor_array COMMAND recon_sensor_array_test)

# sensor tier: a sensor array's sets prepared on the GPU in one batch, each
# frame as GpuFramePrep makes it alone and posed by the calibration. Skips
# without a device.
add_executable(recon_sensor_array_process_test sensor_array_process_test.cpp)
target_link_libraries(
  recon_sensor_array_process_test PRIVATE volumetric_kit::recon_sensor_array
                                          volumetric_kit::core_test_policy)
vr_target_warnings(recon_sensor_array_process_test)
add_test(NAME recon_sensor_array_process
         COMMAND recon_sensor_array_process_test)

# sensor tier: RGB-D sensor defaults, frame-return helpers, copied-frame
# ownership and stable enum names. Host-only.
add_executable(recon_sensor_rgbd_sensor_test sensor_rgbd_sensor_test.cpp)
target_link_libraries(recon_sensor_rgbd_sensor_test
                      PRIVATE volumetric_kit::recon_sensor)
vr_target_warnings(recon_sensor_rgbd_sensor_test)
add_test(NAME recon_sensor_rgbd_sensor COMMAND recon_sensor_rgbd_sensor_test)

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
# must write. Test-only; the codec tests that hold the device to it link it, and
# reach the internal headers through it.
add_library(recon_codec_oracle STATIC codec_oracle.cpp codec_oracle_device.cpp)
target_link_libraries(recon_codec_oracle PUBLIC volumetric_kit::recon_codec)
target_include_directories(
  recon_codec_oracle
  PUBLIC ${PROJECT_SOURCE_DIR}/src/volumetric_kit/recon/codec)
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
# refusals passed through. Skips with no device.
add_executable(recon_codec_decode_grid_test codec_decode_grid_test.cpp)
target_link_libraries(
  recon_codec_decode_grid_test
  PRIVATE volumetric_kit::recon_codec Vulkan::Vulkan
          volumetric_kit::core_test_policy)
vr_target_warnings(recon_codec_decode_grid_test)
add_test(NAME recon_codec_decode_grid COMMAND recon_codec_decode_grid_test)

# Allocation injection stays in its own executable: factory diagnostics and a
# metrics merge after a committed P-frame, followed by the next P-frame.
add_executable(recon_codec_decode_grid_oom_test codec_decode_grid_oom_test.cpp
                                                test_allocation_failure.cpp)
target_link_libraries(
  recon_codec_decode_grid_oom_test
  PRIVATE volumetric_kit::recon_codec Vulkan::Vulkan
          volumetric_kit::core_test_policy)
vr_target_warnings(recon_codec_decode_grid_oom_test)
add_test(NAME recon_codec_decode_grid_oom
         COMMAND recon_codec_decode_grid_oom_test)

# Apple's pixel buffers imported as images (recon_sensor_apple, any Apple build,
# with or without FFmpeg): buffers made here, read back in place, prepared as a
# frame's colour, and the refusals. Skips without a device.
if(APPLE)
  add_executable(recon_sensor_apple_pixel_buffer_importer_test
                 sensor_apple_pixel_buffer_importer_test.cpp)
  target_link_libraries(
    recon_sensor_apple_pixel_buffer_importer_test
    PRIVATE volumetric_kit::core_test_policy volumetric_kit::recon_sensor_apple
            volumetric_kit::recon_sensor_utils "-framework CoreFoundation"
            "-framework CoreVideo")
  vr_target_warnings(recon_sensor_apple_pixel_buffer_importer_test)
  add_test(NAME recon_sensor_apple_pixel_buffer_importer
           COMMAND recon_sensor_apple_pixel_buffer_importer_test)
endif()

# The video decoders (VR_WITH_FFMPEG only). The frame colour test reads the
# colour off frames built by hand, no codec, and the FFmpeg error test maps
# error codes; each compiles the internal source itself, as the Orbbec
# conversion test does. The decoder tests decode the committed clips and JPEGs
# on this machine's hardware, onto the device; on a GPU with no device path they
# skip, unless VR_TEST_HEVC_BACKEND says this leg has one, where they fail.
if(VR_WITH_FFMPEG)
  set(_vr_video_src ${PROJECT_SOURCE_DIR}/src/volumetric_kit/recon/sensor/video)
  add_executable(
    recon_sensor_video_frame_color_test sensor_video_frame_color_test.cpp
                                        ${_vr_video_src}/frame_color.cpp)
  target_link_libraries(
    recon_sensor_video_frame_color_test
    PRIVATE volumetric_kit::recon_sensor_video PkgConfig::VR_FFMPEG)
  target_include_directories(recon_sensor_video_frame_color_test
                             PRIVATE ${_vr_video_src})
  vr_target_warnings(recon_sensor_video_frame_color_test)
  add_test(NAME recon_sensor_video_frame_color
           COMMAND recon_sensor_video_frame_color_test)

  add_executable(recon_sensor_video_ffmpeg_error_test
                 sensor_video_ffmpeg_error_test.cpp ${_vr_video_src}/ffmpeg.cpp)
  target_link_libraries(recon_sensor_video_ffmpeg_error_test
                        PRIVATE volumetric_kit::core_base PkgConfig::VR_FFMPEG)
  target_include_directories(recon_sensor_video_ffmpeg_error_test
                             PRIVATE ${_vr_video_src})
  vr_target_warnings(recon_sensor_video_ffmpeg_error_test)
  add_test(NAME recon_sensor_video_ffmpeg_error
           COMMAND recon_sensor_video_ffmpeg_error_test)

  add_executable(recon_sensor_video_hevc_test sensor_video_hevc_test.cpp)
  target_link_libraries(
    recon_sensor_video_hevc_test
    PRIVATE volumetric_kit::recon_sensor_video
            volumetric_kit::recon_sensor_utils volumetric_kit::core_test_policy)
  target_compile_definitions(
    recon_sensor_video_hevc_test
    PRIVATE VR_HEVC_DATA="${CMAKE_CURRENT_SOURCE_DIR}/data/hevc")
  vr_target_warnings(recon_sensor_video_hevc_test)
  add_test(NAME recon_sensor_video_hevc COMMAND recon_sensor_video_hevc_test)

  add_executable(recon_sensor_video_jpeg_test sensor_video_jpeg_test.cpp)
  target_link_libraries(
    recon_sensor_video_jpeg_test
    PRIVATE volumetric_kit::recon_sensor_video
            volumetric_kit::recon_sensor_utils volumetric_kit::core_test_policy)
  target_compile_definitions(
    recon_sensor_video_jpeg_test
    PRIVATE VR_JPEG_DATA="${CMAKE_CURRENT_SOURCE_DIR}/data/jpeg")
  vr_target_warnings(recon_sensor_video_jpeg_test)
  add_test(NAME recon_sensor_video_jpeg COMMAND recon_sensor_video_jpeg_test)

  if(VR_WITH_CUDA)
    # Host-only ownership and allocation-failure checks. The test supplies the
    # driver table: no CUDA device, driver library, or decoder is opened.
    add_executable(recon_sensor_video_cuda_slot_test
                   sensor_video_cuda_slot_test.cpp test_allocation_failure.cpp)
    target_link_libraries(recon_sensor_video_cuda_slot_test
                          PRIVATE volumetric_kit::recon_core)
    target_include_directories(recon_sensor_video_cuda_slot_test
                               PRIVATE ${_vr_video_src})
    target_include_directories(recon_sensor_video_cuda_slot_test SYSTEM
                               PRIVATE ${CUDAToolkit_INCLUDE_DIRS})
    vr_target_warnings(recon_sensor_video_cuda_slot_test)
    add_test(NAME recon_sensor_video_cuda_slot
             COMMAND recon_sensor_video_cuda_slot_test)

    # The Status of a failed driver or nvJPEG call. It compiles the CUDA source
    # itself, which loads libcuda for the error's name and opens no device.
    add_executable(
      recon_sensor_video_cuda_error_test sensor_video_cuda_error_test.cpp
                                         ${_vr_video_src}/cuda_pictures.cpp)
    target_link_libraries(
      recon_sensor_video_cuda_error_test
      PRIVATE volumetric_kit::recon_core volumetric_kit::core_test_policy
              ${CMAKE_DL_LIBS})
    target_include_directories(recon_sensor_video_cuda_error_test
                               PRIVATE ${_vr_video_src})
    target_include_directories(recon_sensor_video_cuda_error_test SYSTEM
                               PRIVATE ${CUDAToolkit_INCLUDE_DIRS})
    vr_target_warnings(recon_sensor_video_cuda_error_test)
    add_test(NAME recon_sensor_video_cuda_error
             COMMAND recon_sensor_video_cuda_error_test)
  endif()

  if(APPLE)
    # Compile the public JPEG decoder here so the executable's native-call
    # replacements intercept it in both static and shared library builds.
    add_executable(
      recon_sensor_video_vt_jpeg_error_test
      sensor_video_vt_jpeg_error_test.cpp ${_vr_video_src}/jpeg_decoder.cpp
      ${_vr_video_src}/vt_jpeg.cpp)
    target_link_libraries(
      recon_sensor_video_vt_jpeg_error_test
      PRIVATE volumetric_kit::recon_sensor_video
              volumetric_kit::recon_sensor_apple
              "-framework VideoToolbox"
              "-framework CoreMedia"
              "-framework CoreVideo"
              volumetric_kit::core_test_policy)
    target_compile_definitions(
      recon_sensor_video_vt_jpeg_error_test
      PRIVATE VR_JPEG_DATA="${CMAKE_CURRENT_SOURCE_DIR}/data/jpeg")
    vr_target_warnings(recon_sensor_video_vt_jpeg_error_test)
    add_test(NAME recon_sensor_video_vt_jpeg_error
             COMMAND recon_sensor_video_vt_jpeg_error_test)
  endif()
endif()

# eval tier: closest points, the reach clamp, the summary statistics, accuracy /
# coverage and the F-score between meshes a known distance apart, and every
# refusal. Host-only.
add_executable(recon_eval_mesh_distance_test eval_mesh_distance_test.cpp)
target_link_libraries(recon_eval_mesh_distance_test
                      PRIVATE volumetric_kit::recon_eval)
vr_target_warnings(recon_eval_mesh_distance_test)
add_test(NAME recon_eval_mesh_distance COMMAND recon_eval_mesh_distance_test)

# The worker each Orbbec colour decoder decodes on (decode_worker.hpp), driven
# by a fake decode. It names no SDK type, so it is tested without the driver
# too, on every leg, the sanitizers' included; with exceptions, as the driver is
# compiled, since it catches what a decode throws.
add_executable(recon_sensor_orbbec_decode_worker_test
               sensor_orbbec_decode_worker_test.cpp)
target_link_libraries(recon_sensor_orbbec_decode_worker_test
                      PRIVATE volumetric_kit::recon_core)
target_include_directories(
  recon_sensor_orbbec_decode_worker_test
  PRIVATE ${PROJECT_SOURCE_DIR}/src/volumetric_kit/recon/sensor/orbbec)
target_compile_options(recon_sensor_orbbec_decode_worker_test
                       PRIVATE -fexceptions)
vr_target_warnings(recon_sensor_orbbec_decode_worker_test)
add_test(NAME recon_sensor_orbbec_decode_worker
         COMMAND recon_sensor_orbbec_decode_worker_test)

# The Orbbec SDK prerequisite (VR_WITH_ORBBEC only): the copy the build found is
# the one that loads -- runtime version == configured version, since the SDK is
# linked in place from an install outside the repo. It opens no SDK context.
if(VR_WITH_ORBBEC)
  add_executable(recon_orbbec_sdk_smoke_test orbbec_sdk_smoke_test.cpp)
  vr_link_orbbec_sdk(recon_orbbec_sdk_smoke_test)
  target_compile_definitions(
    recon_orbbec_sdk_smoke_test
    PRIVATE VR_ORBBEC_SDK_VERSION_MAJOR=${VR_ORBBEC_SDK_VERSION_MAJOR}
            VR_ORBBEC_SDK_VERSION_MINOR=${VR_ORBBEC_SDK_VERSION_MINOR}
            VR_ORBBEC_SDK_VERSION_PATCH=${VR_ORBBEC_SDK_VERSION_PATCH})
  vr_target_warnings(recon_orbbec_sdk_smoke_test)
  add_test(NAME recon_orbbec_sdk_smoke COMMAND recon_orbbec_sdk_smoke_test)

  # The Orbbec driver's arithmetic, with no camera. frame_conversion.hpp is
  # internal and names SDK types, so the test reaches it by include path; its
  # functions are not exported, so the test compiles their source itself rather
  # than linking a shared build's hidden copies.
  add_executable(
    recon_sensor_orbbec_conversion_test
    sensor_orbbec_conversion_test.cpp
    ${PROJECT_SOURCE_DIR}/src/volumetric_kit/recon/sensor/orbbec/frame_conversion.cpp
  )
  target_link_libraries(recon_sensor_orbbec_conversion_test
                        PRIVATE volumetric_kit::recon_sensor_orbbec)
  vr_link_orbbec_sdk(recon_sensor_orbbec_conversion_test)
  target_include_directories(
    recon_sensor_orbbec_conversion_test
    PRIVATE ${PROJECT_SOURCE_DIR}/src/volumetric_kit/recon/sensor/orbbec)
  vr_target_warnings(recon_sensor_orbbec_conversion_test)
  add_test(NAME recon_sensor_orbbec_conversion
           COMMAND recon_sensor_orbbec_conversion_test)

  # The rig's sync configuration: the SDK's file layout, its refusals, and the
  # comparison OrbbecSensor::open makes against what the camera reads back. No
  # camera; compiles frame_conversion.cpp itself, as the conversion test does.
  add_executable(
    recon_sensor_orbbec_sync_config_test
    sensor_orbbec_sync_config_test.cpp
    ${PROJECT_SOURCE_DIR}/src/volumetric_kit/recon/sensor/orbbec/frame_conversion.cpp
  )
  target_link_libraries(recon_sensor_orbbec_sync_config_test
                        PRIVATE volumetric_kit::recon_sensor_orbbec)
  vr_link_orbbec_sdk(recon_sensor_orbbec_sync_config_test)
  target_include_directories(
    recon_sensor_orbbec_sync_config_test
    PRIVATE ${PROJECT_SOURCE_DIR}/src/volumetric_kit/recon/sensor/orbbec)
  target_compile_definitions(
    recon_sensor_orbbec_sync_config_test
    PRIVATE
      VR_RIG_SYNC_CONFIG="${PROJECT_SOURCE_DIR}/config/femto_mega_sync.json")
  vr_target_warnings(recon_sensor_orbbec_sync_config_test)
  add_test(NAME recon_sensor_orbbec_sync_config
           COMMAND recon_sensor_orbbec_sync_config_test)

  # Public open() against a camera stub (orbbec_camera_stub.cpp): SDK
  # normalization after a sync write must be refused. Compile the sensor here so
  # the stub also works in shared builds; no real camera or SDK context is
  # opened.
  add_executable(
    recon_sensor_orbbec_sync_apply_test
    sensor_orbbec_sync_apply_test.cpp
    orbbec_camera_stub.cpp
    ${PROJECT_SOURCE_DIR}/src/volumetric_kit/recon/sensor/orbbec/orbbec_sensor.cpp
    ${PROJECT_SOURCE_DIR}/src/volumetric_kit/recon/sensor/orbbec/frame_conversion.cpp
  )
  target_link_libraries(recon_sensor_orbbec_sync_apply_test
                        PRIVATE volumetric_kit::recon_sensor_orbbec)
  vr_link_orbbec_sdk(recon_sensor_orbbec_sync_apply_test)
  target_include_directories(
    recon_sensor_orbbec_sync_apply_test
    PRIVATE ${PROJECT_SOURCE_DIR}/src/volumetric_kit/recon/sensor/orbbec)
  vr_target_warnings(recon_sensor_orbbec_sync_apply_test)
  add_test(NAME recon_sensor_orbbec_sync_apply
           COMMAND recon_sensor_orbbec_sync_apply_test)

  # A rig opened from its sync file (open_orbbec_sensors) against the same
  # camera stub, with the fixture files in data/orbbec_sync.
  add_executable(
    recon_sensor_orbbec_open_sensors_test
    sensor_orbbec_open_sensors_test.cpp
    orbbec_camera_stub.cpp
    ${PROJECT_SOURCE_DIR}/src/volumetric_kit/recon/sensor/orbbec/orbbec_sensor.cpp
    ${PROJECT_SOURCE_DIR}/src/volumetric_kit/recon/sensor/orbbec/frame_conversion.cpp
  )
  target_link_libraries(recon_sensor_orbbec_open_sensors_test
                        PRIVATE volumetric_kit::recon_sensor_orbbec)
  vr_link_orbbec_sdk(recon_sensor_orbbec_open_sensors_test)
  target_include_directories(
    recon_sensor_orbbec_open_sensors_test
    PRIVATE ${PROJECT_SOURCE_DIR}/src/volumetric_kit/recon/sensor/orbbec)
  target_compile_definitions(
    recon_sensor_orbbec_open_sensors_test
    PRIVATE VR_SYNC_FIXTURES="${CMAKE_CURRENT_SOURCE_DIR}/data/orbbec_sync")
  vr_target_warnings(recon_sensor_orbbec_open_sensors_test)
  add_test(NAME recon_sensor_orbbec_open_sensors
           COMMAND recon_sensor_orbbec_open_sensors_test)

  # The driver's H.265 colour decoder, with no camera: the committed patch clip
  # wrapped in SDK frames. Internal and unexported, so the test compiles its
  # source itself.
  add_executable(
    recon_sensor_orbbec_hevc_test
    sensor_orbbec_hevc_test.cpp
    ${PROJECT_SOURCE_DIR}/src/volumetric_kit/recon/sensor/orbbec/hevc_color.cpp
    ${PROJECT_SOURCE_DIR}/src/volumetric_kit/recon/sensor/orbbec/picture_frames.cpp
  )
  target_link_libraries(
    recon_sensor_orbbec_hevc_test
    PRIVATE volumetric_kit::recon_sensor_orbbec
            volumetric_kit::recon_sensor_video
            volumetric_kit::recon_sensor_utils volumetric_kit::core_test_policy)
  vr_link_orbbec_sdk(recon_sensor_orbbec_hevc_test)
  target_include_directories(
    recon_sensor_orbbec_hevc_test
    PRIVATE ${PROJECT_SOURCE_DIR}/src/volumetric_kit/recon/sensor/orbbec)
  target_compile_definitions(
    recon_sensor_orbbec_hevc_test
    PRIVATE VR_HEVC_DATA="${CMAKE_CURRENT_SOURCE_DIR}/data/hevc")
  vr_target_warnings(recon_sensor_orbbec_hevc_test)
  add_test(NAME recon_sensor_orbbec_hevc COMMAND recon_sensor_orbbec_hevc_test)

  # The MJPEG colour decoder, with no camera: the committed JPEG wrapped in SDK
  # frames.
  add_executable(
    recon_sensor_orbbec_jpeg_test
    sensor_orbbec_jpeg_test.cpp
    ${PROJECT_SOURCE_DIR}/src/volumetric_kit/recon/sensor/orbbec/jpeg_color.cpp
    ${PROJECT_SOURCE_DIR}/src/volumetric_kit/recon/sensor/orbbec/picture_frames.cpp
  )
  target_link_libraries(
    recon_sensor_orbbec_jpeg_test
    PRIVATE volumetric_kit::recon_sensor_orbbec
            volumetric_kit::recon_sensor_video
            volumetric_kit::recon_sensor_utils volumetric_kit::core_test_policy)
  vr_link_orbbec_sdk(recon_sensor_orbbec_jpeg_test)
  target_include_directories(
    recon_sensor_orbbec_jpeg_test
    PRIVATE ${PROJECT_SOURCE_DIR}/src/volumetric_kit/recon/sensor/orbbec)
  target_compile_definitions(
    recon_sensor_orbbec_jpeg_test
    PRIVATE VR_JPEG_DATA="${CMAKE_CURRENT_SOURCE_DIR}/data/jpeg")
  vr_target_warnings(recon_sensor_orbbec_jpeg_test)
  add_test(NAME recon_sensor_orbbec_jpeg COMMAND recon_sensor_orbbec_jpeg_test)
endif()

# Host asset I/O remains testable without creating a Vulkan device.
add_executable(recon_io_image_test io_image_test.cpp
                                   test_allocation_failure.cpp)
target_link_libraries(recon_io_image_test PRIVATE volumetric_kit::recon_io)
vr_target_warnings(recon_io_image_test)
add_test(NAME recon_io_image COMMAND recon_io_image_test)

add_executable(recon_io_ply_test io_ply_test.cpp)
target_link_libraries(recon_io_ply_test PRIVATE volumetric_kit::recon_io)
vr_target_warnings(recon_io_ply_test)
add_test(NAME recon_io_ply COMMAND recon_io_ply_test)

# The opt-in mesh normalization and topology helpers, on synthetic meshes.
add_executable(recon_io_mesh_normalization_test io_mesh_normalization_test.cpp
                                                test_allocation_failure.cpp)
target_link_libraries(recon_io_mesh_normalization_test
                      PRIVATE volumetric_kit::recon_io)
vr_target_warnings(recon_io_mesh_normalization_test)
add_test(NAME recon_io_mesh_normalization
         COMMAND recon_io_mesh_normalization_test)

# io::check_mesh_file, the one home of a mesh path's checks, and load_mesh
# answering through it. Host only; built with the Assimp backend.
if(TARGET recon_io_assimp)
  add_executable(recon_io_mesh_file_test io_mesh_file_test.cpp)
  target_link_libraries(recon_io_mesh_file_test
                        PRIVATE volumetric_kit::recon_io_assimp)
  vr_target_warnings(recon_io_mesh_file_test)
  add_test(NAME recon_io_mesh_file COMMAND recon_io_mesh_file_test)
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

add_executable(recon_stream_footer_test stream_footer_test.cpp
                                        test_allocation_failure.cpp)
target_link_libraries(recon_stream_footer_test
                      PRIVATE volumetric_kit::recon_stream)
# A 64-bit off_t, as the library's, for pwrite and truncate past 4 GiB.
target_compile_definitions(recon_stream_footer_test
                           PRIVATE _FILE_OFFSET_BITS=64)
vr_target_warnings(recon_stream_footer_test)
add_test(NAME recon_stream_footer COMMAND recon_stream_footer_test)

# Public stream calls contain allocation failures in validation, diagnostics,
# saved errors, and index/footer growth. Host-only, including retry behavior.
add_executable(recon_stream_oom_test stream_oom_test.cpp
                                     test_allocation_failure.cpp)
target_link_libraries(recon_stream_oom_test
                      PRIVATE volumetric_kit::recon_stream)
vr_target_warnings(recon_stream_oom_test)
add_test(NAME recon_stream_oom COMMAND recon_stream_oom_test)

# networking: the server's GOP log with no sockets (join, newest key wins at a
# chunk boundary, the GOP limit, a lost copy, no P-frame without its key) and
# the socket calls (each socket's settings read back, no SIGPIPE, the bytes a
# peer has not acknowledged, a wait a signal interrupts) -- internal and
# unexported, so the tests compile their sources themselves -- then the server
# and the client on loopback, 127.0.0.1 port 0 only: the wire against the
# writer's file, joins, newest key wins, the limits, the rules, teardown,
# signals, the client's errors against raw senders, and a stress run for
# ThreadSanitizer. Host-only.
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

# serve_stream's replay (examples/common/stream_replay.hpp): the records from
# the first key, seq and key pts across passes by the writer's rules, the dues,
# damage left out and an allocation failure past it refused. Host only.
add_executable(recon_example_stream_replay_test example_stream_replay_test.cpp
                                                test_allocation_failure.cpp)
target_include_directories(recon_example_stream_replay_test
                           PRIVATE ${PROJECT_SOURCE_DIR}/examples/common)
target_link_libraries(recon_example_stream_replay_test
                      PRIVATE volumetric_kit::recon_stream)
vr_target_warnings(recon_example_stream_replay_test)
add_test(NAME recon_example_stream_replay
         COMMAND recon_example_stream_replay_test)

# The tests whose code runs threads -- a reader shared by two, the sockets', the
# senders' log, the Orbbec decode workers -- which CI builds and runs again
# under ThreadSanitizer (.github/workflows/_build.yml): one target to build
# them.
add_custom_target(recon_tsan_tests)
add_dependencies(
  recon_tsan_tests
  recon_stream_container_test
  recon_networking_gop_log_test
  recon_networking_socket_test
  recon_networking_server_test
  recon_networking_client_test
  recon_networking_stress_test
  recon_example_info_log_test
  recon_sensor_orbbec_decode_worker_test)
if(VR_WITH_ORBBEC)
  add_dependencies(
    recon_tsan_tests recon_sensor_orbbec_jpeg_test
    recon_sensor_orbbec_hevc_test recon_sensor_orbbec_open_sensors_test)
endif()

# What a sibling that fetches recon gets (subproject/): its configure, from a
# clean tree, with each dependency this build fetched as a local source, so it
# needs no network. The defaults hold, and recon refuses a core the sibling made
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

_vr_add_subproject_test(recon_subproject_defaults defaults)
set_tests_properties(recon_subproject_defaults
                     PROPERTIES FIXTURES_SETUP vr_subproject_defaults)
# The sibling compiles a kernel that includes recon's exported GLSL through the
# root recon_core names (glsl_consumer.cmake).
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
set_tests_properties(recon_tsdf_mesh_integrate_exact PROPERTIES COST 56)
set_tests_properties(recon_eval_mesh_distance PROPERTIES COST 55)
set_tests_properties(recon_codec_device_frame PROPERTIES COST 52)
set_tests_properties(recon_example_codec_stream PROPERTIES COST 49)
set_tests_properties(recon_tsdf_mesh_integrate PROPERTIES COST 36)
set_tests_properties(recon_codec_encoder PROPERTIES COST 30)
set_tests_properties(recon_example_set_codec PROPERTIES COST 29)

# A test skips by exiting with the core's test::kSkipExitCode (gpu_test.hpp). A
# hang (a fence never signalled, a deadlocked thread) fails in 300 s rather than
# holding a shared runner to its job's limit; 300 is well above the largest
# COST.
get_property(
  _vr_tests
  DIRECTORY
  PROPERTY TESTS)
set_tests_properties(${_vr_tests} PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 300)
