# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# Every test is a main() of its own on the core's test support: CHECK,
# VKC_TEST_SANITIZED and the scratch directory (core_test_base), in camera-only
# builds too. A GPU test also links core_test_policy, and one that fails or
# measures allocations core_test_allocation_failure.
link_libraries(volumetric_kit::core_test_base)

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

# The rest also take grid's shared helpers (grid_test_support): a grid's blocks
# made and read on the device, a sphere's field, a codec test's device, a
# stage's rows, a .vrst's bytes and loopback serving.
link_libraries(volumetric_kit::grid_test_support)

# The core tier's own pieces: the version and the log seam's "vr" source.
add_executable(recon_core_smoke_test core_smoke_test.cpp)
target_link_libraries(recon_core_smoke_test PRIVATE volumetric_kit::recon_core)
vr_target_warnings(recon_core_smoke_test)
add_test(NAME recon_core_smoke COMMAND recon_core_smoke_test)

# The decoder tests' skips (decoder_test.hpp), as VR_TEST_HEVC_BACKEND says. The
# GPU tests run on the core's harness (core/testing/gpu_test.hpp), which the
# core tests.
add_executable(recon_decoder_test_harness_test decoder_test_harness_test.cpp)
target_link_libraries(
  recon_decoder_test_harness_test PRIVATE volumetric_kit::recon_core
                                          volumetric_kit::core_test_policy)
vr_target_warnings(recon_decoder_test_harness_test)
add_test(NAME recon_decoder_test_harness
         COMMAND recon_decoder_test_harness_test)

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
# published pinhole) on the device against core/shaders/pinhole.glsl's, bit for
# bit, and the host's pinhole in double (shaders/depth_unproject_cases.comp).
# Here in recon, which has both: the kernel reads recon's src/ and grid's, whose
# copy is internal. Skips where no device is present.
add_executable(recon_volume_depth_unproject_test
               volume_depth_unproject_test.cpp)
vkc_embed_shaders(
  recon_volume_depth_unproject_test
  SYMBOL_PREFIX
  vr_
  INCLUDE_DIRS
  "${PROJECT_SOURCE_DIR}/src"
  "${volumetric_kit_grid_SOURCE_DIR}/src"
  SPIRV_VAL_ARGS
  --scalar-block-layout
  SHADERS
  shaders/depth_unproject_cases.comp)
target_link_libraries(
  recon_volume_depth_unproject_test
  PRIVATE volumetric_kit::recon_core Vulkan::Vulkan
          volumetric_kit::core_test_policy)
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

# The sensor boundary's curve (core/color_space.hpp) and the voxel colour
# format's (volume/voxel_color.hpp), held to each other bit for bit. Here in
# tsdf, which links both, so the volume tier takes no colour vocabulary from
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
# make its depth or pixel NaN, through recon's src/ and the GLSL grid_volume
# exports, as the tsdf kernels read them. Skips with no device.
add_executable(recon_tsdf_integrate_test tsdf_integrate_test.cpp)
target_link_libraries(
  recon_tsdf_integrate_test PRIVATE volumetric_kit::recon_tsdf Vulkan::Vulkan
                                    volumetric_kit::core_test_policy)
vkc_embed_shaders(
  recon_tsdf_integrate_test
  SYMBOL_PREFIX
  vr_
  INCLUDE_DIRS
  "${PROJECT_SOURCE_DIR}/src"
  INCLUDE_TARGETS
  volumetric_kit::grid_volume
  SPIRV_VAL_ARGS
  --scalar-block-layout
  SHADERS
  "${CMAKE_CURRENT_SOURCE_DIR}/shaders/project_pinhole.comp")
vr_target_warnings(recon_tsdf_integrate_test)
add_test(NAME recon_tsdf_integrate COMMAND recon_tsdf_integrate_test)

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
               sensor_gpu_frame_prep_oom_test.cpp)
target_link_libraries(
  recon_sensor_gpu_frame_prep_oom_test
  PRIVATE volumetric_kit::core_test_policy volumetric_kit::recon_sensor_utils
          volumetric_kit::core_test_allocation_failure)
vr_target_warnings(recon_sensor_gpu_frame_prep_oom_test)
add_test(NAME recon_sensor_gpu_frame_prep_oom
         COMMAND recon_sensor_gpu_frame_prep_oom_test)

# Submission-loss lifetime fault injection needs the Unix dynamic loader.
if(UNIX)
  add_executable(recon_sensor_gpu_frame_prep_submit_test
                 sensor_gpu_frame_prep_submit_test.cpp)
  target_link_libraries(
    recon_sensor_gpu_frame_prep_submit_test
    PRIVATE volumetric_kit::core_test_policy volumetric_kit::recon_sensor_utils
            ${CMAKE_DL_LIBS})
  vr_target_warnings(recon_sensor_gpu_frame_prep_submit_test)
  add_test(NAME recon_sensor_gpu_frame_prep_submit
           COMMAND recon_sensor_gpu_frame_prep_submit_test)
endif()

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
               sensor_picture_readback_oom_test.cpp)
target_link_libraries(
  recon_sensor_picture_readback_oom_test
  PRIVATE volumetric_kit::recon_sensor_utils volumetric_kit::core_test_policy
          volumetric_kit::core_test_allocation_failure)
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
  PRIVATE volumetric_kit::recon_sender volumetric_kit::recon_texture
          volumetric_kit::core_test_policy)
vr_target_warnings(recon_example_fuse_keyframe_test)
add_test(NAME recon_example_fuse_keyframe
         COMMAND recon_example_fuse_keyframe_test)

# The core parses and validates scenarios. This test pins the fusing examples'
# templates, their codec options through grid's codec_config.hpp among them, and
# their sender and sensor options, without opening a camera or GPU.
add_executable(recon_example_fuse_config_test example_fuse_config_test.cpp)
target_include_directories(recon_example_fuse_config_test
                           PRIVATE ${PROJECT_SOURCE_DIR}/examples/common)
if(TARGET volumetric_kit::recon_sensor_orbbec)
  target_link_libraries(recon_example_fuse_config_test
                        PRIVATE volumetric_kit::recon_sensor_orbbec)
  target_compile_definitions(recon_example_fuse_config_test
                             PRIVATE VR_TEST_ORBBEC_CONFIG=1)
endif()
# fuse_sender.hpp reports through codec_report.hpp, which compares with eval's,
# and codec_config.hpp maps a mesh source into voxelize's MeshSdfParams.
# recon_sender brings the codec stream and the sensor tiers: naming them too
# would repeat their archives on the link line.
target_link_libraries(
  recon_example_fuse_config_test
  PRIVATE grid_example_common volumetric_kit::core_config
          volumetric_kit::grid_eval volumetric_kit::grid_voxelize
          volumetric_kit::recon_sender)
target_compile_definitions(
  recon_example_fuse_config_test
  PRIVATE VR_CONFIG_FIXTURES="${PROJECT_SOURCE_DIR}/config/examples")
vr_target_warnings(recon_example_fuse_config_test)
add_test(NAME recon_example_fuse_config COMMAND recon_example_fuse_config_test)

# fuse_orbbec's cameras (examples/common/fuse_orbbec_sets.hpp), where the driver
# is built: the one camera a calibration file names, and a camera's report. Host
# only.
if(TARGET volumetric_kit::recon_sensor_orbbec)
  add_executable(recon_example_fuse_orbbec_sets_test
                 example_fuse_orbbec_sets_test.cpp)
  target_include_directories(recon_example_fuse_orbbec_sets_test
                             PRIVATE ${PROJECT_SOURCE_DIR}/examples/common)
  target_link_libraries(
    recon_example_fuse_orbbec_sets_test
    PRIVATE grid_example_common volumetric_kit::recon_sensor_orbbec
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
                                      volumetric_kit::recon_image_io)
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
          volumetric_kit::grid_eval volumetric_kit::recon_sender)
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

# Color-space vocabulary: the exact piecewise sRGB curve, the sensor boundary's
# packed helpers, and the primaries matrices. Pure host math over core, so it
# always runs -- no device, no platform. The kernels' colour format is grid's
# volume's (voxel_color.glsl / voxel_color.hpp), pinned by
# grid_volume_voxel_color, and held to this curve by
# recon_tsdf_voxel_color_space.
add_executable(recon_core_color_space_test core_color_space_test.cpp)
target_link_libraries(recon_core_color_space_test
                      PRIVATE volumetric_kit::recon_core)
vr_target_warnings(recon_core_color_space_test)
add_test(NAME recon_core_color_space COMMAND recon_core_color_space_test)

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

# sensor tier: a run's sets over pushed fake sensors -- each waited for, counted
# and cut to the run's length, its pictures let go, each secondary's skew kept,
# the silence limit from the last set and a lone sync secondary named, and the
# array's refusal passed on. Host-only.
add_executable(recon_sensor_array_rig_sets_test sensor_array_rig_sets_test.cpp)
target_link_libraries(recon_sensor_array_rig_sets_test
                      PRIVATE volumetric_kit::recon_sensor_array)
vr_target_warnings(recon_sensor_array_rig_sets_test)
add_test(NAME recon_sensor_array_rig_sets
         COMMAND recon_sensor_array_rig_sets_test)

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

# Apple's pixel buffers imported as images (recon_sensor_apple, any Apple build,
# with or without FFmpeg): buffers made here, read back in place, prepared as a
# frame's colour, what the import allocates in Metal (metal_allocation.mm), and
# the refusals. Skips without a device.
if(APPLE)
  add_executable(
    recon_sensor_apple_pixel_buffer_importer_test
    sensor_apple_pixel_buffer_importer_test.cpp metal_allocation.mm)
  set_source_files_properties(metal_allocation.mm PROPERTIES COMPILE_OPTIONS
                                                             -fobjc-arc)
  target_link_libraries(
    recon_sensor_apple_pixel_buffer_importer_test
    PRIVATE volumetric_kit::core_test_policy
            volumetric_kit::recon_sensor_apple
            volumetric_kit::recon_sensor_utils
            "-framework CoreFoundation"
            "-framework CoreVideo"
            "-framework IOSurface"
            "-framework Metal"
            "-framework Foundation")
  vr_target_warnings(recon_sensor_apple_pixel_buffer_importer_test)
  add_test(NAME recon_sensor_apple_pixel_buffer_importer
           COMMAND recon_sensor_apple_pixel_buffer_importer_test)
endif()

# The video decoders' host pieces, with no codec and no GPU, so on every leg:
# the colour H.273 codes describe, compiling the internal source itself, as the
# Orbbec conversion test does; and the public H.265 syntax
# (recon_sensor_video_syntax) on the committed clips.
set(_vr_video_src ${PROJECT_SOURCE_DIR}/src/volumetric_kit/recon/sensor/video)
add_executable(
  recon_sensor_video_frame_color_test sensor_video_frame_color_test.cpp
                                      ${_vr_video_src}/frame_color.cpp)
target_link_libraries(recon_sensor_video_frame_color_test
                      PRIVATE volumetric_kit::recon_sensor)
target_include_directories(recon_sensor_video_frame_color_test
                           PRIVATE ${_vr_video_src})
vr_target_warnings(recon_sensor_video_frame_color_test)
add_test(NAME recon_sensor_video_frame_color
         COMMAND recon_sensor_video_frame_color_test)

add_executable(recon_sensor_video_hevc_syntax_test
               sensor_video_hevc_syntax_test.cpp)
target_link_libraries(
  recon_sensor_video_hevc_syntax_test
  PRIVATE volumetric_kit::recon_sensor_video_syntax
          volumetric_kit::core_test_allocation_failure)
target_compile_definitions(
  recon_sensor_video_hevc_syntax_test
  PRIVATE VR_HEVC_DATA="${CMAKE_CURRENT_SOURCE_DIR}/data/hevc")
vr_target_warnings(recon_sensor_video_hevc_syntax_test)
add_test(NAME recon_sensor_video_hevc_syntax
         COMMAND recon_sensor_video_hevc_syntax_test)

# The video decoders (on Apple, and elsewhere with VR_WITH_FFMPEG). The FFmpeg
# helper test maps error codes and a frame's colour fields, compiling ffmpeg.cpp
# itself. The decoder tests decode the committed clips and JPEGs on this
# machine's hardware, onto the device; on a GPU with no device path they skip,
# unless VR_TEST_HEVC_BACKEND says this leg has one, where they fail.
if(APPLE OR VR_WITH_FFMPEG)
  if(VR_WITH_FFMPEG)
    add_executable(recon_sensor_video_ffmpeg_test sensor_video_ffmpeg_test.cpp
                                                  ${_vr_video_src}/ffmpeg.cpp)
    target_link_libraries(
      recon_sensor_video_ffmpeg_test PRIVATE volumetric_kit::recon_sensor
                                             PkgConfig::VR_FFMPEG)
    target_include_directories(recon_sensor_video_ffmpeg_test
                               PRIVATE ${_vr_video_src})
    vr_target_warnings(recon_sensor_video_ffmpeg_test)
    add_test(NAME recon_sensor_video_ffmpeg
             COMMAND recon_sensor_video_ffmpeg_test)
  endif()

  add_executable(recon_sensor_video_hevc_test sensor_video_hevc_test.cpp)
  target_link_libraries(
    recon_sensor_video_hevc_test
    PRIVATE volumetric_kit::recon_sensor_video
            volumetric_kit::recon_sensor_utils volumetric_kit::core_test_policy)
  target_compile_definitions(
    recon_sensor_video_hevc_test
    PRIVATE VR_HEVC_DATA="${CMAKE_CURRENT_SOURCE_DIR}/data/hevc")
  if(VR_WITH_CUDA)
    # It asks libcuda which primary contexts the decoders hold.
    target_include_directories(recon_sensor_video_hevc_test SYSTEM
                               PRIVATE ${CUDAToolkit_INCLUDE_DIRS})
    target_link_libraries(recon_sensor_video_hevc_test PRIVATE ${CMAKE_DL_LIBS})
  endif()
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
    # The public CUDA interop on this machine's GPU, which skips as the decoder
    # tests do; it calls libcuda through a loader of its own.
    add_executable(recon_sensor_video_cuda_interop_test
                   sensor_video_cuda_interop_test.cpp)
    target_link_libraries(
      recon_sensor_video_cuda_interop_test
      PRIVATE volumetric_kit::recon_sensor_video
              volumetric_kit::core_test_policy ${CMAKE_DL_LIBS})
    vr_target_warnings(recon_sensor_video_cuda_interop_test)
    add_test(NAME recon_sensor_video_cuda_interop
             COMMAND recon_sensor_video_cuda_interop_test)

    # The Status of a failed driver or nvJPEG call. It compiles the driver
    # source itself, which loads libcuda for the error's name and opens no
    # device.
    add_executable(
      recon_sensor_video_cuda_error_test sensor_video_cuda_error_test.cpp
                                         ${_vr_video_src}/cuda_driver.cpp)
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
      ${_vr_video_src}/vt_jpeg.cpp ${_vr_video_src}/videotoolbox.cpp)
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

  # The colour controls open() writes and reads back, against the same camera
  # stub, whose properties go through the driver's own sequence
  # (frame_conversion.cpp's write_color_controls).
  add_executable(
    recon_sensor_orbbec_color_controls_test
    sensor_orbbec_color_controls_test.cpp
    orbbec_camera_stub.cpp
    ${PROJECT_SOURCE_DIR}/src/volumetric_kit/recon/sensor/orbbec/orbbec_sensor.cpp
    ${PROJECT_SOURCE_DIR}/src/volumetric_kit/recon/sensor/orbbec/frame_conversion.cpp
  )
  target_link_libraries(recon_sensor_orbbec_color_controls_test
                        PRIVATE volumetric_kit::recon_sensor_orbbec)
  vr_link_orbbec_sdk(recon_sensor_orbbec_color_controls_test)
  target_include_directories(
    recon_sensor_orbbec_color_controls_test
    PRIVATE ${PROJECT_SOURCE_DIR}/src/volumetric_kit/recon/sensor/orbbec)
  vr_target_warnings(recon_sensor_orbbec_color_controls_test)
  add_test(NAME recon_sensor_orbbec_color_controls
           COMMAND recon_sensor_orbbec_color_controls_test)

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

# Host image I/O remains testable without creating a Vulkan device: on the
# family core's base tier, with no mesh tier and no Vulkan (subproject/ checks
# its links).
add_executable(recon_io_image_test io_image_test.cpp)
target_link_libraries(
  recon_io_image_test PRIVATE volumetric_kit::recon_image_io
                              volumetric_kit::core_test_allocation_failure)
vr_target_warnings(recon_io_image_test)
add_test(NAME recon_io_image COMMAND recon_io_image_test)

if(TARGET grid_mesh_io_assimp)
  # grid's mesh_io::load_textured_mesh on a procedural OBJ + MTL + PNG cube it
  # writes, the PNG with recon's image I/O; built with the Assimp backend.
  add_executable(recon_io_textured_mesh_test io_textured_mesh_test.cpp)
  target_link_libraries(
    recon_io_textured_mesh_test PRIVATE volumetric_kit::grid_mesh_io_assimp
                                        volumetric_kit::recon_image_io)
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
          volumetric_kit::grid_mesh volumetric_kit::recon_sensor_array
          Vulkan::Vulkan)
vr_target_warnings(recon_sender_set_sender_test)
add_test(NAME recon_sender_set_sender COMMAND recon_sender_set_sender_test)

# The tests whose code runs threads -- grid's (grid_tsan_tests: a reader shared
# by two, the mesh handoff's producer and consumer, the sockets', the senders'
# log) and the Orbbec decode workers -- which CI builds and runs again under
# ThreadSanitizer (.github/workflows/_build.yml): one target to build them.
add_custom_target(recon_tsan_tests)
add_dependencies(recon_tsan_tests grid_tsan_tests
                 recon_sensor_orbbec_decode_worker_test)
if(VR_WITH_ORBBEC)
  add_dependencies(
    recon_tsan_tests recon_sensor_orbbec_jpeg_test
    recon_sensor_orbbec_hevc_test recon_sensor_orbbec_open_sensors_test)
endif()

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
# The sibling compiles a kernel that includes recon's exported GLSL through the
# root recon_core names, which holds exactly what it exports
# (glsl_consumer.cmake).
add_test(NAME recon_subproject_glsl
         COMMAND "${CMAKE_COMMAND}" --build "${_vr_subproject}/defaults"
                 --target vr_glsl_consumer)
set_tests_properties(recon_subproject_glsl PROPERTIES FIXTURES_REQUIRED
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
# On Apple the decoders use VideoToolbox directly, so VR_WITH_FFMPEG is refused
# with the fix (cmake/vr_ffmpeg.cmake).
if(APPLE)
  _vr_add_subproject_test(recon_subproject_apple_ffmpeg apple_ffmpeg
                          -DVR_WITH_FFMPEG=ON)
  _vr_expect_message(
    recon_subproject_apple_ffmpeg
    "on Apple the video decoders use VideoToolbox directly and need no FFmpeg. Configure this build with -DVR_WITH_FFMPEG=OFF."
  )
endif()

# Start the slowest tests first under `ctest --parallel`. Without a COST, CTest
# runs tests in the order they were added, and a fresh CI runner has no timing
# history, so a slow test added late finished last. Seconds under the sanitizers
# job, the longest in CI, in run 39717939263 (before the 2026-10-22 test trims);
# only the order matters. grid's tests set theirs (grid/tests).
set_tests_properties(recon_sender_set_sender PROPERTIES COST 29)
# Waits out RigSets' 10-s silence limit, under any build.
set_tests_properties(recon_sensor_array_rig_sets PROPERTIES COST 11)

# A test skips by exiting with the core's test::kSkipExitCode (gpu_test.hpp). A
# hang (a fence never signalled, a deadlocked thread) fails in 300 s rather than
# holding a shared runner to its job's limit; 300 is well above the largest
# COST.
get_property(
  _vr_tests
  DIRECTORY
  PROPERTY TESTS)
set_tests_properties(${_vr_tests} PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 300)
