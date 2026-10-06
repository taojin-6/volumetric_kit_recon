# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# Smoke test: compiles the core public headers (Status/Result, POD math,
# version) and links recon_core, proving the tier builds, links, and runs under
# -Werror. A full GoogleTest suite replaces this as the tiers land (see the
# DESIGN.md core implementation and AGENTS.md validation rules).
add_executable(recon_core_smoke_test core_smoke_test.cpp)
target_link_libraries(recon_core_smoke_test PRIVATE volumetric_kit::recon_core)
vr_target_warnings(recon_core_smoke_test)
add_test(NAME recon_core_smoke COMMAND recon_core_smoke_test)

# Vulkan availability smoke: proves the build finds Vulkan and the platform
# driver (MoltenVK on Apple) offers a device that meets recon's requirements
# (core/device_requirements.hpp). The Vulkan foundation itself -- device,
# allocator, buffers, kernels, batches, timers, external memory -- is
# volumetric_kit_core's and is tested there, not here. Exits 0 (skip) where no
# driver/device is present.
add_executable(recon_vulkan_smoke_test vulkan_smoke_test.cpp)
target_link_libraries(recon_vulkan_smoke_test PRIVATE volumetric_kit::recon_core
                                                      Vulkan::Vulkan)
vr_target_warnings(recon_vulkan_smoke_test)
add_test(NAME recon_vulkan_smoke COMMAND recon_vulkan_smoke_test)

# Every tier reports, and reports the DEVICE half. The load-bearing assertion is
# per row: gpu_ms < cpu_ms, since the host span wraps the submit the device span
# sits inside -- equal numbers mean the wall clock was relabelled. Also pins
# that a null out-param measures nothing, and that publishing ends the timer
# window.
add_executable(recon_tier_stage_metrics_test tier_stage_metrics_test.cpp)
target_link_libraries(
  recon_tier_stage_metrics_test
  PRIVATE volumetric_kit::recon_texture volumetric_kit::recon_tsdf
          volumetric_kit::recon_volume volumetric_kit::recon_core)
vr_target_warnings(recon_tier_stage_metrics_test)
add_test(NAME recon_tier_stage_metrics COMMAND recon_tier_stage_metrics_test)

# volume tier: the sparse voxel-hash POD layouts. The static_assert size/offset
# guards in hash_types.hpp are the real test (a build failure there is the
# check); this binary adds runtime sanity and proves the header compiles and
# links through recon_volume.
add_executable(recon_volume_types_test volume_types_test.cpp)
target_link_libraries(recon_volume_types_test
                      PRIVATE volumetric_kit::recon_volume)
vr_target_warnings(recon_volume_types_test)
add_test(NAME recon_volume_types COMMAND recon_volume_types_test)

# volume coordinate math + spatial hash: pure host transforms (world <-> voxel
# <-> block, negative-coordinate block indexing, truncation band) and the bucket
# hash function. CPU-only (no device), so these always run -- not gated on a
# Vulkan driver.
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
# compact, and verify the active set + the HashEntry scalar-layout round-trip on
# the real driver (MoltenVK / NVIDIA). Skips (exit 0) where no device is
# present.
add_executable(recon_volume_hash_map_test volume_hash_map_test.cpp)
target_link_libraries(recon_volume_hash_map_test
                      PRIVATE volumetric_kit::recon_volume Vulkan::Vulkan)
vr_target_warnings(recon_volume_hash_map_test)
add_test(NAME recon_volume_hash_map COMMAND recon_volume_hash_map_test)

# volume block grid: declare independent SoA per-voxel attributes, verify they
# are distinct + correctly-sized buffers keyed by the block pool, the composed
# map allocates, and the error / move paths. Skips (exit 0) with no device.
add_executable(recon_volume_block_grid_test volume_block_grid_test.cpp)
target_link_libraries(recon_volume_block_grid_test
                      PRIVATE volumetric_kit::recon_volume Vulkan::Vulkan)
vr_target_warnings(recon_volume_block_grid_test)
add_test(NAME recon_volume_block_grid COMMAND recon_volume_block_grid_test)

# volume block stamps: every allocation path stamps the blocks it asks for with
# the map's tick, the block pass stamps those holding weight, and
# free_stale_blocks frees those neither for more than max_age ticks; resize
# keeps the records, remove and clear zero them. Skips (exit 0) with no device.
add_executable(recon_volume_block_stamps_test volume_block_stamps_test.cpp)
target_link_libraries(recon_volume_block_stamps_test
                      PRIVATE volumetric_kit::recon_volume Vulkan::Vulkan)
vr_target_warnings(recon_volume_block_stamps_test)
add_test(NAME recon_volume_block_stamps COMMAND recon_volume_block_stamps_test)

# tsdf integrate: fuse a synthetic constant-depth plane into a VoxelBlockGrid
# and verify the classic projective TSDF math per voxel (sdf, truncation, weight
# + cap). Skips (exit 0) with no device.
add_executable(recon_tsdf_integrate_test tsdf_integrate_test.cpp)
target_link_libraries(recon_tsdf_integrate_test
                      PRIVATE volumetric_kit::recon_tsdf Vulkan::Vulkan)
vr_target_warnings(recon_tsdf_integrate_test)
add_test(NAME recon_tsdf_integrate COMMAND recon_tsdf_integrate_test)

# tsdf mesh integrate: write a mesh's truncated distance field (signed through
# pseudonormals, or as a shell) and check every voxel of every written block
# against a brute-force host reference -- an independent closest point, and the
# winding number for inside/outside -- on a sharp tetrahedron, a dented cube, an
# open quad and a triangle soup, plus determinism and every refusal. Skips (exit
# 0) with no device.
add_executable(recon_tsdf_mesh_integrate_test tsdf_mesh_integrate_test.cpp)
target_link_libraries(recon_tsdf_mesh_integrate_test
                      PRIVATE volumetric_kit::recon_tsdf Vulkan::Vulkan)
vr_target_warnings(recon_tsdf_mesh_integrate_test)
add_test(NAME recon_tsdf_mesh_integrate COMMAND recon_tsdf_mesh_integrate_test)
# GPU pre-processing (recon_sensor_utils) on synthetic frames: undistortion in
# the right direction, the colour matrix, a host reference of the sampling, the
# refusals, and fusing the result. Skips (exit 0) where no device is present.
add_executable(recon_sensor_gpu_frame_prep_test sensor_gpu_frame_prep_test.cpp)
target_link_libraries(
  recon_sensor_gpu_frame_prep_test PRIVATE volumetric_kit::recon_sensor_utils
                                           volumetric_kit::recon_tsdf)
vr_target_warnings(recon_sensor_gpu_frame_prep_test)
add_test(NAME recon_sensor_gpu_frame_prep
         COMMAND recon_sensor_gpu_frame_prep_test)

# The device-input overloads of allocate_from_depth and integrate: the same
# frame fused from host arrays and from storage buffers must give the same grid.
# Skips (exit 0) where no device is present.
add_executable(recon_tsdf_device_input_test tsdf_device_input_test.cpp)
target_link_libraries(recon_tsdf_device_input_test
                      PRIVATE volumetric_kit::recon_tsdf Vulkan::Vulkan)
vr_target_warnings(recon_tsdf_device_input_test)
add_test(NAME recon_tsdf_device_input COMMAND recon_tsdf_device_input_test)

# tsdf integrate set: several cameras allocated and fused in one call each must
# give the grid fusing them one at a time gives, bit for bit. Skips (exit 0)
# where no device is present.
add_executable(recon_tsdf_integrate_set_test tsdf_integrate_set_test.cpp)
target_link_libraries(recon_tsdf_integrate_set_test
                      PRIVATE volumetric_kit::recon_tsdf Vulkan::Vulkan)
vr_target_warnings(recon_tsdf_integrate_set_test)
add_test(NAME recon_tsdf_integrate_set COMMAND recon_tsdf_integrate_set_test)

# volume delete: allocate a cube, remove a subset, verify the survivors + heap
# reuse (chain splice / successor pull-up) on the real driver. Skips (exit 0)
# where no device is present.
add_executable(recon_volume_delete_test volume_delete_test.cpp)
target_link_libraries(recon_volume_delete_test
                      PRIVATE volumetric_kit::recon_volume Vulkan::Vulkan)
vr_target_warnings(recon_volume_delete_test)
add_test(NAME recon_volume_delete COMMAND recon_volume_delete_test)

# volume resize: allocate, grow the table to more buckets, and verify the active
# set survives + further allocation past the old capacity works. Skips (exit 0)
# with no device.
add_executable(recon_volume_resize_test volume_resize_test.cpp)
target_link_libraries(recon_volume_resize_test
                      PRIVATE volumetric_kit::recon_volume Vulkan::Vulkan)
vr_target_warnings(recon_volume_resize_test)
add_test(NAME recon_volume_resize COMMAND recon_volume_resize_test)

# volume diagnostics: force a collision chain (coords hashing to one bucket) and
# verify the reported active / overflow / chain-length + heap stats. Skips (exit
# 0) where no device is present.
add_executable(recon_volume_diagnostics_test volume_diagnostics_test.cpp)
target_link_libraries(recon_volume_diagnostics_test
                      PRIVATE volumetric_kit::recon_volume Vulkan::Vulkan)
vr_target_warnings(recon_volume_diagnostics_test)
add_test(NAME recon_volume_diagnostics COMMAND recon_volume_diagnostics_test)

# volume allocate-from-depth / -points: unproject a synthetic one-pixel depth
# frame (and a small point set) and verify exactly the expected (2*tb+1)^3
# truncation-band block cube is allocated on the real driver -- the camera
# unprojection + band dilation, plus the DepthCameraParams scalar-layout upload.
# Skips (exit 0) where no device is present.
add_executable(recon_volume_allocate_test volume_allocate_test.cpp)
target_link_libraries(recon_volume_allocate_test
                      PRIVATE volumetric_kit::recon_volume Vulkan::Vulkan)
vr_target_warnings(recon_volume_allocate_test)
add_test(NAME recon_volume_allocate COMMAND recon_volume_allocate_test)

# volume triangle-mesh allocation: allocate from a small mesh, then verify on
# the real driver that every block holding a voxel within trunc_dist is present
# -- brute-forced on the host against an independent closest-point -- that a
# triangle wider than the band leaves no hole (where the point path does), and
# that the distance prune keeps the bounding box's far corners out. Skips (exit
# 0) where no device is present.
add_executable(recon_volume_allocate_triangles_test
               volume_allocate_triangles_test.cpp)
target_link_libraries(recon_volume_allocate_triangles_test
                      PRIVATE volumetric_kit::recon_volume Vulkan::Vulkan)
vr_target_warnings(recon_volume_allocate_triangles_test)
add_test(NAME recon_volume_allocate_triangles
         COMMAND recon_volume_allocate_triangles_test)

# volume frustum-culled compaction: allocate blocks at known world positions
# (one dead-centre, others behind / beyond-far / off to the side), then verify
# compact_active_blocks_in_frustum returns only the in-view block while plain
# compaction returns all -- proving the plane math + the on-device AABB cull.
# Skips (exit 0) where no device is present.
add_executable(recon_volume_frustum_test volume_frustum_test.cpp)
target_link_libraries(recon_volume_frustum_test
                      PRIVATE volumetric_kit::recon_volume Vulkan::Vulkan)
vr_target_warnings(recon_volume_frustum_test)
add_test(NAME recon_volume_frustum COMMAND recon_volume_frustum_test)

# Sparse marching cubes: mesh an analytic sphere written into a real multi-block
# VoxelBlockGrid, exercising cross-block neighbour sampling, and prove the same
# field meshes identically however it is divided into blocks. Skips (exit 0)
# where no device is present.
add_executable(recon_mesh_marching_cubes_sparse_test
               marching_cubes_sparse_test.cpp)
target_link_libraries(recon_mesh_marching_cubes_sparse_test
                      PRIVATE volumetric_kit::recon_mesh Vulkan::Vulkan)
vr_target_warnings(recon_mesh_marching_cubes_sparse_test)
add_test(NAME recon_mesh_marching_cubes_sparse
         COMMAND recon_mesh_marching_cubes_sparse_test)

# mesh MarchingCubesConfig: prove a consumer's extra buffer usage reaches the
# vertex arena and index run -- on the first allocation AND across a grow -- and
# that an unsupported bit is rejected at create(). recon itself never sets one,
# so this is the only place the non-zero path runs. Skips (exit 0) with no
# device.
add_executable(recon_mesh_marching_cubes_config_test
               marching_cubes_config_test.cpp)
target_link_libraries(recon_mesh_marching_cubes_config_test
                      PRIVATE volumetric_kit::recon_mesh Vulkan::Vulkan)
vr_target_warnings(recon_mesh_marching_cubes_config_test)
add_test(NAME recon_mesh_marching_cubes_config
         COMMAND recon_mesh_marching_cubes_config_test)

# texture projective: texture a mesh of three triangles (one the camera sees
# head-on, one occluded by a closer depth reading, one behind the camera) and
# verify the visible triangle gets the exact projected atlas UVs while the
# others keep the (-1,-1) sentinel; a closer depth map reverts the textured
# triangle, and a translated pose shifts the projection (the world->camera
# transform). Skips (exit 0) where no device is present.
add_executable(recon_texture_projective_test texture_projective_test.cpp)
target_link_libraries(recon_texture_projective_test
                      PRIVATE volumetric_kit::recon_texture Vulkan::Vulkan)
vr_target_warnings(recon_texture_projective_test)
add_test(NAME recon_texture_projective COMMAND recon_texture_projective_test)

# Several views into an atlas, on a synthetic scene: each triangle takes the
# view facing it most squarely among those that see it whole, inside that view's
# tile; the atlas layout and packing; the refusals. Skips where no device is
# present.
add_executable(recon_texture_multiview_test texture_multiview_test.cpp)
target_link_libraries(recon_texture_multiview_test
                      PRIVATE volumetric_kit::recon_texture Vulkan::Vulkan)
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
          Vulkan::Vulkan)
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
# decision exists to prevent, so it is pinned rather than trusted. Skips (exit
# 0) where no device is present.
add_executable(recon_core_color_space_gpu_test core_color_space_gpu_test.cpp)
vr_compile_shaders(recon_core_color_space_gpu_test SHADERS
                   "${CMAKE_CURRENT_SOURCE_DIR}/shaders/color_parity.comp")
target_compile_definitions(
  recon_core_color_space_gpu_test
  PRIVATE
    VR_COLOR_PARITY_SPV="${CMAKE_CURRENT_BINARY_DIR}/shaders/color_parity.comp.spv"
)
target_link_libraries(recon_core_color_space_gpu_test
                      PRIVATE volumetric_kit::recon_core Vulkan::Vulkan)
vr_target_warnings(recon_core_color_space_gpu_test)
add_test(NAME recon_core_color_space_gpu
         COMMAND recon_core_color_space_gpu_test)

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

# camera tier: the camera model (its checks, the lens template against OpenCV's
# numbers), its geometry (rigid checks, Rodrigues against OpenCV), and the
# sensor array's calibration file (poses read, the refusals, a file in the build
# tree). Host-only, so they always run.
add_executable(recon_camera_model_test camera_model_test.cpp)
# recon_sensor brings recon_camera: the test compares the sensor tier's float
# lens with the camera model.
target_link_libraries(recon_camera_model_test
                      PRIVATE volumetric_kit::recon_sensor)
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
target_compile_definitions(
  recon_camera_array_calibration_test
  PRIVATE VR_TEST_SCRATCH_DIR="${CMAKE_CURRENT_BINARY_DIR}")
vr_target_warnings(recon_camera_array_calibration_test)
add_test(NAME recon_camera_array_calibration
         COMMAND recon_camera_array_calibration_test)

# codec tier, host half: CodecParams validation and the DCT's two tables (basis
# orthonormality, the zigzag order against the prior engine's). CPU-only, so it
# always runs. The tables are internal (non-installed), reached by include path.
add_executable(recon_codec_params_test codec_params_test.cpp)
target_link_libraries(recon_codec_params_test
                      PRIVATE volumetric_kit::recon_codec)
target_include_directories(
  recon_codec_params_test
  PRIVATE ${PROJECT_SOURCE_DIR}/src/volumetric_kit/recon/codec)
vr_target_warnings(recon_codec_params_test)
add_test(NAME recon_codec_params COMMAND recon_codec_params_test)

# codec tier, GPU half: the internal DctTransform's forward kernel against a
# double-precision reference, the orthonormal reconstruction bounds, the
# observed mask, batching past one dispatch, the refusals and the moves. Skips
# (exit 0) with no device.
add_executable(recon_codec_dct_test codec_dct_test.cpp)
target_link_libraries(recon_codec_dct_test PRIVATE volumetric_kit::recon_codec
                                                   Vulkan::Vulkan)
target_include_directories(
  recon_codec_dct_test
  PRIVATE ${PROJECT_SOURCE_DIR}/src/volumetric_kit/recon/codec)
vr_target_warnings(recon_codec_dct_test)
add_test(NAME recon_codec_dct COMMAND recon_codec_dct_test)

# codec tier, the device frame writer and reader: every frame the writer writes
# is the host writer's, byte for byte, across block counts, K, segment sizes and
# int32 coordinate extremes, and every frame the reader reads, or corrupt
# payload it refuses, is the host reader's. Skips without a device.
add_executable(recon_codec_device_frame_test codec_device_frame_test.cpp)
target_link_libraries(recon_codec_device_frame_test
                      PRIVATE volumetric_kit::recon_codec Vulkan::Vulkan)
target_include_directories(
  recon_codec_device_frame_test
  PRIVATE ${PROJECT_SOURCE_DIR}/src/volumetric_kit/recon/codec)
vr_target_warnings(recon_codec_device_frame_test)
add_test(NAME recon_codec_device_frame COMMAND recon_codec_device_frame_test)

# codec tier, entropy coding: the rANS reference coder (normalization, round
# trips, a probability-one symbol's zero cost, size against the ideal code
# length, truncation and bit flips) and the v1 intra frame built on it (round
# trips, every writer and reader refusal, the section rules, corruption, and an
# all-zero frame's cost). Both host-only, so they always run; both reach the
# internal headers by include path, as recon_codec_params_test does.
foreach(_vr_codec_host_test rans bitstream)
  add_executable(recon_codec_${_vr_codec_host_test}_test
                 codec_${_vr_codec_host_test}_test.cpp)
  target_link_libraries(recon_codec_${_vr_codec_host_test}_test
                        PRIVATE volumetric_kit::recon_codec)
  target_include_directories(
    recon_codec_${_vr_codec_host_test}_test
    PRIVATE ${PROJECT_SOURCE_DIR}/src/volumetric_kit/recon/codec)
  vr_target_warnings(recon_codec_${_vr_codec_host_test}_test)
  add_test(NAME recon_codec_${_vr_codec_host_test}
           COMMAND recon_codec_${_vr_codec_host_test}_test)
endforeach()

# codec tier, public API: the Encoder (an empty grid, bytes independent of the
# hash table's order, unobserved blocks left out, its rows, refusals, moves) and
# the Decoder with the codec end to end -- a decoded grid within the orthonormal
# bound of its source, meshing to the analytic sphere it came from, frame after
# frame into one grid, untouched on every early refusal, and OutOfMemory that a
# resize recovers. Public headers only, plus the shared codec_fixture.hpp; the
# decoder test links recon_mesh, which the tier may not. Both skip (exit 0) with
# no device.
add_executable(recon_codec_encoder_test codec_encoder_test.cpp)
target_link_libraries(recon_codec_encoder_test
                      PRIVATE volumetric_kit::recon_codec Vulkan::Vulkan)
vr_target_warnings(recon_codec_encoder_test)
add_test(NAME recon_codec_encoder COMMAND recon_codec_encoder_test)

add_executable(recon_codec_decoder_test codec_decoder_test.cpp)
target_link_libraries(
  recon_codec_decoder_test PRIVATE volumetric_kit::recon_codec
                                   volumetric_kit::recon_mesh Vulkan::Vulkan)
vr_target_warnings(recon_codec_decoder_test)
add_test(NAME recon_codec_decoder COMMAND recon_codec_decoder_test)

# The video decoder (VR_WITH_FFMPEG only). The converter test builds frames by
# hand and the back-end test asks the platform, no decoding; both compile the
# internal source themselves, as the Orbbec conversion test does. The decoder
# test decodes the committed clips on software and on every hardware back end
# found; VR_TEST_HEVC_BACKEND makes one required. The JPEG decoder test decodes
# the committed JPEGs, on nvJPEG too in a VR_WITH_CUDA build.
if(VR_WITH_FFMPEG)
  set(_vr_video_src ${PROJECT_SOURCE_DIR}/src/volumetric_kit/recon/sensor/video)
  add_executable(
    recon_sensor_video_converter_test
    sensor_video_converter_test.cpp ${_vr_video_src}/picture_converter.cpp
    ${_vr_video_src}/ffmpeg.cpp)
  target_link_libraries(
    recon_sensor_video_converter_test PRIVATE volumetric_kit::recon_sensor_video
                                              PkgConfig::VR_FFMPEG)
  target_include_directories(recon_sensor_video_converter_test
                             PRIVATE ${_vr_video_src})
  vr_target_warnings(recon_sensor_video_converter_test)
  add_test(NAME recon_sensor_video_converter
           COMMAND recon_sensor_video_converter_test)

  add_executable(
    recon_sensor_video_backend_test
    sensor_video_backend_test.cpp ${_vr_video_src}/hw_backend.cpp
    ${_vr_video_src}/ffmpeg.cpp)
  target_link_libraries(
    recon_sensor_video_backend_test PRIVATE volumetric_kit::recon_sensor_video
                                            PkgConfig::VR_FFMPEG)
  if(APPLE)
    target_link_libraries(recon_sensor_video_backend_test
                          PRIVATE "-framework VideoToolbox")
  endif()
  target_include_directories(recon_sensor_video_backend_test
                             PRIVATE ${_vr_video_src})
  vr_target_warnings(recon_sensor_video_backend_test)
  add_test(NAME recon_sensor_video_backend
           COMMAND recon_sensor_video_backend_test)

  add_executable(recon_sensor_video_hevc_test sensor_video_hevc_test.cpp)
  target_link_libraries(recon_sensor_video_hevc_test
                        PRIVATE volumetric_kit::recon_sensor_video)
  target_compile_definitions(
    recon_sensor_video_hevc_test
    PRIVATE VR_HEVC_DATA="${CMAKE_CURRENT_SOURCE_DIR}/data/hevc"
            VR_TEST_WITH_CUDA=$<BOOL:${VR_WITH_CUDA}>)
  vr_target_warnings(recon_sensor_video_hevc_test)
  add_test(NAME recon_sensor_video_hevc COMMAND recon_sensor_video_hevc_test)

  add_executable(recon_sensor_video_jpeg_test sensor_video_jpeg_test.cpp)
  target_link_libraries(
    recon_sensor_video_jpeg_test PRIVATE volumetric_kit::recon_sensor_video
                                         volumetric_kit::recon_sensor_utils)
  target_compile_definitions(
    recon_sensor_video_jpeg_test
    PRIVATE VR_JPEG_DATA="${CMAKE_CURRENT_SOURCE_DIR}/data/jpeg"
            VR_TEST_WITH_CUDA=$<BOOL:${VR_WITH_CUDA}>)
  vr_target_warnings(recon_sensor_video_jpeg_test)
  add_test(NAME recon_sensor_video_jpeg COMMAND recon_sensor_video_jpeg_test)
endif()

# eval tier: closest points, the reach clamp, the summary statistics, accuracy /
# coverage and the F-score between meshes a known distance apart, and every
# refusal. Host-only.
add_executable(recon_eval_mesh_distance_test eval_mesh_distance_test.cpp)
target_link_libraries(recon_eval_mesh_distance_test
                      PRIVATE volumetric_kit::recon_eval)
vr_target_warnings(recon_eval_mesh_distance_test)
add_test(NAME recon_eval_mesh_distance COMMAND recon_eval_mesh_distance_test)

# The Orbbec SDK prerequisite (VR_WITH_ORBBEC only): the copy the build found is
# the one that loads -- runtime version == configured version, since the SDK is
# linked in place from an install outside the repo -- and an SDK context opens
# and enumerates devices. Needs no camera (zero devices passes); enumeration
# probes the network for Ethernet devices, so it takes a couple of seconds.
if(VR_WITH_ORBBEC)
  add_executable(recon_orbbec_sdk_smoke_test orbbec_sdk_smoke_test.cpp)
  target_link_libraries(recon_orbbec_sdk_smoke_test PRIVATE ob::OrbbecSDK)
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
  target_link_libraries(
    recon_sensor_orbbec_conversion_test
    PRIVATE volumetric_kit::recon_sensor_orbbec ob::OrbbecSDK)
  target_include_directories(
    recon_sensor_orbbec_conversion_test
    PRIVATE ${PROJECT_SOURCE_DIR}/src/volumetric_kit/recon/sensor/orbbec)
  # Without the decoders, it checks that H.265 colour is refused.
  target_compile_definitions(
    recon_sensor_orbbec_conversion_test
    PRIVATE VR_TEST_HEVC=$<TARGET_EXISTS:recon_sensor_video>)
  vr_target_warnings(recon_sensor_orbbec_conversion_test)
  add_test(NAME recon_sensor_orbbec_conversion
           COMMAND recon_sensor_orbbec_conversion_test)

  # The rig's sync configuration: the SDK's file layout, its refusals, and the
  # comparison OrbbecRig::open makes against what the cameras read back. No
  # camera; compiles frame_conversion.cpp itself, as the conversion test does.
  add_executable(
    recon_sensor_orbbec_sync_config_test
    sensor_orbbec_sync_config_test.cpp
    ${PROJECT_SOURCE_DIR}/src/volumetric_kit/recon/sensor/orbbec/frame_conversion.cpp
  )
  target_link_libraries(
    recon_sensor_orbbec_sync_config_test
    PRIVATE volumetric_kit::recon_sensor_orbbec ob::OrbbecSDK)
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

  # The rig's grouping of frames into sync triggers and its start order, driven
  # by synthetic streams -- drops, a silent camera, a slow poll, a clock off by
  # more than the tolerance, a late frame, the start-up frames. No camera.
  # Internal and unexported, so the test compiles the source itself, as the
  # conversion test does.
  add_executable(
    recon_sensor_orbbec_grouping_test
    sensor_orbbec_grouping_test.cpp
    ${PROJECT_SOURCE_DIR}/src/volumetric_kit/recon/sensor/orbbec/trigger_grouping.cpp
  )
  target_link_libraries(recon_sensor_orbbec_grouping_test
                        PRIVATE volumetric_kit::recon_sensor_orbbec)
  target_include_directories(
    recon_sensor_orbbec_grouping_test
    PRIVATE ${PROJECT_SOURCE_DIR}/src/volumetric_kit/recon/sensor/orbbec)
  vr_target_warnings(recon_sensor_orbbec_grouping_test)
  add_test(NAME recon_sensor_orbbec_grouping
           COMMAND recon_sensor_orbbec_grouping_test)

  # The driver's H.265 colour decoder, with no camera: the committed patch clip
  # wrapped in SDK frames (VR_WITH_FFMPEG as well). Internal and unexported, so
  # the test compiles its source itself.
  if(TARGET recon_sensor_video)
    add_executable(
      recon_sensor_orbbec_hevc_test
      sensor_orbbec_hevc_test.cpp
      ${PROJECT_SOURCE_DIR}/src/volumetric_kit/recon/sensor/orbbec/hevc_color.cpp
      ${PROJECT_SOURCE_DIR}/src/volumetric_kit/recon/sensor/orbbec/picture_frames.cpp
    )
    target_link_libraries(
      recon_sensor_orbbec_hevc_test
      PRIVATE volumetric_kit::recon_sensor_orbbec
              volumetric_kit::recon_sensor_video ob::OrbbecSDK)
    target_include_directories(
      recon_sensor_orbbec_hevc_test
      PRIVATE ${PROJECT_SOURCE_DIR}/src/volumetric_kit/recon/sensor/orbbec)
    target_compile_definitions(
      recon_sensor_orbbec_hevc_test
      PRIVATE VR_HEVC_DATA="${CMAKE_CURRENT_SOURCE_DIR}/data/hevc"
              VR_TEST_WITH_CUDA=$<BOOL:${VR_WITH_CUDA}>)
    vr_target_warnings(recon_sensor_orbbec_hevc_test)
    add_test(NAME recon_sensor_orbbec_hevc
             COMMAND recon_sensor_orbbec_hevc_test)

    # The raw MJPEG colour decoder, with no camera: the committed JPEG wrapped
    # in SDK frames.
    add_executable(
      recon_sensor_orbbec_jpeg_test
      sensor_orbbec_jpeg_test.cpp
      ${PROJECT_SOURCE_DIR}/src/volumetric_kit/recon/sensor/orbbec/jpeg_color.cpp
      ${PROJECT_SOURCE_DIR}/src/volumetric_kit/recon/sensor/orbbec/picture_frames.cpp
    )
    target_link_libraries(
      recon_sensor_orbbec_jpeg_test
      PRIVATE volumetric_kit::recon_sensor_orbbec
              volumetric_kit::recon_sensor_video ob::OrbbecSDK)
    target_include_directories(
      recon_sensor_orbbec_jpeg_test
      PRIVATE ${PROJECT_SOURCE_DIR}/src/volumetric_kit/recon/sensor/orbbec)
    target_compile_definitions(
      recon_sensor_orbbec_jpeg_test
      PRIVATE VR_JPEG_DATA="${CMAKE_CURRENT_SOURCE_DIR}/data/jpeg"
              VR_TEST_WITH_CUDA=$<BOOL:${VR_WITH_CUDA}>)
    vr_target_warnings(recon_sensor_orbbec_jpeg_test)
    add_test(NAME recon_sensor_orbbec_jpeg
             COMMAND recon_sensor_orbbec_jpeg_test)
  endif()

  if(WIN32)
    # Windows resolves a DLL on PATH, not through an rpath, and the SDK is
    # linked in place rather than copied beside the test -- it loads its
    # extensions/ from beside its own DLL, so a copy would run without them.
    # ENVIRONMENT_MODIFICATION needs CMake >= 3.22; an older ctest ignores it
    # and the test cannot start. TODO(ci): no leg runs this on Windows.
    set(_vr_orbbec_tests
        recon_orbbec_sdk_smoke recon_sensor_orbbec_conversion
        recon_sensor_orbbec_grouping recon_sensor_orbbec_sync_config)
    if(TARGET recon_sensor_orbbec_hevc_test)
      list(APPEND _vr_orbbec_tests recon_sensor_orbbec_hevc
           recon_sensor_orbbec_jpeg)
    endif()
    set_tests_properties(
      ${_vr_orbbec_tests}
      PROPERTIES ENVIRONMENT_MODIFICATION
                 "PATH=path_list_prepend:$<TARGET_FILE_DIR:ob::OrbbecSDK>")
  endif()
endif()

# Host asset I/O remains testable without creating a Vulkan device.
add_executable(recon_io_image_test io_image_test.cpp)
target_link_libraries(recon_io_image_test PRIVATE volumetric_kit::recon_io)
vr_target_warnings(recon_io_image_test)
add_test(NAME recon_io_image COMMAND recon_io_image_test)

add_executable(recon_io_ply_test io_ply_test.cpp)
target_link_libraries(recon_io_ply_test PRIVATE volumetric_kit::recon_io)
vr_target_warnings(recon_io_ply_test)
add_test(NAME recon_io_ply COMMAND recon_io_ply_test)

# Start the slowest tests first under `ctest --parallel`. Without a COST, CTest
# runs tests in the order they were added, and a fresh CI runner has no timing
# history, so a slow test added late finished last. Seconds under the sanitizers
# job, the longest in CI (2026-10-06); only the order matters.
set_tests_properties(recon_tsdf_mesh_integrate PROPERTIES COST 42)
set_tests_properties(recon_eval_mesh_distance PROPERTIES COST 29)
set_tests_properties(recon_mesh_marching_cubes_sparse PROPERTIES COST 25)
set_tests_properties(recon_codec_decoder PROPERTIES COST 23)
set_tests_properties(recon_codec_device_frame PROPERTIES COST 17)
