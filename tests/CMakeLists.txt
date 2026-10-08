# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# The core tier's own pieces: the version and the log seam's "vr" source.
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
# + cap). It also runs the kernel's projection alone (shaders/project_pinhole)
# on inputs that make its depth or pixel NaN. Skips (exit 0) with no device.
add_executable(recon_tsdf_integrate_test tsdf_integrate_test.cpp)
target_link_libraries(recon_tsdf_integrate_test
                      PRIVATE volumetric_kit::recon_tsdf Vulkan::Vulkan)
include(vr_embed)
vr_embed_shaders(recon_tsdf_integrate_test SHADERS
                 "${CMAKE_CURRENT_SOURCE_DIR}/shaders/project_pinhole.comp")
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

# Allocation failure while staging is an OutOfMemory result, and a retained
# frame survives it. Separate executable for its replacement operator new.
add_executable(recon_sensor_gpu_frame_prep_oom_test
               sensor_gpu_frame_prep_oom_test.cpp test_allocation_failure.cpp)
target_link_libraries(recon_sensor_gpu_frame_prep_oom_test
                      PRIVATE volumetric_kit::recon_sensor_utils)
vr_target_warnings(recon_sensor_gpu_frame_prep_oom_test)
add_test(NAME recon_sensor_gpu_frame_prep_oom
         COMMAND recon_sensor_gpu_frame_prep_oom_test)

# The viewer's retained keyframe survives preparation and integration errors,
# and still supplies depth, coverage and atlas pixels for projective texturing.
# Headless: uses the example's fusion helper, without gfx or a dataset.
add_executable(recon_example_fuse_keyframe_test example_fuse_keyframe_test.cpp)
target_include_directories(recon_example_fuse_keyframe_test
                           PRIVATE ${PROJECT_SOURCE_DIR}/examples/common)
target_link_libraries(
  recon_example_fuse_keyframe_test
  PRIVATE volumetric_kit::recon_sensor_utils volumetric_kit::recon_tsdf
          volumetric_kit::recon_texture)
vr_target_warnings(recon_example_fuse_keyframe_test)
add_test(NAME recon_example_fuse_keyframe
         COMMAND recon_example_fuse_keyframe_test)

# tsdf fuser: a set fused through tsdf::Fuser, growing on the way, matches the
# tier calls on a grid that never needed to; it grows ahead before any failure,
# refuses incomplete bands by default, and keeps fusing only when the caller
# explicitly allows partial coverage. Skips (exit 0) with no device.
add_executable(recon_tsdf_fuser_test tsdf_fuser_test.cpp)
target_link_libraries(recon_tsdf_fuser_test PRIVATE volumetric_kit::recon_tsdf
                                                    Vulkan::Vulkan)
vr_target_warnings(recon_tsdf_fuser_test)
add_test(NAME recon_tsdf_fuser COMMAND recon_tsdf_fuser_test)

# Host OOM stays in Status, resize rolls back, and private live backoff expires.
add_executable(recon_tsdf_fuser_oom_test tsdf_fuser_oom_test.cpp
                                         test_allocation_failure.cpp)
target_link_libraries(recon_tsdf_fuser_oom_test
                      PRIVATE volumetric_kit::recon_tsdf Vulkan::Vulkan)
vr_target_warnings(recon_tsdf_fuser_oom_test)
add_test(NAME recon_tsdf_fuser_oom COMMAND recon_tsdf_fuser_oom_test)

# The device-input overloads of allocate_from_depth and integrate: the same
# frame fused from host arrays and from storage buffers must give the same grid.
# Skips (exit 0) where no device is present.
add_executable(recon_tsdf_device_input_test tsdf_device_input_test.cpp)
target_link_libraries(recon_tsdf_device_input_test
                      PRIVATE volumetric_kit::recon_tsdf Vulkan::Vulkan)
vr_target_warnings(recon_tsdf_device_input_test)
add_test(NAME recon_tsdf_device_input COMMAND recon_tsdf_device_input_test)

# tsdf integrate set: several cameras allocated and fused in one call each must
# give, under classic, the grid fusing them one at a time gives, bit for bit,
# and under dynamic the grid the same set in the other order gives. Skips (exit
# 0) where no device is present.
add_executable(recon_tsdf_integrate_set_test tsdf_integrate_set_test.cpp)
target_link_libraries(recon_tsdf_integrate_set_test
                      PRIVATE volumetric_kit::recon_tsdf Vulkan::Vulkan)
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
  PRIVATE volumetric_kit::recon_tsdf volumetric_kit::recon_mesh Vulkan::Vulkan)
vr_target_warnings(recon_tsdf_integrate_dynamic_set_test)
add_test(NAME recon_tsdf_integrate_dynamic_set
         COMMAND recon_tsdf_integrate_dynamic_set_test)

# tsdf integrate cull: fusing only the blocks the frames' frusta reach must give
# the grid fusing every active block gives, bit for bit, at each edge of that
# reach. Skips (exit 0) where no device is present.
add_executable(recon_tsdf_integrate_cull_test tsdf_integrate_cull_test.cpp)
target_link_libraries(recon_tsdf_integrate_cull_test
                      PRIVATE volumetric_kit::recon_tsdf Vulkan::Vulkan)
vr_target_warnings(recon_tsdf_integrate_cull_test)
add_test(NAME recon_tsdf_integrate_cull COMMAND recon_tsdf_integrate_cull_test)

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

# Stateless volume growth: doubling, an explicit minimum, content retention, and
# rejection of invalid or unreachable sizes. Skips (exit 0) with no device.
add_executable(recon_volume_grid_growth_test volume_grid_growth_test.cpp)
target_link_libraries(recon_volume_grid_growth_test
                      PRIVATE volumetric_kit::recon_volume Vulkan::Vulkan)
vr_target_warnings(recon_volume_grid_growth_test)
add_test(NAME recon_volume_grid_growth COMMAND recon_volume_grid_growth_test)

# volume diagnostics: force a collision chain (coords hashing to one bucket) and
# verify the reported active / overflow / chain-length + heap stats. Skips (exit
# 0) where no device is present.
add_executable(recon_volume_diagnostics_test volume_diagnostics_test.cpp)
target_link_libraries(recon_volume_diagnostics_test
                      PRIVATE volumetric_kit::recon_volume Vulkan::Vulkan)
vr_target_warnings(recon_volume_diagnostics_test)
add_test(NAME recon_volume_diagnostics COMMAND recon_volume_diagnostics_test)

# volume allocate-from-depth: unproject a synthetic one-pixel depth frame and
# verify exactly the expected (2*tb+1)^3 truncation-band block cube is allocated
# on the real driver -- the camera unprojection + band dilation, plus the
# DepthCameraParams scalar-layout upload. Skips (exit 0) where no device is
# present.
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
# compact_active_blocks_in_frusta_on_device keeps only the in-view blocks while
# plain compaction returns all -- proving the plane math + the on-device AABB
# cull -- and that several frusta keep their union. Skips (exit 0) where no
# device is present.
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
target_link_libraries(recon_sensor_array_process_test
                      PRIVATE volumetric_kit::recon_sensor_array)
vr_target_warnings(recon_sensor_array_process_test)
add_test(NAME recon_sensor_array_process
         COMMAND recon_sensor_array_process_test)

# sensor tier: the RGB-D sensor contract, implemented by a fake and driven
# through the base class -- the no_frame/some_frame returns, poll's newest and
# drain's oldest-first, frames that hold their pixels. Host-only.
add_executable(recon_sensor_rgbd_sensor_test sensor_rgbd_sensor_test.cpp)
target_link_libraries(recon_sensor_rgbd_sensor_test
                      PRIVATE volumetric_kit::recon_sensor)
vr_target_warnings(recon_sensor_rgbd_sensor_test)
add_test(NAME recon_sensor_rgbd_sensor COMMAND recon_sensor_rgbd_sensor_test)

# camera tier: the camera model (its checks, the lens template against OpenCV's
# numbers), its geometry (rigid checks, Rodrigues against OpenCV), and the
# sensor array's calibration file (poses read, the refusals, a file in the build
# tree). Host-only, so they always run.
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

# The video decoders (VR_WITH_FFMPEG only). The frame colour test reads the
# colour off frames built by hand, no codec; it compiles the internal source
# itself, as the Orbbec conversion test does. The decoder tests decode the
# committed clips and JPEGs on this machine's hardware, onto the device; on a
# GPU with no device path they skip, unless VR_TEST_HEVC_BACKEND says this leg
# has one, where they fail.
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

  add_executable(recon_sensor_video_hevc_test sensor_video_hevc_test.cpp)
  target_link_libraries(recon_sensor_video_hevc_test
                        PRIVATE volumetric_kit::recon_sensor_video)
  target_compile_definitions(
    recon_sensor_video_hevc_test
    PRIVATE VR_HEVC_DATA="${CMAKE_CURRENT_SOURCE_DIR}/data/hevc")
  vr_target_warnings(recon_sensor_video_hevc_test)
  add_test(NAME recon_sensor_video_hevc COMMAND recon_sensor_video_hevc_test)

  add_executable(recon_sensor_video_jpeg_test sensor_video_jpeg_test.cpp)
  target_link_libraries(
    recon_sensor_video_jpeg_test PRIVATE volumetric_kit::recon_sensor_video
                                         volumetric_kit::recon_sensor_utils)
  target_compile_definitions(
    recon_sensor_video_jpeg_test
    PRIVATE VR_JPEG_DATA="${CMAKE_CURRENT_SOURCE_DIR}/data/jpeg")
  vr_target_warnings(recon_sensor_video_jpeg_test)
  add_test(NAME recon_sensor_video_jpeg COMMAND recon_sensor_video_jpeg_test)

  if(APPLE)
    # Compile the public JPEG decoder here so the executable's native-call
    # replacements intercept it in both static and shared library builds.
    add_executable(
      recon_sensor_video_vt_jpeg_error_test
      sensor_video_vt_jpeg_error_test.cpp ${_vr_video_src}/jpeg_decoder.cpp
      ${_vr_video_src}/vt_jpeg.cpp ${_vr_video_src}/vt_pictures.mm)
    set_source_files_properties(${_vr_video_src}/vt_pictures.mm
                                PROPERTIES COMPILE_OPTIONS -fobjc-arc)
    target_link_libraries(
      recon_sensor_video_vt_jpeg_error_test
      PRIVATE volumetric_kit::recon_sensor_video
              "-framework VideoToolbox"
              "-framework CoreMedia"
              "-framework CoreVideo"
              "-framework IOSurface"
              "-framework Metal"
              "-framework Foundation")
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
  target_link_libraries(
    recon_sensor_orbbec_open_sensors_test
    PRIVATE volumetric_kit::recon_sensor_orbbec ob::OrbbecSDK)
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
    recon_sensor_orbbec_hevc_test PRIVATE volumetric_kit::recon_sensor_orbbec
                                          volumetric_kit::recon_sensor_video)
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
    recon_sensor_orbbec_jpeg_test PRIVATE volumetric_kit::recon_sensor_orbbec
                                          volumetric_kit::recon_sensor_video)
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
add_executable(recon_io_image_test io_image_test.cpp)
target_link_libraries(recon_io_image_test PRIVATE volumetric_kit::recon_io)
vr_target_warnings(recon_io_image_test)
add_test(NAME recon_io_image COMMAND recon_io_image_test)

add_executable(recon_io_ply_test io_ply_test.cpp)
target_link_libraries(recon_io_ply_test PRIVATE volumetric_kit::recon_io)
vr_target_warnings(recon_io_ply_test)
add_test(NAME recon_io_ply COMMAND recon_io_ply_test)

# What a sibling that fetches recon gets (subproject/): its configure, from a
# clean tree, with each dependency this build fetched as a local source, so it
# needs no network.
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
  if(_vr_var MATCHES "^FETCHCONTENT_SOURCE_DIR_" AND NOT "${${_vr_var}}"
                                                     STREQUAL "")
    list(APPEND _vr_fetched_sources "-D${_vr_var}=${${_vr_var}}")
  endif()
endforeach()
set(_vr_subproject "${CMAKE_CURRENT_BINARY_DIR}/subproject")
add_test(NAME recon_subproject_clean COMMAND "${CMAKE_COMMAND}" -E rm -rf
                                             "${_vr_subproject}")
set_tests_properties(recon_subproject_clean PROPERTIES FIXTURES_SETUP
                                                       vr_subproject)
add_test(
  NAME recon_subproject_defaults
  COMMAND
    "${CMAKE_COMMAND}" -S "${CMAKE_CURRENT_SOURCE_DIR}/subproject" -B
    "${_vr_subproject}" -G "${CMAKE_GENERATOR}"
    "-DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}"
    "-DVR_SOURCE_DIR=${PROJECT_SOURCE_DIR}" ${_vr_fetched_sources})
set_tests_properties(recon_subproject_defaults PROPERTIES FIXTURES_REQUIRED
                                                          vr_subproject)

# Start the slowest tests first under `ctest --parallel`. Without a COST, CTest
# runs tests in the order they were added, and a fresh CI runner has no timing
# history, so a slow test added late finished last. Seconds under the sanitizers
# job, the longest in CI (2026-10-06); only the order matters.
set_tests_properties(recon_tsdf_mesh_integrate PROPERTIES COST 42)
set_tests_properties(recon_eval_mesh_distance PROPERTIES COST 29)
set_tests_properties(recon_mesh_marching_cubes_sparse PROPERTIES COST 25)
set_tests_properties(recon_codec_decoder PROPERTIES COST 23)
set_tests_properties(recon_codec_device_frame PROPERTIES COST 17)
