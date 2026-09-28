# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# Smoke test: compiles the core public headers (Status/Result, POD math,
# version) and links recon_core, proving the tier builds, links, and runs under
# -Werror. A full GoogleTest suite replaces this as the tiers land (see the
# CLAUDE.md roadmap and tests + CI task).
add_executable(recon_core_smoke_test core_smoke_test.cpp)
target_link_libraries(recon_core_smoke_test PRIVATE volumetric_kit::recon_core)
vr_target_warnings(recon_core_smoke_test)
add_test(NAME recon_core_smoke COMMAND recon_core_smoke_test)

# Vulkan availability smoke: proves the build finds Vulkan and the platform
# driver (MoltenVK on Apple) exposes a compute-capable device. The only test
# that links the Vulkan loader directly; recon_core carries the include tree for
# core/vulkan.hpp. Exits 0 (skip) where no driver/device is present.
add_executable(recon_vulkan_smoke_test vulkan_smoke_test.cpp)
target_link_libraries(recon_vulkan_smoke_test PRIVATE volumetric_kit::recon_core
                                                      Vulkan::Vulkan)
vr_target_warnings(recon_vulkan_smoke_test)
add_test(NAME recon_vulkan_smoke COMMAND recon_vulkan_smoke_test)

# core tier: the device-memory figures Allocator::memory_stats reports (what the
# viewer's perf overlay shows). Behavioural, not absolute -- the bytes are VMA's
# accounting. Skips where no device is present.
add_executable(recon_core_memory_stats_test core_memory_stats_test.cpp)
target_link_libraries(recon_core_memory_stats_test
                      PRIVATE volumetric_kit::recon_core)
vr_target_warnings(recon_core_memory_stats_test)
add_test(NAME recon_core_memory_stats COMMAND recon_core_memory_stats_test)

# The reporting vocabulary (host-only, runs on every leg) and the GPU timer
# behind it (skips without a device). The load-bearing assertion is that the
# device span comes in UNDER the wall clock around the same blocking submit --
# separating the two is the entire reason GpuTimer exists.
add_executable(recon_core_stage_metrics_test core_stage_metrics_test.cpp)
target_link_libraries(recon_core_stage_metrics_test
                      PRIVATE volumetric_kit::recon_core)
vr_target_warnings(recon_core_stage_metrics_test)
add_test(NAME recon_core_stage_metrics COMMAND recon_core_stage_metrics_test)

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

add_executable(recon_core_buffer_sharing_test core_buffer_sharing_test.cpp)
target_link_libraries(recon_core_buffer_sharing_test
                      PRIVATE volumetric_kit::recon_core)
# queue_family_set.hpp is internal (non-installed): its location under src/ is
# what keeps it out of the public API, so the test reaches it by include path
# rather than the tier exporting it for one caller's benefit.
target_include_directories(
  recon_core_buffer_sharing_test
  PRIVATE ${PROJECT_SOURCE_DIR}/src/volumetric_kit/recon/core)
vr_target_warnings(recon_core_buffer_sharing_test)
add_test(NAME recon_core_buffer_sharing COMMAND recon_core_buffer_sharing_test)

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

# core device seam: Instance + Device create/adopt. Links recon_core, which now
# carries the Vulkan headers/loader. Skips at runtime where no compute device is
# present.
add_executable(recon_device_test device_test.cpp)
target_link_libraries(recon_device_test PRIVATE volumetric_kit::recon_core)
vr_target_warnings(recon_device_test)
add_test(NAME recon_device COMMAND recon_device_test)

# Vulkan compute smoke: the end-to-end de-risk gate for the compute core. Builds
# Instance -> Device -> Allocator -> Buffer -> Descriptor -> ComputePipeline,
# dispatches fill.comp through the platform driver (MoltenVK on Apple), and
# verifies the readback. vr_compile_shaders compiles the GLSL to SPIR-V; the
# .spv path is handed to the test as VR_FILL_SPV. Skips (exit 0) where no
# driver/device is present.
include(vr_shaders)
add_executable(recon_compute_smoke_test compute_smoke_test.cpp)
vr_compile_shaders(recon_compute_smoke_test SHADERS
                   "${CMAKE_CURRENT_SOURCE_DIR}/shaders/fill.comp")
target_compile_definitions(
  recon_compute_smoke_test
  PRIVATE VR_FILL_SPV="${CMAKE_CURRENT_BINARY_DIR}/shaders/fill.comp.spv")
target_link_libraries(recon_compute_smoke_test
                      PRIVATE volumetric_kit::recon_core Vulkan::Vulkan)
vr_target_warnings(recon_compute_smoke_test)
add_test(NAME recon_compute_smoke COMMAND recon_compute_smoke_test)

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

# Move-semantics tests for the compute core's RAII handle owners (Buffer's
# std::function deleter + the UniqueHandle mechanism behind the rest). The
# sanitizer CI job turns these into leak / double-free detectors. Skips (exit 0)
# where no device is present.
add_executable(recon_compute_raii_test compute_raii_test.cpp)
target_link_libraries(recon_compute_raii_test PRIVATE volumetric_kit::recon_core
                                                      Vulkan::Vulkan)
vr_target_warnings(recon_compute_raii_test)
add_test(NAME recon_compute_raii COMMAND recon_compute_raii_test)

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

# sensor tier: the rig calibration file -- the family's config layout, the
# world->camera pose turned into camera->world, Rodrigues' edge cases, the
# refusals, and a round trip through a file in the build tree. Host-only.
add_executable(recon_sensor_rig_calibration_test
               sensor_rig_calibration_test.cpp)
target_link_libraries(recon_sensor_rig_calibration_test
                      PRIVATE volumetric_kit::recon_sensor)
target_compile_definitions(
  recon_sensor_rig_calibration_test
  PRIVATE VR_TEST_SCRATCH_DIR="${CMAKE_CURRENT_BINARY_DIR}")
vr_target_warnings(recon_sensor_rig_calibration_test)
add_test(NAME recon_sensor_rig_calibration
         COMMAND recon_sensor_rig_calibration_test)

# codec tier, host half: CodecParams validation and the DCT's two tables (basis
# orthonormality, the zigzag order against the prior engine's). CPU-only, so it
# always runs. The tables are internal (non-installed), reached by include path
# as core_buffer_sharing_test reaches queue_family_set.hpp.
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

# codec round trip with ground truth: a mesh through tsdf::MeshIntegrator, the
# forward DCT, v1 frame bytes and back, the inverse DCT and marching cubes, with
# the decoded surface measured against the source mesh (accuracy and coverage)
# beside the uncompressed volume's, and the frame's decode compared with what
# was written. Reaches the codec's internal headers by include path, as the
# other codec tests do. Skips (exit 0) with no device.
add_executable(recon_codec_mesh_roundtrip_test codec_mesh_roundtrip_test.cpp)
target_link_libraries(
  recon_codec_mesh_roundtrip_test
  PRIVATE volumetric_kit::recon_codec volumetric_kit::recon_tsdf
          volumetric_kit::recon_mesh Vulkan::Vulkan)
target_include_directories(
  recon_codec_mesh_roundtrip_test
  PRIVATE ${PROJECT_SOURCE_DIR}/src/volumetric_kit/recon/codec)
vr_target_warnings(recon_codec_mesh_roundtrip_test)
add_test(NAME recon_codec_mesh_roundtrip
         COMMAND recon_codec_mesh_roundtrip_test)

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
# found; VR_TEST_HEVC_BACKEND makes one required.
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
    PRIVATE VR_HEVC_DATA="${CMAKE_CURRENT_SOURCE_DIR}/data/hevc")
  vr_target_warnings(recon_sensor_video_hevc_test)
  add_test(NAME recon_sensor_video_hevc COMMAND recon_sensor_video_hevc_test)
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
  vr_target_warnings(recon_sensor_orbbec_conversion_test)
  add_test(NAME recon_sensor_orbbec_conversion
           COMMAND recon_sensor_orbbec_conversion_test)

  # The driver against the camera VR_ORBBEC_TEST_SERIAL names; unset, it skips,
  # so no CI leg opens a camera that happens to share its network.
  add_executable(recon_sensor_orbbec_capture_test
                 sensor_orbbec_capture_test.cpp)
  target_link_libraries(recon_sensor_orbbec_capture_test
                        PRIVATE volumetric_kit::recon_sensor_orbbec)
  # Its H.265 half runs when the driver has the decoder; else it checks the
  # refusal.
  if(TARGET recon_sensor_video)
    target_compile_definitions(recon_sensor_orbbec_capture_test
                               PRIVATE VR_TEST_HEVC=1)
  else()
    target_compile_definitions(recon_sensor_orbbec_capture_test
                               PRIVATE VR_TEST_HEVC=0)
  endif()
  vr_target_warnings(recon_sensor_orbbec_capture_test)
  add_test(NAME recon_sensor_orbbec_capture
           COMMAND recon_sensor_orbbec_capture_test)

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
      PRIVATE VR_HEVC_DATA="${CMAKE_CURRENT_SOURCE_DIR}/data/hevc")
    vr_target_warnings(recon_sensor_orbbec_hevc_test)
    add_test(NAME recon_sensor_orbbec_hevc
             COMMAND recon_sensor_orbbec_hevc_test)
  endif()

  # The rig against real cameras -- only the ones VR_ORBBEC_TEST_RIG names
  # (comma-separated serials); unset, it skips (exit 0).
  add_executable(recon_sensor_orbbec_rig_test sensor_orbbec_rig_test.cpp)
  target_link_libraries(recon_sensor_orbbec_rig_test
                        PRIVATE volumetric_kit::recon_sensor_orbbec)
  if(TARGET recon_sensor_video)
    target_compile_definitions(recon_sensor_orbbec_rig_test
                               PRIVATE VR_TEST_HEVC=1)
  else()
    target_compile_definitions(recon_sensor_orbbec_rig_test
                               PRIVATE VR_TEST_HEVC=0)
  endif()
  target_compile_definitions(
    recon_sensor_orbbec_rig_test
    PRIVATE VR_TEST_SCRATCH_DIR="${CMAKE_CURRENT_BINARY_DIR}")
  vr_target_warnings(recon_sensor_orbbec_rig_test)
  add_test(NAME recon_sensor_orbbec_rig COMMAND recon_sensor_orbbec_rig_test)

  if(WIN32)
    # Windows resolves a DLL on PATH, not through an rpath, and the SDK is
    # linked in place rather than copied beside the test -- it loads its
    # extensions/ from beside its own DLL, so a copy would run without them.
    # ENVIRONMENT_MODIFICATION needs CMake >= 3.22; an older ctest ignores it
    # and the test cannot start. TODO(ci): no leg runs this on Windows.
    set(_vr_orbbec_tests
        recon_orbbec_sdk_smoke recon_sensor_orbbec_conversion
        recon_sensor_orbbec_capture recon_sensor_orbbec_grouping
        recon_sensor_orbbec_sync_config recon_sensor_orbbec_rig)
    if(TARGET recon_sensor_orbbec_hevc_test)
      list(APPEND _vr_orbbec_tests recon_sensor_orbbec_hevc)
    endif()
    set_tests_properties(
      ${_vr_orbbec_tests}
      PROPERTIES ENVIRONMENT_MODIFICATION
                 "PATH=path_list_prepend:$<TARGET_FILE_DIR:ob::OrbbecSDK>")
  endif()
endif()
