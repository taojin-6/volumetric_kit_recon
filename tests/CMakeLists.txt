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

# mesh marching cubes: extract an analytic sphere SDF on a dense grid and verify
# the mesh is a sphere -- vertices on the radius, outward normals, and winding
# consistent with the gradient normal -- on the real driver (MoltenVK / NVIDIA).
# Skips (exit 0) where no device is present.
add_executable(recon_mesh_marching_cubes_test marching_cubes_test.cpp)
target_link_libraries(recon_mesh_marching_cubes_test
                      PRIVATE volumetric_kit::recon_mesh Vulkan::Vulkan)
vr_target_warnings(recon_mesh_marching_cubes_test)
add_test(NAME recon_mesh_marching_cubes COMMAND recon_mesh_marching_cubes_test)

# Sparse marching cubes: mesh an analytic sphere written into a real multi-block
# VoxelBlockGrid, exercising cross-block neighbour sampling, and prove it
# matches the dense path exactly. Skips (exit 0) where no device is present.
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
