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

# Move-semantics tests for the compute core's RAII handle owners (Buffer's
# std::function deleter + the UniqueHandle mechanism behind the rest). The
# sanitizer CI job turns these into leak / double-free detectors. Skips (exit 0)
# where no device is present.
add_executable(recon_compute_raii_test compute_raii_test.cpp)
target_link_libraries(recon_compute_raii_test PRIVATE volumetric_kit::recon_core
                                                      Vulkan::Vulkan)
vr_target_warnings(recon_compute_raii_test)
add_test(NAME recon_compute_raii COMMAND recon_compute_raii_test)
