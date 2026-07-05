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
