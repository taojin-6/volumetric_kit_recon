# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# Orchestrator for the component tiers. Each subdirectory owns its own target
# and build settings; the strict dependency rule (a tier may depend only on
# tiers to its left) is expressed by what each links.

# The camera and sensor tiers are volumetric_kit_sensor's, which third_party/
# adds first: recon's libraries link its frame contract, sensor_frame, alone.
add_subdirectory(volumetric_kit/recon/core)
# tsdf fuses into grid's VoxelBlockGrid (volumetric_kit_grid).
add_subdirectory(volumetric_kit/recon/tsdf)
# texture fills the mesh uv0 by projecting a posed camera onto it; it depends on
# grid's mesh (the Mesh it textures) and the sensor package's frame contract
# (DepthCameraParams + the projection GLSL).
add_subdirectory(volumetric_kit/recon/texture)
# A sender's set step: it links the frame contract, the fuser and grid's codec
# stream.
add_subdirectory(volumetric_kit/recon/sender)
# TODO: add_subdirectory() for interop as that tier lands.
set(_vr_tiers recon_core recon_tsdf recon_texture recon_sender)

# --- recon: umbrella alias --------------------------------------------------
# Consumers link `volumetric_kit::recon` and get whatever tiers are built. It
# carries recon's version, which a consumer's vkc_require_sibling reads.
add_library(recon INTERFACE)
add_library(volumetric_kit::recon ALIAS recon)
set_target_properties(recon PROPERTIES VKC_PACKAGE_VERSION "${PROJECT_VERSION}")
target_link_libraries(recon INTERFACE ${_vr_tiers})
