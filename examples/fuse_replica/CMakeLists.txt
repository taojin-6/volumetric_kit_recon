# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# fuse_replica: read a posed Replica RGB-D sequence -- or, with --orbbec, a live
# Orbbec camera -- fuse TSDF + colour into a sparse volume, extract a
# marching-cubes mesh, and write it to PLY.
add_executable(fuse_replica main.cpp)

target_link_libraries(
  fuse_replica
  PRIVATE vr_example_common volumetric_kit::recon_tsdf
          volumetric_kit::recon_mesh volumetric_kit::recon_volume
          Vulkan::Vulkan)
vr_target_warnings(fuse_replica)

# The live source, when the driver is built (VR_WITH_ORBBEC).
if(TARGET volumetric_kit::recon_sensor_orbbec)
  target_link_libraries(fuse_replica
                        PRIVATE volumetric_kit::recon_sensor_orbbec)
  target_compile_definitions(fuse_replica PRIVATE VR_EXAMPLE_WITH_ORBBEC)
endif()
