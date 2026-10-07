# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# fuse_orbbec: live reconstruction from an Orbbec camera, or a rig of them read
# as a sensor array, to a PLY. recon_sensor_array brings sensor/utils's GPU pass
# with it.
add_executable(fuse_orbbec main.cpp)

target_link_libraries(
  fuse_orbbec
  PRIVATE vr_example_common volumetric_kit::recon_sensor_orbbec
          volumetric_kit::recon_sensor_array volumetric_kit::recon_tsdf
          Vulkan::Vulkan)
vr_target_warnings(fuse_orbbec)
