# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# fuse_orbbec: live reconstruction from an Orbbec camera to a PLY.
add_executable(fuse_orbbec main.cpp)

target_link_libraries(
  fuse_orbbec
  PRIVATE vr_example_common
          volumetric_kit::recon_sensor_orbbec
          volumetric_kit::recon_sensor_utils
          volumetric_kit::recon_tsdf
          volumetric_kit::recon_mesh
          volumetric_kit::recon_volume
          Vulkan::Vulkan)
vr_target_warnings(fuse_orbbec)
