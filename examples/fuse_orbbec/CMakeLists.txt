# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# fuse_orbbec: live reconstruction from an Orbbec camera -- poll frames through
# sensor::OrbbecCapture, fuse them with the shared per-frame fuse, extract a
# marching-cubes mesh and write it to PLY. Built when the driver is
# (VR_WITH_ORBBEC); the dataset example, fuse_replica, stays dataset-only.
add_executable(fuse_orbbec main.cpp)

target_link_libraries(
  fuse_orbbec
  PRIVATE vr_example_common volumetric_kit::recon_sensor_orbbec
          volumetric_kit::recon_tsdf volumetric_kit::recon_mesh
          volumetric_kit::recon_volume Vulkan::Vulkan)
vr_target_warnings(fuse_orbbec)
