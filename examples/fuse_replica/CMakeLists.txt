# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# fuse_replica: read a posed Replica RGB-D sequence, fuse TSDF + colour into a
# sparse volume, extract a marching-cubes mesh, and write it to PLY.
add_executable(fuse_replica main.cpp)

# recon_tsdf: the fuser and its inputs (fuse_frame.hpp's fuse_set);
# vr_example_common brings the sensor package's GPU pass (sensor_prep).
target_link_libraries(
  fuse_replica PRIVATE vr_example_common volumetric_kit::grid_mesh_io
                       volumetric_kit::recon_tsdf Vulkan::Vulkan)
vr_target_warnings(fuse_replica)
