# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# fuse_replica: read a posed Replica RGB-D sequence, fuse TSDF + colour into a
# sparse volume, extract a marching-cubes mesh, and write it to PLY.
add_executable(fuse_replica main.cpp)

# recon_sender: the fuser's inputs (fuse_frame.hpp's fuse_set), with the fuser
# and sensor/utils's GPU pass.
target_link_libraries(
  fuse_replica PRIVATE vr_example_common volumetric_kit::grid_mesh_io
                       volumetric_kit::recon_sender Vulkan::Vulkan)
vr_target_warnings(fuse_replica)
