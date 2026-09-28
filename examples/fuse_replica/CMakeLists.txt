# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# fuse_replica: read a posed Replica RGB-D sequence, fuse TSDF + colour into a
# sparse volume, extract a marching-cubes mesh, and write it to PLY.
add_executable(fuse_replica main.cpp)

target_link_libraries(
  fuse_replica
  PRIVATE vr_example_common volumetric_kit::recon_tsdf
          volumetric_kit::recon_mesh volumetric_kit::recon_volume
          Vulkan::Vulkan)
vr_target_warnings(fuse_replica)
