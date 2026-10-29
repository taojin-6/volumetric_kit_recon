# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# fuse_orbbec: live reconstruction from an Orbbec camera, or a rig of them read
# as a sensor array, to a PLY; output.stream codes each set into a .vrst file,
# and output.serve serves it live. recon_sender is the set step, and brings its
# codec stream, the fuser, the codec and sensor/utils's GPU pass with it; eval
# is the codec sweep's, and voxelize the mesh settings codec_config.hpp maps.
add_executable(fuse_orbbec main.cpp)

target_link_libraries(
  fuse_orbbec
  PRIVATE vr_example_common
          volumetric_kit::recon_eval
          volumetric_kit::recon_mesh_io
          volumetric_kit::recon_sender
          volumetric_kit::recon_voxelize
          volumetric_kit::recon_sensor_orbbec
          volumetric_kit::recon_sensor_array
          Vulkan::Vulkan)
vr_target_warnings(fuse_orbbec)
