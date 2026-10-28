# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# codec_replica: fuse a Replica sequence as fuse_replica does, stream the
# growing volume through the TSDF codec, and report its size, its cost and the
# decoded surface's distance from the source; codec.sweep prints the
# rate-distortion table the codec's defaults are chosen from. output.stream
# records the coded frames in a .vrst file. voxelize is for the mesh settings
# codec_config.hpp maps.
add_executable(codec_replica main.cpp)

target_link_libraries(
  codec_replica
  PRIVATE vr_example_common volumetric_kit::recon_eval
          volumetric_kit::recon_sender volumetric_kit::recon_voxelize
          Vulkan::Vulkan)
vr_target_warnings(codec_replica)
