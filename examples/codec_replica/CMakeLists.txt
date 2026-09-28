# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# codec_replica: fuse a Replica sequence as fuse_replica does, stream the
# growing volume through the TSDF codec, and report its size, its cost and the
# decoded surface's distance from the source; --sweep prints the rate-distortion
# table the codec's defaults are chosen from.
add_executable(codec_replica main.cpp)

target_link_libraries(
  codec_replica
  PRIVATE vr_example_common
          volumetric_kit::recon_codec
          volumetric_kit::recon_eval
          volumetric_kit::recon_sensor_utils
          volumetric_kit::recon_tsdf
          volumetric_kit::recon_mesh
          volumetric_kit::recon_volume
          Vulkan::Vulkan)
vr_target_warnings(codec_replica)
