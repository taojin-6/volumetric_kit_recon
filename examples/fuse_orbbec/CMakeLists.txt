# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# fuse_orbbec: live reconstruction from an Orbbec camera, or a rig of them read
# as a sensor array, to a PLY; output.stream codes each set into a .vrst file,
# and output.serve serves it live. recon_sensor_array brings sensor/utils's GPU
# pass with it; the codec, stream, networking and eval tiers are what the codec
# examples' stream (codec_stream.hpp) needs.
add_executable(fuse_orbbec main.cpp)

target_link_libraries(
  fuse_orbbec
  PRIVATE vr_example_common
          volumetric_kit::recon_codec
          volumetric_kit::recon_eval
          volumetric_kit::recon_networking
          volumetric_kit::recon_sensor_orbbec
          volumetric_kit::recon_sensor_array
          volumetric_kit::recon_stream
          volumetric_kit::recon_tsdf
          Vulkan::Vulkan)
vr_target_warnings(fuse_orbbec)
