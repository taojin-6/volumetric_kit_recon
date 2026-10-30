# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# fuse_realsense: live reconstruction from one RealSense camera's own depth, to
# a PLY, on fuse_orbbec's run (examples/common/live_fusion.hpp). It links what
# that run links (fuse_orbbec's CMakeLists.txt says why each) and the RealSense
# driver in place of the Orbbec one.
add_executable(fuse_realsense main.cpp)

target_link_libraries(
  fuse_realsense
  PRIVATE vr_example_common
          volumetric_kit::grid_eval
          volumetric_kit::grid_mesh_io
          volumetric_kit::recon_sender
          volumetric_kit::grid_voxelize
          volumetric_kit::sensor_realsense
          volumetric_kit::sensor_array
          Vulkan::Vulkan)
vr_target_warnings(fuse_realsense)
