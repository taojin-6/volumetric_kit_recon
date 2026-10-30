# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# fuse_realsense: live reconstruction from one RealSense camera, to a PLY, on
# fuse_orbbec's run (examples/common/live_fusion.hpp). It links what that run
# links (fuse_orbbec's CMakeLists.txt says why each) and the RealSense driver in
# place of the Orbbec one; and, where sensor builds it (VS_WITH_FFS), FFS stereo
# depth (sensor_ffs), which VR_EXAMPLE_FFS turns on in realsense_camera.hpp.
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
if(TARGET volumetric_kit::sensor_ffs)
  target_link_libraries(fuse_realsense PRIVATE volumetric_kit::sensor_ffs)
  target_compile_definitions(fuse_realsense PRIVATE VR_EXAMPLE_FFS=1)
endif()
vr_target_warnings(fuse_realsense)
