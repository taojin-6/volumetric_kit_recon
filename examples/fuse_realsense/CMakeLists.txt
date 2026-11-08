# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# fuse_realsense: live reconstruction from one RealSense camera, to a PLY, on
# fuse_orbbec's run (vr_example_live) with the RealSense driver in place of the
# Orbbec one, and FFS stereo depth where sensor builds it (vr_example_ffs).
add_executable(fuse_realsense main.cpp)

target_link_libraries(
  fuse_realsense PRIVATE vr_example_live volumetric_kit::sensor_realsense
                         vr_example_ffs)
vr_target_warnings(fuse_realsense)
