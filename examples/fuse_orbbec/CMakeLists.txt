# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# fuse_orbbec: live reconstruction from an Orbbec camera, or a rig of them read
# as a sensor array, to a PLY; output.stream codes each set into a .vrst file,
# and output.serve serves it live. It links the live loop (vr_example_live,
# examples/common/CMakeLists.txt says why each library) and the Orbbec driver.
add_executable(fuse_orbbec main.cpp)

target_link_libraries(fuse_orbbec PRIVATE vr_example_live
                                          volumetric_kit::sensor_orbbec)
vr_target_warnings(fuse_orbbec)
