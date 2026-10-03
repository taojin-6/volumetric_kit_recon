# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# The Replica sequence implements the sensor capture contract and loads its
# assets through the production IO tier. Dataset layout stays example-local.
add_library(vr_example_common STATIC replica_capture.cpp)
target_include_directories(vr_example_common
                           PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(vr_example_common PUBLIC volumetric_kit::recon_sensor
                                               volumetric_kit::recon_io)
target_compile_features(vr_example_common PUBLIC cxx_std_17)
vr_target_warnings(vr_example_common)
