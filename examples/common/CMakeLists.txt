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

# Mesh height normalization and topology audit, shared by codec_mesh and its
# test. Built only with the Assimp-backed loader whose geometry it takes.
if(TARGET recon_io_assimp)
  add_library(vr_example_mesh STATIC mesh_normalization.cpp)
  target_include_directories(vr_example_mesh
                             PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
  target_link_libraries(vr_example_mesh PUBLIC volumetric_kit::recon_io_assimp)
  target_compile_features(vr_example_mesh PUBLIC cxx_std_17)
  vr_target_warnings(vr_example_mesh)
endif()
