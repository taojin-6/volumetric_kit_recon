# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# Shared example utilities: the Replica sequence as an ICameraCapture,
# stb-backed image decode, and a tinyply-backed binary PLY writer. A small
# static lib the fuse examples link.
add_library(
  vr_example_common STATIC image_io.cpp stb_impl.cpp replica_capture.cpp
                           ply_writer.cpp tinyply_impl.cpp)

target_include_directories(vr_example_common
                           PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

# recon_sensor is the capture contract ReplicaCapture implements (and brings
# core: the camera structs + Result/Status); recon_mesh brings mesh::Mesh for
# ply_writer. The tsdf tier is linked by the executables that fuse, not here.
# stb_image + tinyply are private (examples-only) build dependencies of the
# image-decode / PLY-writer TUs.
target_link_libraries(
  vr_example_common
  PUBLIC volumetric_kit::recon_sensor volumetric_kit::recon_mesh
  PRIVATE vr_stb_image vr_tinyply)
target_compile_features(vr_example_common PUBLIC cxx_std_17)
vr_target_warnings(vr_example_common)

# The stb_image / tinyply implementation TUs are third-party: build them
# warnings-off so they do not trip the -Werror surface vr_target_warnings
# applies to our own sources.
set_source_files_properties(stb_impl.cpp tinyply_impl.cpp
                            PROPERTIES COMPILE_OPTIONS "-w")

# ReplicaCapture's behavioural test: host-only, dataset-free (it writes a tiny
# synthetic scene from bytes it carries), so it runs on every CI leg -- the
# sanitizer one included, which is where a frame read after the poll that freed
# it would surface. The tsdf tier is not needed: the capture is the thing under
# test, not what consumes it.
if(VR_BUILD_TESTS)
  add_executable(vr_example_replica_capture_test replica_capture_test.cpp)
  target_link_libraries(vr_example_replica_capture_test
                        PRIVATE vr_example_common)
  vr_target_warnings(vr_example_replica_capture_test)
  add_test(NAME vr_example_replica_capture
           COMMAND vr_example_replica_capture_test)
endif()
