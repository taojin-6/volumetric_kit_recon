# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# Shared example utilities: the Replica dataset reader, stb-backed image decode,
# and a tinyply-backed binary PLY writer. A small static lib the fuse examples
# link.
add_library(
  vr_example_common STATIC image_io.cpp stb_impl.cpp replica_dataset.cpp
                           ply_writer.cpp tinyply_impl.cpp)

target_include_directories(vr_example_common
                           PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

# recon_mesh brings mesh::Mesh (ply_writer) and transitively volume + core (the
# dataset reader's math + Result/Status). The tsdf tier is linked by the
# executables that fuse, not here. stb_image + tinyply are private (examples-
# only) build dependencies of the image-decode / PLY-writer TUs.
target_link_libraries(
  vr_example_common
  PUBLIC volumetric_kit::recon_mesh
  PRIVATE vr_stb_image vr_tinyply)
target_compile_features(vr_example_common PUBLIC cxx_std_17)
vr_target_warnings(vr_example_common)

# The stb_image / tinyply implementation TUs are third-party: build them
# warnings-off so they do not trip the -Werror surface vr_target_warnings
# applies to our own sources.
set_source_files_properties(stb_impl.cpp tinyply_impl.cpp
                            PROPERTIES COMPILE_OPTIONS "-w")
