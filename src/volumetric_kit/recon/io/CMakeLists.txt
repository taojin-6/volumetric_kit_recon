# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

include(GenerateExportHeader)

set(_vr_io_generated_inc "${CMAKE_CURRENT_BINARY_DIR}/generated/include")

# Host image decoding / PNG encoding, Mesh -> PLY serialization and the opt-in
# mesh normalization helpers. The implementation-only libraries have no public
# headers or targets.
add_library(recon_io image_io.cpp mesh_normalization.cpp ply_writer.cpp
                     stb_impl.cpp tinyply_impl.cpp)
add_library(volumetric_kit::recon_io ALIAS recon_io)
generate_export_header(
  recon_io
  BASE_NAME
  VR_IO
  EXPORT_MACRO_NAME
  VR_IO_API
  EXPORT_FILE_NAME
  "${_vr_io_generated_inc}/volumetric_kit/recon/io/export.hpp")
set_target_properties(
  recon_io
  PROPERTIES CXX_VISIBILITY_PRESET hidden
             VISIBILITY_INLINES_HIDDEN ON
             POSITION_INDEPENDENT_CODE ON)
get_target_property(_vr_io_type recon_io TYPE)
if(_vr_io_type STREQUAL "STATIC_LIBRARY")
  target_compile_definitions(recon_io PUBLIC VR_IO_STATIC_DEFINE)
endif()
target_compile_features(recon_io PUBLIC cxx_std_17)
# Catch backend / allocation exceptions at the Status boundary even when a
# consuming application globally disables exceptions. Keep this private.
target_compile_options(recon_io PRIVATE -fexceptions)
target_link_libraries(recon_io PUBLIC recon_mesh)
target_include_directories(
  recon_io PUBLIC "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
                  "$<BUILD_INTERFACE:${_vr_io_generated_inc}>")
target_include_directories(recon_io SYSTEM PRIVATE "${VR_STB_INCLUDE_DIR}"
                                                   "${VR_TINYPLY_INCLUDE_DIR}")
vr_target_warnings(recon_io)
# Only vendored implementation TUs opt out of first-party warnings.
set_source_files_properties(stb_impl.cpp tinyply_impl.cpp
                            PROPERTIES COMPILE_OPTIONS -w)
# No fused multiply-add, so a mesh transform's doubles, which a stream may
# store, are the same whichever compiler built them.
set_source_files_properties(mesh_normalization.cpp PROPERTIES COMPILE_OPTIONS
                                                              -ffp-contract=off)

# An installed prerequisite, like the sensor decoders. Keeping it separate lets
# image / PLY consumers build without Assimp on every platform.
if(VR_WITH_ASSIMP)
  add_library(recon_io_assimp mesh_io.cpp)
  add_library(volumetric_kit::recon_io_assimp ALIAS recon_io_assimp)
  generate_export_header(
    recon_io_assimp
    BASE_NAME
    VR_IO_ASSIMP
    EXPORT_MACRO_NAME
    VR_IO_ASSIMP_API
    EXPORT_FILE_NAME
    "${_vr_io_generated_inc}/volumetric_kit/recon/io/assimp_export.hpp")
  set_target_properties(
    recon_io_assimp
    PROPERTIES CXX_VISIBILITY_PRESET hidden
               VISIBILITY_INLINES_HIDDEN ON
               POSITION_INDEPENDENT_CODE ON)
  get_target_property(_vr_io_assimp_type recon_io_assimp TYPE)
  if(_vr_io_assimp_type STREQUAL "STATIC_LIBRARY")
    target_compile_definitions(recon_io_assimp
                               PUBLIC VR_IO_ASSIMP_STATIC_DEFINE)
  endif()
  target_compile_features(recon_io_assimp PUBLIC cxx_std_17)
  target_compile_options(recon_io_assimp PRIVATE -fexceptions)
  # recon_io carries the public and generated include directories.
  target_link_libraries(
    recon_io_assimp
    PUBLIC recon_io
    PRIVATE assimp::assimp)
  vr_target_warnings(recon_io_assimp)
endif()
