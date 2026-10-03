# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

include(GenerateExportHeader)
include(GNUInstallDirs)

set(_vr_io_generated_inc "${CMAKE_CURRENT_BINARY_DIR}/generated/include")
set(_vr_io_install_inc "${CMAKE_INSTALL_INCLUDEDIR}/volumetric_kit/recon/io")

# Host image decoding / PNG encoding and Mesh -> PLY serialization. The
# implementation-only libraries have no public headers or exported targets.
add_library(recon_io image_io.cpp ply_writer.cpp stb_impl.cpp tinyply_impl.cpp)
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
target_compile_options(
  recon_io PRIVATE "$<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-fexceptions>"
                   "$<$<CXX_COMPILER_ID:MSVC>:/EHsc>")
target_link_libraries(recon_io PUBLIC recon_mesh)
target_include_directories(
  recon_io
  PUBLIC "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
         "$<BUILD_INTERFACE:${_vr_io_generated_inc}>"
         "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")
target_include_directories(recon_io SYSTEM PRIVATE "${VR_STB_INCLUDE_DIR}"
                                                   "${VR_TINYPLY_INCLUDE_DIR}")
vr_target_warnings(recon_io)
# Only vendored implementation TUs opt out of first-party warnings.
set_source_files_properties(
  stb_impl.cpp tinyply_impl.cpp
  PROPERTIES COMPILE_OPTIONS "$<IF:$<CXX_COMPILER_ID:MSVC>,/w,-w>")

if(VR_INSTALL)
  install(
    TARGETS recon_io
    EXPORT volumetric_kit_reconTargets
    ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
  install(
    FILES "${PROJECT_SOURCE_DIR}/include/volumetric_kit/recon/io/image_io.hpp"
          "${PROJECT_SOURCE_DIR}/include/volumetric_kit/recon/io/ply_writer.hpp"
          "${_vr_io_generated_inc}/volumetric_kit/recon/io/export.hpp"
    DESTINATION "${_vr_io_install_inc}")
endif()

# An installed prerequisite, like the sensor decoders. Keeping it separate lets
# image / PLY consumers build and install without Assimp on every platform.
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
  target_compile_options(
    recon_io_assimp
    PRIVATE "$<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-fexceptions>"
            "$<$<CXX_COMPILER_ID:MSVC>:/EHsc>")
  target_link_libraries(
    recon_io_assimp
    PUBLIC recon_io
    PRIVATE assimp::assimp)
  target_include_directories(
    recon_io_assimp
    PUBLIC "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
           "$<BUILD_INTERFACE:${_vr_io_generated_inc}>"
           "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")
  vr_target_warnings(recon_io_assimp)

  if(VR_INSTALL)
    install(
      TARGETS recon_io_assimp
      EXPORT volumetric_kit_reconTargets
      ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
      LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
      RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
    install(
      FILES "${PROJECT_SOURCE_DIR}/include/volumetric_kit/recon/io/mesh_io.hpp"
            "${_vr_io_generated_inc}/volumetric_kit/recon/io/assimp_export.hpp"
      DESTINATION "${_vr_io_install_inc}")
  endif()
endif()
