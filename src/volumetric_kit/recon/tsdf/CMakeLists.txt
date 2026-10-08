# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# --- recon_tsdf: Tier 3 -----------------------------------------------------
# TSDF integration: fuses posed depth frames into a VoxelBlockGrid's per-voxel
# tsdf/weight attributes via a GLSL compute kernel (classic projective
# integration first; dynamic follows), and the Fuser that takes a set of frames
# through the grid's growth, allocation and integration. Depends on recon_volume
# (VoxelBlockGrid, GridGrowth, DepthCameraParams, the BlockIndex/VoxelGridParams
# ABI), per the strict left-to-right tier rule.
include(GenerateExportHeader)
include(GNUInstallDirs)

set(_vr_tsdf_generated_inc "${CMAKE_CURRENT_BINARY_DIR}/generated/include")

add_library(recon_tsdf)
add_library(volumetric_kit::recon_tsdf ALIAS recon_tsdf)

target_sources(recon_tsdf PRIVATE fuser.cpp mesh_integrator.cpp
                                  tsdf_integrator.cpp)

# Per-tier export header (VR_TSDF_API / VR_TSDF_STATIC_DEFINE), isolated so the
# macros never collide with another tier's on one include path.
generate_export_header(
  recon_tsdf
  BASE_NAME
  VR_TSDF
  EXPORT_MACRO_NAME
  VR_TSDF_API
  EXPORT_FILE_NAME
  "${_vr_tsdf_generated_inc}/volumetric_kit/recon/tsdf/export.hpp")

set_target_properties(
  recon_tsdf
  PROPERTIES CXX_VISIBILITY_PRESET hidden
             VISIBILITY_INLINES_HIDDEN ON
             POSITION_INDEPENDENT_CODE ON)

get_target_property(_vr_tsdf_type recon_tsdf TYPE)
if(_vr_tsdf_type STREQUAL "STATIC_LIBRARY")
  target_compile_definitions(recon_tsdf PUBLIC VR_TSDF_STATIC_DEFINE)
endif()

target_compile_features(recon_tsdf PUBLIC cxx_std_17)

# PUBLIC: the tsdf public header exposes volume types (VoxelBlockGrid,
# DepthCameraParams) + core types (Result), so consumers compile against
# recon_volume (and, transitively, recon_core) too.
target_link_libraries(recon_tsdf PUBLIC recon_volume)

target_include_directories(
  recon_tsdf
  PUBLIC "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
         "$<BUILD_INTERFACE:${_vr_tsdf_generated_inc}>"
         "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")

# Compile the GLSL kernels to SPIR-V and embed them as constexpr byte arrays
# (each .cpp #includes its generated *.spv.hpp on a PRIVATE path), so recon_tsdf
# ships the technique with no runtime shader files. The two mesh kernels include
# volume's and core's GLSL by root-relative path, as the mesh tier's do.
include(vr_embed)
vr_embed_shaders(
  recon_tsdf
  SHADERS
  "${CMAKE_CURRENT_SOURCE_DIR}/shaders/tsdf_integrate.comp"
  "${CMAKE_CURRENT_SOURCE_DIR}/shaders/mesh_bin.comp"
  "${CMAKE_CURRENT_SOURCE_DIR}/shaders/mesh_scan_counts.comp"
  "${CMAKE_CURRENT_SOURCE_DIR}/shaders/mesh_scan_sums.comp"
  "${CMAKE_CURRENT_SOURCE_DIR}/shaders/mesh_scan_add.comp"
  "${CMAKE_CURRENT_SOURCE_DIR}/shaders/mesh_compact_bins.comp"
  "${CMAKE_CURRENT_SOURCE_DIR}/shaders/mesh_bin_ranges.comp"
  "${CMAKE_CURRENT_SOURCE_DIR}/shaders/mesh_integrate.comp")

vr_target_warnings(recon_tsdf)

# Install the target into the shared export set + this tier's headers (the tsdf/
# subtree) + its generated export header. Mirrors recon_volume so a partial
# build never installs a header for a tier it did not build.
if(VR_INSTALL)
  install(
    TARGETS recon_tsdf
    EXPORT volumetric_kit_reconTargets
    ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
  install(
    DIRECTORY "${PROJECT_SOURCE_DIR}/include/volumetric_kit/recon/tsdf"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/volumetric_kit/recon"
    FILES_MATCHING
    PATTERN "*.hpp")
  install(
    DIRECTORY "${_vr_tsdf_generated_inc}/"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
    FILES_MATCHING
    PATTERN "*.hpp")
endif()
