# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# --- recon_tsdf: Tier 3 -----------------------------------------------------
# TSDF integration: fuses posed depth frames into a VoxelBlockGrid's per-voxel
# tsdf/weight attributes via a GLSL compute kernel (classic projective
# integration first; dynamic follows), and the Fuser that takes a set of frames
# through the grid's growth, allocation and integration. Depends on recon_volume
# (VoxelBlockGrid, grow_grid, DepthCameraParams, the BlockIndex/VoxelGridParams
# ABI), per the strict left-to-right tier rule.
include(GenerateExportHeader)

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

# PRIVATE src/: recon_core's internal dispatch.hpp, as recon_volume reaches it.
target_include_directories(
  recon_tsdf
  PUBLIC "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
         "$<BUILD_INTERFACE:${_vr_tsdf_generated_inc}>"
  PRIVATE "${PROJECT_SOURCE_DIR}/src")

# Compile the GLSL kernels to SPIR-V and embed them as constexpr byte arrays
# (each .cpp #includes its generated *.spv.hpp on a PRIVATE path), so recon_tsdf
# ships the technique with no runtime shader files. The two mesh kernels include
# volume's and core's GLSL by root-relative path, as the mesh tier's do.
include(vr_embed)
vr_embed_shaders(
  recon_tsdf
  SHADERS
  "${CMAKE_CURRENT_SOURCE_DIR}/shaders/tsdf_integrate.comp"
  "${CMAKE_CURRENT_SOURCE_DIR}/shaders/tsdf_clear.comp"
  "${CMAKE_CURRENT_SOURCE_DIR}/shaders/mesh_bin.comp"
  "${CMAKE_CURRENT_SOURCE_DIR}/shaders/mesh_scan_counts.comp"
  "${CMAKE_CURRENT_SOURCE_DIR}/shaders/mesh_scan_sums.comp"
  "${CMAKE_CURRENT_SOURCE_DIR}/shaders/mesh_scan_add.comp"
  "${CMAKE_CURRENT_SOURCE_DIR}/shaders/mesh_compact_bins.comp"
  "${CMAKE_CURRENT_SOURCE_DIR}/shaders/mesh_bin_ranges.comp"
  "${CMAKE_CURRENT_SOURCE_DIR}/shaders/mesh_integrate.comp")

vr_target_warnings(recon_tsdf)
