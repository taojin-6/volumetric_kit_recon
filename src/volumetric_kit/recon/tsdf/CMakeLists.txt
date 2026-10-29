# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# --- recon_tsdf: Tier 3 -----------------------------------------------------
# TSDF integration: fuses posed depth frames into a VoxelBlockGrid's per-voxel
# tsdf/weight attributes via a GLSL compute kernel (classic projective
# integration first; dynamic follows), the Fuser that takes a set of frames
# through the grid's growth, allocation and integration, and the grid it starts
# from. Depends on recon_volume (VoxelBlockGrid, grow_grid, DepthPinhole, the
# BlockIndex/VoxelGridParams ABI), per the strict left-to-right tier rule.
include(GenerateExportHeader)

set(_vr_tsdf_generated_inc "${CMAKE_CURRENT_BINARY_DIR}/generated/include")

add_library(recon_tsdf)
add_library(volumetric_kit::recon_tsdf ALIAS recon_tsdf)

target_sources(recon_tsdf PRIVATE fuser.cpp fusion_grid.cpp tsdf_integrator.cpp)

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
# DepthPinhole), core types (Result) and the sensor package's frame vocabulary
# (DepthCameraParams, ColorEncoding), so consumers compile against recon_volume
# (and, transitively, recon_core) and sensor_frame too.
target_link_libraries(recon_tsdf PUBLIC recon_volume
                                        volumetric_kit::sensor_frame)

# PRIVATE src/: recon_core's internal dispatch.hpp, as recon_volume reaches it.
target_include_directories(
  recon_tsdf
  PUBLIC "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
         "$<BUILD_INTERFACE:${_vr_tsdf_generated_inc}>"
  PRIVATE "${PROJECT_SOURCE_DIR}/src")

# Compile the GLSL kernels to SPIR-V and embed them as constexpr byte arrays
# (each .cpp #includes its generated *.spv.hpp on a PRIVATE path), so recon_tsdf
# ships the technique with no runtime shader files. They include the GLSL
# sensor_frame and recon_volume export, not src/: the camera's pinhole and the
# grid's contract, never its hash table.
include(vr_embed)
vr_embed_shaders(
  recon_tsdf
  INCLUDE_TARGETS
  volumetric_kit::sensor_frame
  volumetric_kit::recon_volume
  SHADERS
  "${CMAKE_CURRENT_SOURCE_DIR}/shaders/tsdf_integrate.comp"
  "${CMAKE_CURRENT_SOURCE_DIR}/shaders/tsdf_clear.comp")

vr_target_warnings(recon_tsdf)
