# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# --- recon_texture: Tier 5 --------------------------------------------------
# Projective texturing: fills a mesh's per-vertex uv0 with a posed camera's
# image coordinates for the triangles it sees unoccluded (per-vertex color
# fallback elsewhere), via a GLSL compute kernel. Depends on recon_mesh (the
# Mesh it textures) and recon_volume (DepthCameraParams + the projection ABI),
# per the strict left-to-right tier rule.
include(GenerateExportHeader)

set(_vr_texture_generated_inc "${CMAKE_CURRENT_BINARY_DIR}/generated/include")

add_library(recon_texture)
add_library(volumetric_kit::recon_texture ALIAS recon_texture)

target_sources(recon_texture PRIVATE projective_texturer.cpp texture_atlas.cpp)

# Per-tier export header (VR_TEXTURE_API / VR_TEXTURE_STATIC_DEFINE), isolated
# so the macros never collide with another tier's on one include path.
generate_export_header(
  recon_texture
  BASE_NAME
  VR_TEXTURE
  EXPORT_MACRO_NAME
  VR_TEXTURE_API
  EXPORT_FILE_NAME
  "${_vr_texture_generated_inc}/volumetric_kit/recon/texture/export.hpp")

set_target_properties(
  recon_texture
  PROPERTIES CXX_VISIBILITY_PRESET hidden
             VISIBILITY_INLINES_HIDDEN ON
             POSITION_INDEPENDENT_CODE ON)

get_target_property(_vr_texture_type recon_texture TYPE)
if(_vr_texture_type STREQUAL "STATIC_LIBRARY")
  target_compile_definitions(recon_texture PUBLIC VR_TEXTURE_STATIC_DEFINE)
endif()

target_compile_features(recon_texture PUBLIC cxx_std_17)

# PUBLIC: the texture public header exposes mesh types (Mesh/Vertex), the sensor
# package's frame vocabulary (DepthCameraParams) and core types (Result), so
# consumers compile against recon_mesh + recon_volume (and, transitively,
# recon_core) and sensor_frame too.
target_link_libraries(recon_texture PUBLIC recon_mesh recon_volume
                                           volumetric_kit::sensor_frame)

target_include_directories(
  recon_texture PUBLIC "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
                       "$<BUILD_INTERFACE:${_vr_texture_generated_inc}>")

# Compile the GLSL view-selection kernel to SPIR-V and embed it as a constexpr
# byte array (projective_texturer.cpp #includes the generated *.spv.hpp on a
# PRIVATE path), so recon_texture ships the technique with no runtime shader
# files. They include the GLSL sensor_frame exports (pinhole.glsl), not src/.
include(vr_embed)
vr_embed_shaders(
  recon_texture INCLUDE_TARGETS volumetric_kit::sensor_frame SHADERS
  "${CMAKE_CURRENT_SOURCE_DIR}/shaders/texture_score.comp"
  "${CMAKE_CURRENT_SOURCE_DIR}/shaders/texture_multiview.comp")

vr_target_warnings(recon_texture)
