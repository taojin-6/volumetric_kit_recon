# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# --- recon_track --------------------------------------------------------------
# Tracking a self-posed camera's frames against a model on the device: the one
# backend interface (IFitter) and its kinds (PointToPlane, projective
# point-to-plane ICP over grid's RayCaster), the per-chain judge, and the
# Tracker that runs a driver and its shadows on the same frames. It links
# grid_volume (the model and its cast) and the sensor package's frame contract,
# not tsdf: the filters it casts through arrive as values.
include(GenerateExportHeader)

set(_vr_track_generated_inc "${CMAKE_CURRENT_BINARY_DIR}/generated/include")

add_library(recon_track)
add_library(volumetric_kit::recon_track ALIAS recon_track)

target_sources(recon_track PRIVATE correction_filter.cpp create_fitter.cpp
                                   point_to_plane.cpp tracker.cpp)

# Per-tier export header (VR_TRACK_API / VR_TRACK_STATIC_DEFINE).
generate_export_header(
  recon_track
  BASE_NAME
  VR_TRACK
  EXPORT_MACRO_NAME
  VR_TRACK_API
  EXPORT_FILE_NAME
  "${_vr_track_generated_inc}/volumetric_kit/recon/track/export.hpp")

set_target_properties(
  recon_track
  PROPERTIES CXX_VISIBILITY_PRESET hidden
             VISIBILITY_INLINES_HIDDEN ON
             POSITION_INDEPENDENT_CODE ON)

get_target_property(_vr_track_type recon_track TYPE)
if(_vr_track_type STREQUAL "STATIC_LIBRARY")
  target_compile_definitions(recon_track PUBLIC VR_TRACK_STATIC_DEFINE)
endif()

target_compile_features(recon_track PUBLIC cxx_std_17)

# PUBLIC: the headers expose grid's volume types (VoxelBlockGrid, CastFilter),
# core types and the frame contract (DeviceFrame).
target_link_libraries(recon_track PUBLIC recon_core grid_volume
                                         volumetric_kit::sensor_frame)

target_include_directories(
  recon_track PUBLIC "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
                     "$<BUILD_INTERFACE:${_vr_track_generated_inc}>")

# The kernels include the sensor's exported pinhole and the tier's own
# track_common.glsl, through recon's src/.
include(vr_embed)
vr_embed_shaders(
  recon_track
  INCLUDE_TARGETS
  volumetric_kit::sensor_frame
  WITH_SRC
  SHADERS
  "${CMAKE_CURRENT_SOURCE_DIR}/shaders/track_filter.comp"
  "${CMAKE_CURRENT_SOURCE_DIR}/shaders/track_down.comp"
  "${CMAKE_CURRENT_SOURCE_DIR}/shaders/track_vmap.comp"
  "${CMAKE_CURRENT_SOURCE_DIR}/shaders/track_scale.comp"
  "${CMAKE_CURRENT_SOURCE_DIR}/shaders/track_reduce_point_to_plane.comp"
  "${CMAKE_CURRENT_SOURCE_DIR}/shaders/track_solve.comp"
  "${CMAKE_CURRENT_SOURCE_DIR}/shaders/track_pose.comp")

vr_target_warnings(recon_track)
