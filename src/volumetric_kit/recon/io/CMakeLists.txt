# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

include(GenerateExportHeader)

set(_vr_io_generated_inc "${CMAKE_CURRENT_BINARY_DIR}/generated/include")

# --- recon_image_io: host image decoding and PNG encoding ---------------------
# On the family core's base tier alone, with no Vulkan and no other recon tier,
# so a capture app writes its images without the mesh stack. stb's
# implementation is private: no public header or target.
add_library(recon_image_io image_io.cpp stb_impl.cpp)
add_library(volumetric_kit::recon_image_io ALIAS recon_image_io)
generate_export_header(
  recon_image_io
  BASE_NAME
  VR_IMAGE_IO
  EXPORT_MACRO_NAME
  VR_IMAGE_IO_API
  EXPORT_FILE_NAME
  "${_vr_io_generated_inc}/volumetric_kit/recon/io/image_io_export.hpp")
set_target_properties(
  recon_image_io
  PROPERTIES CXX_VISIBILITY_PRESET hidden
             VISIBILITY_INLINES_HIDDEN ON
             POSITION_INDEPENDENT_CODE ON)
get_target_property(_vr_image_io_type recon_image_io TYPE)
if(_vr_image_io_type STREQUAL "STATIC_LIBRARY")
  target_compile_definitions(recon_image_io PUBLIC VR_IMAGE_IO_STATIC_DEFINE)
endif()
target_compile_features(recon_image_io PUBLIC cxx_std_17)
# Catch backend / allocation exceptions at the Status boundary even when a
# consuming application globally disables exceptions. Keep this private.
target_compile_options(recon_image_io PRIVATE -fexceptions)
# PUBLIC: the header returns the core's Status/Result.
target_link_libraries(recon_image_io PUBLIC volumetric_kit::core_base)
target_include_directories(
  recon_image_io PUBLIC "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
                        "$<BUILD_INTERFACE:${_vr_io_generated_inc}>")
target_include_directories(recon_image_io SYSTEM
                           PRIVATE "${VR_STB_INCLUDE_DIR}")
vr_target_warnings(recon_image_io)

# Only the vendored implementation TU opts out of first-party warnings.
set_source_files_properties(stb_impl.cpp PROPERTIES COMPILE_OPTIONS -w)
