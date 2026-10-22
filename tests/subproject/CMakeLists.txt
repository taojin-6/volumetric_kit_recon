# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# A stand-in for a sibling that fetches recon's source from VR_SOURCE_DIR, as
# the recon_subproject_* tests configure it: recon is then not the top-level
# project, so its tests, examples and -Werror must default off, and the consumer
# builds the library tiers alone, under its own compiler's warnings. With all of
# recon, recon_subproject_glsl builds its kernel on recon's exported GLSL
# (glsl_consumer.cmake), and recon_subproject_headers compiles the public
# headers that reach the volume tier, none of which may hold the hash table's
# internals (internal_names.cpp).
#
# With -DVR_CORE_FIRST=ON it makes the core available first, as an application
# that also fetches calib or gfx does, with VKC_WITH_VULKAN as given, so recon's
# pin yields to it; -DVR_CONSUMER_CORE_VERSION=<version> stamps that version on
# the core, standing in for an older one.
cmake_minimum_required(VERSION 3.21)
project(vr_subproject_consumer LANGUAGES CXX)

if(VR_CORE_FIRST)
  include(FetchContent)
  set(_core_source "${FETCHCONTENT_SOURCE_DIR_VOLUMETRIC_KIT_CORE}")
  FetchContent_Declare(volumetric_kit_core SOURCE_DIR "${_core_source}")
  FetchContent_MakeAvailable(volumetric_kit_core)
  if(VR_CONSUMER_CORE_VERSION)
    get_target_property(_core volumetric_kit::core_base ALIASED_TARGET)
    set_target_properties(${_core} PROPERTIES VKC_CORE_VERSION
                                              ${VR_CONSUMER_CORE_VERSION})
  endif()
endif()
add_subdirectory("${VR_SOURCE_DIR}" volumetric_kit_recon)
foreach(_option VR_BUILD_TESTS VR_BUILD_EXAMPLES VR_WARNINGS_AS_ERRORS)
  if(${_option})
    message(FATAL_ERROR "${_option} must default OFF for a non-top-level recon")
  endif()
endforeach()

# A camera-only recon (VR_CAMERA_ONLY) makes no core vulkan tier and no other
# recon tier, and fetches no dependency of the other tiers. That it finds no
# Vulkan is the caller's CMAKE_DISABLE_FIND_PACKAGE_Vulkan, as Vulkan's target
# would be local to recon's directory, not visible here.
if(VR_EXPECT_CAMERA_ONLY)
  foreach(_target volumetric_kit::core_vulkan volumetric_kit::recon_core)
    if(TARGET ${_target})
      message(FATAL_ERROR "a camera-only recon made ${_target}")
    endif()
  endforeach()
  include(FetchContent)
  foreach(_dependency stb tinyply)
    FetchContent_GetProperties(${_dependency})
    if(${_dependency}_POPULATED)
      message(FATAL_ERROR "a camera-only recon fetched ${_dependency}")
    endif()
  endforeach()
endif()

# The umbrella reaches recon_camera, so it must name only tiers that exist.
add_executable(vr_camera_consumer camera_consumer.cpp)
target_link_libraries(vr_camera_consumer PRIVATE volumetric_kit::recon)

if(NOT TARGET volumetric_kit::recon_core)
  return()
endif()
include("${CMAKE_CURRENT_LIST_DIR}/../glsl_consumer.cmake")

# The public headers internal_names.cpp compiles, found in recon's include/ so a
# new one is checked too: every volume header, and every other recon header that
# includes one.
set(_include "${VR_SOURCE_DIR}/include")
file(
  GLOB_RECURSE _headers
  RELATIVE "${_include}"
  "${_include}/volumetric_kit/recon/*.hpp")
set(_volume_headers "")
foreach(_header IN LISTS _headers)
  file(STRINGS "${_include}/${_header}" _includes_volume
       REGEX "#include \"volumetric_kit/recon/volume/")
  if(_header MATCHES "^volumetric_kit/recon/volume/" OR _includes_volume)
    string(APPEND _volume_headers "#include \"${_header}\"\n")
  endif()
endforeach()
if(NOT _volume_headers)
  message(FATAL_ERROR "no public volume header under ${_include}")
endif()
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/vr_volume_headers.hpp"
     "${_volume_headers}")
# Compiled with the umbrella's usage requirements, not linked to it, so the
# check builds this one file, not recon.
add_library(vr_public_headers OBJECT EXCLUDE_FROM_ALL internal_names.cpp)
target_compile_features(vr_public_headers PRIVATE cxx_std_17)
target_include_directories(
  vr_public_headers
  PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}"
    "$<TARGET_PROPERTY:volumetric_kit::recon,INTERFACE_INCLUDE_DIRECTORIES>")
target_compile_definitions(
  vr_public_headers
  PRIVATE
    "$<TARGET_PROPERTY:volumetric_kit::recon,INTERFACE_COMPILE_DEFINITIONS>")
