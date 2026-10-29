# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# A stand-in for a sibling that fetches recon's source from VR_SOURCE_DIR, as
# the recon_subproject_* tests configure it: recon is then not the top-level
# project, so its tests, examples and -Werror must default off, and the consumer
# builds the library tiers alone, under its own compiler's warnings. With all of
# recon, image I/O links the core's base tier alone, and recon_subproject_glsl
# builds its kernel on recon's exported GLSL (glsl_consumer.cmake). grid's own
# consumer (its tests/subproject) checks grid's.
#
# With -DVR_CORE_FIRST=ON it makes the core available first, as an application
# that also fetches calib or gfx does, with VKC_WITH_VULKAN as given, so recon's
# pin yields to it; -DVR_CONSUMER_CORE_VERSION=<version> stamps that version on
# the core, standing in for an older one. -DVR_REQUIRE_RECON=<version> is the
# consumer's floor on recon.
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

# With -DVR_REQUIRE_RECON=<version>, the consumer needs a recon that new, and
# checks it as a sibling does, with the core's vkc_require_sibling.
if(VR_REQUIRE_RECON)
  vkc_require_sibling(volumetric_kit::recon ${VR_REQUIRE_RECON})
endif()

# A camera-only recon (VR_CAMERA_ONLY) makes no core vulkan tier and no other
# recon tier, adds no grid, and fetches no dependency of the other tiers. That
# it finds no Vulkan is the caller's CMAKE_DISABLE_FIND_PACKAGE_Vulkan, as
# Vulkan's target would be local to recon's directory, not visible here.
if(VR_EXPECT_CAMERA_ONLY)
  foreach(_target volumetric_kit::core_vulkan volumetric_kit::recon_core
                  volumetric_kit::grid)
    if(TARGET ${_target})
      message(FATAL_ERROR "a camera-only recon made ${_target}")
    endif()
  endforeach()
  include(FetchContent)
  foreach(_dependency volumetric_kit_grid stb tinyply)
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

# Image I/O, which a capture app links to write its images, links the core's
# base tier alone: no Vulkan and no mesh tier.
get_target_property(_image_io volumetric_kit::recon_image_io ALIASED_TARGET)
get_target_property(_image_io_links ${_image_io} LINK_LIBRARIES)
if(NOT _image_io_links STREQUAL "volumetric_kit::core_base")
  message(FATAL_ERROR "recon_image_io links ${_image_io_links}, "
                      "not volumetric_kit::core_base alone")
endif()

include("${CMAKE_CURRENT_LIST_DIR}/../glsl_consumer.cmake")
