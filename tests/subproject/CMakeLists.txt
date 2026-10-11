# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# A stand-in for a sibling that uses recon. With VR_SOURCE_DIR it fetches
# recon's source, as the recon_subproject_* tests configure it: recon is then
# not the top-level project, so its tests, examples and -Werror must default
# off, and the consumer builds the library tiers alone, under its own compiler's
# warnings. Without, it finds an installed recon (camera_only.cmake).
#
# With -DVR_CORE_FIRST=ON it makes the core available first, as an application
# that also fetches calib or gfx does, with VKC_WITH_VULKAN as given, so recon's
# pin yields to it; -DVR_CONSUMER_CORE_VERSION=<version> stamps that version on
# the core, standing in for an older one.
cmake_minimum_required(VERSION 3.21)
project(vr_subproject_consumer LANGUAGES CXX)

if(DEFINED VR_SOURCE_DIR)
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
      message(
        FATAL_ERROR "${_option} must default OFF for a non-top-level recon")
    endif()
  endforeach()
else()
  find_package(volumetric_kit_recon CONFIG REQUIRED)
endif()

# A camera-only recon (VR_CAMERA_ONLY), as calib's solver takes it, makes no
# target that reaches Vulkan (Vulkan's own is visible here only from a package),
# and fetches no dependency of the other tiers.
if(VR_EXPECT_CAMERA_ONLY)
  foreach(_target Vulkan::Vulkan volumetric_kit::core_vulkan
                  volumetric_kit::recon_core)
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

add_executable(vr_camera_consumer camera_consumer.cpp)
target_link_libraries(vr_camera_consumer PRIVATE volumetric_kit::recon_camera)
