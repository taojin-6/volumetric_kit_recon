# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# A stand-in for a sibling that fetches recon's source from VR_SOURCE_DIR, as
# the recon_subproject_* tests configure it: recon is then not the top-level
# project, so its tests, examples and -Werror must default off, and the consumer
# builds the library tiers alone, under its own compiler's warnings. recon
# exports no GLSL: grid's own consumer (its tests/subproject) checks grid's, and
# the sensor package's (sensor/tests/subproject) the package's.
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
