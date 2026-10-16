# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# A stand-in for a project that finds an installed recon, configured by the
# recon_package_* tests with the core and recon packages they name. Linking the
# umbrella target makes generation check that every imported target in its link
# interface exists; recon_package_glsl builds its kernel on recon's installed
# GLSL (glsl_consumer.cmake). Found or not, recon's package config leaves none
# of its own variables (_vr_*) in the caller's scope.
cmake_minimum_required(VERSION 3.21)
project(vr_package_consumer LANGUAGES CXX)

# The _vr_ variables in this scope, which only recon's package config can have
# set here.
macro(vr_consumer_leftovers out)
  get_cmake_property(${out} VARIABLES)
  list(FILTER ${out} INCLUDE REGEX "^_vr_")
endmacro()

# With VR_CONSUMER_QUIET, a project that can do without recon finds it QUIET and
# reports what it got (recon_package_quiet_not_found).
if(VR_CONSUMER_QUIET)
  find_package(volumetric_kit_recon CONFIG QUIET)
  vr_consumer_leftovers(vr_consumer_left)
  message(STATUS "volumetric_kit_recon found: '${volumetric_kit_recon_FOUND}'"
                 "; left behind: '${vr_consumer_left}'")
  return()
endif()

find_package(volumetric_kit_recon REQUIRED CONFIG)
vr_consumer_leftovers(vr_consumer_left)
if(vr_consumer_left)
  message(FATAL_ERROR "recon's package config left ${vr_consumer_left}")
endif()
add_executable(vr_package_consumer consumer.cpp)
target_link_libraries(vr_package_consumer PRIVATE volumetric_kit::recon)
include("${CMAKE_CURRENT_LIST_DIR}/../glsl_consumer.cmake")
