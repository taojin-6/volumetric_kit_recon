# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# A stand-in for a sibling that fetches recon, configured by the
# recon_subproject_defaults test. recon is not the top-level project here, so
# its tests, examples and -Werror must default off: the consumer builds the
# library tiers alone, under its own compiler's warnings.
cmake_minimum_required(VERSION 3.21)
project(vr_subproject_consumer LANGUAGES CXX)

add_subdirectory("${VR_SOURCE_DIR}" volumetric_kit_recon)
foreach(_option VR_BUILD_TESTS VR_BUILD_EXAMPLES VR_WARNINGS_AS_ERRORS)
  if(${_option})
    message(FATAL_ERROR "${_option} must default OFF for a non-top-level recon")
  endif()
endforeach()
