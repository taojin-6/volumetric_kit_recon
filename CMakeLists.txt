# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

cmake_minimum_required(VERSION 3.21)

project(
  volumetric_kit_recon
  VERSION 0.0.1
  DESCRIPTION
    "Production-grade Vulkan TSDF reconstruction + compression backend (MoltenVK on Apple) for the volumetric_kit family"
  HOMEPAGE_URL "https://github.com/taojin-6/volumetric_kit_recon"
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# --- Options -----------------------------------------------------------------
# PROJECT_IS_TOP_LEVEL (set by project() since CMake 3.21, our floor) is ON only
# when we are the root project, not pulled in via add_subdirectory/FetchContent;
# the "build the extras" and "generate install rules" defaults follow from it.
option(VR_BUILD_EXAMPLES "Build example programs" ${PROJECT_IS_TOP_LEVEL})
option(VR_BUILD_TESTS "Build tests" ${PROJECT_IS_TOP_LEVEL})
option(VR_INSTALL "Generate install/export rules" ${PROJECT_IS_TOP_LEVEL})
option(VR_WARNINGS_AS_ERRORS "Treat first-party compiler warnings as errors" ON)

# Semicolon list of sanitizers (e.g. "address;undefined"); empty = off. Applied
# globally -- see cmake/vr_sanitizers.cmake for why.
set(VR_SANITIZE
    ""
    CACHE STRING "Sanitizers to enable, e.g. address;undefined (empty = off)")

include(vr_warnings)
include(vr_sanitizers)

# --- Dependencies + library --------------------------------------------------
# The Vulkan loader (linked at runtime) + headers, from the system SDK (MoltenVK
# on Apple). Public: the core tier reaches Vulkan through core/vulkan.hpp, so the
# requirement is carried on the exported target and the package Config re-runs
# find_dependency(Vulkan). Mirrors volumetric_kit_gfx.
find_package(Vulkan REQUIRED)

# third_party/ is optional today (only the header-only deps a later tier vendors,
# e.g. VMA); pick it up automatically once it exists. TODO: drop the guard once
# it does.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/third_party/CMakeLists.txt")
  add_subdirectory(third_party)
endif()

add_subdirectory(src)

if(VR_BUILD_TESTS AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
  enable_testing()
  add_subdirectory(tests)
endif()

if(VR_BUILD_EXAMPLES AND EXISTS
                         "${CMAKE_CURRENT_SOURCE_DIR}/examples/CMakeLists.txt")
  add_subdirectory(examples)
endif()
