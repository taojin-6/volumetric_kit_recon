# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

cmake_minimum_required(VERSION 3.21)

project(
  volumetric_kit_recon
  VERSION 0.1.0
  DESCRIPTION
    "Production-grade Vulkan TSDF reconstruction backend (MoltenVK on Apple) for the volumetric_kit family, on its grid package"
  HOMEPAGE_URL "https://github.com/taojin-6/volumetric_kit_recon"
  LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# --- Options -----------------------------------------------------------------
# PROJECT_IS_TOP_LEVEL (set by project() since CMake 3.21, our floor) is ON only
# when we are the root project, not pulled in via add_subdirectory/FetchContent;
# the "build the extras" and -Werror defaults follow from it. A consumer's newer
# compiler then shows it our new warnings rather than failing its build
# (tests/subproject checks the extras and -Werror). recon has no install rules:
# a consumer fetches it by commit (DECISIONS.md, 2026-10-22).
#
# The camera and sensor options are volumetric_kit_sensor's (VS_WITH_ORBBEC,
# VS_WITH_FFMPEG, VS_WITH_CUDA); a -D on recon's configure reaches it as a cache
# variable. Its camera-only build is the package's alone: recon needs its frame
# contract.
if(PROJECT_IS_TOP_LEVEL)
  set(_vr_examples_default ON)
else()
  set(_vr_examples_default OFF)
endif()
option(VR_BUILD_EXAMPLES "Build example programs" ${_vr_examples_default})
option(VR_BUILD_TESTS "Build tests" ${PROJECT_IS_TOP_LEVEL})
option(VR_WARNINGS_AS_ERRORS "Treat first-party compiler warnings as errors"
       ${PROJECT_IS_TOP_LEVEL})

# Semicolon list of sanitizers (e.g. "address;undefined"); empty = off. Applied
# globally, the fetched packages included -- see cmake/vr_sanitizers.cmake for
# why.
set(VR_SANITIZE
    ""
    CACHE STRING "Sanitizers to enable, e.g. address;undefined (empty = off)")

if(APPLE)
  # Objective-C++ for the sensor package's files that reach Metal
  # (sensor_apple's pixel_buffer_importer.mm), which recon's programs link:
  # enabled here, the highest directory common to them all, at the C++ standard
  # the rest compiles at.
  enable_language(OBJCXX)
  set(CMAKE_OBJCXX_STANDARD 17)
  set(CMAKE_OBJCXX_STANDARD_REQUIRED ON)
  set(CMAKE_OBJCXX_EXTENSIONS OFF)
endif()

include(vr_warnings)
include(vr_sanitizers)

# --- Dependencies + library --------------------------------------------------
# The Vulkan loader (linked at runtime) + headers, from the system SDK (MoltenVK
# on Apple). Public: recon's headers reach Vulkan through the core's
# core/vulkan/vulkan.hpp, so the requirement is carried on recon's targets.
# Mirrors volumetric_kit_gfx.
find_package(Vulkan REQUIRED)

# Pinned dependencies: volumetric_kit_core, volumetric_kit_sensor and
# volumetric_kit_grid, linked PUBLIC. The oldest core recon's libraries build
# with is checked in the core that arrives (third_party/, which also checks the
# config tier the examples and tests need): 0.31.0, whose RigViewer scenarios
# name the frames rig_viewer saves offscreen (output.screenshot); 0.30.0
# brought the output.max_viewers fuse_orbbec serves by, 0.29.0 the
# vulkan/fwd.hpp the public headers include, and 0.28.0 the test support the
# tests run on. The oldest sensor is 0.1.0, its first, whose sensor_frame
# recon's libraries link; the oldest grid is VR_GRID_MIN_VERSION: 0.3.0, whose
# codec_outputs passes output.max_viewers to the server.
set(VR_CORE_MIN_VERSION 0.31.0)
set(VR_SENSOR_MIN_VERSION 0.1.0)
set(VR_GRID_MIN_VERSION 0.3.0)
add_subdirectory(third_party)

add_subdirectory(src)

if(VR_BUILD_TESTS AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
  enable_testing()
  add_subdirectory(tests)
endif()

if(VR_BUILD_EXAMPLES AND EXISTS
                         "${CMAKE_CURRENT_SOURCE_DIR}/examples/CMakeLists.txt")
  add_subdirectory(examples)
endif()
