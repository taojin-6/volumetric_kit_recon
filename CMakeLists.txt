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
# VS_WITH_REALSENSE, VS_WITH_DECKLINK, VS_WITH_FFMPEG, VS_WITH_CUDA,
# VS_WITH_FFS, VS_WITH_PORTAUDIO); a -D on recon's configure reaches it as a
# cache variable, and a driver it builds builds its live example
# (examples/CMakeLists.txt), as its stereo depth (sensor_ffs) gives
# fuse_realsense FFS depth and its microphone (sensor_microphone) the live
# examples source.audio; fuse_decklink is built always and runs only with the
# DeckLink driver and FFS both. Its camera-only build is the package's alone:
# recon needs its frame contract.
if(PROJECT_IS_TOP_LEVEL)
  set(_vr_examples_default ON)
else()
  set(_vr_examples_default OFF)
endif()
option(VR_BUILD_EXAMPLES "Build example programs" ${_vr_examples_default})
option(VR_BUILD_TESTS "Build tests" ${PROJECT_IS_TOP_LEVEL})
option(VR_WARNINGS_AS_ERRORS "Treat first-party compiler warnings as errors"
       ${PROJECT_IS_TOP_LEVEL})
# The phone lane's registration by the origin tag (examples/common/live_tag.hpp)
# runs on volumetric_kit_calib's tag detector and solve, fetched pinned
# (third_party/) for the examples and tests, and so on OpenCV 5, an installed
# prerequisite. Off, a phone needs source.mobile_registration. CMake 3.28 for
# the fetch's EXCLUDE_FROM_ALL, which builds only what the examples link.
option(
  VR_WITH_CALIB
  "fuse_orbbec registers a phone by the origin tag (fetches calib; OpenCV 5; CMake 3.28)"
  OFF)
if(VR_WITH_CALIB)
  if(CMAKE_VERSION VERSION_LESS 3.28)
    message(FATAL_ERROR "VR_WITH_CALIB needs CMake 3.28 or newer")
  endif()
  # Found here, so its imported targets reach the tests, which draw markers.
  find_package(OpenCV 5.0 CONFIG QUIET COMPONENTS core imgproc objdetect
                                                  geometry calib)
  if(NOT OpenCV_FOUND)
    message(
      FATAL_ERROR
        "VR_WITH_CALIB=ON needs OpenCV 5: Homebrew's opencv on macOS; on Linux "
        "calib's tools/install-opencv.sh, then "
        "-DOpenCV_DIR=<prefix>/lib/cmake/opencv5")
  endif()
endif()

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
# volumetric_kit_grid, linked PUBLIC. third_party/ refuses an arriving sibling
# older than its floor below (and a core without the config tier the examples
# and tests need). A floor is the oldest release with everything recon, its
# examples and its tests use; the phone lane set all three: core 0.49.0's
# optional source.mobile_registration, sensor 0.40.0's DepthNoise::known and
# grid 0.15.0's StreamClient::send_note. CHANGELOG records each rise and why.
set(VR_CORE_MIN_VERSION 0.49.0)
set(VR_SENSOR_MIN_VERSION 0.40.0)
set(VR_GRID_MIN_VERSION 0.15.0)
add_subdirectory(third_party)
# The tag is found in the phone's pictures, which only sensor's video decoders
# read: refused here rather than once a phone has dialled.
if(VR_WITH_CALIB AND NOT TARGET volumetric_kit::sensor_video)
  message(
    FATAL_ERROR
      "VR_WITH_CALIB=ON needs sensor's video decoders (sensor_video): built "
      "on macOS always, on Linux with -DVS_WITH_FFMPEG=ON and "
      "-DVS_WITH_CUDA=ON (NVDEC)")
endif()

add_subdirectory(src)

if(VR_BUILD_TESTS AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt")
  enable_testing()
  add_subdirectory(tests)
endif()

if(VR_BUILD_EXAMPLES AND EXISTS
                         "${CMAKE_CURRENT_SOURCE_DIR}/examples/CMakeLists.txt")
  add_subdirectory(examples)
  # session_check, a session checked by hand on the examples' link, built with
  # the tests (tools/session_check).
  if(VR_BUILD_TESTS)
    add_subdirectory(tools/session_check)
  endif()
endif()
