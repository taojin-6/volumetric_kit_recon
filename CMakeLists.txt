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
# Objective-C++ for the one file that reaches Metal (sensor/video's
# vt_pictures.mm), at the C++ standard the rest compiles at.
if(APPLE)
  enable_language(OBJCXX)
  set(CMAKE_OBJCXX_STANDARD 17)
  set(CMAKE_OBJCXX_STANDARD_REQUIRED ON)
  set(CMAKE_OBJCXX_EXTENSIONS OFF)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# --- Options -----------------------------------------------------------------
# PROJECT_IS_TOP_LEVEL (set by project() since CMake 3.21, our floor) is ON only
# when we are the root project, not pulled in via add_subdirectory/FetchContent;
# the "build the extras", "generate install rules" and -Werror defaults follow
# from it. A consumer's newer compiler then shows it our new warnings rather
# than failing its build (tests/subproject checks the extras and -Werror).
option(VR_BUILD_EXAMPLES "Build example programs" ${PROJECT_IS_TOP_LEVEL})
option(VR_BUILD_TESTS "Build tests" ${PROJECT_IS_TOP_LEVEL})
option(VR_INSTALL "Generate install/export rules" ${PROJECT_IS_TOP_LEVEL})
option(VR_WARNINGS_AS_ERRORS "Treat first-party compiler warnings as errors"
       ${PROJECT_IS_TOP_LEVEL})
# OFF by default: the Orbbec SDK is a prerequisite you install, not a vendored
# dependency, and it ships no iOS build -- recon cross-compiles to iOS
# unchanged. See cmake/vr_orbbec.cmake for how the build finds it.
option(VR_WITH_ORBBEC
       "Build the Orbbec-SDK-backed code (requires an installed Orbbec SDK)"
       OFF)
# OFF by default for the same reasons: FFmpeg is a prerequisite you install
# (cmake/vr_ffmpeg.cmake), and the iOS build takes none.
option(
  VR_WITH_FFMPEG
  "Build the hardware video decoders (requires FFmpeg, and VR_WITH_CUDA off Apple)"
  OFF)
# OFF by default too: off Apple, the video decoders decode on NVIDIA's hardware
# through CUDA, so VR_WITH_FFMPEG needs it there (cmake/vr_cuda.cmake).
option(
  VR_WITH_CUDA
  "Decode video on NVIDIA's hardware, onto the device (requires the CUDA toolkit)"
  OFF)
option(VR_WITH_ASSIMP
       "Build the Assimp-backed mesh loader (requires an installed Assimp)" OFF)

# Semicolon list of sanitizers (e.g. "address;undefined"); empty = off. Applied
# globally -- see cmake/vr_sanitizers.cmake for why.
set(VR_SANITIZE
    ""
    CACHE STRING "Sanitizers to enable, e.g. address;undefined (empty = off)")

include(vr_warnings)
include(vr_sanitizers)

# --- Dependencies + library --------------------------------------------------
# The Vulkan loader (linked at runtime) + headers, from the system SDK (MoltenVK
# on Apple). Public: recon's headers reach Vulkan through the core's
# core/vulkan/vulkan.hpp, so the requirement is carried on the exported target
# and the package Config re-runs find_dependency(Vulkan). Mirrors
# volumetric_kit_gfx.
find_package(Vulkan REQUIRED)

# The Orbbec SDK (ob::OrbbecSDK), found the same way: installed once, outside
# the repo, and never fetched (the 2026-09-24 decision).
if(VR_WITH_ORBBEC)
  include(vr_orbbec)
endif()
if(VR_WITH_FFMPEG)
  include(vr_ffmpeg)
endif()
if(VR_WITH_CUDA)
  include(vr_cuda)
endif()
if(VR_WITH_ASSIMP)
  find_package(assimp CONFIG REQUIRED)
endif()

# Pinned dependencies: volumetric_kit_core, linked PUBLIC and installed beside
# recon, and implementation dependencies that expose private include paths to
# the tiers below and never become part of the installed link interface.
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
