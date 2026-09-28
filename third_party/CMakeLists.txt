# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# Vendored dependencies (pinned FetchContent). recon takes Vulkan and glm from
# the system (find_package in the root + core CMakeLists), so what is vendored
# here is header-only with no system package to rely on across our targets
# (Linux/Android/macOS/iOS/Windows): VMA, and nlohmann/json below. Pinned to a
# tag, fetched shallow. recon_core compiles VMA in one TU, so it is a hard
# dependency: the root CMakeLists adds this subdirectory unconditionally and the
# fetch below runs on the first configure (a network is required then;
# afterwards the FetchContent cache serves it).
#
# The include dir is exported to the parent scope; the core tier compiles VMA's
# implementation in a single TU (core/vma_impl.cpp) and attaches this dir as a
# SYSTEM include so VMA's headers stay out of our -Werror surface.

include(FetchContent)

# --- VMA: Vulkan Memory Allocator (header-only; built in one recon_core TU) ---
# Matches volumetric_kit_gfx's pin so the two siblings allocate through the same
# VMA version -- relevant once they share a VkDevice at the interop seam.
FetchContent_Declare(
  VulkanMemoryAllocator
  GIT_REPOSITORY
    https://github.com/GPUOpen-LibrariesAndSDKs/VulkanMemoryAllocator.git
  GIT_TAG v3.4.0
  GIT_SHALLOW TRUE)

FetchContent_MakeAvailable(VulkanMemoryAllocator)

# Expose the vendored header location to the library target (see the SYSTEM
# include + vma_impl.cpp in core/CMakeLists.txt).
set(VR_VMA_INCLUDE_DIR
    "${vulkanmemoryallocator_SOURCE_DIR}/include"
    PARENT_SCOPE)

# --- nlohmann/json (header-only; parsed in one recon_sensor TU) ---------------
# The rig pose file (sensor/rig_poses.hpp) is JSON. The release's source
# archive, hash-pinned; only its include/ tree is used, so its CMake is not run
# (SOURCE_SUBDIR names a directory the archive does not have). Like VMA, it is a
# PRIVATE SYSTEM include of the one target that parses, never part of the
# installed interface. volumetric_kit_gfx carries the same library through
# tinygltf.
if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW) # extracted files get the extraction time
endif()
FetchContent_Declare(
  nlohmann_json
  URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz
  URL_HASH
    SHA256=42f6e95cad6ec532fd372391373363b62a14af6d771056dbfc86160e6dfff7aa
  SOURCE_SUBDIR do-not-add-json-cmake)
FetchContent_MakeAvailable(nlohmann_json)

set(VR_JSON_INCLUDE_DIR
    "${nlohmann_json_SOURCE_DIR}/include"
    PARENT_SCOPE)
