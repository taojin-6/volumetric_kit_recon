# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# Vendored dependencies (pinned FetchContent). recon takes Vulkan and glm from
# the system (find_package in the root + core CMakeLists), so the only thing
# vendored here is VMA -- header-only, with no system package to rely on across
# our targets (Linux/Android/macOS/iOS/Windows). Pinned to a tag, fetched
# shallow. This subdirectory is added by the root CMakeLists only when it
# exists, so a core-only checkout without a network can still configure.
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
  GIT_TAG v3.3.0
  GIT_SHALLOW TRUE)

FetchContent_MakeAvailable(VulkanMemoryAllocator)

# Expose the vendored header location to the library target (see the SYSTEM
# include + vma_impl.cpp in core/CMakeLists.txt).
set(VR_VMA_INCLUDE_DIR
    "${vulkanmemoryallocator_SOURCE_DIR}/include"
    PARENT_SCOPE)
