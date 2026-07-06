// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The single translation unit that instantiates the Vulkan Memory Allocator.
// VMA is header-only: exactly one TU in the whole library must define
// VMA_IMPLEMENTATION, and this is it. No other file includes <vk_mem_alloc.h>
// -- the allocator/buffer wrappers keep the VmaAllocator/VmaAllocation handles
// behind their own headers so VMA never reaches a public include or a consumer.
//
// VMA_STATIC_VULKAN_FUNCTIONS resolves Vulkan entry points against the
// link-time loader (Vulkan::Vulkan) that the rest of recon already links --
// matching the loader choice documented in core/vulkan.hpp. If recon later
// adopts volk for the iOS/Android loader, this switches to the
// dynamic-functions path here (plus a functions table fed to
// vmaCreateAllocator), with no churn elsewhere.

#define VMA_STATIC_VULKAN_FUNCTIONS 1
#define VMA_DYNAMIC_VULKAN_FUNCTIONS 0

// Vulkan is reached through the umbrella header, never <vulkan/vulkan.h>
// directly (the loader-choice rule in core/vulkan.hpp) -- VMA needs the Vulkan
// prototypes in scope before its implementation is expanded below.
#include "volumetric_kit/recon/core/vulkan.hpp"

#define VMA_IMPLEMENTATION
#include <vk_mem_alloc.h>
