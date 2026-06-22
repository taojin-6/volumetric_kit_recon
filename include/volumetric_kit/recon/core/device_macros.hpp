// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file device_macros.hpp
/// @brief Host/device portability macros for code shared between the CPU and a
///        GPU backend.
///
/// On a normal host C++ translation unit these expand to nothing. Under nvcc
/// (CUDA) they expand to the CUDA execution-space qualifiers, so the same POD
/// struct definitions and small inline helpers can be reused inside device
/// kernels. (Metal kernels live in `.metal` files with their own qualifiers and
/// include only the POD layouts, not these helpers.)
///
/// Named `VR_*` on purpose: the salvaged code used `VK_*`-prefixed macros,
/// which collide visually with Vulkan's `VK_` namespace -- renamed here.

#if defined(__CUDACC__)
#define VR_DEVICE_HOST __host__ __device__
#define VR_DEVICE __device__
#define VR_HOST __host__
#else
#define VR_DEVICE_HOST
#define VR_DEVICE
#define VR_HOST
#endif

#if defined(__CUDACC__)
#define VR_ALIGN(n) __align__(n)
#else
#define VR_ALIGN(n) alignas(n)
#endif
