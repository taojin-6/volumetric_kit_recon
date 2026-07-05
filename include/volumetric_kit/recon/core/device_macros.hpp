// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file device_macros.hpp
/// @brief Host/device portability macros for POD layouts and small inline
///        helpers shared between the CPU and the native-CUDA accelerator.
///
/// On the Vulkan/host build these expand to nothing. Under nvcc they expand to
/// the CUDA execution-space qualifiers, so the shared POD struct definitions
/// and the small inline helpers built on them (e.g. the @c Voxel accessors, the
/// guarded @c normalize) can be reused inside CUDA device kernels -- the
/// optional NVIDIA accelerator layered under the Vulkan baseline (locked
/// decision, 2026-07-04). The Vulkan compute path does not use these: GLSL
/// shaders are separate source that mirror the same POD layouts via scalar
/// block layout (`GL_EXT_scalar_block_layout`), not by including these headers.
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
