// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file fwd.hpp
/// @brief Forward declarations of the core's Vulkan classes recon's headers
///        name by pointer or reference.
///
/// A header that only names a `Device`, `Buffer` or `CommandBatch` by pointer
/// or reference includes this rather than the full header -- and so includes no
/// Vulkan, which keeps the capture contract (sensor/raw_frame.hpp) compilable
/// without a GPU API. The classes are volumetric_kit_core's, so they are
/// declared in its namespace.
///
/// @code
/// #include "volumetric_kit/recon/core/fwd.hpp"
///
/// core::Status record(core::CommandBatch& batch, const core::Device& device);
/// @endcode

namespace volumetric_kit::core {
class Allocator;
class Buffer;
class CommandBatch;
class Device;
class GpuStageScope;
class Image;
class StorageInput;
}  // namespace volumetric_kit::core
