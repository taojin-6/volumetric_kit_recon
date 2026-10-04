// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file fwd.hpp
/// @brief Forward declarations of the core's Vulkan foundation, under recon's
///        names.
///
/// A header that only names a `Device`, `Buffer` or `CommandBatch` by pointer
/// or reference includes this rather than the full header -- and so includes no
/// Vulkan, which keeps the capture contract (sensor/raw_frame.hpp) compilable
/// without a GPU API. The classes are volumetric_kit_core's, so they are
/// declared in its namespace and named in recon's; declaring them in recon's
/// namespace instead would make a second, unrelated class of the same name.
///
/// @code
/// #include "volumetric_kit/recon/core/fwd.hpp"
///
/// Status record(CommandBatch& batch, const Device& device);
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

namespace volumetric_kit::recon {

using core::Allocator;
using core::Buffer;
using core::CommandBatch;
using core::Device;
using core::GpuStageScope;
using core::Image;
using core::StorageInput;

}  // namespace volumetric_kit::recon
