// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file instance.hpp
/// @brief The Vulkan instance + optional validation, plus compute-capable
///        physical-device selection.

#include <string>
#include <vector>

#include "volumetric_kit/recon/core/export.hpp"
#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/core/vulkan.hpp"

namespace volumetric_kit::recon {

/// @brief Parameters for @ref Instance::create.
struct InstanceConfig {
  /// Application name reported to the driver in `VkApplicationInfo`.
  std::string app_name = "volumetric_kit_recon";
  /// Enable the Khronos validation layer when it is available (a no-op, with a
  /// logged warning, when the layer is not installed).
  bool enable_validation = false;
  /// Extra instance extensions to request (the interop bootstrap may add some).
  std::vector<const char*> extra_instance_extensions;
};

/// @brief Owns a `VkInstance` (and, with validation, its debug messenger).
///
/// Split from @ref Device so a headless compute backend, a multi-GPU setup, or
/// a device shared with the renderer all compose on one instance. Portability
/// enumeration is enabled automatically where the loader offers it, so MoltenVK
/// devices are visible on Apple.
///
/// @code
/// Result<Instance> instance = Instance::create({});
/// if (!instance) return instance.status();
/// Result<VkPhysicalDevice> gpu = instance.value().select_physical_device();
/// if (!gpu) return gpu.status();
/// @endcode
class VR_CORE_API Instance {
 public:
  /// @brief Create the instance (Vulkan >= 1.2; timeline semaphores are 1.2
  ///        core), enabling validation when requested and available and
  ///        portability enumeration when the loader offers it.
  /// @param config  App name, validation toggle, and extra extensions.
  /// @return The instance, or a non-OK @ref Status carrying the
  ///         `vkCreateInstance` `VkResult`.
  static Result<Instance> create(const InstanceConfig& config);

  ~Instance();
  Instance(Instance&& other) noexcept;
  Instance& operator=(Instance&& other) noexcept;
  Instance(const Instance&) = delete;
  Instance& operator=(const Instance&) = delete;

  /// @return The owned `VkInstance` (`VK_NULL_HANDLE` when moved-from).
  VkInstance handle() const noexcept { return instance_; }
  /// @return Whether the Khronos validation layer is enabled on this instance.
  bool validation_enabled() const noexcept { return validation_enabled_; }

  /// @brief Pick the best physical device that exposes a **compute-capable**
  ///        queue family: prefers discrete > integrated > virtual > CPU, and
  ///        requires Vulkan >= 1.2 (the floor @ref Device::create also needs).
  /// @return The chosen device, or @ref Status::Code::Unsupported when none
  ///         qualifies.
  Result<VkPhysicalDevice> select_physical_device() const;

 private:
  Instance() = default;
  void destroy() noexcept;

  VkInstance instance_ = VK_NULL_HANDLE;
  bool validation_enabled_ = false;
};

}  // namespace volumetric_kit::recon
