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
  /// Request `VK_EXT_debug_utils` whenever the loader offers it, **independent
  /// of** @ref enable_validation.
  ///
  /// On by default because the extension is what lets a GPU profiler name what
  /// it is looking at: @ref Device resolves the label entry points from it, so
  /// every dispatch carries its kernel's name and every buffer recon allocates
  /// carries its own (see @ref Device::set_object_name). Nsight renders those
  /// as trace ranges and named resources; MoltenVK maps them onto Metal debug
  /// groups and `MTLResource` labels, so one instance flag serves the profiler
  /// on either platform.
  ///
  /// Costs nothing when no profiler is attached -- the label entry points are
  /// driver stubs, so the labels are a predictable branch and nothing more --
  /// which is the whole reason this defaults on rather than sitting behind a
  /// build flag: attaching a profiler to a Release binary must not require
  /// rebuilding the binary being profiled.
  ///
  /// Set false only to hold a shipping instance to the extensions it strictly
  /// needs; @ref Instance::debug_utils_enabled then reports false and every
  /// label site becomes a no-op.
  bool request_debug_utils = true;
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
  ///
  /// When validation is enabled and `VK_EXT_debug_utils` is available, a debug
  /// messenger is created that routes layer output through the core log handler
  /// (@ref log_message), so diagnostics reach a consumer's installed sink
  /// rather than the layer's default stderr. If the extension is missing the
  /// layer still runs (a warning is logged, output goes to its default sink).
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
  /// @return Whether `VK_EXT_debug_utils` is enabled on this instance -- the
  ///         precondition for @ref Device resolving the label entry points.
  ///
  /// False when @ref InstanceConfig::request_debug_utils was cleared or the
  /// loader does not offer the extension. An embedder creating its own
  /// instance and handing recon the device reports the same fact through
  /// @ref AdoptedDevice::enabled_debug_utils, Vulkan offering no way to query
  /// it back.
  bool debug_utils_enabled() const noexcept { return debug_utils_enabled_; }

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
  // The validation-layer message sink, created only when validation is enabled
  // and VK_EXT_debug_utils is available; destroyed before the instance. Reset
  // on every ownership transfer.
  VkDebugUtilsMessengerEXT debug_messenger_ = VK_NULL_HANDLE;
  bool validation_enabled_ = false;
  // Whether VK_EXT_debug_utils was enabled at creation -- the messenger above
  // needs it, but so do the label entry points Device resolves, which is why
  // this is tracked separately from validation_enabled_. Reset on every
  // ownership transfer.
  bool debug_utils_enabled_ = false;
};

}  // namespace volumetric_kit::recon
