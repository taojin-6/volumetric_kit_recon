// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file device.hpp
/// @brief The logical device: a compute (+ transfer) queue, a command pool, and
///        the create-or-adopt seam that lets recon run standalone or share one
///        `VkDevice` with the renderer.

#include <cstdint>
#include <mutex>
#include <vector>

#include "volumetric_kit/recon/core/export.hpp"
#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/core/vulkan.hpp"

namespace volumetric_kit::recon {

/// @brief Parameters for @ref Device::create / @ref Device::adopt.
struct DeviceConfig {
  /// Core (1.0) device features to enable (fed into
  /// `VkPhysicalDeviceFeatures2`).
  VkPhysicalDeviceFeatures features = {};
  /// Device extensions to enable beyond those implied above; each is validated
  /// against the device's supported list (create) or its declared enabled list
  /// (adopt).
  std::vector<const char*> extra_device_extensions;
};

/// @brief What recon needs from a `VkDevice`, published so an embedder sharing
///        one device across libraries can merge everyone's requirements, create
///        a device satisfying the union, and hand it to each via @ref
///        Device::adopt.
///
/// Raw Vulkan data only, so the bundle carries no type a sibling library must
/// import. Derived from a @ref DeviceConfig by @ref Device::requirements.
struct DeviceRequirements {
  /// Minimum device Vulkan version (recon targets 1.2 core: timeline
  /// semaphores).
  std::uint32_t api_version = VK_API_VERSION_1_2;
  /// Queue capabilities at least one assigned queue must carry.
  VkQueueFlags queue_flags = VK_QUEUE_COMPUTE_BIT | VK_QUEUE_TRANSFER_BIT;
  /// Device extensions to enable.
  std::vector<const char*> device_extensions;
  /// Core (1.0) features to enable.
  VkPhysicalDeviceFeatures features = {};
  /// `timelineSemaphore` (1.2 core) — recon's sync primitive and the interop
  /// handoff.
  bool timeline_semaphore = true;
};

/// @brief A `VkDevice` the caller already created, plus what the caller ENABLED
///        on it, handed to @ref Device::adopt.
///
/// The `enabled_*` fields exist because Vulkan gives no way to query which
/// extensions/features were enabled at device-creation time; the creator
/// declares them so @ref Device::adopt can verify by set-comparison. The
/// pointed-to arrays need only outlive the `adopt` call.
struct AdoptedDevice {
  VkInstance instance = VK_NULL_HANDLE;
  VkPhysicalDevice physical_device = VK_NULL_HANDLE;
  VkDevice device = VK_NULL_HANDLE;

  /// The compute-capable queue assigned to recon, and its family.
  std::uint32_t compute_family = 0;
  VkQueue compute_queue = VK_NULL_HANDLE;
  /// When non-null, the assigned queue is shared with another library; every
  /// `vkQueueSubmit` on it must hold this mutex (Vulkan requires queue submits
  /// be externally synchronized).
  std::mutex* submit_mutex = nullptr;

  /// What the creator enabled on `device` (for `adopt`'s set-comparison
  /// verify).
  const char* const* enabled_device_extensions = nullptr;
  std::uint32_t enabled_device_extension_count = 0;
};

/// @brief Owns *or borrows* a `VkDevice`, its compute (+ transfer) queue, and a
///        compute command pool. Holds no surface/swapchain — recon is headless.
///
/// @warning The @ref Instance / device in the create or adopt inputs must
///          outlive this object; it stores only borrowed handles.
///
/// @code
/// Result<Device> device = Device::create(instance.handle(), physical, {});
/// if (!device) return device.status();
/// @endcode
class VR_CORE_API Device {
 public:
  /// @brief Create and OWN a logical device on @p physical (Vulkan >= 1.2 with
  /// a
  ///        compute queue family), enabling `timelineSemaphore` and any
  ///        `config` features/extensions.
  /// @param instance  The instance @p physical belongs to; must outlive the
  ///                  returned device (a lifetime contract, unused at
  ///                  creation).
  /// @param physical  The physical device to build on.
  /// @param config    Features and extensions to enable.
  /// @return The device, or a non-OK @ref Status:
  ///         @ref Status::Code::InvalidArgument for a null @p physical;
  ///         @ref Status::Code::Unsupported below Vulkan 1.2, without a compute
  ///         family, or missing a requested extension / `timelineSemaphore`.
  static Result<Device> create(VkInstance instance, VkPhysicalDevice physical,
                               const DeviceConfig& config);

  /// @brief Adopt a `VkDevice` an embedder already created, **without owning
  ///        it** — the destructor leaves the `VkDevice` alone (it still creates
  ///        and owns its own command pool). Use this to run recon on a device
  ///        shared with the renderer.
  /// @param adopted  The existing handles, the compute queue assigned to recon,
  ///                 and what the creator enabled on the device.
  /// @param config   The same config recon would pass to @ref create; its needs
  ///                 are validated against @p adopted.
  /// @return The (non-owning) device, or a non-OK @ref Status:
  ///         @ref Status::Code::InvalidArgument for null handles;
  ///         @ref Status::Code::Unsupported when @p adopted is below
  ///         Vulkan 1.2, its assigned queue family lacks compute, or a required
  ///         extension/feature was not enabled on it.
  static Result<Device> adopt(const AdoptedDevice& adopted,
                              const DeviceConfig& config);

  /// @brief The device requirements implied by @p config — the set an embedder
  ///        merges with other libraries' to build one shared device.
  static DeviceRequirements requirements(const DeviceConfig& config);

  ~Device();
  Device(Device&& other) noexcept;
  Device& operator=(Device&& other) noexcept;
  Device(const Device&) = delete;
  Device& operator=(const Device&) = delete;

  /// @return The logical device (`VK_NULL_HANDLE` when moved-from).
  VkDevice handle() const noexcept { return device_; }
  /// @return The physical device it was created on / adopted from.
  VkPhysicalDevice physical_device() const noexcept { return physical_; }
  /// @return The compute-capable queue-family index.
  std::uint32_t compute_family() const noexcept { return compute_family_; }
  /// @return The compute queue.
  VkQueue compute_queue() const noexcept { return compute_queue_; }
  /// @return The compute-family command pool (created `RESET_COMMAND_BUFFER`).
  VkCommandPool command_pool() const noexcept { return command_pool_; }
  /// @return Whether this wrapper owns (and will destroy) the `VkDevice`.
  ///         `false` for a device obtained through @ref adopt.
  bool owns_device() const noexcept { return owns_device_; }
  /// @return The mutex guarding submits on a shared queue, or `nullptr` when
  /// the
  ///         queue is exclusively this device's.
  std::mutex* submit_mutex() const noexcept { return submit_mutex_; }

 private:
  Device() = default;
  void destroy() noexcept;

  VkPhysicalDevice physical_ = VK_NULL_HANDLE;
  VkDevice device_ = VK_NULL_HANDLE;
  VkCommandPool command_pool_ = VK_NULL_HANDLE;
  // False when the device was adopted (@ref adopt): destroy() then tears down
  // only the command pool this wrapper made and leaves the VkDevice to its
  // owner. Reset on every ownership transfer.
  bool owns_device_ = true;
  std::mutex* submit_mutex_ = nullptr;
  std::uint32_t compute_family_ = 0;
  VkQueue compute_queue_ = VK_NULL_HANDLE;
};

}  // namespace volumetric_kit::recon
