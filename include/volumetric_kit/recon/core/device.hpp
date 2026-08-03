// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file device.hpp
/// @brief The logical device: a compute (+ transfer) queue, a command pool, and
///        the create-or-adopt seam that lets recon run standalone or share one
///        `VkDevice` with the renderer.

#include <cstdint>
#include <functional>
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
  /// Queue capabilities at least one assigned queue must carry. recon needs a
  /// **compute** queue; a compute-capable family implicitly supports transfer
  /// (Vulkan guarantees transfer operations on any compute or graphics queue),
  /// so only the compute bit is required. An embedder building the shared
  /// device must not additionally demand `VK_QUEUE_TRANSFER_BIT`, which a
  /// conformant compute family may legally not advertise.
  VkQueueFlags queue_flags = VK_QUEUE_COMPUTE_BIT;
  /// Device extensions to enable.
  std::vector<const char*> device_extensions;
  /// Core (1.0) features to enable.
  VkPhysicalDeviceFeatures features = {};
  /// `timelineSemaphore` (1.2 core) — recon's sync primitive and the interop
  /// handoff.
  bool timeline_semaphore = true;
  /// `scalarBlockLayout` (1.2 core) — the buffer ABI every recon compute shader
  /// reads its POD structs through (2026-07-05); required.
  bool scalar_block_layout = true;
};

/// @brief A `VkDevice` the caller already created, plus what the caller ENABLED
///        on it, handed to @ref Device::adopt.
///
/// The `enabled_*` fields exist because Vulkan gives no way to query which
/// extensions or features were enabled at device-creation time; the creator
/// declares them so @ref Device::adopt can verify recon's needs are met. The
/// pointed-to extension array need only outlive the `adopt` call.
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

  /// Device extensions the creator enabled on `device` (for `adopt`'s
  /// set-comparison verify).
  const char* const* enabled_device_extensions = nullptr;
  std::uint32_t enabled_device_extension_count = 0;
  /// Core (1.0) features the creator enabled on `device`. @ref Device::adopt
  /// rejects the device if any feature recon's @ref DeviceConfig requests is
  /// not set here.
  VkPhysicalDeviceFeatures enabled_features = {};
  /// Whether the creator enabled `timelineSemaphore` on `device`. It is Vulkan
  /// 1.2 core but must still be *enabled* at device creation, and that cannot
  /// be queried back, so the creator declares it; recon's default config
  /// requires it.
  bool enabled_timeline_semaphore = false;
  /// Whether the creator enabled `scalarBlockLayout` on `device` (1.2 core, but
  /// must be enabled at creation and can't be queried back). recon requires it.
  bool enabled_scalar_block_layout = false;
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
  /// @brief The capabilities @ref compute_family advertises (`VK_QUEUE_*`).
  ///
  /// recon requires only `VK_QUEUE_COMPUTE_BIT`, so the family it is handed may
  /// be compute-*only* -- a dedicated async-compute family on a discrete GPU,
  /// or anything an embedder assigns through @ref adopt. That matters because
  /// Vulkan permits a pipeline barrier to name only stages the recording
  /// command buffer's queue family supports, and
  /// `VK_PIPELINE_STAGE_VERTEX_INPUT_BIT` requires `VK_QUEUE_GRAPHICS_BIT`; the
  /// shared @ref dispatch consults this before widening its destination scope
  /// for a renderer. Read from the driver at create/adopt, never assumed.
  VkQueueFlags compute_family_flags() const noexcept {
    return compute_family_flags_;
  }
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

  /// @brief Submit to the compute queue, holding @ref submit_mutex when the
  ///        queue is shared with another library (an adopted device).
  ///
  /// Vulkan requires queue submits be externally synchronized; on a shared
  /// queue the neutral bootstrap hands recon a mutex, and every submit must
  /// hold it. Prefer @ref submit_single_time for a one-shot dispatch; this is
  /// the lower-level primitive for a caller batching its own command buffers.
  /// @param count    Number of `VkSubmitInfo`s in @p submits.
  /// @param submits  The submit batch.
  /// @param fence    Fence signalled on completion (may be `VK_NULL_HANDLE`).
  /// @return The `VkResult` from `vkQueueSubmit`.
  VkResult queue_submit(std::uint32_t count, const VkSubmitInfo* submits,
                        VkFence fence) const;

  /// @brief Record a one-time command buffer, submit it to the compute queue,
  ///        and block until the GPU finishes — the simplest dispatch primitive.
  ///
  /// Allocates a primary command buffer from @ref command_pool, begins it
  /// (`ONE_TIME_SUBMIT`), invokes @p record to fill it (bind pipeline, bind
  /// descriptors, push constants, dispatch, barriers), then ends, submits
  /// (through @ref queue_submit, so it is shared-queue-safe), and waits on an
  /// internal fence. Every transient — command buffer and fence — is freed
  /// before returning. Blocking, so it is a bring-up / single-shot primitive;
  /// the fusion tiers will batch many dispatches per submit on their own.
  ///
  /// @warning Not thread-safe: it allocates, records, and frees a command
  ///          buffer on @ref command_pool, which Vulkan requires be externally
  ///          synchronized. @ref submit_mutex guards only the queue submit, not
  ///          the pool, so concurrent calls on one @ref Device must be
  ///          serialized by the caller.
  /// @param record  Records compute commands into the given command buffer.
  /// @return OK once the work completes, or a non-OK @ref Status if any Vulkan
  ///         step fails.
  Status submit_single_time(
      const std::function<void(VkCommandBuffer)>& record) const;

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
  VkQueueFlags compute_family_flags_ = 0;
  VkQueue compute_queue_ = VK_NULL_HANDLE;
};

}  // namespace volumetric_kit::recon
