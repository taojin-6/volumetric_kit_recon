// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/recon/core/device.hpp"

#include <algorithm>
#include <cstddef>
#include <cstring>
#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "vk_physical_device.hpp"
#include "volumetric_kit/recon/core/gpu_timer.hpp"
#include "volumetric_kit/recon/core/vk_result.hpp"

namespace volumetric_kit::recon {
namespace {

// Runs `cleanup` when it leaves scope, unless release()d first.
// submit_single_time uses it to free its one-shot transients on every exit
// path -- and to deliberately leak them (rather than free objects the GPU may
// still be using) when the fence wait fails.
class ScopeGuard {
 public:
  explicit ScopeGuard(std::function<void()> cleanup)
      : cleanup_(std::move(cleanup)) {}
  ScopeGuard(const ScopeGuard&) = delete;
  ScopeGuard& operator=(const ScopeGuard&) = delete;
  ~ScopeGuard() {
    if (cleanup_) {
      cleanup_();
    }
  }
  void release() noexcept { cleanup_ = nullptr; }

 private:
  std::function<void()> cleanup_;
};

// VK_KHR_portability_subset's name macro lives in vulkan_beta.h (gated by
// VK_ENABLE_BETA_EXTENSIONS); the string is stable, so we use it directly.
constexpr const char* kPortabilitySubset = "VK_KHR_portability_subset";

// The device's supported extensions, enumerated once (empty on an enumeration
// error, so a missing extension reads as "unsupported" rather than crashing).
std::vector<VkExtensionProperties> supported_device_extensions(
    VkPhysicalDevice physical) {
  std::uint32_t count = 0;
  vkEnumerateDeviceExtensionProperties(physical, nullptr, &count, nullptr);
  std::vector<VkExtensionProperties> exts(count);
  if (vkEnumerateDeviceExtensionProperties(physical, nullptr, &count,
                                           exts.data()) != VK_SUCCESS) {
    exts.clear();
  }
  return exts;
}

// Whether the physical device reports timelineSemaphore support (Vulkan 1.2
// core, queried through VkPhysicalDeviceFeatures2). Used only on the create
// path, where the instance is recon's own >= 1.2 one, so vkGetPhysicalDevice-
// Features2 (Vulkan 1.1) is always dispatchable.
bool supports_timeline_semaphore(VkPhysicalDevice physical) {
  VkPhysicalDeviceTimelineSemaphoreFeatures timeline{};
  timeline.sType =
      VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_TIMELINE_SEMAPHORE_FEATURES;
  VkPhysicalDeviceFeatures2 supported{};
  supported.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
  supported.pNext = &timeline;
  vkGetPhysicalDeviceFeatures2(physical, &supported);
  return timeline.timelineSemaphore == VK_TRUE;
}

// Whether the physical device reports scalarBlockLayout support (Vulkan 1.2
// core) -- the buffer ABI every recon compute shader reads its POD structs
// through (GL_EXT_scalar_block_layout; see the 2026-07-05 decision).
bool supports_scalar_block_layout(VkPhysicalDevice physical) {
  VkPhysicalDeviceScalarBlockLayoutFeatures scalar{};
  scalar.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_SCALAR_BLOCK_LAYOUT_FEATURES;
  VkPhysicalDeviceFeatures2 supported{};
  supported.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
  supported.pNext = &scalar;
  vkGetPhysicalDeviceFeatures2(physical, &supported);
  return scalar.scalarBlockLayout == VK_TRUE;
}

// VkPhysicalDeviceFeatures is a POD of only VkBool32 members (no padding), so
// treat it as a flat array to check "every feature recon requested is enabled"
// without hand-listing its ~55 fields.
bool features_subset_enabled(const VkPhysicalDeviceFeatures& requested,
                             const VkPhysicalDeviceFeatures& enabled) {
  constexpr std::size_t kCount =
      sizeof(VkPhysicalDeviceFeatures) / sizeof(VkBool32);
  const auto* req = reinterpret_cast<const VkBool32*>(&requested);
  const auto* en = reinterpret_cast<const VkBool32*>(&enabled);
  for (std::size_t i = 0; i < kCount; ++i) {
    if (req[i] == VK_TRUE && en[i] != VK_TRUE) {
      return false;
    }
  }
  return true;
}

}  // namespace

DeviceRequirements Device::requirements(const DeviceConfig& config) {
  // Everything but the config-derived features/extensions comes from the
  // DeviceRequirements member initializers (api 1.2, a compute queue, timeline
  // semaphore on).
  DeviceRequirements reqs;
  reqs.features = config.features;
  reqs.device_extensions = config.extra_device_extensions;
  // portability_subset is enabled by whoever creates the device (spec-required
  // when present); it is not a caller requirement, so it is not listed here.
  return reqs;
}

Result<Device> Device::create(VkInstance instance, VkPhysicalDevice physical,
                              const DeviceConfig& config) {
  // `instance` is a lifetime contract (the device stores only handles).
  (void)instance;
  if (physical == VK_NULL_HANDLE) {
    return Status::invalid_argument("Device::create: physical device is null");
  }
  if (detail::physical_api_version(physical) < VK_API_VERSION_1_2) {
    return Status::unsupported("device does not support Vulkan 1.2");
  }
  std::optional<std::uint32_t> compute = detail::find_compute_family(physical);
  if (!compute) {
    return Status::unsupported("no compute queue family");
  }

  // Enumerate the device's supported extensions once, then test each request
  // against that in-memory list rather than re-enumerating per name.
  const std::vector<VkExtensionProperties> supported =
      supported_device_extensions(physical);
  auto is_supported = [&](const char* name) {
    return std::any_of(supported.begin(), supported.end(),
                       [&](const VkExtensionProperties& e) {
                         return std::strcmp(e.extensionName, name) == 0;
                       });
  };

  std::vector<const char*> extensions;
  auto already = [&](const char* name) {
    return std::any_of(
        extensions.begin(), extensions.end(),
        [&](const char* e) { return std::strcmp(e, name) == 0; });
  };
  for (const char* name : config.extra_device_extensions) {
    if (already(name)) {
      continue;
    }
    if (!is_supported(name)) {
      return Status::unsupported(
          std::string("required device extension missing: ") + name);
    }
    extensions.push_back(name);
  }
  // The spec requires enabling VK_KHR_portability_subset whenever a device
  // exposes it (e.g. MoltenVK).
  if (is_supported(kPortabilitySubset) && !already(kPortabilitySubset)) {
    extensions.push_back(kPortabilitySubset);
  }

  if (!supports_timeline_semaphore(physical)) {
    return Status::unsupported(
        "device does not support timelineSemaphore (Vulkan 1.2 core)");
  }
  if (!supports_scalar_block_layout(physical)) {
    return Status::unsupported(
        "device does not support scalarBlockLayout (Vulkan 1.2 core) -- the "
        "recon compute-shader buffer ABI");
  }

  const float priority = 1.0f;
  VkDeviceQueueCreateInfo queue_info{};
  queue_info.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
  queue_info.queueFamilyIndex = *compute;
  queue_info.queueCount = 1;
  queue_info.pQueuePriorities = &priority;

  // Enable features through VkPhysicalDeviceFeatures2 (which supersedes
  // pEnabledFeatures), with timelineSemaphore (1.2 core) chained on.
  VkPhysicalDeviceTimelineSemaphoreFeatures timeline{};
  timeline.sType =
      VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_TIMELINE_SEMAPHORE_FEATURES;
  timeline.timelineSemaphore = VK_TRUE;
  // scalarBlockLayout (1.2 core): the buffer ABI every recon compute shader
  // reads its POD structs through (2026-07-05). Chained ahead of timeline.
  VkPhysicalDeviceScalarBlockLayoutFeatures scalar{};
  scalar.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_SCALAR_BLOCK_LAYOUT_FEATURES;
  scalar.scalarBlockLayout = VK_TRUE;
  scalar.pNext = &timeline;
  VkPhysicalDeviceFeatures2 features2{};
  features2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
  features2.features = config.features;
  features2.pNext = &scalar;

  VkDeviceCreateInfo create_info{};
  create_info.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
  create_info.pNext = &features2;
  create_info.queueCreateInfoCount = 1;
  create_info.pQueueCreateInfos = &queue_info;
  create_info.enabledExtensionCount =
      static_cast<std::uint32_t>(extensions.size());
  create_info.ppEnabledExtensionNames =
      extensions.empty() ? nullptr : extensions.data();

  Device device;
  device.physical_ = physical;
  device.compute_family_ = *compute;
  device.compute_family_flags_ = detail::queue_family_flags(physical, *compute);
  VR_VK_TRY(vkCreateDevice(physical, &create_info, nullptr, &device.device_));
  vkGetDeviceQueue(device.device_, *compute, 0, &device.compute_queue_);

  VkCommandPoolCreateInfo pool_info{};
  pool_info.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
  pool_info.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
  pool_info.queueFamilyIndex = *compute;
  VR_VK_TRY(vkCreateCommandPool(device.device_, &pool_info, nullptr,
                                &device.command_pool_));
  return device;
}

Result<Device> Device::adopt(const AdoptedDevice& adopted,
                             const DeviceConfig& config) {
  if (adopted.instance == VK_NULL_HANDLE ||
      adopted.physical_device == VK_NULL_HANDLE ||
      adopted.device == VK_NULL_HANDLE ||
      adopted.compute_queue == VK_NULL_HANDLE) {
    return Status::invalid_argument(
        "Device::adopt: instance, physical device, device, and compute queue "
        "must all be non-null");
  }
  if (detail::physical_api_version(adopted.physical_device) <
      VK_API_VERSION_1_2) {
    return Status::unsupported("adopted device does not support Vulkan 1.2");
  }
  if (!detail::queue_family_has_compute(adopted.physical_device,
                                        adopted.compute_family)) {
    return Status::unsupported(
        "Device::adopt: assigned queue family is not compute-capable");
  }

  // Vulkan cannot be asked what a logical device enabled, so recon's needs are
  // verified against the creator's declaration in AdoptedDevice, not by
  // querying the device. That is also why adopt touches only Vulkan 1.0
  // physical-device queries: it makes no assumption about the API version the
  // embedder negotiated on `adopted.instance`.
  const DeviceRequirements reqs = requirements(config);
  if (!reqs.device_extensions.empty()) {
    if (adopted.enabled_device_extensions == nullptr) {
      return Status::unsupported(
          "Device::adopt: adopted device did not declare enabled extensions");
    }
    auto is_enabled = [&](const char* name) {
      for (std::uint32_t i = 0; i < adopted.enabled_device_extension_count;
           ++i) {
        if (std::strcmp(adopted.enabled_device_extensions[i], name) == 0) {
          return true;
        }
      }
      return false;
    };
    for (const char* name : reqs.device_extensions) {
      if (!is_enabled(name)) {
        return Status::unsupported(
            std::string("Device::adopt: required extension not enabled on the "
                        "adopted device: ") +
            name);
      }
    }
  }
  // Every core feature recon requested must have been enabled by the creator.
  if (!features_subset_enabled(reqs.features, adopted.enabled_features)) {
    return Status::unsupported(
        "Device::adopt: a VkPhysicalDeviceFeatures bit recon requires was not "
        "enabled on the adopted device");
  }
  // timelineSemaphore likewise can't be queried post-creation; the creator
  // declares whether it enabled it (1.2 core, but optional to enable).
  if (reqs.timeline_semaphore && !adopted.enabled_timeline_semaphore) {
    return Status::unsupported(
        "Device::adopt: timelineSemaphore was not enabled on the adopted "
        "device");
  }
  // scalarBlockLayout likewise must be enabled at creation and can't be queried
  // back; the creator declares it. recon's compute-shader ABI requires it.
  if (reqs.scalar_block_layout && !adopted.enabled_scalar_block_layout) {
    return Status::unsupported(
        "Device::adopt: scalarBlockLayout was not enabled on the adopted "
        "device");
  }

  Device device;
  device.owns_device_ = false;  // borrowed -- the dtor must not destroy it
  device.physical_ = adopted.physical_device;
  device.device_ = adopted.device;
  device.compute_family_ = adopted.compute_family;
  // Read from the driver, not declared by the embedder: unlike the enabled
  // extensions and features, a family's capabilities *are* queryable, so there
  // is nothing here for a hand-written payload to get wrong.
  device.compute_family_flags_ = detail::queue_family_flags(
      adopted.physical_device, adopted.compute_family);
  device.compute_queue_ = adopted.compute_queue;
  device.submit_mutex_ = adopted.submit_mutex;

  // The command pool is this wrapper's own resource on the shared device --
  // made here (and destroyed in destroy()) even though the device is borrowed.
  VkCommandPoolCreateInfo pool_info{};
  pool_info.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
  pool_info.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
  pool_info.queueFamilyIndex = adopted.compute_family;
  VR_VK_TRY(vkCreateCommandPool(device.device_, &pool_info, nullptr,
                                &device.command_pool_));
  return device;
}

Device::Device(Device&& other) noexcept
    : physical_(other.physical_),
      device_(other.device_),
      command_pool_(other.command_pool_),
      owns_device_(other.owns_device_),
      submit_mutex_(other.submit_mutex_),
      compute_family_(other.compute_family_),
      compute_family_flags_(other.compute_family_flags_),
      compute_queue_(other.compute_queue_) {
  other.physical_ = VK_NULL_HANDLE;
  other.device_ = VK_NULL_HANDLE;
  other.command_pool_ = VK_NULL_HANDLE;
  other.owns_device_ = true;
  other.submit_mutex_ = nullptr;
  other.compute_family_ = 0;
  other.compute_family_flags_ = 0;
  other.compute_queue_ = VK_NULL_HANDLE;
}

Device& Device::operator=(Device&& other) noexcept {
  if (this != &other) {
    destroy();
    physical_ = other.physical_;
    device_ = other.device_;
    command_pool_ = other.command_pool_;
    owns_device_ = other.owns_device_;
    submit_mutex_ = other.submit_mutex_;
    compute_family_ = other.compute_family_;
    compute_family_flags_ = other.compute_family_flags_;
    compute_queue_ = other.compute_queue_;
    other.physical_ = VK_NULL_HANDLE;
    other.device_ = VK_NULL_HANDLE;
    other.command_pool_ = VK_NULL_HANDLE;
    other.owns_device_ = true;
    other.submit_mutex_ = nullptr;
    other.compute_family_ = 0;
    other.compute_family_flags_ = 0;
    other.compute_queue_ = VK_NULL_HANDLE;
  }
  return *this;
}

Device::~Device() { destroy(); }

void Device::destroy() noexcept {
  if (command_pool_ != VK_NULL_HANDLE) {
    vkDestroyCommandPool(device_, command_pool_, nullptr);
    command_pool_ = VK_NULL_HANDLE;
  }
  if (device_ != VK_NULL_HANDLE) {
    // Only destroy a device this wrapper created; an adopted one belongs to its
    // owner (the shared bootstrap), which outlives us.
    if (owns_device_) {
      vkDestroyDevice(device_, nullptr);
    }
    device_ = VK_NULL_HANDLE;
  }
  owns_device_ = true;
  submit_mutex_ = nullptr;
  physical_ = VK_NULL_HANDLE;
  compute_family_ = 0;
  compute_family_flags_ = 0;
  compute_queue_ = VK_NULL_HANDLE;
}

VkResult Device::queue_submit(std::uint32_t count, const VkSubmitInfo* submits,
                              VkFence fence) const {
  // Vulkan requires queue submits be externally synchronized. When the compute
  // queue is shared with another library (an adopted device), the neutral
  // bootstrap hands us a mutex to serialize submits on it; hold it for the
  // submit. When the queue is exclusively ours (created path), there is nothing
  // to lock and the unengaged lock is a no-op.
  std::unique_lock<std::mutex> lock;
  if (submit_mutex_ != nullptr) {
    lock = std::unique_lock<std::mutex>(*submit_mutex_);
  }
  return vkQueueSubmit(compute_queue_, count, submits, fence);
}

Status Device::submit_single_time(
    const std::function<void(VkCommandBuffer)>& record) const {
  // Delegated rather than duplicated: a null timer makes the timed overload
  // byte-for-byte this one, and one body is one place for the fence-failure
  // leak rule below to be got right.
  return submit_single_time(record, nullptr, nullptr);
}

Status Device::submit_single_time(
    const std::function<void(VkCommandBuffer)>& record, GpuTimer* timer,
    const char* label) const {
  VkCommandBufferAllocateInfo alloc_info{};
  alloc_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
  alloc_info.commandPool = command_pool_;
  alloc_info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
  alloc_info.commandBufferCount = 1;
  VkCommandBuffer cmd = VK_NULL_HANDLE;
  VR_VK_TRY(vkAllocateCommandBuffers(device_, &alloc_info, &cmd));

  // Free the command buffer on every exit path below -- including the VR_VK_TRY
  // early returns (recon has no standalone CommandBuffer type yet; a one-shot
  // dispatch does not need one).
  ScopeGuard free_cmd(
      [&] { vkFreeCommandBuffers(device_, command_pool_, 1, &cmd); });

  VkCommandBufferBeginInfo begin{};
  begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
  begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
  VR_VK_TRY(vkBeginCommandBuffer(cmd, &begin));
  // The span brackets exactly what `record` puts in the buffer, so it measures
  // device execution and excludes the allocate/begin/end/submit around it --
  // which is the whole distinction this overload exists to draw.
  const std::uint32_t span =
      timer != nullptr ? timer->begin(cmd, label) : GpuTimer::kNoSpan;
  record(cmd);
  if (timer != nullptr) {
    timer->end(cmd, span);
  }
  VR_VK_TRY(vkEndCommandBuffer(cmd));

  VkFenceCreateInfo fence_info{};
  fence_info.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
  VkFence fence = VK_NULL_HANDLE;
  VR_VK_TRY(vkCreateFence(device_, &fence_info, nullptr, &fence));
  ScopeGuard destroy_fence([&] { vkDestroyFence(device_, fence, nullptr); });

  VkSubmitInfo submit{};
  submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
  submit.commandBufferCount = 1;
  submit.pCommandBuffers = &cmd;
  VR_VK_TRY(queue_submit(1, &submit, fence));

  // Block until the GPU signals the fence. If the wait itself fails (device
  // lost / out of memory) the submit may still be pending, so the command
  // buffer and fence must NOT be freed -- disarm the guards and leak them
  // rather than free objects the GPU could still touch (a use-after-free).
  const VkResult waited =
      vkWaitForFences(device_, 1, &fence, VK_TRUE, UINT64_MAX);
  if (waited != VK_SUCCESS) {
    free_cmd.release();
    destroy_fence.release();
    return vk_error(waited, "vkWaitForFences");
  }

  // Only here, and only on this path. The queries are readable because the
  // fence has signalled; on the failure path above the submit may still be
  // pending, so there is nothing valid to read and the timer keeps the span
  // unresolved -- which report_into skips, rather than publishing a zero that
  // reads as a fast dispatch.
  if (timer != nullptr) {
    VR_TRY(timer->resolve());
  }
  return {};
}

}  // namespace volumetric_kit::recon
