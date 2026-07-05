// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/recon/core/device.hpp"

#include <algorithm>
#include <cstring>
#include <optional>
#include <string>
#include <vector>

#include "volumetric_kit/recon/core/vk_result.hpp"

namespace volumetric_kit::recon {
namespace {

// VK_KHR_portability_subset's name macro lives in vulkan_beta.h (gated by
// VK_ENABLE_BETA_EXTENSIONS); the string is stable, so we use it directly.
constexpr const char* kPortabilitySubset = "VK_KHR_portability_subset";

std::optional<std::uint32_t> find_compute_family(VkPhysicalDevice physical) {
  std::uint32_t count = 0;
  vkGetPhysicalDeviceQueueFamilyProperties(physical, &count, nullptr);
  std::vector<VkQueueFamilyProperties> families(count);
  vkGetPhysicalDeviceQueueFamilyProperties(physical, &count, families.data());
  // A compute-capable family implicitly supports transfer, so one queue covers
  // both of recon's needs.
  for (std::uint32_t i = 0; i < count; ++i) {
    if ((families[i].queueFlags & VK_QUEUE_COMPUTE_BIT) != 0) {
      return i;
    }
  }
  return std::nullopt;
}

bool queue_family_has_compute(VkPhysicalDevice physical, std::uint32_t family) {
  std::uint32_t count = 0;
  vkGetPhysicalDeviceQueueFamilyProperties(physical, &count, nullptr);
  if (family >= count) {
    return false;
  }
  std::vector<VkQueueFamilyProperties> families(count);
  vkGetPhysicalDeviceQueueFamilyProperties(physical, &count, families.data());
  return (families[family].queueFlags & VK_QUEUE_COMPUTE_BIT) != 0;
}

bool device_supports_extension(VkPhysicalDevice physical, const char* name) {
  std::uint32_t count = 0;
  vkEnumerateDeviceExtensionProperties(physical, nullptr, &count, nullptr);
  std::vector<VkExtensionProperties> exts(count);
  vkEnumerateDeviceExtensionProperties(physical, nullptr, &count, exts.data());
  for (const VkExtensionProperties& e : exts) {
    if (std::strcmp(e.extensionName, name) == 0) {
      return true;
    }
  }
  return false;
}

std::uint32_t device_api_version(VkPhysicalDevice physical) {
  VkPhysicalDeviceProperties props{};
  vkGetPhysicalDeviceProperties(physical, &props);
  return props.apiVersion;
}

// Whether the physical device reports timelineSemaphore support (Vulkan 1.2
// core, queried through VkPhysicalDeviceFeatures2).
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

}  // namespace

DeviceRequirements Device::requirements(const DeviceConfig& config) {
  DeviceRequirements reqs;
  reqs.api_version = VK_API_VERSION_1_2;
  reqs.queue_flags = VK_QUEUE_COMPUTE_BIT | VK_QUEUE_TRANSFER_BIT;
  reqs.timeline_semaphore = true;
  reqs.features = config.features;
  for (const char* name : config.extra_device_extensions) {
    reqs.device_extensions.push_back(name);
  }
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
  if (device_api_version(physical) < VK_API_VERSION_1_2) {
    return Status::unsupported("device does not support Vulkan 1.2");
  }
  std::optional<std::uint32_t> compute = find_compute_family(physical);
  if (!compute) {
    return Status::unsupported("no compute queue family");
  }

  std::vector<const char*> extensions;
  auto already = [&](const char* name) {
    return std::any_of(
        extensions.begin(), extensions.end(),
        [&](const char* e) { return std::strcmp(e, name) == 0; });
  };
  auto require = [&](const char* name) -> Status {
    if (!device_supports_extension(physical, name)) {
      return Status::unsupported(
          std::string("required device extension missing: ") + name);
    }
    extensions.push_back(name);
    return Status{};
  };
  for (const char* name : config.extra_device_extensions) {
    if (already(name)) {
      continue;
    }
    VR_TRY(require(name));
  }
  // The spec requires enabling VK_KHR_portability_subset whenever a device
  // exposes it (e.g. MoltenVK).
  if (device_supports_extension(physical, kPortabilitySubset) &&
      !already(kPortabilitySubset)) {
    extensions.push_back(kPortabilitySubset);
  }

  if (!supports_timeline_semaphore(physical)) {
    return Status::unsupported(
        "device does not support timelineSemaphore (Vulkan 1.2 core)");
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
  VkPhysicalDeviceFeatures2 features2{};
  features2.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
  features2.features = config.features;
  features2.pNext = &timeline;

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
  if (device_api_version(adopted.physical_device) < VK_API_VERSION_1_2) {
    return Status::unsupported("adopted device does not support Vulkan 1.2");
  }
  if (!queue_family_has_compute(adopted.physical_device,
                                adopted.compute_family)) {
    return Status::unsupported(
        "Device::adopt: assigned queue family is not compute-capable");
  }

  // Every extension recon needs must be in the creator's declared enabled set
  // -- Vulkan cannot be asked what a logical device enabled.
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
  // Necessary condition for timelineSemaphore: the physical device supports it.
  // Whether it was actually enabled on the logical device rests on the creator
  // honoring requirements().
  if (reqs.timeline_semaphore &&
      !supports_timeline_semaphore(adopted.physical_device)) {
    return Status::unsupported(
        "adopted device does not support timelineSemaphore");
  }

  Device device;
  device.owns_device_ = false;  // borrowed -- the dtor must not destroy it
  device.physical_ = adopted.physical_device;
  device.device_ = adopted.device;
  device.compute_family_ = adopted.compute_family;
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
      compute_queue_(other.compute_queue_) {
  other.physical_ = VK_NULL_HANDLE;
  other.device_ = VK_NULL_HANDLE;
  other.command_pool_ = VK_NULL_HANDLE;
  other.owns_device_ = true;
  other.submit_mutex_ = nullptr;
  other.compute_family_ = 0;
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
    compute_queue_ = other.compute_queue_;
    other.physical_ = VK_NULL_HANDLE;
    other.device_ = VK_NULL_HANDLE;
    other.command_pool_ = VK_NULL_HANDLE;
    other.owns_device_ = true;
    other.submit_mutex_ = nullptr;
    other.compute_family_ = 0;
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
  compute_queue_ = VK_NULL_HANDLE;
}

}  // namespace volumetric_kit::recon
