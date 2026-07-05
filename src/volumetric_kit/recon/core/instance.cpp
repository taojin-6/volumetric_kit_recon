// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/recon/core/instance.hpp"

#include <cstring>
#include <utility>
#include <vector>

#include "volumetric_kit/recon/core/log.hpp"
#include "volumetric_kit/recon/core/vk_result.hpp"

namespace volumetric_kit::recon {
namespace {

constexpr const char* kValidationLayer = "VK_LAYER_KHRONOS_validation";

// Whether the Khronos validation layer is installed on this system.
bool validation_layer_available() {
  uint32_t count = 0;
  vkEnumerateInstanceLayerProperties(&count, nullptr);
  std::vector<VkLayerProperties> layers(count);
  vkEnumerateInstanceLayerProperties(&count, layers.data());
  for (const VkLayerProperties& l : layers) {
    if (std::strcmp(l.layerName, kValidationLayer) == 0) {
      return true;
    }
  }
  return false;
}

// Whether `physical` exposes at least one compute-capable queue family and
// reports Vulkan >= 1.2 (the floor Device::create needs for timeline
// semaphores).
bool is_compute_capable(VkPhysicalDevice physical) {
  VkPhysicalDeviceProperties props{};
  vkGetPhysicalDeviceProperties(physical, &props);
  if (props.apiVersion < VK_API_VERSION_1_2) {
    return false;
  }
  uint32_t count = 0;
  vkGetPhysicalDeviceQueueFamilyProperties(physical, &count, nullptr);
  std::vector<VkQueueFamilyProperties> families(count);
  vkGetPhysicalDeviceQueueFamilyProperties(physical, &count, families.data());
  for (const VkQueueFamilyProperties& f : families) {
    if ((f.queueFlags & VK_QUEUE_COMPUTE_BIT) != 0) {
      return true;
    }
  }
  return false;
}

// A preference score for picking among compute-capable devices.
int device_type_score(VkPhysicalDevice physical) {
  VkPhysicalDeviceProperties props{};
  vkGetPhysicalDeviceProperties(physical, &props);
  switch (props.deviceType) {
    case VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU:
      return 4;
    case VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU:
      return 3;
    case VK_PHYSICAL_DEVICE_TYPE_VIRTUAL_GPU:
      return 2;
    case VK_PHYSICAL_DEVICE_TYPE_CPU:
      return 1;
    default:
      return 0;
  }
}

}  // namespace

Result<Instance> Instance::create(const InstanceConfig& config) {
  VkApplicationInfo app{};
  app.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
  app.pApplicationName = config.app_name.c_str();
  // Timeline semaphores are Vulkan 1.2 core, and the interop handoff uses them;
  // negotiate at least 1.2 so the device-level feature query below is valid.
  app.apiVersion = VK_API_VERSION_1_2;

  const bool want_validation =
      config.enable_validation && validation_layer_available();
  if (config.enable_validation && !want_validation) {
    log_message(LogLevel::Warning,
                "validation requested but VK_LAYER_KHRONOS_validation is not "
                "installed; continuing without it");
  }

  std::vector<const char*> layers;
  if (want_validation) {
    layers.push_back(kValidationLayer);
  }

  // Build the extension list once, then create; on a loader that lacks
  // portability enumeration we retry without it (see below).
  auto make_instance = [&](bool with_portability, VkInstance* out) -> VkResult {
    std::vector<const char*> extensions = config.extra_instance_extensions;
#ifdef VK_KHR_portability_enumeration
    if (with_portability) {
      extensions.push_back(VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME);
    }
#else
    (void)with_portability;
#endif

    VkInstanceCreateInfo ci{};
    ci.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
    ci.pApplicationInfo = &app;
    ci.enabledLayerCount = static_cast<uint32_t>(layers.size());
    ci.ppEnabledLayerNames = layers.empty() ? nullptr : layers.data();
    ci.enabledExtensionCount = static_cast<uint32_t>(extensions.size());
    ci.ppEnabledExtensionNames =
        extensions.empty() ? nullptr : extensions.data();
#ifdef VK_KHR_portability_enumeration
    if (with_portability) {
      ci.flags |= VK_INSTANCE_CREATE_ENUMERATE_PORTABILITY_BIT_KHR;
    }
#endif
    return vkCreateInstance(&ci, nullptr, out);
  };

  Instance instance;
  VkResult r = make_instance(/*with_portability=*/true, &instance.instance_);
  if (r == VK_ERROR_EXTENSION_NOT_PRESENT ||
      r == VK_ERROR_INCOMPATIBLE_DRIVER) {
    r = make_instance(/*with_portability=*/false, &instance.instance_);
  }
  if (r != VK_SUCCESS) {
    return vk_error(r, "vkCreateInstance");
  }
  instance.validation_enabled_ = want_validation;
  return instance;
}

Result<VkPhysicalDevice> Instance::select_physical_device() const {
  uint32_t count = 0;
  vkEnumeratePhysicalDevices(instance_, &count, nullptr);
  if (count == 0) {
    return Status::unsupported("no Vulkan physical devices");
  }
  std::vector<VkPhysicalDevice> devices(count);
  vkEnumeratePhysicalDevices(instance_, &count, devices.data());

  VkPhysicalDevice best = VK_NULL_HANDLE;
  int best_score = -1;
  for (VkPhysicalDevice d : devices) {
    if (!is_compute_capable(d)) {
      continue;
    }
    const int score = device_type_score(d);
    if (score > best_score) {
      best_score = score;
      best = d;
    }
  }
  if (best == VK_NULL_HANDLE) {
    return Status::unsupported(
        "no compute-capable Vulkan 1.2+ device available");
  }
  return best;
}

Instance::Instance(Instance&& other) noexcept
    : instance_(other.instance_),
      validation_enabled_(other.validation_enabled_) {
  other.instance_ = VK_NULL_HANDLE;
  other.validation_enabled_ = false;
}

Instance& Instance::operator=(Instance&& other) noexcept {
  if (this != &other) {
    destroy();
    instance_ = other.instance_;
    validation_enabled_ = other.validation_enabled_;
    other.instance_ = VK_NULL_HANDLE;
    other.validation_enabled_ = false;
  }
  return *this;
}

Instance::~Instance() { destroy(); }

void Instance::destroy() noexcept {
  if (instance_ != VK_NULL_HANDLE) {
    vkDestroyInstance(instance_, nullptr);
    instance_ = VK_NULL_HANDLE;
  }
  validation_enabled_ = false;
}

}  // namespace volumetric_kit::recon
