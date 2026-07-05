// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/recon/core/instance.hpp"

#include <algorithm>
#include <cstring>
#include <string>
#include <vector>

#include "vk_physical_device.hpp"
#include "volumetric_kit/recon/core/log.hpp"
#include "volumetric_kit/recon/core/vk_result.hpp"

namespace volumetric_kit::recon {
namespace {

constexpr const char* kValidationLayer = "VK_LAYER_KHRONOS_validation";

// Whether the Khronos validation layer is installed on this system.
bool validation_layer_available() {
  std::uint32_t count = 0;
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

// Whether the named instance extension is available from the loader.
bool instance_extension_available(const char* name) {
  std::uint32_t count = 0;
  vkEnumerateInstanceExtensionProperties(nullptr, &count, nullptr);
  std::vector<VkExtensionProperties> exts(count);
  if (vkEnumerateInstanceExtensionProperties(nullptr, &count, exts.data()) !=
      VK_SUCCESS) {
    return false;
  }
  for (const VkExtensionProperties& e : exts) {
    if (std::strcmp(e.extensionName, name) == 0) {
      return true;
    }
  }
  return false;
}

// Route validation-layer messages into the pluggable core log handler, mapping
// Vulkan severities onto LogLevel. Returns VK_FALSE so the offending Vulkan
// call is not aborted (the required return for a non-debugging messenger).
VKAPI_ATTR VkBool32 VKAPI_CALL debug_callback(
    VkDebugUtilsMessageSeverityFlagBitsEXT severity,
    VkDebugUtilsMessageTypeFlagsEXT /*types*/,
    const VkDebugUtilsMessengerCallbackDataEXT* data, void* /*user*/) {
  LogLevel level = LogLevel::Info;
  if ((severity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT) != 0) {
    level = LogLevel::Error;
  } else if ((severity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT) !=
             0) {
    level = LogLevel::Warning;
  }
  const char* message = (data != nullptr && data->pMessage != nullptr)
                            ? data->pMessage
                            : "(none)";
  log_message(level, std::string("[vulkan] ") + message);
  return VK_FALSE;
}

// The messenger config recon uses: warnings + errors, all message types.
VkDebugUtilsMessengerCreateInfoEXT debug_messenger_info() {
  VkDebugUtilsMessengerCreateInfoEXT info{};
  info.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
  info.messageSeverity = VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT |
                         VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
  info.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT |
                     VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT |
                     VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
  info.pfnUserCallback = debug_callback;
  return info;
}

// A preference score for picking among compute-capable devices.
int device_type_score(VkPhysicalDeviceType type) {
  switch (type) {
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
  // The debug messenger (which routes layer output through the log handler)
  // needs VK_EXT_debug_utils. If validation is on but the extension is absent,
  // the layer still runs -- its output just goes to its own default sink.
  const bool want_debug_messenger =
      want_validation &&
      instance_extension_available(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
  if (want_validation && !want_debug_messenger) {
    log_message(LogLevel::Warning,
                "validation enabled but VK_EXT_debug_utils is unavailable; "
                "layer messages will not route through the log handler");
  }

  std::vector<const char*> layers;
  if (want_validation) {
    layers.push_back(kValidationLayer);
  }

  // Build the extension list once, then create; on a loader that lacks
  // portability enumeration we retry without it (see below).
  auto make_instance = [&](bool with_portability, VkInstance* out) -> VkResult {
    std::vector<const char*> extensions = config.extra_instance_extensions;
    // Append each extension only if the caller did not already supply it, so
    // ppEnabledExtensionNames never lists a duplicate.
    auto add_unique = [&](const char* name) {
      const bool present =
          std::any_of(extensions.begin(), extensions.end(),
                      [&](const char* e) { return std::strcmp(e, name) == 0; });
      if (!present) {
        extensions.push_back(name);
      }
    };
    if (want_debug_messenger) {
      add_unique(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
    }
#ifdef VK_KHR_portability_enumeration
    if (with_portability) {
      add_unique(VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME);
    }
#else
    (void)with_portability;
#endif

    VkInstanceCreateInfo ci{};
    ci.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
    ci.pApplicationInfo = &app;
    ci.enabledLayerCount = static_cast<std::uint32_t>(layers.size());
    ci.ppEnabledLayerNames = layers.empty() ? nullptr : layers.data();
    ci.enabledExtensionCount = static_cast<std::uint32_t>(extensions.size());
    ci.ppEnabledExtensionNames =
        extensions.empty() ? nullptr : extensions.data();
    // Chain a messenger create-info so the layer also validates instance
    // creation and destruction, routing those messages to our callback too.
    VkDebugUtilsMessengerCreateInfoEXT dbg = debug_messenger_info();
    if (want_debug_messenger) {
      ci.pNext = &dbg;
    }
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

  // Stand up the debug messenger now the instance exists. Its entry point is an
  // extension function, fetched through vkGetInstanceProcAddr. A failure here
  // is non-fatal: the layer still validates, just without our log routing.
  if (want_debug_messenger) {
    auto create_messenger =
        reinterpret_cast<PFN_vkCreateDebugUtilsMessengerEXT>(
            vkGetInstanceProcAddr(instance.instance_,
                                  "vkCreateDebugUtilsMessengerEXT"));
    if (create_messenger != nullptr) {
      VkDebugUtilsMessengerCreateInfoEXT dbg = debug_messenger_info();
      if (create_messenger(instance.instance_, &dbg, nullptr,
                           &instance.debug_messenger_) != VK_SUCCESS) {
        instance.debug_messenger_ = VK_NULL_HANDLE;
        log_message(LogLevel::Warning,
                    "failed to create the Vulkan debug messenger; validation "
                    "messages will not route through the log handler");
      }
    }
  }
  return instance;
}

Result<VkPhysicalDevice> Instance::select_physical_device() const {
  std::uint32_t count = 0;
  VR_VK_TRY(vkEnumeratePhysicalDevices(instance_, &count, nullptr));
  if (count == 0) {
    return Status::unsupported("no Vulkan physical devices");
  }
  std::vector<VkPhysicalDevice> devices(count);
  const VkResult r =
      vkEnumeratePhysicalDevices(instance_, &count, devices.data());
  if (r != VK_SUCCESS && r != VK_INCOMPLETE) {
    return vk_error(r, "vkEnumeratePhysicalDevices");
  }
  devices.resize(count);  // VK_INCOMPLETE: the list grew; keep what we got.

  VkPhysicalDevice best = VK_NULL_HANDLE;
  int best_score = -1;
  for (VkPhysicalDevice d : devices) {
    // Read properties once per candidate and use them for both the 1.2 floor
    // (the level Device::create needs for timeline semaphores) and the score.
    VkPhysicalDeviceProperties props{};
    vkGetPhysicalDeviceProperties(d, &props);
    if (props.apiVersion < VK_API_VERSION_1_2) {
      continue;
    }
    if (!detail::find_compute_family(d).has_value()) {
      continue;
    }
    const int score = device_type_score(props.deviceType);
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
      debug_messenger_(other.debug_messenger_),
      validation_enabled_(other.validation_enabled_) {
  other.instance_ = VK_NULL_HANDLE;
  other.debug_messenger_ = VK_NULL_HANDLE;
  other.validation_enabled_ = false;
}

Instance& Instance::operator=(Instance&& other) noexcept {
  if (this != &other) {
    destroy();
    instance_ = other.instance_;
    debug_messenger_ = other.debug_messenger_;
    validation_enabled_ = other.validation_enabled_;
    other.instance_ = VK_NULL_HANDLE;
    other.debug_messenger_ = VK_NULL_HANDLE;
    other.validation_enabled_ = false;
  }
  return *this;
}

Instance::~Instance() { destroy(); }

void Instance::destroy() noexcept {
  // The messenger must go before the instance that owns it. Its destroy entry
  // point is an extension function, fetched through vkGetInstanceProcAddr.
  if (debug_messenger_ != VK_NULL_HANDLE) {
    auto destroy_messenger =
        reinterpret_cast<PFN_vkDestroyDebugUtilsMessengerEXT>(
            vkGetInstanceProcAddr(instance_,
                                  "vkDestroyDebugUtilsMessengerEXT"));
    if (destroy_messenger != nullptr) {
      destroy_messenger(instance_, debug_messenger_, nullptr);
    }
    debug_messenger_ = VK_NULL_HANDLE;
  }
  if (instance_ != VK_NULL_HANDLE) {
    vkDestroyInstance(instance_, nullptr);
    instance_ = VK_NULL_HANDLE;
  }
  validation_enabled_ = false;
}

}  // namespace volumetric_kit::recon
