// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/viewer/shared_device.hpp
/// @brief The neutral bootstrap: one `VkInstance` + one `VkDevice` built from
///        the union of recon's and gfx's published requirements, for both to
///        adopt.
///
/// This is the embedder half of the create/adopt seam both libraries expose.
/// Neither owns the device -- an app that wants them to share one builds it
/// from `Device::requirements` on each side, then hands the same handles to
/// `recon::Device::adopt` and `gfx::app::WindowedApp::adopt`. That is what
/// makes zero-copy possible at all: a `VkBuffer` is valid only on the device
/// that created it, so recon geometry can be drawn by gfx *only* if there is
/// one device.
///
/// Deliberately a plain function over raw Vulkan rather than a wrapper type:
/// it is the one place in this tree that is neither library's, and both
/// libraries' RAII owners start *after* it.

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <vector>

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#include "volumetric_kit/gfx/app/windowed_app.hpp"
#include "volumetric_kit/gfx/core/device.hpp"
#include "volumetric_kit/recon/core/device.hpp"

namespace fuse_viewer {

namespace vr = volumetric_kit::recon;
namespace vg = volumetric_kit::gfx;

/// @brief How the two libraries' queues were carved out of the shared device.
///
/// Ordered best-first. The bootstrap takes the first one the hardware allows;
/// @ref SharedDevice::queue_plan records which.
enum class QueuePlan {
  /// One family, two queues: independent submission *and* no queue-family
  /// ownership transfer on a buffer recon writes and gfx reads. What the
  /// interop-seam-B plan assumes.
  kTwoQueuesOneFamily,
  /// Two families, one queue each: still independent submission, but a shared
  /// buffer now needs `VK_SHARING_MODE_CONCURRENT` or an explicit
  /// release/acquire pair when seam B lands. Taken on MoltenVK, which reports
  /// several graphics+compute+present families of exactly one queue each.
  kTwoFamilies,
  /// One family, one queue, shared under a mutex: submits from the fuse thread
  /// and the render thread serialize. Last resort -- it costs the concurrency
  /// the background fuse thread exists for.
  kSharedQueue,
};

/// @brief One Vulkan device both libraries adopt, plus everything each needs
///        to adopt it.
///
/// Owns the raw instance/device (destroyed in @ref ~SharedDevice, after both
/// adopters have released their wrappers) and the surface the swapchain
/// presents to.
struct SharedDevice {
  VkInstance instance = VK_NULL_HANDLE;
  VkPhysicalDevice physical_device = VK_NULL_HANDLE;
  VkDevice device = VK_NULL_HANDLE;
  /// Created here so device selection can test present support, then handed
  /// to the renderer, which owns and destroys it -- see @ref release_surface.
  VkSurfaceKHR surface = VK_NULL_HANDLE;

  /// Which of the three carvings @ref build_shared_device settled on.
  QueuePlan queue_plan = QueuePlan::kSharedQueue;
  /// gfx's family (graphics + present) and recon's (compute). Equal under
  /// every plan but @ref QueuePlan::kTwoFamilies.
  std::uint32_t graphics_family = 0;
  std::uint32_t compute_family = 0;
  VkQueue graphics_queue = VK_NULL_HANDLE;  ///< gfx's (also presents).
  VkQueue compute_queue = VK_NULL_HANDLE;   ///< recon's.

  /// Non-null only under @ref QueuePlan::kSharedQueue, where both libraries
  /// were handed the same `VkQueue` -- Vulkan requires queue submits be
  /// externally synchronized, and both `adopt` paths route through this when
  /// set. Points at @ref submit_mutex_storage.
  std::mutex* submit_mutex = nullptr;
  /// Backs @ref submit_mutex. A member (not a static) so its address dies with
  /// this object; the deleted moves below keep that address stable.
  std::mutex submit_mutex_storage;

  /// Exactly what was enabled on @ref device, for each `adopt` to verify.
  /// Written by @ref build_shared_device from the very values it fed
  /// `vkCreateDevice`, never restated by hand -- the `adopt` checks are the
  /// point of the seam, and a payload that asserts more than the device got
  /// would defeat them.
  std::vector<const char*> enabled_extensions;
  VkPhysicalDeviceFeatures enabled_features{};
  bool enabled_timeline_semaphore = false;
  bool enabled_scalar_block_layout = false;
  bool enabled_dynamic_rendering = false;
  /// Whether the *instance* got `VK_EXT_debug_utils`. An instance-level fact,
  /// so gfx cannot query it from the device it adopts -- and left false, every
  /// debug label and object name gfx records is a silent no-op, which is
  /// exactly what an Xcode or RenderDoc capture of this viewer must not be.
  bool enabled_debug_utils = false;

  ~SharedDevice() {
    // Both libraries' wrappers must already be gone: they borrow these.
    if (device != VK_NULL_HANDLE) vkDestroyDevice(device, nullptr);
    if (surface != VK_NULL_HANDLE && instance != VK_NULL_HANDLE) {
      vkDestroySurfaceKHR(instance, surface, nullptr);
    }
    if (instance != VK_NULL_HANDLE) vkDestroyInstance(instance, nullptr);
  }

  /// @brief Give up ownership of @ref surface to the caller.
  ///
  /// gfx's surface factory contract is that the app adopts and later destroys
  /// what the factory returns, but this bootstrap had to create the surface
  /// first -- picking a physical device requires testing present support
  /// against a real surface. So it is created here and handed over, and this
  /// stops tracking it; destroying it twice would be a use-after-free at
  /// teardown.
  VkSurfaceKHR release_surface() {
    VkSurfaceKHR released = surface;
    surface = VK_NULL_HANDLE;
    return released;
  }

  SharedDevice() = default;
  SharedDevice(const SharedDevice&) = delete;
  SharedDevice& operator=(const SharedDevice&) = delete;
  SharedDevice(SharedDevice&&) = delete;
  SharedDevice& operator=(SharedDevice&&) = delete;
};

namespace detail {

/// @brief `a | b`, field by field, over a struct that is all `VkBool32`.
inline VkPhysicalDeviceFeatures merge_features(
    const VkPhysicalDeviceFeatures& a, const VkPhysicalDeviceFeatures& b) {
  static_assert(sizeof(VkPhysicalDeviceFeatures) % sizeof(VkBool32) == 0,
                "VkPhysicalDeviceFeatures is not a flat VkBool32 struct");
  constexpr std::size_t kCount =
      sizeof(VkPhysicalDeviceFeatures) / sizeof(VkBool32);
  VkBool32 lhs[kCount];
  VkBool32 rhs[kCount];
  std::memcpy(lhs, &a, sizeof(lhs));
  std::memcpy(rhs, &b, sizeof(rhs));
  for (std::size_t i = 0; i < kCount; ++i) {
    lhs[i] = (lhs[i] != VK_FALSE || rhs[i] != VK_FALSE) ? VK_TRUE : VK_FALSE;
  }
  VkPhysicalDeviceFeatures merged{};
  std::memcpy(&merged, lhs, sizeof(lhs));
  return merged;
}

/// @brief Index of the first bit set in @p wanted but clear in @p supported,
///        or `-1` when @p wanted is a subset.
inline int first_unsupported_feature(
    const VkPhysicalDeviceFeatures& wanted,
    const VkPhysicalDeviceFeatures& supported) {
  constexpr std::size_t kCount =
      sizeof(VkPhysicalDeviceFeatures) / sizeof(VkBool32);
  VkBool32 want[kCount];
  VkBool32 have[kCount];
  std::memcpy(want, &wanted, sizeof(want));
  std::memcpy(have, &supported, sizeof(have));
  for (std::size_t i = 0; i < kCount; ++i) {
    if (want[i] != VK_FALSE && have[i] == VK_FALSE) return static_cast<int>(i);
  }
  return -1;
}

inline void log_vk(const char* what, VkResult result) {
  std::fprintf(stderr, "shared device: %s failed (VkResult %d)\n", what,
               static_cast<int>(result));
}

}  // namespace detail

/// @brief Knobs the embedder controls; everything else is derived from what
///        the two libraries publish.
struct SharedDeviceConfig {
  /// Enable `VK_LAYER_KHRONOS_validation` + `VK_EXT_debug_utils` when the
  /// loader has them. `WindowedApp::adopt` ignores its own
  /// `enable_validation` (the embedder owns the instance), so this is the only
  /// way to validate the shared-device path -- and shared device lifetime plus
  /// cross-library queue synchronization is exactly what the layer catches.
  bool enable_validation = false;
  /// Reported to the driver in `VkApplicationInfo`.
  const char* app_name = "fuse_viewer";
};

/// @brief Build one instance + device satisfying both libraries, and the
///        window's surface on it.
///
/// @param window  The GLFW window to present to; its required instance
///                extensions are enabled and its surface created here.
/// @param config  Embedder-owned knobs (validation, app name).
/// @param out     Filled on success; destroys what it holds on scope exit.
/// @return `false`, with a specific reason on stderr, when the loader, the
///         hardware, or the driver cannot satisfy the union. The caller must
///         treat that as fatal: running the two libraries on separate devices
///         would silently give up the shared-`VkBuffer` seam this bootstrap
///         exists to establish.
inline bool build_shared_device(GLFWwindow* window,
                                const SharedDeviceConfig& config,
                                SharedDevice& out) {
  // --- 1. Merge what each library publishes. --------------------------------
  // Neither is consulted about the other: each states its needs, the embedder
  // satisfies the union. gfx needs present; recon needs compute.
  vg::DeviceConfig gfx_config;
  gfx_config.needs_present = true;
  const vr::DeviceRequirements recon_req = vr::Device::requirements({});
  const vg::DeviceRequirements gfx_req = vg::Device::requirements(gfx_config);

  // The higher floor wins. This must NOT inherit recon's 1.2: MoltenVK caps a
  // physical device's advertised apiVersion to whatever the instance asked
  // for, so an instance created at recon's floor would make a 1.3-capable
  // device look 1.2 and fail gfx's check.
  const std::uint32_t api_version = recon_req.api_version > gfx_req.api_version
                                        ? recon_req.api_version
                                        : gfx_req.api_version;

  // Ask the loader before demanding that floor of it, so a too-old loader says
  // so instead of surfacing as an opaque vkCreateInstance failure.
  std::uint32_t loader_version = VK_API_VERSION_1_0;
  if (vkEnumerateInstanceVersion(&loader_version) != VK_SUCCESS) {
    loader_version = VK_API_VERSION_1_0;
  }
  if (loader_version < api_version) {
    std::fprintf(stderr,
                 "shared device: the loader supports Vulkan %u.%u but the "
                 "merged floor is %u.%u\n",
                 VK_API_VERSION_MAJOR(loader_version),
                 VK_API_VERSION_MINOR(loader_version),
                 VK_API_VERSION_MAJOR(api_version),
                 VK_API_VERSION_MINOR(api_version));
    return false;
  }

  std::vector<const char*> extensions;
  auto add_extension = [&extensions](const char* name) {
    for (const char* have : extensions) {
      if (std::strcmp(have, name) == 0) return;
    }
    extensions.push_back(name);
  };
  for (const char* name : recon_req.device_extensions) add_extension(name);
  for (const char* name : gfx_req.device_extensions) add_extension(name);

  const VkPhysicalDeviceFeatures wanted_features =
      detail::merge_features(recon_req.features, gfx_req.features);

  // --- 2. Instance: the window's extensions, at the merged floor. -----------
  std::uint32_t glfw_extension_count = 0;
  const char** glfw_extensions =
      glfwGetRequiredInstanceExtensions(&glfw_extension_count);
  if (glfw_extensions == nullptr) {
    std::fprintf(stderr, "shared device: GLFW reports no Vulkan support\n");
    return false;
  }

  bool validation_available = false;
  if (config.enable_validation) {
    std::uint32_t layer_count = 0;
    vkEnumerateInstanceLayerProperties(&layer_count, nullptr);
    std::vector<VkLayerProperties> layers(layer_count);
    if (vkEnumerateInstanceLayerProperties(&layer_count, layers.data()) ==
        VK_SUCCESS) {
      for (const VkLayerProperties& layer : layers) {
        if (std::strcmp(layer.layerName, "VK_LAYER_KHRONOS_validation") == 0) {
          validation_available = true;
          break;
        }
      }
    }
    if (!validation_available) {
      std::fprintf(stderr,
                   "shared device: VK_LAYER_KHRONOS_validation not installed; "
                   "continuing without validation\n");
    }
  }
  const char* const validation_layer = "VK_LAYER_KHRONOS_validation";

  VkApplicationInfo app_info{};
  app_info.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
  app_info.pApplicationName = config.app_name;
  app_info.apiVersion = api_version;

  // Portability enumeration is not universal: a directly-linked MoltenVK (iOS,
  // no loader) does not expose it at all. Try with, retry without -- the same
  // fallback recon's own Instance::create carries.
  auto make_instance = [&](bool with_portability) -> VkResult {
    std::vector<const char*> instance_extensions(
        glfw_extensions, glfw_extensions + glfw_extension_count);
    if (validation_available) {
      instance_extensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
    }
    VkInstanceCreateInfo instance_info{};
    instance_info.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
    instance_info.pApplicationInfo = &app_info;
#ifdef VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME
    if (with_portability) {
      instance_extensions.push_back(
          VK_KHR_PORTABILITY_ENUMERATION_EXTENSION_NAME);
      instance_info.flags |= VK_INSTANCE_CREATE_ENUMERATE_PORTABILITY_BIT_KHR;
    }
#else
    (void)with_portability;
#endif
    instance_info.enabledExtensionCount =
        static_cast<std::uint32_t>(instance_extensions.size());
    instance_info.ppEnabledExtensionNames = instance_extensions.data();
    instance_info.enabledLayerCount = validation_available ? 1u : 0u;
    instance_info.ppEnabledLayerNames =
        validation_available ? &validation_layer : nullptr;
    return vkCreateInstance(&instance_info, nullptr, &out.instance);
  };
  VkResult instance_result = make_instance(/*with_portability=*/true);
  if (instance_result == VK_ERROR_EXTENSION_NOT_PRESENT ||
      instance_result == VK_ERROR_INCOMPATIBLE_DRIVER) {
    instance_result = make_instance(/*with_portability=*/false);
  }
  if (instance_result != VK_SUCCESS) {
    detail::log_vk("vkCreateInstance", instance_result);
    return false;
  }

  const VkResult surface_result =
      glfwCreateWindowSurface(out.instance, window, nullptr, &out.surface);
  if (surface_result != VK_SUCCESS) {
    detail::log_vk("glfwCreateWindowSurface", surface_result);
    return false;
  }

  // --- 3. Queue families, best plan first. ---------------------------------
  // The order matters and is not cosmetic. One family with two queues is
  // ideal: concurrent submission *and* no ownership transfer. Two families is
  // second: still concurrent, but seam B's shared buffer will need
  // VK_SHARING_MODE_CONCURRENT or an explicit release/acquire. One shared
  // queue is last: it serializes fusion against rendering, which is precisely
  // what the background fuse thread exists to avoid. MoltenVK reports several
  // graphics+compute+present families of one queue each, so on Apple the first
  // plan is unreachable and the second is what actually runs -- taking the
  // third there would hand back the concurrency the two-device design had.
  std::uint32_t device_count = 0;
  VkResult enum_result =
      vkEnumeratePhysicalDevices(out.instance, &device_count, nullptr);
  if (enum_result != VK_SUCCESS && enum_result != VK_INCOMPLETE) {
    detail::log_vk("vkEnumeratePhysicalDevices", enum_result);
    return false;
  }
  std::vector<VkPhysicalDevice> devices(device_count);
  if (device_count > 0) {
    enum_result =
        vkEnumeratePhysicalDevices(out.instance, &device_count, devices.data());
    if (enum_result != VK_SUCCESS && enum_result != VK_INCOMPLETE) {
      detail::log_vk("vkEnumeratePhysicalDevices", enum_result);
      return false;
    }
  }
  if (devices.empty()) {
    std::fprintf(stderr, "shared device: no Vulkan physical device\n");
    return false;
  }

  const VkQueueFlags gfx_flags = gfx_req.queue_flags | VK_QUEUE_GRAPHICS_BIT;
  // recon warns against adding VK_QUEUE_TRANSFER_BIT here: a conformant
  // compute family may legally not advertise it, and compute implies transfer.
  const VkQueueFlags recon_flags = recon_req.queue_flags;

  bool found = false;
  for (VkPhysicalDevice candidate : devices) {
    std::uint32_t family_count = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(candidate, &family_count, nullptr);
    std::vector<VkQueueFamilyProperties> families(family_count);
    vkGetPhysicalDeviceQueueFamilyProperties(candidate, &family_count,
                                             families.data());

    auto has = [&](std::uint32_t i, VkQueueFlags flags) {
      return (families[i].queueFlags & flags) == flags;
    };
    auto presents = [&](std::uint32_t i) {
      VkBool32 present = VK_FALSE;
      const VkResult r = vkGetPhysicalDeviceSurfaceSupportKHR(
          candidate, i, out.surface, &present);
      return r == VK_SUCCESS && present == VK_TRUE;
    };
    auto take = [&](QueuePlan plan, std::uint32_t g, std::uint32_t c) {
      out.physical_device = candidate;
      out.queue_plan = plan;
      out.graphics_family = g;
      out.compute_family = c;
      out.submit_mutex =
          plan == QueuePlan::kSharedQueue ? &out.submit_mutex_storage : nullptr;
      found = true;
    };

    for (std::uint32_t i = 0; i < family_count && !found; ++i) {
      if (has(i, gfx_flags) && has(i, recon_flags) &&
          families[i].queueCount >= 2 && presents(i)) {
        take(QueuePlan::kTwoQueuesOneFamily, i, i);
      }
    }
    for (std::uint32_t i = 0; i < family_count && !found; ++i) {
      if (!has(i, gfx_flags) || !presents(i)) continue;
      for (std::uint32_t j = 0; j < family_count && !found; ++j) {
        if (j != i && has(j, recon_flags)) {
          take(QueuePlan::kTwoFamilies, i, j);
        }
      }
    }
    for (std::uint32_t i = 0; i < family_count && !found; ++i) {
      if (has(i, gfx_flags) && has(i, recon_flags) && presents(i)) {
        take(QueuePlan::kSharedQueue, i, i);
      }
    }
    if (found) break;
  }
  if (!found) {
    std::fprintf(stderr,
                 "shared device: no physical device exposes a present-capable "
                 "graphics family alongside a compute family\n");
    return false;
  }

  // --- 4. The device, satisfying the union. --------------------------------
  std::uint32_t available_count = 0;
  vkEnumerateDeviceExtensionProperties(out.physical_device, nullptr,
                                       &available_count, nullptr);
  std::vector<VkExtensionProperties> available(available_count);
  const VkResult ext_result = vkEnumerateDeviceExtensionProperties(
      out.physical_device, nullptr, &available_count, available.data());
  if (ext_result != VK_SUCCESS && ext_result != VK_INCOMPLETE) {
    detail::log_vk("vkEnumerateDeviceExtensionProperties", ext_result);
    return false;
  }
  auto device_supports = [&](const char* name) {
    for (const VkExtensionProperties& extension : available) {
      if (std::strcmp(extension.extensionName, name) == 0) return true;
    }
    return false;
  };
  // Spec-required whenever a portability driver exposes it, and it is the
  // device *creator's* obligation -- which gfx's requirements deliberately
  // omit, since it is nobody's "need".
  if (device_supports("VK_KHR_portability_subset")) {
    add_extension("VK_KHR_portability_subset");
  }
  // Name what is missing rather than letting vkCreateDevice fail opaquely.
  for (const char* name : extensions) {
    if (!device_supports(name)) {
      std::fprintf(stderr,
                   "shared device: the device does not support a required "
                   "extension: %s\n",
                   name);
      return false;
    }
  }

  // Same for features: check support before enabling, so a device that cannot
  // do recon's shader ABI says which bit it lacks.
  VkPhysicalDeviceDynamicRenderingFeatures supported_dynamic_rendering{};
  supported_dynamic_rendering.sType =
      VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_DYNAMIC_RENDERING_FEATURES;
  VkPhysicalDeviceTimelineSemaphoreFeatures supported_timeline{};
  supported_timeline.sType =
      VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_TIMELINE_SEMAPHORE_FEATURES;
  supported_timeline.pNext = &supported_dynamic_rendering;
  VkPhysicalDeviceScalarBlockLayoutFeatures supported_scalar{};
  supported_scalar.sType =
      VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_SCALAR_BLOCK_LAYOUT_FEATURES;
  supported_scalar.pNext = &supported_timeline;
  VkPhysicalDeviceFeatures2 supported{};
  supported.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
  supported.pNext = &supported_scalar;
  vkGetPhysicalDeviceFeatures2(out.physical_device, &supported);

  const bool want_timeline =
      recon_req.timeline_semaphore || gfx_req.timeline_semaphore;
  const bool want_scalar = recon_req.scalar_block_layout;
  const bool want_dynamic_rendering = gfx_req.dynamic_rendering;
  struct FeatureCheck {
    bool wanted;
    VkBool32 supported;
    const char* name;
  };
  for (const FeatureCheck& check :
       {FeatureCheck{want_timeline, supported_timeline.timelineSemaphore,
                     "timelineSemaphore"},
        FeatureCheck{want_scalar, supported_scalar.scalarBlockLayout,
                     "scalarBlockLayout"},
        FeatureCheck{want_dynamic_rendering,
                     supported_dynamic_rendering.dynamicRendering,
                     "dynamicRendering"}}) {
    if (check.wanted && check.supported == VK_FALSE) {
      std::fprintf(stderr,
                   "shared device: the device does not support a required "
                   "feature: %s\n",
                   check.name);
      return false;
    }
  }
  const int missing_core =
      detail::first_unsupported_feature(wanted_features, supported.features);
  if (missing_core >= 0) {
    std::fprintf(stderr,
                 "shared device: the device does not support "
                 "VkPhysicalDeviceFeatures bit %d\n",
                 missing_core);
    return false;
  }

  const float priorities[2] = {1.0f, 1.0f};
  const bool one_family = out.graphics_family == out.compute_family;
  const std::uint32_t compute_queue_index =
      out.queue_plan == QueuePlan::kTwoQueuesOneFamily ? 1u : 0u;
  std::vector<VkDeviceQueueCreateInfo> queue_infos;
  VkDeviceQueueCreateInfo queue_info{};
  queue_info.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
  queue_info.pQueuePriorities = priorities;
  queue_info.queueFamilyIndex = out.graphics_family;
  queue_info.queueCount = compute_queue_index == 1u ? 2u : 1u;
  queue_infos.push_back(queue_info);
  if (!one_family) {
    queue_info.queueFamilyIndex = out.compute_family;
    queue_info.queueCount = 1u;
    queue_infos.push_back(queue_info);
  }

  // The union of both libraries' feature requests. Chained rather than merged
  // by hand so each library's published requirement is visible at the call.
  VkPhysicalDeviceDynamicRenderingFeatures dynamic_rendering{};
  dynamic_rendering.sType =
      VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_DYNAMIC_RENDERING_FEATURES;
  dynamic_rendering.dynamicRendering =
      want_dynamic_rendering ? VK_TRUE : VK_FALSE;
  // gfx publishes any *further* extended-feature structs it would enable as an
  // opaque chain and documents enabling them as the embedder's job -- its
  // Device::adopt cannot introspect one. Splice it on the tail. The const_cast
  // is safe: vkCreateDevice only reads pNext. (Null today; this keeps it from
  // being silently dropped the day it is not.)
  dynamic_rendering.pNext = const_cast<void*>(gfx_req.feature_chain);
  VkPhysicalDeviceTimelineSemaphoreFeatures timeline{};
  timeline.sType =
      VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_TIMELINE_SEMAPHORE_FEATURES;
  timeline.timelineSemaphore = want_timeline ? VK_TRUE : VK_FALSE;
  timeline.pNext = &dynamic_rendering;
  // recon reads its buffers through scalar block layout (its shader ABI), so
  // the shared device must enable it even though gfx does not ask.
  VkPhysicalDeviceScalarBlockLayoutFeatures scalar_layout{};
  scalar_layout.sType =
      VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_SCALAR_BLOCK_LAYOUT_FEATURES;
  scalar_layout.scalarBlockLayout = want_scalar ? VK_TRUE : VK_FALSE;
  scalar_layout.pNext = &timeline;

  VkPhysicalDeviceFeatures2 features{};
  features.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_FEATURES_2;
  features.features = wanted_features;
  features.pNext = &scalar_layout;

  VkDeviceCreateInfo device_info{};
  device_info.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
  device_info.pNext = &features;
  device_info.queueCreateInfoCount =
      static_cast<std::uint32_t>(queue_infos.size());
  device_info.pQueueCreateInfos = queue_infos.data();
  device_info.enabledExtensionCount =
      static_cast<std::uint32_t>(extensions.size());
  device_info.ppEnabledExtensionNames = extensions.data();
  const VkResult device_result =
      vkCreateDevice(out.physical_device, &device_info, nullptr, &out.device);
  if (device_result != VK_SUCCESS) {
    detail::log_vk("vkCreateDevice", device_result);
    return false;
  }

  // Record what was enabled, from the same values vkCreateDevice just saw.
  out.enabled_extensions = extensions;
  out.enabled_features = wanted_features;
  out.enabled_timeline_semaphore = want_timeline;
  out.enabled_scalar_block_layout = want_scalar;
  out.enabled_dynamic_rendering = want_dynamic_rendering;
  // Not config.enable_validation: the layer may have been absent, in which case
  // the instance above skipped the extension too and continued without it.
  out.enabled_debug_utils = validation_available;

  vkGetDeviceQueue(out.device, out.graphics_family, 0, &out.graphics_queue);
  vkGetDeviceQueue(out.device, out.compute_family, compute_queue_index,
                   &out.compute_queue);

  switch (out.queue_plan) {
    case QueuePlan::kTwoQueuesOneFamily:
      std::printf(
          "shared device: one VkDevice, family %u, 2 queues (gfx + recon)\n",
          out.graphics_family);
      break;
    case QueuePlan::kTwoFamilies:
      std::printf(
          "shared device: one VkDevice, family %u (gfx) + family %u (recon), "
          "1 queue each\n",
          out.graphics_family, out.compute_family);
      break;
    case QueuePlan::kSharedQueue:
      std::printf(
          "shared device: one VkDevice, family %u, 1 queue shared under a "
          "mutex -- fusion and rendering will serialize\n",
          out.graphics_family);
      break;
  }
  return true;
}

/// @brief The payload `recon::Device::adopt` needs from a @ref SharedDevice.
inline vr::AdoptedDevice recon_adopt_payload(const SharedDevice& shared) {
  vr::AdoptedDevice adopted;
  adopted.instance = shared.instance;
  adopted.physical_device = shared.physical_device;
  adopted.device = shared.device;
  adopted.compute_family = shared.compute_family;
  adopted.compute_queue = shared.compute_queue;
  adopted.submit_mutex = shared.submit_mutex;
  adopted.enabled_device_extensions = shared.enabled_extensions.data();
  adopted.enabled_device_extension_count =
      static_cast<std::uint32_t>(shared.enabled_extensions.size());
  adopted.enabled_features = shared.enabled_features;
  // Read back from what the bootstrap enabled, never asserted here: recon's
  // adopt trusts this declaration in place of a query Vulkan does not offer,
  // so a hand-written `true` would turn its verification into a no-op.
  adopted.enabled_timeline_semaphore = shared.enabled_timeline_semaphore;
  adopted.enabled_scalar_block_layout = shared.enabled_scalar_block_layout;
  return adopted;
}

/// @brief The payload `gfx::app::WindowedApp::adopt` needs from a
///        @ref SharedDevice.
inline vg::AdoptedDevice gfx_adopt_payload(const SharedDevice& shared) {
  vg::AdoptedDevice adopted;
  adopted.instance = shared.instance;
  adopted.physical_device = shared.physical_device;
  adopted.device = shared.device;
  adopted.graphics_family = shared.graphics_family;
  adopted.graphics_queue = shared.graphics_queue;
  // The graphics family was chosen for its present support, so the present
  // queue is the graphics queue.
  adopted.has_present = true;
  adopted.present_family = shared.graphics_family;
  adopted.present_queue = shared.graphics_queue;
  adopted.submit_mutex = shared.submit_mutex;
  adopted.enabled_device_extensions = shared.enabled_extensions.data();
  adopted.enabled_device_extension_count =
      static_cast<std::uint32_t>(shared.enabled_extensions.size());
  adopted.enabled_features = shared.enabled_features;
  // Read back from what the bootstrap enabled, never asserted here -- as in
  // recon_adopt_payload above. gfx's adopt verifies against this declaration
  // rather than against physical-device support, because Vulkan cannot be asked
  // what a *logical* device enabled: every 1.3 physical device reports
  // dynamicRendering as supported whatever vkCreateDevice was passed. The
  // fields default to "not enabled", so omitting them is not a lost
  // optimisation but a refused adopt at startup.
  adopted.enabled_timeline_semaphore = shared.enabled_timeline_semaphore;
  adopted.enabled_dynamic_rendering = shared.enabled_dynamic_rendering;
  adopted.enabled_debug_utils = shared.enabled_debug_utils;
  return adopted;
}

}  // namespace fuse_viewer
