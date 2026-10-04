// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/viewer/shared_device.hpp
/// @brief The viewers' one `VkInstance` + `VkDevice`, built from recon's and
///        gfx's requirements, for both to adopt.
///
/// This is the embedder half of the create/adopt seam both libraries expose.
/// Neither owns the device: each publishes its requirements, the family's
/// `SharedDevice` (volumetric_kit_core) satisfies their union and makes the
/// window's surface on the instance, and each library adopts the same handles
/// -- recon through `Device::adopt`, gfx through `WindowedApp::adopt`. That is
/// what makes zero-copy possible at all: a `VkBuffer` is valid only on the
/// device that created it, so recon geometry can be drawn by gfx *only* if
/// there is one device.
///
/// What is left here is the viewer's side: the GLFW surface, and gfx's
/// requirements and payload in the core's terms, as gfx still has a device type
/// of its own.

#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#include "volumetric_kit/core/vulkan/shared_device.hpp"
#include "volumetric_kit/gfx/app/windowed_app.hpp"
#include "volumetric_kit/gfx/core/device.hpp"
#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/vk_result.hpp"

namespace fuse_viewer {

namespace vkc = volumetric_kit::core;
namespace vr = volumetric_kit::recon;
namespace vg = volumetric_kit::gfx;

/// @brief The shared device, and what the viewers read off it.
///
/// Holds the core's `SharedDevice`, which owns the instance, device and
/// surface and destroys them after both adopters have released their wrappers:
/// declare this before the app and recon's device so it outlives them.
struct SharedDevice {
  /// The family's shared device; null until @ref build_shared_device fills it.
  std::unique_ptr<vkc::SharedDevice> device;
  /// The instance both libraries adopt.
  VkInstance instance = VK_NULL_HANDLE;
  /// gfx's family (graphics + present) and recon's (compute). Equal under
  /// every plan but `QueuePlan::TwoFamilies`.
  std::uint32_t graphics_family = 0;
  std::uint32_t compute_family = 0;

  /// @brief Give up ownership of the surface to the caller.
  ///
  /// gfx's surface factory contract is that the app adopts and later destroys
  /// what the factory returns, but the surface had to exist first -- picking a
  /// physical device requires testing present support against a real surface.
  /// So it is created with the device and handed over, and stops being tracked
  /// there; destroying it twice would be a use-after-free at teardown.
  /// @return The surface, or `VK_NULL_HANDLE` once released.
  VkSurfaceKHR release_surface() {
    return device != nullptr ? device->release_surface() : VK_NULL_HANDLE;
  }
};

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

namespace detail {

// gfx's requirements in the core's terms.
// TODO: drop once gfx adopts the core's DeviceRequirements.
inline vkc::DeviceRequirements core_requirements(
    const vg::DeviceRequirements& gfx) {
  vkc::DeviceRequirements reqs;
  reqs.api_version = gfx.api_version;
  reqs.queue_flags = gfx.queue_flags;
  reqs.needs_present = gfx.needs_present;
  for (const char* name : gfx.device_extensions) {
    reqs.extensions.emplace_back(name);
  }
  reqs.features = gfx.features;
  reqs.timeline_semaphore = gfx.timeline_semaphore;
  reqs.dynamic_rendering = gfx.dynamic_rendering;
  // The core raises the bits it enables in the chain's structs, so it takes
  // them mutable; gfx hands out the chain its DeviceConfig was given.
  reqs.feature_chain = const_cast<void*>(gfx.feature_chain);
  return reqs;
}

}  // namespace detail

/// @brief Build one instance + device satisfying both libraries, and the
///        window's surface on it.
///
/// @param window  The GLFW window to present to; its required instance
///                extensions are enabled and its surface created here.
/// @param config  Embedder-owned knobs (validation, app name).
/// @param out     Filled on success.
/// @return `false`, with a specific reason on stderr, when the loader, the
///         hardware, or the driver cannot satisfy the union. The caller must
///         treat that as fatal: running the two libraries on separate devices
///         would silently give up the shared-`VkBuffer` seam this bootstrap
///         exists to establish.
inline bool build_shared_device(GLFWwindow* window,
                                const SharedDeviceConfig& config,
                                SharedDevice& out) {
  vkc::SharedDeviceConfig shared;
  shared.instance.app_name = config.app_name;
  shared.instance.enable_validation = config.enable_validation;
  std::uint32_t glfw_extension_count = 0;
  const char** glfw_extensions =
      glfwGetRequiredInstanceExtensions(&glfw_extension_count);
  if (glfw_extensions == nullptr) {
    std::fprintf(stderr,
                 "shared device: GLFW found no Vulkan surface support\n");
    return false;
  }
  for (std::uint32_t i = 0; i < glfw_extension_count; ++i) {
    shared.instance.extensions.push_back(glfw_extensions[i]);
  }

  // Neither library is consulted about the other: each states its needs, and
  // the shared device satisfies the union. gfx needs present; recon compute.
  vg::DeviceConfig gfx_config;
  gfx_config.needs_present = true;
  shared.compute = vr::device_requirements();
  shared.graphics =
      detail::core_requirements(vg::Device::requirements(gfx_config));
  shared.make_surface =
      [window](VkInstance instance) -> vr::Result<VkSurfaceKHR> {
    VkSurfaceKHR surface = VK_NULL_HANDLE;
    VR_VK_TRY(glfwCreateWindowSurface(instance, window, nullptr, &surface));
    return surface;
  };

  vr::Result<std::unique_ptr<vkc::SharedDevice>> made =
      vkc::SharedDevice::create(shared);
  if (!made) {
    std::fprintf(stderr, "shared device: %s\n",
                 made.status().message().c_str());
    return false;
  }
  out.device = std::move(made).value();
  out.instance = out.device->instance().handle();
  out.graphics_family = out.device->graphics_family();
  out.compute_family = out.device->compute_family();
  std::printf("shared device: %s\n", out.device->summary().c_str());
  if (out.device->plan() == vkc::QueuePlan::SharedQueue) {
    std::fprintf(stderr,
                 "shared device: one queue shared under a mutex -- fusion and "
                 "rendering will serialize\n");
  }
  return true;
}

/// @brief The payload `recon::Device::adopt` needs from a @ref SharedDevice.
inline vr::AdoptedDevice recon_adopt_payload(const SharedDevice& shared) {
  return shared.device->compute_payload();
}

/// @brief The payload `gfx::app::WindowedApp::adopt` needs from a
///        @ref SharedDevice: the core's graphics payload, field for field.
// TODO: drop once gfx adopts the core's AdoptedDevice.
inline vg::AdoptedDevice gfx_adopt_payload(const SharedDevice& shared) {
  const vkc::AdoptedDevice core = shared.device->graphics_payload();
  vg::AdoptedDevice adopted;
  adopted.instance = core.instance;
  adopted.physical_device = core.physical_device;
  adopted.device = core.device;
  adopted.graphics_family = core.queue_family;
  adopted.graphics_queue = core.queue;
  adopted.has_present = core.has_present;
  adopted.present_family = core.present_family;
  adopted.present_queue = core.present_queue;
  // The graphics queue presents, so its mutex covers both.
  adopted.submit_mutex = core.submit_mutex;
  adopted.enabled_device_extensions = core.enabled_extensions;
  adopted.enabled_device_extension_count = core.enabled_extension_count;
  adopted.enabled_features = core.enabled_features;
  // Read back from what the shared device enabled, never asserted here: gfx's
  // adopt verifies against this declaration rather than against
  // physical-device support, because Vulkan cannot be asked what a *logical*
  // device enabled.
  adopted.enabled_timeline_semaphore = core.enabled_timeline_semaphore;
  adopted.enabled_dynamic_rendering = core.enabled_dynamic_rendering;
  adopted.enabled_debug_utils = core.enabled_debug_utils;
  return adopted;
}

}  // namespace fuse_viewer
