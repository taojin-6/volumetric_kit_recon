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
/// What is left here is the viewer's side: the GLFW surface. recon and gfx
/// both state their requirements as the core's `DeviceRequirements` and adopt
/// the core's `AdoptedDevice`, so the shared device's payloads go to each
/// unconverted: `graphics_payload()` to `WindowedApp::adopt`,
/// `compute_payload()` to recon's `Device::adopt`.

#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/core/vulkan/shared_device.hpp"
#include "volumetric_kit/core/vulkan/vk_result.hpp"
#include "volumetric_kit/gfx/app/windowed_app.hpp"
#include "volumetric_kit/gfx/core/device_requirements.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"

namespace fuse_viewer {

namespace vkc = volumetric_kit::core;
namespace vr = volumetric_kit::recon;
namespace vg = volumetric_kit::gfx;

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
/// The returned device owns the instance, device and surface, and destroys
/// them after both adopters have released their wrappers: declare it before
/// the app and recon's device so it outlives them. The surface is handed to
/// gfx's factory through `release_surface()`, since picking a present-capable
/// device needed it first.
///
/// @param window  The GLFW window to present to; its required instance
///                extensions are enabled and its surface created here.
/// @param config  Embedder-owned knobs (validation, app name).
/// @return The device; null, with a specific reason on stderr, when the
///         loader, the hardware, or the driver cannot satisfy the union. The
///         caller must treat that as fatal: running the two libraries on
///         separate devices would silently give up the shared-`VkBuffer` seam
///         this bootstrap exists to establish.
inline std::unique_ptr<vkc::SharedDevice> build_shared_device(
    GLFWwindow* window, const SharedDeviceConfig& config) {
  vkc::SharedDeviceConfig shared;
  shared.instance.app_name = config.app_name;
  shared.instance.enable_validation = config.enable_validation;
  std::uint32_t glfw_extension_count = 0;
  const char** glfw_extensions =
      glfwGetRequiredInstanceExtensions(&glfw_extension_count);
  if (glfw_extensions == nullptr) {
    std::fprintf(stderr,
                 "shared device: GLFW found no Vulkan surface support\n");
    return nullptr;
  }
  for (std::uint32_t i = 0; i < glfw_extension_count; ++i) {
    shared.instance.extensions.push_back(glfw_extensions[i]);
  }

  // Neither library is consulted about the other: each states its needs, and
  // the shared device satisfies the union. gfx needs present; recon compute.
  shared.compute = vr::device_requirements();
  shared.graphics = vg::device_requirements();
  shared.graphics.needs_present = true;
  shared.make_surface =
      [window](VkInstance instance) -> vkc::Result<VkSurfaceKHR> {
    VkSurfaceKHR surface = VK_NULL_HANDLE;
    VKC_VK_TRY(glfwCreateWindowSurface(instance, window, nullptr, &surface));
    return surface;
  };

  vkc::Result<std::unique_ptr<vkc::SharedDevice>> made =
      vkc::SharedDevice::create(shared);
  if (!made) {
    std::fprintf(stderr, "shared device: %s\n",
                 made.status().message().c_str());
    return nullptr;
  }
  std::unique_ptr<vkc::SharedDevice> device = std::move(made).value();
  std::printf("shared device: %s\n", device->summary().c_str());
  if (device->plan() == vkc::QueuePlan::SharedQueue) {
    std::fprintf(stderr,
                 "shared device: one queue shared under a mutex -- fusion and "
                 "rendering will serialize\n");
  }
  return device;
}

}  // namespace fuse_viewer
