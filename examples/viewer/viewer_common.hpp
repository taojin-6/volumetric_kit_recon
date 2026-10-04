// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/viewer/viewer_common.hpp
/// @brief What the live viewers share beyond the device bootstrap: the scope
///        guards their teardown order rests on, the render side of recon's
///        mesh ring -- the release mark and the check that a published mesh
///        can be bound as geometry -- and the orbit camera's state.

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <thread>
#include <vector>

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>
#include <imgui_impl_glfw.h>
#include <glm/glm.hpp>

#include "volumetric_kit/gfx/app/windowed_app.hpp"
#include "volumetric_kit/gfx/core/vulkan.hpp"
#include "volumetric_kit/recon/mesh/device_mesh.hpp"

namespace fuse_viewer {

namespace vg = volumetric_kit::gfx;
namespace rmesh = volumetric_kit::recon::mesh;

/// @return The window's framebuffer size, at least 1 x 1.
inline VkExtent2D window_extent(GLFWwindow* window) {
  int width = 0, height = 0;
  glfwGetFramebufferSize(window, &width, &height);
  return {static_cast<std::uint32_t>(std::max(1, width)),
          static_cast<std::uint32_t>(std::max(1, height))};
}

/// Signals a fuse thread to quit and joins it on scope exit, so an exception
/// unwinding the render loop cannot destroy a still-joinable std::thread
/// (which would call std::terminate). Declared right after the thread so it
/// runs first at scope exit -- while the recon resources the thread borrows
/// are still live.
struct QuitJoin {
  std::thread& thread;
  std::atomic<bool>& quit;
  ~QuitJoin() {
    quit.store(true);
    if (thread.joinable()) thread.join();
  }
};

/// Shuts the ImGui GLFW platform backend down at scope exit. Declared *after*
/// the overlay so it runs first: ImGui_ImplGlfw_Shutdown touches the ImGui
/// context the overlay owns, so it must not outlive it.
struct ImGuiGlfwShutdown {
  bool active;
  ~ImGuiGlfwShutdown() {
    if (active) ImGui_ImplGlfw_Shutdown();
  }
};

/// Detaches the profiler from the app's frame loop at scope exit, before the
/// profiler itself is destroyed -- the loop holds a bare pointer to it.
struct ProfilerDetach {
  vg::app::WindowedApp& app;
  ~ProfilerDetach() { app.set_profiler(nullptr); }
};

/// @brief Retire @p slot's generation and return the mark the fuse thread
///        may release recon's ring through.
///
/// Called once begin_frame has fence-waited @p slot, so the frame that last
/// used it has completed. What remains in @p frame_generations is exactly the
/// generations frames still in flight are reading, read as a SET: one
/// generation is normally drawn by several consecutive frames, so the retired
/// frame's is often still read by a newer one, and releasing on it would hand
/// recon a slot a live vkCmdDrawIndexedIndirect reads (a grow frees its
/// buffers outright). Everything strictly below their minimum is finished.
///
/// With no other frame in flight holding one, the floor is what this frame is
/// about to draw, @p live_generation. Only when nothing has been committed at
/// all (generation 0, since recon numbers extracts from 1) does everything
/// taken so far, @p newest_taken, become releasable -- the path that drains
/// the ring when takes are accepted but never drawn.
inline std::uint64_t retire_and_release_mark(
    std::vector<std::uint64_t>& frame_generations, std::uint32_t slot,
    std::uint64_t live_generation, std::uint64_t newest_taken) {
  frame_generations[slot] = 0;
  std::uint64_t oldest_in_flight = 0;
  for (const std::uint64_t g : frame_generations) {
    if (g != 0 && (oldest_in_flight == 0 || g < oldest_in_flight)) {
      oldest_in_flight = g;
    }
  }
  if (oldest_in_flight == 0) oldest_in_flight = live_generation;
  return oldest_in_flight > 0 ? oldest_in_flight - 1 : newest_taken;
}

/// @brief Why gfx may not bind @p mesh as geometry, or null if it may.
///
/// Verified, not assumed: recon reports the usage and sharing mode its
/// buffers were made with because Vulkan cannot be asked, and binding one
/// that lacks a usage bit is undefined with layers off. The sharing mode is
/// the term that can vary: reading an EXCLUSIVE buffer from a family that does
/// not own it is undefined, and on Apple undefined in the way that appears to
/// work. Checked only when @p cross_family, since recon collapses the pair to
/// EXCLUSIVE on one family, which is correct there.
///
/// Not for an empty mesh, which may carry null handles by design and draws
/// nothing.
inline const char* unbindable_reason(const rmesh::DeviceMesh& mesh,
                                     bool cross_family) {
  if (cross_family && mesh.sharing_mode != VK_SHARING_MODE_CONCURRENT) {
    return "its buffers are EXCLUSIVE but recon and gfx are on different "
           "queue families, so binding them would be undefined";
  }
  if (!mesh.valid() ||
      (mesh.vertex_usage & VK_BUFFER_USAGE_VERTEX_BUFFER_BIT) == 0 ||
      (mesh.index_usage & VK_BUFFER_USAGE_INDEX_BUFFER_BIT) == 0 ||
      (mesh.indirect_usage & VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT) == 0) {
    return "usage bits or handles missing";
  }
  return nullptr;
}

// A turntable around `target`, about a camera's image-up axis. recon's world
// is the capture's, whose cameras follow OpenCV (+Y down), so gfx's
// OrbitCamera -- which fixes world +Y as up -- would stand it on its head.
struct OrbitView {
  glm::vec3 target{0.0f};
  glm::vec3 up{0.0f, -1.0f, 0.0f};
  glm::vec3 forward{0.0f, 0.0f, 1.0f};
  glm::vec3 right{1.0f, 0.0f, 0.0f};
  float distance = 2.0f;
  float azimuth = 0.0f;
  float elevation = 0.0f;

  // Look from a capture pose at the point `distance` ahead of it.
  static OrbitView from_pose(const glm::mat4& c2w, float distance) {
    OrbitView v;
    v.forward = glm::normalize(glm::vec3(c2w[2]));
    v.up = -glm::normalize(glm::vec3(c2w[1]));
    v.right = glm::normalize(glm::cross(v.forward, v.up));
    v.target = glm::vec3(c2w[3]) + distance * v.forward;
    v.distance = distance;
    return v;
  }
  // At azimuth = elevation = 0 the eye is `distance` behind the target along
  // `forward`: where the camera looks from, when it looks at it.
  glm::vec3 eye() const {
    const glm::vec3 around =
        std::cos(azimuth) * -forward + std::sin(azimuth) * right;
    return target +
           distance * (std::cos(elevation) * around + std::sin(elevation) * up);
  }
};

// Scroll arrives through a callback; the render loop reads what built up.
struct ScrollInput {
  double pending = 0.0;
};

}  // namespace fuse_viewer
