// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/viewer/viewer_common.hpp
/// @brief What `fuse_viewer` and `rig_viewer` share beyond the device
///        bootstrap and recon's `mesh::MeshExchange`: the scope guards their
///        teardown order rests on, and the render camera the fuse thread
///        meshes.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <thread>

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>
#include <imgui_impl_glfw.h>
#include <glm/glm.hpp>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/vulkan.hpp"
#include "volumetric_kit/gfx/app/windowed_app.hpp"
#include "volumetric_kit/recon/mesh/device_mesh.hpp"
#include "volumetric_kit/recon/mesh/marching_cubes.hpp"
#include "volumetric_kit/recon/volume/frustum.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"
#include "volumetric_kit/recon/volume/voxel_hash_map.hpp"

namespace fuse_viewer {

namespace vg = volumetric_kit::gfx;
namespace vkc = volumetric_kit::core;
namespace rmesh = volumetric_kit::recon::mesh;
namespace vol = volumetric_kit::recon::volume;

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

/// @brief The render camera, handed from the render thread to the fuse
///        thread, which meshes what it sees (the 2026-10-06 decision).
class SharedView {
 public:
  /// Margin the view's frustum is widened by, for the frames the view moves
  /// on before the mesh is drawn.
  static constexpr float kMargin = 0.25f;  // metres

  /// Render thread: this frame's view. The serial moves only when it does.
  void publish(const glm::mat4& view_proj) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (view_proj != view_proj_) {
      view_proj_ = view_proj;
      ++serial_;
    }
  }

  /// Fuse thread: whether the view moved since the last @ref extract.
  bool moved() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return serial_ != meshed_;
  }

  /// Fuse thread: mesh the blocks inside the view, widened by @ref kMargin;
  /// the whole map until a view is published. @p timings, when given, carries
  /// the compaction as its `compact_ms`.
  vkc::Result<rmesh::DeviceMesh> extract(rmesh::MarchingCubes& extractor,
                                         vol::VoxelBlockGrid& volume,
                                         rmesh::ExtractTimings* timings) {
    glm::mat4 view_proj;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      view_proj = view_proj_;
      meshed_ = serial_;
    }
    if (meshed_ == 0) return extractor.extract_device(volume, 0.0f, timings);
    const auto start = std::chrono::steady_clock::now();
    VKC_ASSIGN(const vol::DeviceBlockList visible,
               volume.map().compact_active_blocks_in_frusta_on_device(
                   {vol::make_frustum_planes(view_proj, kMargin)}));
    const double compact_ms = std::chrono::duration<double, std::milli>(
                                  std::chrono::steady_clock::now() - start)
                                  .count();
    auto mesh = extractor.extract_device(volume, 0.0f, visible, timings);
    if (timings != nullptr) timings->compact_ms = compact_ms;
    return mesh;
  }

 private:
  mutable std::mutex mutex_;
  glm::mat4 view_proj_{1.0f};
  std::uint64_t serial_ = 0;  // 0: no view yet
  std::uint64_t meshed_ = 0;  // the serial last meshed; the fuse thread's
};

}  // namespace fuse_viewer
