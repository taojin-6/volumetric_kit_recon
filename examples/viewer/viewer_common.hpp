// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/viewer/viewer_common.hpp
/// @brief What `fuse_viewer` and `rig_viewer` share beyond gfx's `gfx_glfw`
///        window glue and recon's `mesh::MeshExchange`: the device both
///        libraries adopt, the scope guards their teardown order rests on,
///        the render pass each frame records, and the render camera the
///        fuse thread meshes.

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <mutex>
#include <thread>

#include <glm/glm.hpp>

#include "fuse_device.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/shared_device.hpp"
#include "volumetric_kit/core/vulkan/vulkan.hpp"
#include "volumetric_kit/gfx/app/shared_device_config.hpp"
#include "volumetric_kit/gfx/app/windowed_app.hpp"
#include "volumetric_kit/gfx/core/profiler.hpp"
#include "volumetric_kit/gfx/core/render_target.hpp"
#include "volumetric_kit/gfx/glfw/shared_device.hpp"
#include "volumetric_kit/gfx/ui/imgui_overlay.hpp"
#include "volumetric_kit/recon/mesh/device_mesh.hpp"
#include "volumetric_kit/recon/mesh/marching_cubes.hpp"
#include "volumetric_kit/recon/volume/frustum.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"
#include "volumetric_kit/recon/volume/voxel_hash_map.hpp"

namespace fuse_viewer {

namespace vg = volumetric_kit::gfx;
namespace vkc = volumetric_kit::core;
namespace vr = volumetric_kit::recon;
namespace rmesh = volumetric_kit::recon::mesh;
namespace vol = volumetric_kit::recon::volume;

/// @brief The instance and renderer settings a viewer's device is built with.
/// @param app         The config gfx's app is adopted with: its name, and the
///                    graphics requirements `WindowedApp::adopt` verifies.
/// @param validation  Whether to enable the validation layer
///                    (`window.validation`). `WindowedApp::adopt` ignores the
///                    app's own setting, so this is the only one.
/// @return @p app's name and device requirements, and @p validation.
inline vg::app::WindowDeviceConfig viewer_device_config(
    const vg::app::WindowedAppConfig& app, bool validation) {
  vg::app::WindowDeviceConfig config;
  config.instance.app_name = app.app_name;
  config.instance.enable_validation = validation;
  config.graphics = app.device;
  return config;
}

/// @brief The one device gfx, recon and the sensor package all adopt, with
///        @p window's surface: gfx's `make_shared_device` for recon's and the
///        sensor package's requirements and @ref viewer_device_config. Prints
///        the device's summary.
/// @param window      The window the app presents to; it must outlive the
///                    device.
/// @param app         As @ref viewer_device_config.
/// @param validation  As @ref viewer_device_config.
/// @return The device, to be declared before, and so outlive, everything
///         adopted or made on it; or `make_shared_device`'s failure, which
///         ends the run: two devices cannot share a `VkBuffer`.
inline vkc::Result<std::unique_ptr<vkc::SharedDevice>> make_viewer_device(
    GLFWwindow* window, const vg::app::WindowedAppConfig& app,
    bool validation) {
  VKC_ASSIGN(const vkc::DeviceRequirements reqs,
             vr_example::fuse_device_requirements());
  VKC_ASSIGN(std::unique_ptr<vkc::SharedDevice> shared,
             vg::glfw::make_shared_device(
                 window, reqs, viewer_device_config(app, validation)));
  std::printf("shared device: %s\n", shared->summary().c_str());
  return shared;
}

/// @brief recon's and the sensor package's share of @p shared: the device
///        both adopt, held to their requirements.
inline vkc::Result<vkc::Device> adopt_fuse_device(
    const vkc::SharedDevice& shared) {
  VKC_ASSIGN(const vkc::DeviceRequirements reqs,
             vr_example::fuse_device_requirements());
  return vkc::Device::adopt(shared.compute_payload(), reqs);
}

/// @brief gfx's app on @p shared, presenting to the surface @p shared made:
///        picking a present-capable device needed it, so the app takes that
///        one, and owns it, rather than making a second.
/// @param shared  From @ref make_viewer_device; it must outlive the app.
/// @param app     The config @p shared was built for.
/// @return The app, or `WindowedApp::adopt`'s failure.
inline vkc::Result<vg::app::WindowedApp> adopt_viewer_app(
    vkc::SharedDevice& shared, const vg::app::WindowedAppConfig& app) {
  return vg::app::WindowedApp::adopt(
      shared.graphics_payload(), app,
      [&shared](VkInstance instance) -> vkc::Result<VkSurfaceKHR> {
        // adopt calls this with the payload's instance, so this trips only if
        // the two stop coming from one SharedDevice: the surface would then
        // belong to another instance than the swapchain built on it.
        if (instance != shared.instance().handle()) {
          return vkc::Status::invalid_argument(
              "surface factory: the app adopted a different VkInstance than "
              "the bootstrap created the surface on");
        }
        return shared.release_surface();
      });
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

/// Detaches the profiler from the app's frame loop at scope exit, before the
/// profiler itself is destroyed -- the loop holds a bare pointer to it.
struct ProfilerDetach {
  vg::app::WindowedApp& app;
  ~ProfilerDetach() { app.set_profiler(nullptr); }
};

/// @brief Records a frame's one render pass into @p target: the clear,
///        @p draw_mesh, then @p overlay's UI.
///
/// One GPU scope, "render pass", spans the pass; "mesh draw" and
/// "overlay draw" inside it are CPU-only. Apple GPUs sample a timestamp only
/// at a pass's edge, so a GPU scope inside the pass reads about 0 ms there.
/// @param profiler   The frame's profiler, between its `begin_frame` and
///                   `end_frame`.
/// @param cmd        The frame's command buffer, outside any render pass.
/// @param target     The frame's target, in its attachment layouts.
/// @param draw_mesh  Called with @p cmd inside the pass to draw the mesh.
/// @param overlay    The UI, or null for none.
template <class DrawMesh>
void record_view_pass(vg::Profiler& profiler, VkCommandBuffer cmd,
                      const vg::RenderTarget& target, DrawMesh&& draw_mesh,
                      vg::ui::ImGuiOverlay* overlay) {
  vg::RenderTargetBeginInfo begin_info;
  begin_info.clear_color.float32[0] = 0.05f;
  begin_info.clear_color.float32[1] = 0.05f;
  begin_info.clear_color.float32[2] = 0.07f;
  begin_info.clear_color.float32[3] = 1.0f;
  const vg::Profiler::Scope pass = profiler.gpu_scope(cmd, "render pass");
  target.begin(cmd, begin_info);
  {
    const vg::Profiler::Scope mesh = profiler.cpu_scope("mesh draw");
    draw_mesh(cmd);
  }
  if (overlay != nullptr) {
    const vg::Profiler::Scope ui = profiler.cpu_scope("overlay draw");
    overlay->render(cmd);
  }
  target.end(cmd);
}

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
