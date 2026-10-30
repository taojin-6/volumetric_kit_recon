// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// rig_viewer: live reconstruction from a synced rig of Orbbec cameras, drawn
// as it grows and textured from every camera at once. fuse_viewer's sibling
// for a live rig rather than a recorded sequence: the same shared device
// (gfx_glfw's make_shared_device), the same fuse thread and mesh ring, and the
// same HybridMeshPipeline drawing recon's buffers in place.
//
// Nothing the cameras produce visits the host after it arrives. Each camera of
// the sync file is a sensor::OrbbecSensor, and a sensor::SensorArray reads
// them as one: it hands out sets as captured, their colour left on the GPU by
// the hardware decoder, and SensorArray::process undistorts and converts every
// camera's frame there, in one batch; the frames fuse through the device-input
// overloads; the marching-cubes mesh is textured from every camera of the set
// (texture::ProjectiveTexturer, each view's depth on the device, its colour
// camera its own, and its colour the coverage that keeps the lens's black
// corners off the mesh); and the atlas is filled by copying each camera's
// colour buffer into its tile of a gfx image, recorded in the renderer's own
// frame before the draw. The only copies are device to device.
//
// What makes the atlas copy safe, each checked or derived rather than assumed:
//   * sharing -- the frame prep's colour is made CONCURRENT across recon's
//     and gfx's queue families (GpuFramePrepConfig::color_queue_families),
//     since MoltenVK hands the two libraries different families, where reading
//     an EXCLUSIVE buffer is undefined in the way that appears to work.
//     Verified per buffer before its camera textures the mesh, so a buffer gfx
//     cannot copy costs that camera's triangles rather than the mesh's texture.
//   * lifetime -- the atlas is gfx's StreamedAtlas, whose images are reused
//     only once the frame loop's timeline passes the last frame that drew
//     them, and the colour buffers a copy reads are held until the frame that
//     copies them completes (viewer_atlas.hpp's LiveAtlas).
//   * visibility -- as for the mesh: the frame prep fence-waits its batch
//     before process returns, and gfx's later vkQueueSubmit makes those
//     writes visible to the copy.
//
// The atlas is laid out once, from the rig's colour cameras, with
// texture::side_by_side_atlas (four cameras make a 2 x 2 grid), so it never
// changes size. A set missing a camera textures that camera from its last
// frame, into the same tile (Options::hold_ms).
//
// The mesh is unshared: texturing from several views chooses a camera per
// triangle, which a vertex shared between triangles that chose differently
// cannot express (see ProjectiveTexturer).
//
//   rig_viewer --config config/examples/rig_viewer.json
//   rig_viewer --help
//
// Without source.calibration every camera sits at the world origin, which fuses
// a rig of several into nonsense; the viewer says so and runs anyway. Drag with
// the left button to orbit, the right to pan, and scroll to zoom; the View
// panel switches shading, texturing and the viewpoint, and tunes the fusion
// and texturing knobs below live. Closing the window, or Ctrl+C in the
// terminal, stops the rig before the process ends.
//
// The panels show the newest set. At the end the run prints its rates, the
// array's and each camera's counters, the atlas copies' device time and a
// table per set (vr_example::SetLog, as fuse_orbbec's): each set's rows, the
// device's counters across it and its latency, to the hand-over of its mesh
// or, for a set the viewer did not mesh, to its last stage, the two tabled
// apart.

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#define GLFW_INCLUDE_NONE  // window and input only: gfx_glfw makes the surface
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <glm/glm.hpp>

#include "config.hpp"
#include "fuse_frame.hpp"        // fuse_set, free_stale_blocks_every
#include "fuse_orbbec_sets.hpp"  // on_host_clock, print_rig_stats
#include "interrupt.hpp"
#include "recon_gfx_bridge.hpp"  // to_live_mesh, and the vertex-layout asserts
#include "rig_report.hpp"
#include "set_log.hpp"
#include "viewer_atlas.hpp"
#include "viewer_common.hpp"

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/base/stage_metrics.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/buffer.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/core/vulkan/image.hpp"
#include "volumetric_kit/core/vulkan/shared_device.hpp"
#include "volumetric_kit/core/vulkan/vulkan.hpp"
#include "volumetric_kit/grid/mesh/device_mesh.hpp"
#include "volumetric_kit/grid/mesh/marching_cubes.hpp"
#include "volumetric_kit/grid/mesh/mesh_exchange.hpp"
#include "volumetric_kit/grid/volume/voxel_block_grid.hpp"
#include "volumetric_kit/grid/volume/voxel_hash_map.hpp"
#include "volumetric_kit/recon/texture/projective_texturer.hpp"
#include "volumetric_kit/recon/texture/texture_atlas.hpp"
#include "volumetric_kit/recon/tsdf/fuser.hpp"
#include "volumetric_kit/recon/tsdf/fusion_grid.hpp"
#include "volumetric_kit/sensor/array/sensor_array.hpp"
#include "volumetric_kit/sensor/camera/array_calibration.hpp"
#include "volumetric_kit/sensor/camera/camera_model.hpp"
#include "volumetric_kit/sensor/prep/gpu_frame_prep.hpp"
#include "volumetric_kit/sensor/rgbd_sensor.hpp"

#include "volumetric_kit/gfx/app/windowed_app.hpp"
#include "volumetric_kit/gfx/camera/camera.hpp"
#include "volumetric_kit/gfx/core/frame_metrics.hpp"
#include "volumetric_kit/gfx/core/profiler.hpp"
#include "volumetric_kit/gfx/core/render_target.hpp"
#include "volumetric_kit/gfx/glfw/imgui_glfw.hpp"
#include "volumetric_kit/gfx/glfw/window.hpp"
#include "volumetric_kit/gfx/pipelines/hybrid_mesh_pipeline.hpp"
#include "volumetric_kit/gfx/pipelines/live_mesh.hpp"
#include "volumetric_kit/gfx/ui/imgui_overlay.hpp"
#include "volumetric_kit/gfx/ui/metrics_panel.hpp"
#include "volumetric_kit/gfx/windowing/frame_loop.hpp"
#include "volumetric_kit/gfx/windowing/swapchain.hpp"

namespace vr = volumetric_kit::recon;
namespace vkc = volumetric_kit::core;
namespace vol = volumetric_kit::grid::volume;
namespace rtsdf = volumetric_kit::recon::tsdf;
namespace rmesh = volumetric_kit::grid::mesh;
namespace rtex = volumetric_kit::recon::texture;
namespace rsensor = volumetric_kit::sensor;
namespace rcamera = volumetric_kit::sensor::camera;
namespace vg = volumetric_kit::gfx;
namespace glfw = volumetric_kit::gfx::glfw;
namespace vgp = volumetric_kit::gfx::pipelines;
namespace win = volumetric_kit::gfx::windowing;

namespace {

// How long the fuse thread waits for a set before saying the rig is silent. It
// keeps waiting after saying so: a window is not a batch run, and a camera
// coming back is worth drawing.
constexpr auto kSilenceLimit = std::chrono::seconds(10);

// The sets the table at the end covers, the newest, and as many atlas copies:
// 10 minutes at 25 Hz, about 15 MB of rows, so a window left open does not
// grow without bound.
constexpr std::size_t kLoggedSets = 15000;

// The three ways the hybrid pipeline shades, as the View panel offers them.
using Shading = vgrid_example::config::Shading;

std::uint32_t shading_flags(Shading shading) {
  switch (shading) {
    case Shading::Lit:
      return vgp::kHybridMeshLit;
    case Shading::Unlit:
      return 0u;
    case Shading::Normals:
      return vgp::kHybridMeshNormals;
  }
  return vgp::kHybridMeshLit;
}

using Options = vgrid_example::config::Config;

// A turntable around `target`, about the primary camera's image-up axis.
// recon's world is the rig's, whose cameras follow OpenCV (+Y down), so gfx's
// OrbitCamera -- which fixes world +Y as up -- would stand it on its head.
struct OrbitView {
  glm::vec3 target{0.0f};
  glm::vec3 up{0.0f, -1.0f, 0.0f};
  glm::vec3 forward{0.0f, 0.0f, 1.0f};
  glm::vec3 right{1.0f, 0.0f, 0.0f};
  float distance = 2.0f;
  float azimuth = 0.0f;
  float elevation = 0.0f;

  // At azimuth = elevation = 0 the eye is `distance` behind the target along
  // `forward`: where the primary camera looks from, when it looks at it.
  glm::vec3 eye() const {
    const glm::vec3 around =
        std::cos(azimuth) * -forward + std::sin(azimuth) * right;
    return target +
           distance * (std::cos(elevation) * around + std::sin(elevation) * up);
  }
};

// The point nearest every camera's optical axis in the least-squares sense --
// where a rig built around a subject is looking. `fallback` when the axes are
// (nearly) parallel, as one camera's always is, or meet behind any camera.
glm::vec3 axes_meet(const std::vector<glm::mat4>& poses, glm::vec3 fallback) {
  glm::mat3 a(0.0f);
  glm::vec3 b(0.0f);
  for (const glm::mat4& c2w : poses) {
    const glm::vec3 d = glm::normalize(glm::vec3(c2w[2]));
    const glm::mat3 away = glm::mat3(1.0f) - glm::outerProduct(d, d);
    a += away;
    b += away * glm::vec3(c2w[3]);
  }
  if (poses.size() < 2 || std::fabs(glm::determinant(a)) < 1e-6f) {
    return fallback;
  }
  const glm::vec3 p = glm::inverse(a) * b;
  for (const glm::mat4& c2w : poses) {
    if (glm::dot(p - glm::vec3(c2w[3]), glm::normalize(glm::vec3(c2w[2]))) <
        0.1f) {
      return fallback;
    }
  }
  return p;
}

// Scroll arrives through a callback; the render loop reads what built up.
struct ScrollInput {
  double pending = 0.0;
};

// A camera's newest frame with colour, and the set it came in (sets count
// from 1).
struct NewestFrame {
  rsensor::DeviceFrame frame;
  std::uint64_t set = 0;
};

// The frame a remesh textures one camera from, and whether it is held: from
// an earlier set.
struct TextureSource {
  const rsensor::DeviceFrame* frame = nullptr;
  bool held = false;
};

// Per camera, its newest frame: this set's, or an earlier one taken at most
// `hold_ns` before `set_ns` on the host's clock (Options::hold_ms).
std::vector<TextureSource> texture_sources(
    const std::vector<std::optional<NewestFrame>>& newest, std::uint64_t set,
    std::uint64_t set_ns, std::uint64_t hold_ns) {
  std::vector<TextureSource> out(newest.size());
  for (std::size_t c = 0; c < newest.size(); ++c) {
    if (!newest[c]) continue;
    const bool held = newest[c]->set != set;
    const std::uint64_t taken = newest[c]->frame.timestamp_ns;
    if (!held || (set_ns >= taken && set_ns - taken <= hold_ns)) {
      out[c] = {&newest[c]->frame, held};
    }
  }
  return out;
}

// Owns the WindowedApp and every device resource, so they are gone before
// main destroys the window (see fuse_viewer's run()).
int run(GLFWwindow* window, const Options& opt) {
  // --- One VkDevice, adopted by both libraries (as fuse_viewer) ------------
  vg::app::WindowedAppConfig config;
  config.app_name = "rig_viewer";
  config.swapchain.extent = glfw::framebuffer_extent(window);
  config.swapchain.depth_format = VK_FORMAT_D32_SFLOAT;
  config.frames_in_flight = 2;
  auto shared_r =
      fuse_viewer::make_viewer_device(window, config, opt.window.validation);
  if (!shared_r) {
    std::fprintf(stderr, "shared device: %s\n",
                 shared_r.status().message().c_str());
    return 1;
  }
  const std::unique_ptr<vkc::SharedDevice> shared = std::move(shared_r).value();

  auto app_r = fuse_viewer::adopt_viewer_app(*shared, config);
  if (!app_r.ok()) {
    std::fprintf(stderr, "WindowedApp::adopt: %s\n",
                 app_r.status().message().c_str());
    return 1;
  }
  vg::app::WindowedApp app = std::move(app_r).value();

  auto recon_device_result = fuse_viewer::adopt_fuse_device(*shared);
  if (!recon_device_result) {
    std::fprintf(stderr, "recon Device::adopt: %s\n",
                 recon_device_result.status().message().c_str());
    return 1;
  }
  auto recon_allocator_result = vkc::Allocator::create(
      shared->instance().handle(), recon_device_result.value());
  if (!recon_allocator_result) {
    std::fprintf(stderr, "recon allocator: %s\n",
                 recon_allocator_result.status().message().c_str());
    return 1;
  }
  vkc::Device& rdevice = recon_device_result.value();
  vkc::Allocator& rallocator = recon_allocator_result.value();

  // --- The cameras, opened onto the shared device as one array -------------
  // One OrbbecSensor per camera of the sync file, after the device, which
  // they decode onto and which must outlive them. Opened here, on the main
  // thread, so a camera that does not answer ends the run with its reason
  // before the window starts drawing; the fuse thread starts them.
  rsensor::SensorArray::Options array_options;
  if (!opt.source.calibration.empty()) {
    auto calibration = rcamera::read_array_calibration(opt.source.calibration);
    if (!calibration) {  // the message names the file
      std::fprintf(stderr, "rig_viewer: %s\n",
                   calibration.status().message().c_str());
      return 1;
    }
    array_options.calibration = std::move(calibration).value();
  }
  array_options.device = &rdevice;
  array_options.allocator = &rallocator;
  // The colour buffers are what gfx copies into its atlas, so the array's
  // passes share them with gfx's family too -- the same reasoning as the mesh
  // buffers below, and the same unconditional pair. Depth stays EXCLUSIVE:
  // the viewer draws no points.
  array_options.prep.color_queue_families[0] = shared->compute_family();
  array_options.prep.color_queue_families[1] = shared->graphics_family();
  array_options.prep.color_queue_family_count = 2;
  // The prep crops each camera's frame to what both its cameras see, so
  // depth its colour camera does not see is not fused.
  // The streams, the same for every camera, at the calibration's colour
  // exposure, opened and printed as fuse_orbbec opens them.
  auto sensors = vr_example::open_orbbec_cameras(opt, array_options.calibration,
                                                 rdevice, rallocator);
  if (!sensors) {  // the message names the file or the camera
    std::fprintf(stderr, "rig_viewer: %s\n",
                 sensors.status().message().c_str());
    return 1;
  }
  auto array_result =
      rsensor::SensorArray::open(std::move(sensors).value(), array_options);
  if (!array_result) {
    std::fprintf(stderr, "rig_viewer: %s\n",
                 array_result.status().message().c_str());
    return 1;
  }
  rsensor::SensorArray array = std::move(array_result).value();
  const std::size_t cameras = array.size();
  std::vector<std::string> serials;
  std::vector<rcamera::CameraModel> color_cameras;
  std::vector<glm::mat4> camera_poses;
  for (std::size_t i = 0; i < cameras; ++i) {
    const rsensor::SensorInfo& info = array.sensor(i).info();
    if (!info.color) {
      std::fprintf(stderr, "rig_viewer: camera %s has no colour\n",
                   info.id.c_str());
      return 1;
    }
    serials.push_back(info.id);
    color_cameras.push_back(*info.color);
    // Where the array poses the camera's frames: the origin without a
    // calibration.
    const rcamera::SensorCalibration* calibrated =
        rcamera::find_sensor(array_options.calibration, info.id);
    camera_poses.push_back(calibrated != nullptr && calibrated->color_to_world
                               ? glm::mat4(*calibrated->color_to_world)
                               : glm::mat4(1.0f));
    const rcamera::CameraModel& cam = color_cameras.back();
    const glm::vec3 at(camera_poses.back()[3]);
    std::printf("camera %zu: %s%s, colour %ux%u, at (%.3f, %.3f, %.3f) m\n", i,
                serials.back().c_str(),
                i == array.primary() ? " (primary)" : "", cam.size.width,
                cam.size.height, at.x, at.y, at.z);
  }
  if (opt.source.calibration.empty() && cameras > 1) {
    std::fprintf(
        stderr,
        "rig_viewer: no source.calibration, so every camera sits at the "
        "origin and the %zu cameras fuse over one another\n",
        cameras);
  }

  // --- recon: volume, fuser, extractor, texturer --------------------------
  auto grid_result = rtsdf::create_fusion_grid(
      rdevice, rallocator, opt.fusion.voxel_size_m, opt.fusion.truncation_m);
  if (!grid_result) {
    std::fprintf(stderr, "grid: %s\n", grid_result.status().message().c_str());
    return 1;
  }
  vol::VoxelBlockGrid volume = std::move(grid_result).value();
  rtsdf::FuserConfig fuser_config;
  fuser_config.max_grows_per_set = 2;
  // Keep the live viewer updating at capacity.
  fuser_config.allow_partial = true;
  auto fuser_result = rtsdf::Fuser::create(rdevice, rallocator, fuser_config);
  if (!fuser_result) {
    std::fprintf(stderr, "fuser: %s\n",
                 fuser_result.status().message().c_str());
    return 1;
  }
  rtsdf::Fuser fuser = std::move(fuser_result).value();
  // What gfx needs of the mesh buffers, as fuse_viewer states it: the draw's
  // usage bits, both queue families, and a ring a slot deeper than the frames
  // in flight. Unshared, for the several-view texturing.
  rmesh::MarchingCubesConfig mc_config;
  mc_config.extra_vertex_usage = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
  mc_config.extra_index_usage = VK_BUFFER_USAGE_INDEX_BUFFER_BIT;
  mc_config.queue_families[0] = shared->compute_family();
  mc_config.queue_families[1] = shared->graphics_family();
  mc_config.queue_family_count = 2;
  mc_config.slot_count = config.frames_in_flight + 1;
  auto extractor_result =
      rmesh::MarchingCubes::create(rdevice, rallocator, mc_config);
  if (!extractor_result) {
    std::fprintf(stderr, "marching cubes: %s\n",
                 extractor_result.status().message().c_str());
    return 1;
  }
  rmesh::MarchingCubes extractor = std::move(extractor_result).value();
  auto texturer_result = rtex::ProjectiveTexturer::create(rdevice, rallocator);
  if (!texturer_result) {
    std::fprintf(stderr, "texturer: %s\n",
                 texturer_result.status().message().c_str());
    return 1;
  }
  rtex::ProjectiveTexturer texturer = std::move(texturer_result).value();

  // The atlas: one tile per camera, laid out once from the rig's colour
  // cameras. A set textures from the cameras it has, into their own tiles.
  rtex::AtlasLayout layout;
  {
    std::vector<rtex::TextureView> sizes(cameras);
    for (std::size_t i = 0; i < cameras; ++i) {
      sizes[i].image_width = color_cameras[i].size.width;
      sizes[i].image_height = color_cameras[i].size.height;
    }
    auto laid_out =
        rtex::side_by_side_atlas(sizes, texturer.max_atlas_extent());
    if (!laid_out) {
      std::fprintf(stderr, "atlas: %s\n", laid_out.status().message().c_str());
      return 1;
    }
    layout = std::move(laid_out).value();
    std::printf("atlas: %ux%u, %zu tiles\n", layout.width, layout.height,
                layout.tiles.size());
  }

  // --- gfx: pipeline, atlas, profiler, overlay -----------------------------
  auto pipeline_result = vgp::HybridMeshPipeline::create(
      app.device(), app.allocator(), app.swapchain().layout());
  if (!pipeline_result.ok()) {
    std::fprintf(stderr, "pipeline: %s\n",
                 pipeline_result.status().message().c_str());
    return 1;
  }
  vgp::HybridMeshPipeline pipeline = std::move(pipeline_result).value();
  fuse_viewer::LiveAtlas atlas(
      pipeline, app.allocator(), app.frame_loop().timeline(),
      {layout.width, layout.height}, config.frames_in_flight);

  vg::ProfilerConfig profiler_config;
  profiler_config.frames_in_flight = config.frames_in_flight;
  auto profiler_result = vg::Profiler::create(app.device(), profiler_config);
  if (!profiler_result.ok()) {
    std::fprintf(stderr, "profiler: %s\n",
                 profiler_result.status().message().c_str());
    return 1;
  }
  vg::Profiler profiler = std::move(profiler_result).value();
  profiler.set_memory_source(&app.allocator());
  app.set_profiler(&profiler);
  const fuse_viewer::ProfilerDetach profiler_guard{app};

  // Before ImGui's GLFW backend, which chains to the callback it finds.
  ScrollInput scroll;
  glfwSetWindowUserPointer(window, &scroll);
  glfwSetScrollCallback(window, [](GLFWwindow* w, double, double dy) {
    static_cast<ScrollInput*>(glfwGetWindowUserPointer(w))->pending += dy;
  });

  std::optional<vg::ui::ImGuiOverlay> overlay;
  // Declared after the overlay, so it shuts down first, while the context it
  // binds is alive.
  std::unique_ptr<glfw::ImGuiGlfw> imgui_glfw;
  if (opt.window.overlay) {
    vg::ui::ImGuiOverlayConfig overlay_config;
    overlay_config.layout = app.swapchain().layout();
    overlay_config.min_image_count = app.swapchain().image_count();
    overlay_config.image_count = app.swapchain().image_count();
    auto overlay_result = vg::ui::ImGuiOverlay::create(
        app.device(), app.instance_handle(), overlay_config);
    if (!overlay_result.ok()) {
      std::fprintf(stderr, "overlay: %s\n",
                   overlay_result.status().message().c_str());
      return 1;
    }
    overlay = std::move(overlay_result).value();
    auto imgui_glfw_result =
        glfw::ImGuiGlfw::create(window, overlay->context());
    if (!imgui_glfw_result) {
      std::fprintf(stderr, "ImGuiGlfw: %s\n",
                   imgui_glfw_result.status().message().c_str());
      return 1;
    }
    imgui_glfw = std::move(imgui_glfw_result).value();
  }

  // The colour-by-camera view's sources: one device-local buffer a camera,
  // its tile's size, filled with that camera's colour (sRGB bytes, as the
  // atlas holds). Made the first time the view is switched on, since at 4K
  // they are 33 MB a camera, and filled on the device in that frame's command
  // buffer: host-visible, a discrete GPU would copy them across the bus on
  // every remesh.
  const std::array<std::array<std::uint8_t, 3>, 8> kCameraColours = {{
      {230, 60, 60},
      {60, 200, 80},
      {60, 110, 240},
      {240, 210, 50},
      {210, 70, 210},
      {60, 210, 220},
      {245, 140, 40},
      {200, 200, 200},
  }};
  std::vector<vkc::Buffer> solid_buffers;
  auto ensure_solid = [&](VkCommandBuffer cmd) -> bool {
    if (solid_buffers.size() == cameras) return true;
    solid_buffers.clear();
    for (std::size_t c = 0; c < cameras; ++c) {
      const rtex::AtlasTile& tile = layout.tiles[c];
      vkc::BufferDesc desc;
      desc.size = VkDeviceSize(tile.width) * tile.height * 4u;
      desc.usage =
          VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
      desc.memory = vkc::MemoryUsage::DeviceOnly;
      auto buffer = app.allocator().create_buffer(desc);
      if (!buffer.ok()) {
        std::fprintf(stderr, "rig_viewer: colour-by-camera buffer: %s\n",
                     buffer.status().message().c_str());
        solid_buffers.clear();
        return false;
      }
      solid_buffers.push_back(std::move(buffer).value());
    }
    // Filled only once every buffer exists, so a failed allocation above
    // frees none that a recorded fill names.
    for (std::size_t c = 0; c < cameras; ++c) {
      const auto& rgb = kCameraColours[c % kCameraColours.size()];
      const std::uint32_t word = std::uint32_t{rgb[0]} |
                                 (std::uint32_t{rgb[1]} << 8) |
                                 (std::uint32_t{rgb[2]} << 16) | 0xFF000000u;
      vkCmdFillBuffer(cmd, solid_buffers[c].handle(), 0, VK_WHOLE_SIZE, word);
    }
    // The fills land before the atlas copy reads them, later in this buffer.
    VkMemoryBarrier filled{};
    filled.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
    filled.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    filled.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
    vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TRANSFER_BIT,
                         VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 1, &filled, 0,
                         nullptr, 0, nullptr);
    return true;
  };

  // --- The view -------------------------------------------------------------
  // Around where the cameras look, from the primary's side.
  OrbitView home;
  float vfov = 1.0f;
  {
    const glm::mat4& c2w = camera_poses[array.primary()];
    const glm::vec3 eye(c2w[3]);
    home.forward = glm::normalize(glm::vec3(c2w[2]));
    home.up = -glm::normalize(glm::vec3(c2w[1]));  // image up is -cameraY
    home.right = glm::normalize(glm::cross(home.forward, home.up));
    const float depth_mid =
        0.5f * (opt.fusion.min_depth_m + opt.fusion.max_depth_m);
    home.target = axes_meet(camera_poses, eye + home.forward * depth_mid);
    home.distance = std::max(0.3f, glm::length(home.target - eye));
    const rcamera::CameraModel& cam = color_cameras[array.primary()];
    vfov = 2.0f *
           std::atan(
               static_cast<float>(cam.size.height) /
               (2.0f * std::max(1.0f, static_cast<float>(cam.intrinsics.fy))));
  }
  OrbitView view = home;
  // Looking out of camera i, set by the View panel, until a drag orbits away.
  std::optional<std::size_t> from_camera;
  Shading shading = opt.render.shading;
  std::atomic<bool> texture_on{opt.render.texture};
  // The fusion and texturing knobs the View panel tunes while the rig runs,
  // read by the fuse thread at each set and each remesh.
  std::atomic<bool> dynamic_on{opt.fusion.dynamic};
  std::atomic<float> max_weight{opt.fusion.max_weight};
  std::atomic<float> occlusion{opt.fusion.occlusion_m};
  // Render-thread state: whether the next atlas copy fills each tile with its
  // camera's colour rather than its image.
  bool show_sources = opt.render.show_sources;

  // --- Fuse thread ----------------------------------------------------------
  // The array, its frame prep, fusion, extraction and texturing all run here;
  // the render thread only copies the atlas and draws. The mesh crosses with
  // its atlas job through the exchange; the rest is under share_mtx.
  const bool cross_family =
      shared->graphics_family() != shared->compute_family();
  rmesh::MeshExchangeConfig exchange_config;
  exchange_config.frames_in_flight = config.frames_in_flight;
  exchange_config.cross_family = cross_family;
  rmesh::MeshExchange<fuse_viewer::AtlasJob> exchange(exchange_config);
  fuse_viewer::SharedView shared_view;  // the render camera, meshed
  std::mutex share_mtx;
  std::vector<vkc::StageRow> shared_fuse_stages;
  fuse_viewer::RigPanel shared_panel;
  std::atomic<bool> fusing_done{false};
  // Set by whatever ended fusion early, so a scripted run exits non-zero.
  std::atomic<bool> fuse_failed{false};
  std::atomic<bool> quit{false};
  // Each set's figures and the sets' rate, recorded by the fuse thread alone
  // and printed once it has joined. A set's trigger is on the system clock
  // only where every camera stamps on the host's, as a rig's do.
  vr_example::SetLog log([&rdevice] { return rdevice.counters(); },
                         kLoggedSets);
  fuse_viewer::CountRate set_rate;
  const bool host_clock = vr_example::on_host_clock(array);

  std::thread fuse_thread([&]() {
    try {
      vkc::StageMetrics fuse_stages;
      // The remesh rows, held between remeshes (see fuse_viewer).
      vkc::StageMetrics remesh_stages;
      std::size_t cameras_textured = 0;
      std::size_t cameras_held = 0;
      std::uint64_t held_views = 0;
      std::uint64_t short_remeshes = 0;
      std::array<std::uint64_t, fuse_viewer::kUntexturedReasons> untextured{};
      bool texture_error_reported = false;
      // Each camera's newest frame with colour, which the remeshes texture
      // from (texture_sources).
      std::vector<std::optional<NewestFrame>> newest(cameras);
      const std::uint64_t hold_ns =
          static_cast<std::uint64_t>(opt.fusion.hold_ms) * 1000000u;
      std::vector<bool> uncopyable_reported(cameras, false);

      // Texture `mesh` from each camera's source (texture_sources: the set's
      // frame, or the camera's held one) and publish the two together: uv0
      // index into those images, so the mesh and its atlas are one value.
      auto publish = [&](const rmesh::DeviceMesh& mesh,
                         const std::vector<TextureSource>& sources) {
        fuse_viewer::AtlasJob job;
        std::size_t views_held = 0;
        std::size_t why = fuse_viewer::kTextureOff;
        if (!texture_on.load()) {
          why = fuse_viewer::kTextureOff;
        } else if (mesh.empty()) {
          why = fuse_viewer::kEmptyMesh;
        } else {
          why = fuse_viewer::kNoColour;
          std::vector<rtex::TextureView> views;
          rtex::AtlasLayout present{layout.width, layout.height, {}};
          for (std::size_t c = 0; c < sources.size() && c < cameras; ++c) {
            if (sources[c].frame == nullptr) continue;
            const rsensor::DeviceFrame& f = *sources[c].frame;
            if (!fuse_viewer::copyable(*f.color, layout.tiles[c],
                                       cross_family)) {
              // Said once a camera: its format is fixed at open, so every
              // set would repeat it.
              if (!uncopyable_reported[c]) {
                std::fprintf(stderr,
                             "rig_viewer: camera %zu's colour cannot be "
                             "copied into the atlas (not a copy source, "
                             "short of its tile, or EXCLUSIVE across two "
                             "queue families); texturing without it\n",
                             c);
                uncopyable_reported[c] = true;
              }
              continue;
            }
            // The frame's own buffers, held by the view for the call: its
            // depth, and its colour as the coverage, so a vertex where the
            // lens saw nothing is not textured black from this camera.
            rtex::TextureView v;
            v.cam = f.depth_camera;
            v.depth_buffer = f.depth;
            v.color_camera = f.color_camera;
            v.coverage = f.color;
            v.fallback = sources[c].held;
            views.push_back(std::move(v));
            present.tiles.push_back(layout.tiles[c]);
            job.tiles.push_back({f.color, layout.tiles[c], c});
            views_held += sources[c].held ? 1 : 0;
          }
          if (!views.empty()) {
            const vkc::Status textured = texturer.texture(
                mesh, views, present, occlusion.load(), &remesh_stages);
            if (textured.ok()) {
              cameras_textured = views.size();
              cameras_held = views_held;
              held_views += views_held;
              short_remeshes += views.size() < cameras ? 1 : 0;
            } else {
              why = fuse_viewer::kTextureFailed;
              // Said once: a refusal here is a configuration fault (a frame
              // whose colour is not its tile's size, say), and every remesh
              // would repeat it.
              if (!texture_error_reported) {
                std::fprintf(stderr, "rig_viewer: texture: %s\n",
                             textured.message().c_str());
                texture_error_reported = true;
              }
              job = fuse_viewer::AtlasJob{};
            }
          } else {
            job = fuse_viewer::AtlasJob{};
          }
        }
        if (job.empty()) {
          cameras_textured = 0;
          cameras_held = 0;
          ++untextured[why];
        }
        exchange.publish(mesh, std::move(job));
      };
      // render.texture_stats: read `mesh` back and count, per camera, the
      // triangles it textured (uv0 inside its tile; a triangle's three share
      // one) and those no camera did, marking the cameras textured from a held
      // frame. A readback of the whole mesh, hence a flag.
      std::uint64_t remeshes = 0;
      auto print_texture_stats = [&](const rmesh::DeviceMesh& mesh,
                                     const std::vector<TextureSource>& from) {
        const auto start = std::chrono::steady_clock::now();
        auto host = extractor.download(mesh);
        if (!host) {
          std::fprintf(stderr, "rig_viewer: texture stats: %s\n",
                       host.status().message().c_str());
          return;
        }
        const fuse_viewer::TextureShares shares =
            fuse_viewer::count_texture_shares(host.value().vertices, layout);
        std::vector<bool> held(from.size());
        for (std::size_t c = 0; c < from.size(); ++c) held[c] = from[c].held;
        fuse_viewer::print_texture_shares(
            shares, held, occlusion.load(), dynamic_on.load(),
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start)
                .count());
      };
      // `published`, when given, runs once the mesh is handed over, before
      // render.texture_stats' readback, which so counts in no set.
      auto remesh = [&](const std::vector<TextureSource>& sources,
                        const std::function<void()>& published) {
        remesh_stages.clear();
        vkc::Result<rmesh::DeviceMesh> extracted = [&]() {
          vkc::StageScope scope(remesh_stages, "extract");
          return shared_view.extract(extractor, volume, nullptr);
        }();
        // Published even when empty: it holds a ring slot all the same.
        if (extracted) {
          publish(extracted.value(), sources);
          if (published) published();
          // Before the next extract, which retires this mesh.
          if (opt.render.texture_stats && remeshes++ % 30 == 0) {
            print_texture_stats(extracted.value(), sources);
          }
        } else {
          std::fprintf(stderr, "rig_viewer: extract: %s\n",
                       extracted.status().message().c_str());
        }
      };

      const vkc::Status started = array.start();
      if (!started.ok()) {
        std::fprintf(stderr, "rig_viewer: start: %s\n",
                     started.message().c_str());
        fuse_failed.store(true);
      }
      // Whether the map and the device are still fit to mesh: false after a
      // failed frame prep or fuse, not after a capture failure, which leaves
      // what was fused intact.
      bool map_usable = true;
      std::uint64_t sets = 0;
      std::uint64_t frames_fused = 0;
      auto last_set = std::chrono::steady_clock::now();
      std::uint64_t last_set_ns = 0;
      bool said_silent = false;
      // Re-mesh when the view moved while no set fused, from the last set's
      // frames: an idle rig, or one a run.sets run has stopped. Such a remesh
      // is no set's.
      auto remesh_if_view_moved = [&]() {
        if (!shared_view.moved() ||
            !exchange.release_and_may_publish(extractor)) {
          return false;
        }
        remesh(texture_sources(newest, sets, last_set_ns, hold_ns), {});
        return true;
      };
      while (started.ok() && !quit.load()) {
        const auto poll_start = std::chrono::steady_clock::now();
        auto polled = array.poll_set();
        const double poll_ms =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - poll_start)
                .count();
        if (!polled) {
          std::fprintf(stderr, "rig_viewer: poll: %s\n",
                       polled.status().message().c_str());
          fuse_failed.store(true);
          break;
        }
        if (!polled.value()) {
          // Published only when it changes: this path runs every millisecond.
          const bool silent =
              std::chrono::steady_clock::now() - last_set > kSilenceLimit;
          if (silent != said_silent) {
            said_silent = silent;
            std::lock_guard<std::mutex> lock(share_mtx);
            shared_panel.silent = silent;
          }
          if (!remesh_if_view_moved()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          }
          continue;
        }
        last_set = std::chrono::steady_clock::now();
        said_silent = false;
        // The set's figures in the log: its poll, then, from its arrival, its
        // stages and, when it ran one, its remesh's; and its latency, to the
        // mesh's hand-over or, unmeshed, to its last stage.
        vr_example::SetLog::Set& s = log.begin();
        s.kind = "unmeshed";
        s.rows.add_cpu("poll", poll_ms);
        const rsensor::FrameSet& set = *polled.value();
        s.arrived(set.arrival_ns, host_clock ? set.timestamp_ns : 0);
        const std::uint64_t set_ns = set.timestamp_ns;
        last_set_ns = set_ns;
        // Released before this set's prep when no later set may texture from
        // them, so it can reuse their buffers; till then a view-change remesh
        // textures from them.
        if (hold_ns == 0 || !texture_on.load()) {
          newest.assign(cameras, std::nullopt);
        }
        // One "frame prep" row for the set: the cameras prepare in one batch.
        auto prepared = array.process(set, &s.rows);
        if (!prepared) {
          std::fprintf(stderr, "rig_viewer: frame prep: %s\n",
                       prepared.status().message().c_str());
          fuse_failed.store(true);
          map_usable = false;
          break;
        }
        const std::vector<std::optional<rsensor::DeviceFrame>>& frames =
            prepared.value().frames;
        const vkc::Status fused = vr_example::fuse_set(
            fuser, volume, frames, max_weight.load(),
            vr_example::integration_mode(dynamic_on.load()), &s.rows);
        if (!fused.ok()) {
          std::fprintf(stderr, "rig_viewer: fuse: %s\n",
                       fused.message().c_str());
          fuse_failed.store(true);
          map_usable = false;
          break;
        }
        for (const std::optional<rsensor::DeviceFrame>& frame : frames) {
          frames_fused += frame ? 1 : 0;
        }
        ++sets;
        set_rate.at(sets, last_set);
        // Ahead of the remesh, so it meshes the smaller set.
        vr_example::free_stale_blocks_every(volume, sets, opt.fusion.free_after,
                                            "rig_viewer", &s.rows);
        for (std::size_t c = 0; c < frames.size() && c < cameras; ++c) {
          if (frames[c] && frames[c]->has_color()) {
            newest[c] = NewestFrame{*frames[c], sets};
          }
        }
        // Fusion alone, before the remesh rows go in: those describe the
        // newest remesh, not this set, and are reported beside it.
        const double fuse_ms = s.rows.total_cpu_ms(/*exclude=*/"poll");
        // The panel shows the newest set's rows, seeded in display order so
        // the remesh rows read 0 until the first remesh rather than drop out,
        // and the newest remesh's beside them.
        fuse_stages.clear();
        for (const char* stage : {"poll", "frame prep", "allocate", "resize",
                                  "integrate", "  ..active set", "  ..allocate",
                                  "block stamps", "extract", "texture"}) {
          fuse_stages.seed(stage);
        }
        fuse_stages.merge(s.rows);
        // A remesh's rows join the set that ran it, once, before its hand-over
        // ends the set, so their waits count in its "set".
        const auto meshed = [&s, &remesh_stages] {
          s.rows.merge(remesh_stages);
          s.kind = "meshed";
          s.published();
        };

        // The last set of a run.sets run is always meshed: it meshes the last
        // sets fused, and only a view change supersedes it. So it waits a
        // moment for the renderer to collect the previous mesh, then extracts
        // whether or not it did, as fuse_viewer's final extract does -- a
        // minimized window never collects, and skipping lost the surface with
        // nothing said. Not once the window has closed, where an extract and
        // texture would only stall the join.
        const bool last = opt.run.sets > 0 &&
                          sets >= static_cast<std::uint64_t>(opt.run.sets);
        if (last) exchange.wait_collected(std::chrono::seconds(1), quit);
        if (!quit.load() &&
            (last ||
             sets % static_cast<std::uint64_t>(opt.fusion.remesh_every) == 0)) {
          if (exchange.release_and_may_publish(extractor)) {
            remesh(texture_sources(newest, sets, set_ns, hold_ns), meshed);
          } else if (last) {
            std::fprintf(stderr,
                         "rig_viewer: the renderer never collected the last "
                         "mesh (window hidden, or drawing stopped); meshing "
                         "the last set anyway\n");
            remesh(texture_sources(newest, sets, set_ns, hold_ns), meshed);
          }
        }
        fuse_stages.merge(remesh_stages);  // this set's, or the one held
        s.published();  // unmeshed, its fusion was its last stage; else no-op
        {
          const vkc::MemoryStats memory = rallocator.memory_stats();
          const vkc::Result<float> lf = volume.map().load_factor();
          const rsensor::SensorArrayStats stats = array.stats();
          std::lock_guard<std::mutex> lock(share_mtx);
          shared_fuse_stages = fuse_stages.rows();
          shared_panel.fuse_ms = fuse_ms;
          shared_panel.remesh_ms = remesh_stages.total_cpu_ms();
          shared_panel.sets_fused = sets;
          shared_panel.frames_fused = frames_fused;
          shared_panel.cameras_textured = cameras_textured;
          shared_panel.cameras_held = cameras_held;
          shared_panel.held_views = held_views;
          shared_panel.short_remeshes = short_remeshes;
          shared_panel.untextured = untextured;
          shared_panel.stats = stats;
          shared_panel.recon_memory = memory;
          shared_panel.map_blocks = volume.grid().num_blocks;
          shared_panel.map_load_factor = lf ? lf.value() : -1.0f;
          shared_panel.silent = false;
        }
        if (last) break;
      }
      // A run.sets run has stopped fusing, or the capture failed with the map
      // intact, but the view still moves: re-mesh what it sees whenever it
      // changes, until the window closes.
      if (started.ok() && map_usable && !quit.load()) {
        array.stop();
        fusing_done.store(true);
        while (!quit.load()) {
          if (!remesh_if_view_moved()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
          }
        }
      }
    } catch (const std::exception& e) {
      std::fprintf(stderr, "rig_viewer: fuse thread aborted: %s\n", e.what());
      fuse_failed.store(true);
    }
    // Outside the try, so an exception stops the cameras too.
    array.stop();
    fusing_done.store(true);
  });
  fuse_viewer::QuitJoin fuse_guard{fuse_thread, quit};

  // --- Render thread (main) -------------------------------------------------
  // A taken version's atlas job is recorded into this frame's command buffer.
  bool atlas_error_said = false;  // until an atlas commits again
  std::vector<vkc::StageRow> fuse_stages_snapshot;
  fuse_viewer::RigPanel panel;
  double last_x = 0.0, last_y = 0.0;
  bool have_last = false;

  std::printf(
      "rig_viewer: %zu cameras, fusing on a background thread; close "
      "the window to quit\n",
      cameras);
  int exit_code = 0;
  int drawn = 0;
  std::uint64_t atlas_copies = 0;
  // The mesh rate's window: when it began, and the generation drawn then.
  auto rate_start = std::chrono::steady_clock::now();
  std::uint64_t rate_generation = 0;
  double mesh_rate = 0.0;
  // For the end: the meshes drawn a second while the rig fuses, and the
  // atlas copies' device time.
  fuse_viewer::CountRate mesh_updates;
  fuse_viewer::AtlasCopyTimes copy_times(config.frames_in_flight, kLoggedSets);
  while (glfwWindowShouldClose(window) == GLFW_FALSE &&
         (opt.run.frames == 0 || drawn < opt.run.frames)) {
    glfwPollEvents();
    // Ctrl+C closes the window: the loop leaves as on a click, the fuse
    // thread stops the rig, and the cameras are released.
    if (vgrid_example::interrupted()) {
      glfwSetWindowShouldClose(window, GLFW_TRUE);
      continue;
    }

    auto frame = app.begin_frame(glfw::framebuffer_size(window));
    if (!frame.ok()) {
      std::fprintf(stderr, "begin_frame: %s\n",
                   frame.status().message().c_str());
      exit_code = 1;
      break;
    }
    if (!frame.value().has_value()) {
      glfwWaitEventsTimeout(0.02);
      continue;
    }
    const win::Frame& render_frame = *frame.value();
    // The frame loop has begun the profiler's frame, which published the
    // frame this slot carried before.
    copy_times.begun(render_frame.slot, profiler.metrics().sections);
    atlas.poll();

    // Retire this slot's last frame, take the newest mesh, and commit it
    // with its atlas, copied in this command buffer -- before the frame's
    // rendering begins, since a copy may not sit inside it. A copy refused,
    // or an atlas that could not be made, keeps the mesh parked for the next
    // frame.
    bool copied = false;
    {
      vg::Profiler::Scope copy_scope =
          profiler.gpu_scope(render_frame.cmd, fuse_viewer::kAtlasCopyRow);
      const rmesh::ExchangeOutcome outcome = exchange.begin_frame(
          render_frame.slot,
          [&](const rmesh::DeviceMesh&, const fuse_viewer::AtlasJob& job) {
            // Every tile is one the fuse thread found copyable, since it
            // left out any camera whose colour is not. The solid colours are
            // filled in this command buffer, so ahead of the copy.
            const bool solid =
                !job.empty() && show_sources && ensure_solid(render_frame.cmd);
            const vkc::Status committed =
                atlas.commit(render_frame.cmd, render_frame.number, job,
                             solid ? &solid_buffers : nullptr);
            if (!committed.ok()) {
              // Retried every frame, so said once until it succeeds.
              if (!atlas_error_said) {
                std::fprintf(stderr, "rig_viewer: atlas: %s\n",
                             committed.message().c_str());
                atlas_error_said = true;
              }
              return false;
            }
            atlas_error_said = false;
            atlas_copies += job.empty() ? 0 : 1;
            copied = !job.empty();
            return true;
          });
      if (outcome == rmesh::ExchangeOutcome::kRefused) {
        std::fprintf(stderr,
                     "rig_viewer: the extracted mesh cannot be bound as "
                     "geometry (%s); drawing stops here\n",
                     exchange.refused());
      }
    }
    const rmesh::DeviceMesh& live_view = exchange.live();
    {
      std::lock_guard<std::mutex> lock(share_mtx);
      fuse_stages_snapshot = shared_fuse_stages;
      // The mesh rows are filled below, from what this frame draws.
      panel = shared_panel;
    }
    panel.vertices = live_view.vertex_count;
    panel.triangles = live_view.triangle_count;
    panel.mesh_version = live_view.generation;
    {
      // A generation is one extract, and every extract is committed, so their
      // rise over a second is the meshes drawn in it.
      const auto now = std::chrono::steady_clock::now();
      const double seconds =
          std::chrono::duration<double>(now - rate_start).count();
      if (seconds >= 1.0) {
        mesh_rate =
            static_cast<double>(live_view.generation - rate_generation) /
            seconds;
        rate_generation = live_view.generation;
        rate_start = now;
      }
      panel.mesh_rate = mesh_rate;
      // Once a set has fused and until fusion stops, so neither a mesh of
      // the empty map before the first set nor a remesh for a view change
      // after the last stretches the span.
      if (panel.sets_fused > 0 && !fusing_done.load()) {
        mesh_updates.at(live_view.generation, now);
      }
    }

    // --- Input: orbit, pan, zoom, unless a panel has the mouse -------------
    {
      const bool ui_mouse = overlay && ImGui::GetIO().WantCaptureMouse;
      double x = 0.0, y = 0.0;
      // Window coordinates, not glfw::cursor_in_framebuffer: framebuffer
      // pixels would double orbit and pan speed on a HiDPI display.
      glfwGetCursorPos(window, &x, &y);
      const float dx = have_last ? static_cast<float>(x - last_x) : 0.0f;
      const float dy = have_last ? static_cast<float>(y - last_y) : 0.0f;
      last_x = x;
      last_y = y;
      have_last = true;
      const glm::vec3 eye = view.eye();
      const glm::vec3 look = glm::normalize(view.target - eye);
      const glm::vec3 side = glm::normalize(glm::cross(look, view.up));
      const glm::vec3 lift = glm::cross(side, look);
      if (!ui_mouse &&
          glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
        view.azimuth -= dx * 0.005f;
        view.elevation =
            std::clamp(view.elevation + dy * 0.005f, -1.55f, 1.55f);
        from_camera.reset();
      }
      if (!ui_mouse &&
          glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS) {
        view.target += (-dx * side + dy * lift) * view.distance * 0.0015f;
        from_camera.reset();
      }
      if (!ui_mouse && scroll.pending != 0.0) {
        view.distance = std::max(
            0.05f,
            view.distance * std::pow(0.9f, static_cast<float>(scroll.pending)));
        from_camera.reset();
      }
      scroll.pending = 0.0;
    }

    const VkExtent2D extent = app.swapchain().extent();
    const float aspect = static_cast<float>(extent.width) /
                         static_cast<float>(std::max(1u, extent.height));
    const float far_plane =
        2.0f * (view.distance + std::max(opt.fusion.max_depth_m, 1.0f));
    glm::mat4 view_proj(1.0f);
    if (from_camera) {
      const glm::mat4& c2w = camera_poses[*from_camera];
      const glm::vec3 eye(c2w[3]);
      view_proj = vg::camera::Camera::look_at_perspective(
                      eye, eye + glm::vec3(c2w[2]), -glm::vec3(c2w[1]), vfov,
                      aspect, 0.05f, far_plane)
                      .view_proj();
    } else {
      view_proj =
          vg::camera::Camera::look_at_perspective(
              view.eye(), view.target, view.up, vfov, aspect, 0.05f, far_plane)
              .view_proj();
    }
    shared_view.publish(view_proj);  // for the fuse thread to mesh

    if (overlay) {
      imgui_glfw->new_frame();
      overlay->new_frame();
      ImGui::SetNextWindowPos(ImVec2(16.0f, 16.0f), ImGuiCond_FirstUseEver);
      ImGui::SetNextWindowSize(ImVec2(400.0f, 360.0f), ImGuiCond_FirstUseEver);
      vg::FrameMetrics metrics = profiler.metrics();
      metrics.sections.insert(metrics.sections.end(),
                              fuse_stages_snapshot.begin(),
                              fuse_stages_snapshot.end());
      vg::ui::draw_metrics_panel(metrics, "Performance");
      ImGui::SetNextWindowPos(ImVec2(16.0f, 388.0f), ImGuiCond_FirstUseEver);
      ImGui::SetNextWindowSize(ImVec2(400.0f, 260.0f), ImGuiCond_FirstUseEver);
      fuse_viewer::draw_rig_panel(panel, serials, kSilenceLimit.count());
      ImGui::SetNextWindowPos(ImVec2(432.0f, 16.0f), ImGuiCond_FirstUseEver);
      if (ImGui::Begin("View")) {
        int mode = static_cast<int>(shading);
        ImGui::RadioButton("unlit", &mode, static_cast<int>(Shading::Unlit));
        ImGui::SameLine();
        ImGui::RadioButton("lit", &mode, static_cast<int>(Shading::Lit));
        ImGui::SameLine();
        ImGui::RadioButton("normals", &mode,
                           static_cast<int>(Shading::Normals));
        shading = static_cast<Shading>(mode);
        bool texture = texture_on.load();
        if (ImGui::Checkbox("texture from the cameras (next remesh)",
                            &texture)) {
          texture_on.store(texture);
        }
        float occlusion_cm = 100.0f * occlusion.load();
        if (ImGui::SliderFloat("texture occlusion (cm)", &occlusion_cm, 0.5f,
                               20.0f, "%.1f")) {
          occlusion.store(occlusion_cm / 100.0f);
        }
        ImGui::Checkbox("colour by camera", &show_sources);
        if (show_sources) {
          for (std::size_t c = 0; c < cameras; ++c) {
            const auto& rgb = kCameraColours[c % kCameraColours.size()];
            ImGui::ColorButton(
                serials[c].c_str(),
                ImVec4(rgb[0] / 255.0f, rgb[1] / 255.0f, rgb[2] / 255.0f, 1.0f),
                ImGuiColorEditFlags_NoTooltip, ImVec2(12, 12));
            ImGui::SameLine();
            ImGui::Text("camera %zu  %s", c, serials[c].c_str());
          }
          ImGui::TextDisabled("fused colour: textured by no camera");
        }
        ImGui::Separator();
        bool dynamic = dynamic_on.load();
        if (ImGui::Checkbox("fusion resets on change (dynamic)", &dynamic)) {
          dynamic_on.store(dynamic);
        }
        float weight = max_weight.load();
        if (ImGui::SliderFloat("max weight", &weight, 1.0f, 50.0f, "%.0f")) {
          max_weight.store(weight);
        }
        ImGui::Separator();
        if (ImGui::Button("orbit view")) {
          view = home;
          from_camera.reset();
        }
        for (std::size_t i = 0; i < cameras; ++i) {
          ImGui::SameLine();
          char label[32];
          std::snprintf(label, sizeof(label), "camera %zu", i);
          if (ImGui::Button(label)) from_camera = i;
        }
        if (fusing_done.load()) {
          ImGui::Text(fuse_failed.load()
                          ? "fusion stopped on an error (see the terminal)"
                          : "fusion stopped");
        }
      }
      ImGui::End();
    }

    fuse_viewer::record_view_pass(
        profiler, render_frame.cmd, *render_frame.target,
        [&](VkCommandBuffer cmd) {
          if (!live_view.valid() || live_view.empty()) return;
          // recon's buffers, named rather than copied, as in fuse_viewer.
          const vgp::HybridMeshDraw draw{fuse_viewer::to_live_mesh(live_view)};
          vgp::HybridMeshFrame hybrid_frame;
          hybrid_frame.extent = extent;
          hybrid_frame.view_proj = view_proj;
          hybrid_frame.light_dir = -view.up + 0.5f * view.right - view.forward;
          hybrid_frame.flags = shading_flags(shading);
          hybrid_frame.atlas = atlas.use(render_frame.number);
          hybrid_frame.draws = &draw;
          hybrid_frame.draw_count = 1;
          pipeline.submit(cmd, hybrid_frame);
        },
        overlay ? &*overlay : nullptr);

    const vkc::Status present = app.end_frame(render_frame);
    if (!present.ok() && !win::swapchain_stale(present)) {
      std::fprintf(stderr, "end_frame: %s\n", present.message().c_str());
      exit_code = 1;
      break;
    }
    // Submitted, a stale present's frame too, so the profiler publishes its
    // figures as the slot recurs.
    if (copied) copy_times.copied(render_frame.slot);
    if (drawn % 120 == 0) {
      fuse_viewer::print_rig_progress(drawn, panel, atlas_copies);
    }
    ++drawn;
  }

  // Joined here rather than by fuse_guard, so fuse_failed is final when read.
  quit.store(true);
  fuse_thread.join();
  if (const vkc::Status idle = app.wait_idle(); !idle) {
    std::fprintf(stderr, "wait_idle: %s\n", idle.message().c_str());
    exit_code = 1;
  }
  // The fuse thread said why on stderr; the exit code is for a script.
  if (fuse_failed.load()) exit_code = 1;
  // The fuse thread has joined, so what it recorded is final.
  fuse_viewer::print_rig_summary(drawn, panel, atlas_copies,
                                 set_rate.per_second(),
                                 mesh_updates.per_second());
  vr_example::print_rig_stats(array, nullptr);
  std::fputs(copy_times.report().c_str(), stdout);
  const std::string title =
      log.dropped() == 0
          ? std::string("per set")
          : "per set (the newest " + std::to_string(log.sets().size()) +
                " of " + std::to_string(log.sets().size() + log.dropped()) +
                ")";
  std::fputs(log.table(title).c_str(), stdout);
  return exit_code;
}

}  // namespace

int run_main(const Options& opt) {
  vgrid_example::catch_interrupts();
  if (glfwInit() != GLFW_TRUE) {
    std::fprintf(stderr, "glfwInit failed\n");
    return 1;
  }
  glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
  GLFWwindow* window = glfwCreateWindow(opt.window.width, opt.window.height,
                                        "rig_viewer", nullptr, nullptr);
  if (window == nullptr) {
    std::fprintf(stderr, "glfwCreateWindow failed\n");
    glfwTerminate();
    return 1;
  }
  const int rc = run(window, opt);
  glfwDestroyWindow(window);
  glfwTerminate();
  return rc;
}

int main(int argc, char** argv) {
  return vgrid_example::configured_main(
      argc, argv, vgrid_example::config::Application::RigViewer, "rig_viewer",
      run_main);
}
