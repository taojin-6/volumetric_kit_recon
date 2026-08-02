// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// fuse_viewer: the live recon -> gfx interop demo. Opens a window and fuses a
// posed Replica RGB-D sequence into a sparse TSDF+colour volume frame by frame
// (volumetric_kit_recon), periodically re-extracting a marching-cubes mesh,
// handing it across the interop seam (a host mesh), and drawing the growing,
// coloured reconstruction each frame through volumetric_kit_gfx's
// HybridMeshPipeline with an orbiting camera -- the nvblox FuserVisualizer
// analogue. Two devices (recon fuses on its own, gfx renders on its own); the
// mesh bridges them on the host.
//
// Each re-meshed frame is projectively textured with its own keyframe (the
// texture tier): the triangles that keyframe saw unoccluded render at full
// sensor resolution, the rest fall back to fused voxel colour. --no-texture
// disables it (an A/B against the pure vertex-colour path).
//
// Two Dear ImGui panels (gfx's ui tier) show where the time and the memory go:
// Performance (the renderer's fps + GPU spans, with recon's per-frame fuse
// stages appended) and Reconstruction (mesh/volume counters + recon's device
// memory). --no-overlay turns both off.
//
//   fuse_viewer <scene_dir> [--voxel 0.02] [--max-frames N] [--fuse-per-tick K]
//               [--remesh-every N] [--width 1280] [--height 720] [--unlit]
//               [--no-texture] [--preload] [--no-overlay]

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <glm/glm.hpp>

#include "dataset.hpp"
#include "example_camera.hpp"  // vr_example::make_depth_camera
#include "image_io.hpp"        // vr_example::pack_color_rgba8
#include "recon_gfx_bridge.hpp"
#include "stage_metrics.hpp"  // fuse_viewer::StageTimes / StageScope

#include "volumetric_kit/recon/core/allocator.hpp"
#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/instance.hpp"
#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/mesh/marching_cubes.hpp"
#include "volumetric_kit/recon/mesh/mesh.hpp"
#include "volumetric_kit/recon/texture/projective_texturer.hpp"
#include "volumetric_kit/recon/tsdf/tsdf_integrator.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"
#include "volumetric_kit/recon/volume/voxel_grid.hpp"
#include "volumetric_kit/recon/volume/voxel_hash_map.hpp"

#include "volumetric_kit/gfx/app/windowed_app.hpp"
#include "volumetric_kit/gfx/camera/camera.hpp"
#include "volumetric_kit/gfx/core/descriptor.hpp"
#include "volumetric_kit/gfx/core/frame_metrics.hpp"
#include "volumetric_kit/gfx/core/profiler.hpp"
#include "volumetric_kit/gfx/core/render_target.hpp"
#include "volumetric_kit/gfx/core/result.hpp"
#include "volumetric_kit/gfx/core/sampler.hpp"
#include "volumetric_kit/gfx/core/texture_upload.hpp"
#include "volumetric_kit/gfx/core/vulkan.hpp"
#include "volumetric_kit/gfx/pipelines/gpu_mesh.hpp"
#include "volumetric_kit/gfx/pipelines/hybrid_mesh_pipeline.hpp"
#include "volumetric_kit/gfx/ui/imgui_overlay.hpp"
#include "volumetric_kit/gfx/ui/metrics_panel.hpp"
#include "volumetric_kit/gfx/windowing/frame_loop.hpp"
#include "volumetric_kit/gfx/windowing/swapchain.hpp"

namespace vr = volumetric_kit::recon;
namespace vol = volumetric_kit::recon::volume;
namespace rtsdf = volumetric_kit::recon::tsdf;
namespace rmesh = volumetric_kit::recon::mesh;
namespace rtex = volumetric_kit::recon::texture;
namespace vg = volumetric_kit::gfx;
namespace vgp = volumetric_kit::gfx::pipelines;
namespace win = volumetric_kit::gfx::windowing;

namespace {

struct Options {
  std::string scene_dir;
  std::string cam_params;
  float voxel = 0.02f;
  float trunc = 0.08f;
  float max_depth = 8.0f;
  int max_frames = 400;
  int fuse_per_tick = 1;
  int remesh_every = 1;  // re-extract + re-upload every N fused frames
  int width = 1280;
  int height = 720;
  bool lit = true;
  bool texture = true;   // project each keyframe onto the growing mesh (uv0)
  bool preload = false;  // decode every frame up front (RAM for decode time)
  bool overlay = true;   // Dear ImGui performance + reconstruction panels
};

bool parse_args(int argc, char** argv, Options& o) {
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto v = [&]() -> const char* {
      return (i + 1 < argc) ? argv[++i] : nullptr;
    };
    if (a == "--cam-params") {
      const char* x = v();
      if (!x) return false;
      o.cam_params = x;
    } else if (a == "--voxel") {
      const char* x = v();
      if (!x) return false;
      o.voxel = std::strtof(x, nullptr);
    } else if (a == "--max-depth") {
      const char* x = v();
      if (!x) return false;
      o.max_depth = std::strtof(x, nullptr);
    } else if (a == "--max-frames") {
      const char* x = v();
      if (!x) return false;
      o.max_frames = std::atoi(x);
    } else if (a == "--fuse-per-tick") {
      const char* x = v();
      if (!x) return false;
      o.fuse_per_tick = std::max(1, std::atoi(x));
    } else if (a == "--remesh-every") {
      const char* x = v();
      if (!x) return false;
      o.remesh_every = std::max(1, std::atoi(x));
    } else if (a == "--width") {
      const char* x = v();
      if (!x) return false;
      o.width = std::atoi(x);
    } else if (a == "--height") {
      const char* x = v();
      if (!x) return false;
      o.height = std::atoi(x);
    } else if (a == "--unlit") {
      o.lit = false;
    } else if (a == "--no-texture") {
      o.texture = false;
    } else if (a == "--preload") {
      o.preload = true;
    } else if (a == "--no-overlay") {
      o.overlay = false;
    } else if (!a.empty() && a[0] == '-') {
      std::fprintf(stderr, "unknown flag %s\n", a.c_str());
      return false;
    } else if (o.scene_dir.empty()) {
      o.scene_dir = a;
    } else {
      std::fprintf(stderr, "unexpected arg %s\n", a.c_str());
      return false;
    }
  }
  if (o.scene_dir.empty()) {
    std::fprintf(stderr,
                 "usage: fuse_viewer <scene_dir> [--voxel m] [--max-frames n] "
                 "[--fuse-per-tick k] [--remesh-every n] [--unlit] "
                 "[--no-texture] [--preload] [--no-overlay]\n");
    return false;
  }
  // strtof parses "nan"/"inf" without error, and a non-finite knob slips the
  // downstream guards (NaN compares false to every bound) to reach the grid
  // params and the GPU -- a silent, degenerate reconstruction. Reject up front.
  if (!std::isfinite(o.voxel) || o.voxel <= 0.0f) {
    std::fprintf(stderr, "--voxel must be finite and > 0\n");
    return false;
  }
  if (!std::isfinite(o.max_depth) || o.max_depth <= 0.0f) {
    std::fprintf(stderr, "--max-depth must be finite and > 0\n");
    return false;
  }
  if (o.cam_params.empty()) o.cam_params = o.scene_dir + "/../cam_params.json";
  return true;
}

VkExtent2D window_extent(GLFWwindow* window) {
  int width = 0, height = 0;
  glfwGetFramebufferSize(window, &width, &height);
  return {static_cast<std::uint32_t>(std::max(1, width)),
          static_cast<std::uint32_t>(std::max(1, height))};
}

// Signals a fuse thread to quit and joins it on scope exit, so an exception
// unwinding the render loop cannot destroy a still-joinable std::thread (which
// would call std::terminate). Declared right after the thread so it runs first
// at scope exit -- while the recon resources the thread borrows are still live.
struct QuitJoin {
  std::thread& thread;
  std::atomic<bool>& quit;
  ~QuitJoin() {
    quit.store(true);
    if (thread.joinable()) thread.join();
  }
};

// Shuts the ImGui GLFW platform backend down at scope exit. Declared *after*
// the overlay so it runs first: ImGui_ImplGlfw_Shutdown touches the ImGui
// context the overlay owns, so it must not outlive it.
struct ImGuiGlfwShutdown {
  bool active;
  ~ImGuiGlfwShutdown() {
    if (active) ImGui_ImplGlfw_Shutdown();
  }
};

// Detaches the profiler from the app's frame loop at scope exit, before the
// profiler itself is destroyed -- the loop holds a bare pointer to it.
struct ProfilerDetach {
  vg::app::WindowedApp& app;
  ~ProfilerDetach() { app.set_profiler(nullptr); }
};

// What the reconstruction side is holding, shown beside the renderer's frame
// metrics. gfx's FrameMetrics carries one memory pair (its own device), so
// recon's device memory + the volume/mesh counters live in their own panel
// rather than being squeezed into that contract.
struct ReconstructionPanel {
  std::size_t fused_frames = 0;
  std::size_t total_frames = 0;
  std::size_t vertices = 0;
  std::size_t triangles = 0;
  std::uint64_t mesh_version = 0;
  std::int32_t map_buckets = 0;
  std::int32_t map_blocks = 0;  // heap *capacity* (bucket_size * num_buckets)
  double fuse_ms = 0.0;
  std::uint64_t preloaded_bytes = 0;
  vr::MemoryStats recon_memory;
  rmesh::ExtractTimings extract;
};

// Bytes -> MiB, for display only. Mebibytes (1024^2), matching the unit gfx's
// draw_metrics_panel prints beside this one.
double to_mebibytes(std::uint64_t bytes) {
  return static_cast<double>(bytes) / (1024.0 * 1024.0);
}

// Build the reconstruction panel into the ImGui frame the caller is driving
// (it calls neither NewFrame nor Render), mirroring gfx's draw_metrics_panel.
void draw_reconstruction_panel(const ReconstructionPanel& panel) {
  if (!ImGui::Begin("Reconstruction")) {
    ImGui::End();
    return;
  }
  ImGui::Text("fused    %zu / %zu frames", panel.fused_frames,
              panel.total_frames);
  ImGui::Text("fuse     %.2f ms/frame", panel.fuse_ms);
  ImGui::Separator();
  ImGui::Text("mesh v%llu",
              static_cast<unsigned long long>(panel.mesh_version));
  ImGui::Text("  %zu vertices / %zu triangles", panel.vertices,
              panel.triangles);
  // Capacity, not occupancy: num_blocks is bucket_size * num_buckets, the size
  // of the block heap every per-voxel attribute array is dimensioned by. It is
  // what a resize doubles, so it is the memory story; how *full* the map is
  // needs the hash map's diagnostics scan (a device readback, too costly here).
  ImGui::Text("map  %d buckets / %d block capacity", panel.map_buckets,
              panel.map_blocks);
  ImGui::Separator();
  // recon's device memory: its own VMA allocator's share of the device. On a
  // shared/adopted device each library allocates separately, so this is recon's
  // footprint, not the process total.
  for (std::uint32_t heap = 0; heap < panel.recon_memory.heap_count; ++heap) {
    const vr::HeapStats& stats = panel.recon_memory.heaps[heap];
    if (stats.budget_bytes == 0 && stats.usage_bytes == 0) continue;
    ImGui::Text("recon heap %u  %.1f / %.1f MiB", heap,
                to_mebibytes(stats.usage_bytes),
                to_mebibytes(stats.budget_bytes));
  }
  if (panel.preloaded_bytes != 0) {
    ImGui::Text("frame cache   %.0f MiB (host)",
                to_mebibytes(panel.preloaded_bytes));
  }
  ImGui::End();
}

// One live atlas version: the keyframe image the current mesh's uv0 index into,
// plus the descriptor set that binds it to the hybrid pipeline. Each newly
// textured mesh builds a fresh bundle carrying its OWN pool (gfx frees a set
// only with its pool, never individually), so the whole bundle self-frees when
// its last owner drops it. The render thread keeps the in-flight versions alive
// across the frame ring via shared_ptr, so an atlas a still-pending frame bound
// outlives its replacement -- the "per-slot atlas ringing" a live-updated
// texture needs. `set` is declared after `pool` only for tidy teardown; the set
// is a non-owning handle, so the order is not load-bearing.
struct AtlasVersion {
  vg::Texture tex;
  vg::DescriptorPool pool;
  vg::DescriptorSet set;
};

// Owns the WindowedApp (and the VkSurfaceKHR built from `window`) plus every
// device resource, so they all destruct BEFORE main destroys the window -- the
// gfx run()/main() split. Destroying a surface/swapchain after its window is a
// use-after-free, notably on MoltenVK where the surface wraps the window's
// CAMetalLayer.
int run(GLFWwindow* window, const Options& opt) {
  std::uint32_t ext_count = 0;
  const char** glfw_exts = glfwGetRequiredInstanceExtensions(&ext_count);
  vg::app::WindowedAppConfig config;
  config.app_name = "fuse_viewer";
  config.instance_extensions.assign(glfw_exts, glfw_exts + ext_count);
  config.swapchain.extent = window_extent(window);
  config.swapchain.depth_format = VK_FORMAT_D32_SFLOAT;
  config.frames_in_flight = 2;
  auto app_r = vg::app::WindowedApp::create(
      config, [window](VkInstance inst) -> vg::Result<VkSurfaceKHR> {
        VkSurfaceKHR surface = VK_NULL_HANDLE;
        const VkResult r =
            glfwCreateWindowSurface(inst, window, nullptr, &surface);
        if (r != VK_SUCCESS)
          return vg::Status::error(r, "glfwCreateWindowSurface failed");
        return surface;
      });
  if (!app_r.ok()) {
    std::fprintf(stderr, "WindowedApp: %s\n", app_r.status().message().c_str());
    return 1;
  }
  vg::app::WindowedApp app = std::move(app_r).value();

  // --- recon pipeline (its own device), fused incrementally -----------------
  auto recon_instance = vr::Instance::create({});
  if (!recon_instance) {
    std::fprintf(stderr, "recon instance: %s\n",
                 recon_instance.status().message().c_str());
    return 1;
  }
  auto rgpu = recon_instance.value().select_physical_device();
  if (!rgpu) {
    std::fprintf(stderr, "recon gpu: %s\n", rgpu.status().message().c_str());
    return 1;
  }
  auto recon_device_result =
      vr::Device::create(recon_instance.value().handle(), rgpu.value(), {});
  if (!recon_device_result) {
    std::fprintf(stderr, "recon device: %s\n",
                 recon_device_result.status().message().c_str());
    return 1;
  }
  auto recon_allocator_result = vr::Allocator::create(
      recon_instance.value().handle(), recon_device_result.value());
  if (!recon_allocator_result) {
    std::fprintf(stderr, "recon allocator: %s\n",
                 recon_allocator_result.status().message().c_str());
    return 1;
  }
  vr::Device& rdevice = recon_device_result.value();
  vr::Allocator& rallocator = recon_allocator_result.value();

  auto dataset_result =
      vr_example::ReplicaDataset::open(opt.scene_dir, opt.cam_params);
  if (!dataset_result) {
    std::fprintf(stderr, "dataset: %s\n",
                 dataset_result.status().message().c_str());
    return 1;
  }
  vr_example::ReplicaDataset dataset = std::move(dataset_result).value();
  const vr_example::CameraModel& cam = dataset.camera();

  vol::VoxelGridParams grid{};
  grid.voxel_size = opt.voxel;
  grid.block_size = 8;
  grid.voxels_per_block = 512;
  grid.trunc_dist = opt.trunc;
  grid.bucket_size = 8;
  grid.num_buckets = 16384;
  grid.num_blocks = grid.bucket_size * grid.num_buckets;
  grid.max_chain = 128;
  const vol::AttributeSpec attrs[] = {{"tsdf", sizeof(float)},
                                      {"weight", sizeof(float)},
                                      {"color", sizeof(std::uint32_t)}};
  auto grid_result =
      vol::VoxelBlockGrid::create(rdevice, rallocator, grid, attrs, 3);
  if (!grid_result) {
    std::fprintf(stderr, "grid: %s\n", grid_result.status().message().c_str());
    return 1;
  }
  vol::VoxelBlockGrid volume = std::move(grid_result).value();
  auto integrator_result = rtsdf::TsdfIntegrator::create(rdevice, rallocator);
  if (!integrator_result) {
    std::fprintf(stderr, "integrator: %s\n",
                 integrator_result.status().message().c_str());
    return 1;
  }
  rtsdf::TsdfIntegrator integrator = std::move(integrator_result).value();
  auto extractor_result = rmesh::MarchingCubes::create(rdevice, rallocator);
  if (!extractor_result) {
    std::fprintf(stderr, "marching cubes: %s\n",
                 extractor_result.status().message().c_str());
    return 1;
  }
  rmesh::MarchingCubes extractor = std::move(extractor_result).value();
  // Projective texturer (recon device; used, like the integrator/extractor,
  // only on the fuse thread below). Cheap to keep even when --no-texture, but
  // build it only when texturing so the disabled path stays a pure A/B.
  std::optional<rtex::ProjectiveTexturer> texturer;
  if (opt.texture) {
    auto texture_result = rtex::ProjectiveTexturer::create(rdevice, rallocator);
    if (!texture_result) {
      std::fprintf(stderr, "texturer: %s\n",
                   texture_result.status().message().c_str());
      return 1;
    }
    texturer = std::move(texture_result).value();
  }

  const std::size_t frame_count = std::min<std::size_t>(
      dataset.frame_count(),
      static_cast<std::size_t>(std::max(0, opt.max_frames)));
  const float vfov = 2.0f * std::atan(static_cast<float>(cam.height) /
                                      (2.0f * std::max(1.0f, cam.fy)));

  // --- gfx: pipeline + 1x1 white atlas set ----------------------------------
  auto pipeline_result = vgp::HybridMeshPipeline::create(
      app.device().handle(), app.swapchain().layout());
  if (!pipeline_result.ok()) {
    std::fprintf(stderr, "pipeline: %s\n",
                 pipeline_result.status().message().c_str());
    return 1;
  }
  vgp::HybridMeshPipeline pipeline = std::move(pipeline_result).value();

  // --- gfx profiler: the renderer's own per-frame CPU/GPU timings ------------
  // The render side gets real GPU spans (this device reports 64
  // timestampValidBits through MoltenVK) plus fps and whole-frame CPU time;
  // attaching it to the app makes the frame loop drive begin_frame/end_frame.
  // recon's stages are measured separately -- see stage_metrics.hpp for why
  // they are wall-clock CPU rows.
  vg::ProfilerConfig profiler_config;
  profiler_config.frames_in_flight = config.frames_in_flight;
  auto profiler_result = vg::Profiler::create(app.device(), profiler_config);
  if (!profiler_result.ok()) {
    std::fprintf(stderr, "profiler: %s\n",
                 profiler_result.status().message().c_str());
    return 1;
  }
  vg::Profiler profiler = std::move(profiler_result).value();
  // Fill the snapshot's memory pair from the *renderer's* allocator, so the
  // Performance panel reports the mesh/atlas/swapchain footprint on gfx's
  // device; recon's own allocator is on a second device and is reported
  // separately by the Reconstruction panel below. `app` outlives the profiler
  // (declared before it), which set_memory_source requires.
  profiler.set_memory_source(&app.allocator());
  app.set_profiler(&profiler);
  const ProfilerDetach profiler_guard{app};

  // --- gfx ui: the Dear ImGui performance overlay ----------------------------
  // Optional: --no-overlay skips both the context and the platform backend, so
  // the disabled path costs nothing and stays a clean A/B. The pipeline bakes
  // the swapchain layout, which survives a resize, so the overlay is built
  // once.
  std::optional<vg::ui::ImGuiOverlay> overlay;
  if (opt.overlay) {
    vg::ui::ImGuiOverlayConfig overlay_config;
    overlay_config.layout = app.swapchain().layout();
    overlay_config.min_image_count = app.swapchain().image_count();
    overlay_config.image_count = app.swapchain().image_count();
    auto overlay_result = vg::ui::ImGuiOverlay::create(
        app.device(), app.instance().handle(), overlay_config);
    if (!overlay_result.ok()) {
      std::fprintf(stderr, "overlay: %s\n",
                   overlay_result.status().message().c_str());
      return 1;
    }
    overlay = std::move(overlay_result).value();
    // The platform (GLFW) half of ImGui is the example's to own -- gfx's ui
    // tier deliberately wraps only the Vulkan renderer backend so it stays
    // windowing-free. Shut it down before the overlay's context dies, below.
    ImGui::SetCurrentContext(overlay->context());
    if (!ImGui_ImplGlfw_InitForVulkan(window, true)) {
      std::fprintf(stderr, "ImGui_ImplGlfw_InitForVulkan failed\n");
      return 1;
    }
  }
  // Runs before `overlay` is destroyed (reverse declaration order), which the
  // ImGui backend requires: its Shutdown touches the context the overlay owns.
  const ImGuiGlfwShutdown imgui_glfw_guard{opt.overlay};

  // One sampler shared by every atlas version (immutable; outlives them all).
  auto sampler_result = vg::Sampler::create(app.device().handle());
  if (!sampler_result.ok()) {
    std::fprintf(stderr, "sampler: %s\n",
                 sampler_result.status().message().c_str());
    return 1;
  }
  vg::Sampler sampler = std::move(sampler_result).value();

  // Build one atlas bundle (texture + its own pool + a combined-image-sampler
  // set bound to `sampler`) from RGBA8 pixels. Returns nullptr on failure so a
  // transient upload error keeps the previous atlas rather than crashing.
  auto build_atlas =
      [&](const std::uint8_t* pixels, std::uint32_t width,
          std::uint32_t height) -> std::shared_ptr<AtlasVersion> {
    vg::ImageUploadDesc upload_desc;
    upload_desc.extent = {width, height};
    upload_desc.format = VK_FORMAT_R8G8B8A8_UNORM;
    upload_desc.pixels = pixels;
    upload_desc.size = static_cast<std::size_t>(width) * height * 4;
    auto texture_result =
        vg::upload_texture(app.device(), app.allocator(), upload_desc);
    if (!texture_result.ok()) {
      std::fprintf(stderr, "atlas upload: %s\n",
                   texture_result.status().message().c_str());
      return nullptr;
    }
    const VkDescriptorPoolSize pool_size{
        VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1};
    auto pool_result =
        vg::DescriptorPool::create(app.device().handle(), &pool_size, 1, 1);
    if (!pool_result.ok()) {
      std::fprintf(stderr, "atlas pool: %s\n",
                   pool_result.status().message().c_str());
      return nullptr;
    }
    vg::DescriptorPool atlas_pool = std::move(pool_result).value();
    auto set_result = atlas_pool.allocate(pipeline.descriptor_set_layout(0));
    if (!set_result.ok()) {
      std::fprintf(stderr, "atlas set: %s\n",
                   set_result.status().message().c_str());
      return nullptr;
    }
    auto atlas_version = std::make_shared<AtlasVersion>();
    atlas_version->tex = std::move(texture_result).value();
    atlas_version->pool = std::move(atlas_pool);
    atlas_version->set = std::move(set_result).value();
    atlas_version->set.write_combined_image_sampler(
        0, atlas_version->tex.view(), sampler.handle(),
        VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    return atlas_version;
  };

  // The initial + fallback atlas: a 1x1 white texel, bound whenever the current
  // mesh carries no keyframe (texturing off, or nothing in line of sight), so
  // the hybrid shader cleanly takes the per-vertex-colour path (uv0 sentinel).
  const std::uint8_t white[4] = {255, 255, 255, 255};
  std::shared_ptr<AtlasVersion> white_atlas = build_atlas(white, 1, 1);
  if (!white_atlas) return 1;
  std::shared_ptr<AtlasVersion> current_atlas = white_atlas;

  // --- Background fuse thread: load + decode + fuse + extract off the render
  // thread (per-frame JPEG/PNG decode is CPU-heavy and would otherwise gate the
  // loop, starving both GPUs). It publishes the newest coloured mesh + the
  // trajectory; the render thread only uploads + draws, so the window stays at
  // full frame rate and the two devices' queues run concurrently. recon's
  // device is used solely on this thread.
  // -------------------------------------------
  std::mutex share_mtx;
  std::optional<vg::assets::Mesh> pending_mesh;  // newest mesh awaiting upload
  std::vector<std::uint8_t> pending_atlas;  // its keyframe RGBA8 (empty = none)
  std::uint64_t published_version = 0;
  std::vector<glm::mat4> shared_poses;  // trajectory, grows as frames fuse
  std::atomic<std::size_t> fused_count{0};
  std::atomic<bool> fusing_done{false};
  std::atomic<bool> quit{false};
  // Newest fused frame's stage breakdown + the volume's footprint, published
  // for the overlay under share_mtx alongside the mesh.
  std::vector<vg::FrameMetrics::Section> shared_fuse_stages;
  double shared_fuse_ms = 0.0;
  vr::MemoryStats shared_recon_memory;
  std::int32_t shared_map_buckets = 0;
  std::int32_t shared_map_blocks = 0;
  std::uint64_t shared_preloaded_bytes = 0;
  rmesh::ExtractTimings shared_extract;

  std::thread fuse_thread([&]() {
    // Any throw escaping this thread function (e.g. a decode/allocation
    // bad_alloc) would call std::terminate; contain it so shutdown stays clean.
    try {
      // Scratch for this thread only; copied under share_mtx once per frame.
      fuse_viewer::StageTimes fuse_stages;
      // Held across frames so the panel keeps showing the newest remesh's
      // sizes between remeshes, rather than blanking to zero.
      rmesh::ExtractTimings extract_stats;
      // Texture `rm` with one keyframe, then publish it plus that keyframe's
      // colour image as the atlas its uv0 index into. On any texturing failure
      // -- or when --no-texture -- the atlas stays empty and the render thread
      // binds the white dummy (every triangle falls back to fused voxel
      // colour). uv0 must be filled BEFORE to_gfx_mesh, which copies it across.
      auto publish = [&](const rmesh::DeviceMesh& device_mesh,
                         const float* depth,
                         const vol::DepthCameraParams& depth_camera,
                         const std::vector<std::uint32_t>& color) {
        std::vector<std::uint8_t> atlas;
        if (texturer && depth != nullptr && !device_mesh.empty()) {
          vr::Status texture_status;
          {
            // Textures the extractor's buffers in place -- no upload, no
            // readback; the geometry has not left the device since it was
            // meshed.
            fuse_viewer::StageScope scope(fuse_stages, "texture");
            texture_status =
                texturer->texture(device_mesh, depth, depth_camera);
          }
          if (texture_status.ok()) {
            // Its own row, not folded into "texture": repacking a full sensor
            // frame to RGBA8 is host work of the same order as the texturing
            // dispatch, so charging it to the GPU pass would misattribute it.
            fuse_viewer::StageScope scope(fuse_stages, "atlas pack");
            atlas = vr_example::pack_color_rgba8(color);
          } else {
            std::fprintf(stderr, "fuse_viewer: texture: %s\n",
                         texture_status.message().c_str());
          }
        }
        // The single host copy of the whole pipeline, taken after texturing
        // has written uv0 in place -- so the mesh crosses to the host once
        // instead of the three times the host-mesh path cost.
        rmesh::Mesh recon_mesh;
        {
          fuse_viewer::StageScope scope(fuse_stages, "download");
          auto downloaded = extractor.download(device_mesh);
          if (!downloaded) {
            std::fprintf(stderr, "fuse_viewer: download: %s\n",
                         downloaded.status().message().c_str());
            return;
          }
          recon_mesh = std::move(downloaded).value();
        }
        vg::assets::Mesh gfx_mesh;
        {
          fuse_viewer::StageScope scope(fuse_stages, "to_gfx_mesh");
          gfx_mesh = fuse_viewer::to_gfx_mesh(recon_mesh);
        }
        std::lock_guard<std::mutex> lock(share_mtx);
        pending_mesh = std::move(gfx_mesh);
        pending_atlas = std::move(atlas);
        ++published_version;
      };
      // Decode the whole sequence up front when asked, so the loop below is
      // gated by fusion rather than by JPEG/PNG decode (~75% of a streaming
      // loop). Done here, on the fuse thread, so the window is already up and
      // responsive while it works.
      if (opt.preload) {
        std::printf(
            "preloading %.0f MB...\n",
            static_cast<double>(dataset.preload_bytes_projected(frame_count)) /
                (1024 * 1024));
        // `quit` stops the decode at the next frame boundary, so closing the
        // window mid-preload does not leave the join at shutdown waiting out
        // the whole sequence -- the same reason the final extract below is
        // skipped once the user has quit.
        auto cached_frames = dataset.preload(frame_count, 1, &quit);
        if (cached_frames) {
          std::printf(
              "preloaded %zu frames (%.0f MB)\n", cached_frames.value(),
              static_cast<double>(dataset.preloaded_bytes()) / (1024 * 1024));
        } else {
          // frame() still decodes on demand, so a failed preload costs speed,
          // not the run.
          std::fprintf(stderr, "fuse_viewer: preload: %s (streaming instead)\n",
                       cached_frames.status().message().c_str());
        }
        // Sampled once, here: preloaded_bytes() walks the cache, and the cache
        // is only immutable now that the decode has finished.
        const std::uint64_t cache_bytes = dataset.preloaded_bytes();
        std::lock_guard<std::mutex> lock(share_mtx);
        shared_preloaded_bytes = cache_bytes;
      }
      // The newest fused frame, retained so the final extract (after the loop)
      // textures with the last keyframe rather than losing its texture.
      std::optional<vr_example::FrameView> last_frame;
      for (std::size_t i = 0; i < frame_count && !quit.load(); ++i) {
        // Stage spans are per fused frame: the overlay shows the newest frame's
        // breakdown, not a running total. Seed every row this frame *could*
        // fill, in display order, so the remesh-only stages report 0 between
        // remeshes instead of dropping out and shuffling the table.
        fuse_stages.clear();
        for (const char* stage :
             {"frame", "allocate", "integrate", "extract", "  ..compact",
              "  ..neighbour lut", "  ..inputs", "  ..arena alloc",
              "  ..descriptors", "  ..dispatch", "  ..readback", "texture",
              "download", "atlas pack", "to_gfx_mesh"}) {
          fuse_stages.seed(stage);
        }
        // A preload cache hit, else a disk read + JPEG/PNG decode (the CPU
        // cost the preload exists to hoist out of this loop). Timed either way,
        // so --preload's effect is visible as this row collapsing to ~0.
        auto frame_result = [&]() {
          fuse_viewer::StageScope scope(fuse_stages, "frame");
          return dataset.frame(i);
        }();
        if (!frame_result) break;
        vr_example::FrameView view = std::move(frame_result).value();
        const vr_example::RgbdFrame& frame = *view;
        {
          std::lock_guard<std::mutex> lock(share_mtx);
          shared_poses.push_back(frame.cam_to_world);
        }
        const vol::DepthCameraParams depth_camera =
            vr_example::make_depth_camera(cam, frame.cam_to_world,
                                          opt.max_depth);
        rtsdf::ColorCameraParams color_camera{};
        color_camera.fx = cam.fx;
        color_camera.fy = cam.fy;
        color_camera.cx = cam.cx;
        color_camera.cy = cam.cy;
        color_camera.width = cam.width;
        color_camera.height = cam.height;
        color_camera.cam_to_world = frame.cam_to_world;
        const rtsdf::ColorFrame color_frame{frame.color.data(), color_camera};

        // Grow the map to fit this frame's surface band; surface any hard
        // failure instead of silently integrating a partially-allocated frame.
        bool allocated = false;
        {
          // One row for the whole retry: a frame that overflows the map and
          // resizes reports allocate + resize together, which is what that
          // frame actually cost.
          fuse_viewer::StageScope scope(fuse_stages, "allocate");
          for (int attempt = 0; attempt < 5; ++attempt) {
            auto failed = volume.map().allocate_from_depth(frame.depth.data(),
                                                           depth_camera);
            if (!failed) {
              std::fprintf(stderr, "fuse_viewer: allocate (frame %zu): %s\n", i,
                           failed.status().message().c_str());
              break;
            }
            if (failed.value() == 0) {
              allocated = true;
              break;
            }
            const vr::Status rs = volume.resize(volume.grid().num_buckets * 2);
            if (!rs.ok()) {
              std::fprintf(stderr, "fuse_viewer: resize (frame %zu): %s\n", i,
                           rs.message().c_str());
              break;
            }
          }
        }
        if (!allocated) {
          std::fprintf(
              stderr, "fuse_viewer: map overflow at frame %zu; stopping fuse\n",
              i);
          break;
        }
        vr::Status integrate_status;
        {
          fuse_viewer::StageScope scope(fuse_stages, "integrate");
          integrate_status = integrator.integrate(
              volume, frame.depth.data(), depth_camera, 20.0f,
              rtsdf::IntegrationMode::Classic, &color_frame);
        }
        if (!integrate_status.ok()) {
          std::fprintf(stderr, "fuse_viewer: integrate (frame %zu): %s\n", i,
                       integrate_status.message().c_str());
          break;
        }
        fused_count.store(i + 1);
        if ((i % static_cast<std::size_t>(opt.remesh_every)) == 0) {
          rmesh::ExtractTimings extract_timings;
          vr::Result<rmesh::DeviceMesh> extracted = [&]() {
            fuse_viewer::StageScope scope(fuse_stages, "extract");
            return extractor.extract_device(volume, 0.0f, &extract_timings);
          }();
          // Break the extract row down in place: the phases sum to it, so the
          // table reads as a hierarchy rather than double-counting.
          fuse_stages.add("  ..compact", extract_timings.compact_ms);
          fuse_stages.add("  ..neighbour lut",
                          extract_timings.neighbour_lut_ms);
          fuse_stages.add("  ..inputs", extract_timings.input_upload_ms);
          fuse_stages.add("  ..arena alloc", extract_timings.arena_alloc_ms);
          fuse_stages.add("  ..descriptors", extract_timings.descriptor_ms);
          fuse_stages.add("  ..dispatch", extract_timings.dispatch_ms);
          fuse_stages.add("  ..readback", extract_timings.readback_ms);
          extract_stats = extract_timings;
          if (extracted && !extracted.value().empty())
            publish(extracted.value(), frame.depth.data(), depth_camera,
                    frame.color);
        }
        // Publish this frame's stage breakdown + the volume's device memory for
        // the overlay. Sampled here (not in the render thread) because the
        // recon allocator belongs to the fuse thread's device; VMA's own
        // synchronisation makes the read safe either way.
        {
          const vr::MemoryStats recon_memory = rallocator.memory_stats();
          std::lock_guard<std::mutex> lock(share_mtx);
          shared_fuse_stages = fuse_stages.sections();
          // Fusion cost, so the dataset read is excluded: it is dataloading,
          // not fusion, and while streaming it dwarfs the rest (~10 ms of
          // JPEG/PNG decode). It stays visible as its own `frame` row.
          shared_fuse_ms = fuse_stages.total_ms(/*exclude=*/"frame");
          shared_recon_memory = recon_memory;
          shared_map_buckets = volume.grid().num_buckets;
          shared_map_blocks = volume.grid().num_blocks;
          shared_extract = extract_stats;
        }
        // Retain this frame (the newest keyframe) for the final extract below.
        last_frame = std::move(view);
      }
      // Skip the full-volume final extract when the user has already quit, so
      // the join at shutdown does not stall on a whole marching-cubes pass.
      if (!quit.load()) {
        auto m = extractor.extract_device(volume);  // final mesh
        if (m && !m.value().empty()) {
          // Texture the final mesh with the last keyframe (its depth camera
          // rebuilt from the retained frame), or leave it untextured if no
          // frame ever fused.
          static const std::vector<std::uint32_t> kNoColor;
          if (last_frame) {
            const vr_example::RgbdFrame& keyframe = **last_frame;
            publish(m.value(), keyframe.depth.data(),
                    vr_example::make_depth_camera(cam, keyframe.cam_to_world,
                                                  opt.max_depth),
                    keyframe.color);
          } else {
            publish(m.value(), nullptr, vol::DepthCameraParams{}, kNoColor);
          }
        }
      }
    } catch (const std::exception& e) {
      std::fprintf(stderr, "fuse_viewer: fuse thread aborted: %s\n", e.what());
    }
    fusing_done.store(true);
    std::printf("fuse thread: done (%zu frames)\n", fused_count.load());
  });
  QuitJoin fuse_guard{fuse_thread, quit};

  // --- Render thread (main): pick up the newest mesh + trajectory, upload,
  // draw following the capture path.
  // ------------------------------------------------
  std::vector<std::shared_ptr<vgp::GpuMesh>> slot_mesh(config.frames_in_flight);
  std::vector<std::shared_ptr<AtlasVersion>> slot_atlas(
      config.frames_in_flight);
  std::shared_ptr<vgp::GpuMesh> current_gpu;  // latest upload, shared by slots
  std::uint64_t uploaded_version = 0;         // version held in current_gpu
  vg::assets::Mesh current_mesh;              // host source for the next upload
  std::uint64_t current_version = 0;          // version of current_mesh
  std::vector<std::uint8_t>
      current_atlas_px;  // its keyframe pixels (empty=none)
  std::vector<glm::mat4> poses;
  std::size_t view_frame = 0;
  // The fuse thread's newest published stage rows + counters, copied out under
  // share_mtx each frame so the panels read a consistent snapshot.
  std::vector<vg::FrameMetrics::Section> fuse_stages_snapshot;
  ReconstructionPanel recon_panel;

  std::printf(
      "fuse_viewer: %zu frames, fusing on a background thread; close the "
      "window to quit\n",
      frame_count);
  int tick = 0;
  int exit_code = 0;
  while (glfwWindowShouldClose(window) == GLFW_FALSE) {
    glfwPollEvents();

    // Grab the newest published mesh + any new poses (cheap; the heavy load /
    // decode / fuse all runs on the background thread).
    {
      std::lock_guard<std::mutex> lock(share_mtx);
      if (pending_mesh && published_version != current_version) {
        current_mesh = std::move(*pending_mesh);
        pending_mesh.reset();
        current_atlas_px = std::move(pending_atlas);
        pending_atlas.clear();  // moved-from vector -> defined empty state
        current_version = published_version;
      }
      // Append only the new tail (shared_poses only grows) rather than
      // re-copying the whole trajectory each frame it changes.
      if (poses.size() < shared_poses.size())
        poses.insert(poses.end(), shared_poses.begin() + poses.size(),
                     shared_poses.end());
      fuse_stages_snapshot = shared_fuse_stages;
      recon_panel.fuse_ms = shared_fuse_ms;
      recon_panel.recon_memory = shared_recon_memory;
      recon_panel.map_buckets = shared_map_buckets;
      recon_panel.map_blocks = shared_map_blocks;
      recon_panel.preloaded_bytes = shared_preloaded_bytes;
      recon_panel.extract = shared_extract;
    }
    const std::size_t done_frames = fused_count.load();
    const bool done = fusing_done.load();
    recon_panel.fused_frames = done_frames;
    recon_panel.total_frames = frame_count;
    recon_panel.vertices = current_mesh.vertices.size();
    recon_panel.triangles = current_mesh.indices.size() / 3;
    recon_panel.mesh_version = current_version;

    auto frame = app.begin_frame(window_extent(window));
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

    // Upload the newest host mesh once when its version changes (not once per
    // ring slot): a shared_ptr keeps a GpuMesh alive until every slot that drew
    // it has cycled (each release gated by begin_frame's per-slot fence wait),
    // so one upload safely feeds all slots.
    {
      // Scoped over the whole check, not just the upload, so the row reports
      // ~0 on a frame with no new mesh instead of vanishing from the table.
      vg::Profiler::Scope upload_scope = profiler.cpu_scope("mesh upload");
      if (current_version != 0 && uploaded_version != current_version) {
        // Build this version's atlas first (its keyframe image, else the white
        // dummy), then upload the mesh, and commit both together -- so the
        // drawn mesh and the atlas its uv0 index into always come from the SAME
        // version. A transient atlas- or mesh-upload failure leaves the
        // previous coherent pair in place and retries next frame (the viewer
        // keeps drawing), rather than binding a new mesh against a stale atlas.
        std::shared_ptr<AtlasVersion> next =
            current_atlas_px.empty()
                ? white_atlas
                : build_atlas(current_atlas_px.data(), cam.width, cam.height);
        if (next) {
          auto gpu =
              vgp::upload_mesh(app.device(), app.allocator(), current_mesh);
          if (gpu.ok()) {
            current_gpu =
                std::make_shared<vgp::GpuMesh>(std::move(gpu).value());
            current_atlas = std::move(next);
            uploaded_version = current_version;
          }
        }
      }
    }
    // This slot adopts the latest mesh + its atlas, releasing whatever it drew
    // last frame (safe: begin_frame fence-waited this slot). Holding the atlas
    // per slot keeps a version bound by an in-flight frame alive past its swap.
    slot_mesh[render_frame.slot] = current_gpu;
    slot_atlas[render_frame.slot] = current_atlas;

    // Follow the trajectory: the frontier (latest fused pose) while fusing,
    // then replay the path in a loop once done.
    const VkExtent2D extent = app.swapchain().extent();
    const float aspect = static_cast<float>(extent.width) /
                         static_cast<float>(std::max(1u, extent.height));
    if (!poses.empty()) {
      if (!done)
        view_frame = poses.size() - 1;  // follow the frontier (latest pose)
      else if (tick % 2 == 0)
        view_frame = (view_frame + 1) % poses.size();
    }
    glm::mat4 view_proj(1.0f);
    if (!poses.empty()) {
      const glm::mat4& c2w = poses[std::min(view_frame, poses.size() - 1)];
      const glm::vec3 eye(c2w[3]);
      const glm::vec3 fwd(c2w[2]);             // OpenCV camera looks down +Z
      const glm::vec3 up(-glm::vec3(c2w[1]));  // image-up is -cameraY (y down)
      view_proj =
          vg::camera::Camera::look_at_perspective(
              eye, eye + fwd, up, vfov, aspect, 0.05f, 2.0f * opt.max_depth)
              .view_proj();
    }
    const bool has_mesh =
        slot_mesh[render_frame.slot] && slot_mesh[render_frame.slot]->valid();
    if (tick % 120 == 0)
      std::printf("render tick %d: fused %zu/%zu, mesh v%llu, drawing=%d\n",
                  tick, done_frames, frame_count,
                  (unsigned long long)current_version, has_mesh ? 1 : 0);

    // Build the ImGui frame here -- after begin_frame has committed to a real
    // frame, so every new_frame is paired with exactly one render (ImGui
    // forbids two un-rendered frames), and before recording, so the panels are
    // ready when the overlay records its draw data inside the pass below.
    if (overlay) {
      ImGui_ImplGlfw_NewFrame();
      overlay->new_frame();
      // Stack the two panels instead of letting ImGui default both to the same
      // spot (which hides one under the other on a fresh layout), and give each
      // room for its full contents -- the stage table and the memory rows are
      // clipped away at ImGui's default size. FirstUseEver, so a drag or resize
      // sticks and imgui.ini keeps it.
      ImGui::SetNextWindowPos(ImVec2(16.0f, 16.0f), ImGuiCond_FirstUseEver);
      ImGui::SetNextWindowSize(ImVec2(400.0f, 380.0f), ImGuiCond_FirstUseEver);
      // The renderer's own snapshot (fps, whole-frame CPU, its GPU spans,
      // its device memory), with recon's fuse stages appended as rows.
      vg::FrameMetrics metrics = profiler.metrics();
      metrics.sections.insert(metrics.sections.end(),
                              fuse_stages_snapshot.begin(),
                              fuse_stages_snapshot.end());
      vg::ui::draw_metrics_panel(metrics, "Performance");
      ImGui::SetNextWindowPos(ImVec2(16.0f, 408.0f), ImGuiCond_FirstUseEver);
      ImGui::SetNextWindowSize(ImVec2(400.0f, 250.0f), ImGuiCond_FirstUseEver);
      draw_reconstruction_panel(recon_panel);
    }

    vg::RenderTargetBeginInfo begin_info;
    begin_info.clear_color.float32[0] = 0.05f;
    begin_info.clear_color.float32[1] = 0.05f;
    begin_info.clear_color.float32[2] = 0.07f;
    begin_info.clear_color.float32[3] = 1.0f;
    render_frame.target->begin(render_frame.cmd, begin_info);
    {
      vg::Profiler::Scope draw_scope =
          profiler.gpu_scope(render_frame.cmd, "mesh draw");
      if (has_mesh) {
        const vgp::HybridMeshDraw draw{slot_mesh[render_frame.slot].get()};
        vgp::HybridMeshFrame hybrid_frame;
        hybrid_frame.extent = extent;
        hybrid_frame.view_proj = view_proj;
        hybrid_frame.light_dir = glm::vec3(0.4f, 0.9f, 0.5f);
        hybrid_frame.flags = opt.lit ? vgp::kHybridMeshLit : 0u;
        hybrid_frame.atlas = slot_atlas[render_frame.slot]->set.handle();
        hybrid_frame.draws = &draw;
        hybrid_frame.draw_count = 1;
        pipeline.submit(render_frame.cmd, hybrid_frame);
      }
    }
    if (overlay) {
      vg::Profiler::Scope overlay_scope =
          profiler.gpu_scope(render_frame.cmd, "overlay draw");
      overlay->render(render_frame.cmd);
    }
    render_frame.target->end(render_frame.cmd);

    const vg::Status present = app.end_frame(render_frame);
    if (!present.ok() && !win::swapchain_stale(present)) {
      std::fprintf(stderr, "end_frame: %s\n", present.message().c_str());
      exit_code = 1;
      break;
    }
    ++tick;
  }

  quit.store(true);  // stop the fuse thread promptly; QuitJoin joins it on exit
  app.wait_idle();
  return exit_code;
}

}  // namespace

int main(int argc, char** argv) {
  Options opt;
  if (!parse_args(argc, argv, opt)) return 2;

  if (glfwInit() != GLFW_TRUE) {
    std::fprintf(stderr, "glfwInit failed\n");
    return 1;
  }
  glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
  GLFWwindow* window =
      glfwCreateWindow(opt.width, opt.height, "fuse_viewer", nullptr, nullptr);
  if (window == nullptr) {
    std::fprintf(stderr, "glfwCreateWindow failed\n");
    glfwTerminate();
    return 1;
  }

  // run() owns the WindowedApp + every device resource; they destruct as it
  // returns, before the window is destroyed (see run()'s note).
  const int rc = run(window, opt);

  glfwDestroyWindow(window);
  glfwTerminate();
  return rc;
}
