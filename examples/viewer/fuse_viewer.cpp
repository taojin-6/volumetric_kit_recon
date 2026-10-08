// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// fuse_viewer: the live recon -> gfx interop demo. Opens a window and fuses a
// posed Replica RGB-D sequence -- polled through the sensor tier's IRgbdSensor
// interface and prepared on the GPU (sensor::GpuFramePrep), so a live camera
// is a construction-site swap -- into a sparse TSDF+colour volume frame by
// frame (volumetric_kit_recon),
// periodically re-extracting a marching-cubes mesh, and
// drawing the growing, coloured reconstruction each frame through
// volumetric_kit_gfx's HybridMeshPipeline following the capture trajectory --
// the nvblox FuserVisualizer analogue. Both libraries run on ONE VkDevice,
// built by the neutral bootstrap in shared_device.hpp and adopted by each.
//
// The mesh crosses the seam as HANDLES, not bytes -- interop seam B. recon's
// marching-cubes kernel writes its vertex arena, index run and
// VkDrawIndexedIndirectCommand, and gfx binds those very buffers as a
// pipelines::LiveMesh and issues vkCmdDrawIndexedIndirect, so the index count
// is read GPU-side out of the command recon wrote and never crosses the CPU
// either. What used to be here was extractor.download -> to_gfx_mesh ->
// upload_mesh: a full readback plus a full re-upload of geometry that never
// conceptually left the device (~50 MB each way on a ~790 k-vertex room scan),
// every remesh.
//
// Three things make that safe, and all three are checked rather than assumed:
//   * usage + sharing mode -- Vulkan cannot be asked what a VkBuffer was
//     created with, so recon reports both on the DeviceMesh and this file
//     verifies them before binding. On Apple the two libraries land on
//     different queue families, where reading an EXCLUSIVE buffer is undefined
//     with nothing to report it.
//   * lifetime -- recon rings its output slots, and mesh::MeshExchange
//     releases a generation once every frame that drew it has retired, which
//     begin_frame's wait for the frame slot's last frame says. No semaphore
//     crosses this seam: the extract is gated on the host.
//   * visibility -- the fuse thread blocks on its dispatch's fence inside
//     submit_single_time before it publishes anything, and gfx's later
//     vkQueueSubmit makes those device writes visible to the draw, so no
//     barrier is recorded here. Note what this does NOT rest on: recon's
//     dispatch barrier names DRAW_INDIRECT unconditionally but VERTEX_INPUT
//     only where its queue family advertises graphics, since Vulkan forbids
//     naming that stage on a compute-only one -- and the bootstrap matches
//     recon's family on VK_QUEUE_COMPUTE_BIT alone, so off Apple it can land
//     on exactly such a family.
//
// Each re-meshed frame is projectively textured with its own keyframe (the
// texture tier): the triangles that keyframe saw unoccluded render at full
// sensor resolution, the rest fall back to fused voxel colour. The keyframe's
// colour buffer is copied into gfx's atlas on the device, in the frame that
// first draws the mesh, as rig_viewer copies each camera's (viewer_atlas.hpp's
// LiveAtlas). --no-texture disables it (an A/B against the pure vertex-colour
// path).
//
// Two Dear ImGui panels (gfx's ui tier) show where the time and the memory go:
// Performance (the renderer's fps + GPU spans, with recon's per-frame fuse
// stages appended) and Reconstruction (mesh/volume counters + recon's device
// memory). --no-overlay turns both off.
//
//   fuse_viewer <scene_dir> [--voxel 0.02] [--trunc m] [--min-depth m]
//               [--max-depth m] [--max-frames N] [--remesh-every N]
//               [--width 1280] [--height 720] [--unlit] [--no-texture]
//               [--preload] [--no-overlay] [--validation] ...

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
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

#include "cli.hpp"
#include "fuse_frame.hpp"  // vr_example::fuse_keyframe
#include "fusion_flags.hpp"
#include "recon_gfx_bridge.hpp"  // to_live_mesh, and the vertex-layout asserts
#include "replica_flags.hpp"
#include "replica_sensor.hpp"  // vr_example::ReplicaSensor
#include "shared_device.hpp"
#include "viewer_atlas.hpp"
#include "viewer_common.hpp"
#include "viewer_panels.hpp"

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/core/vulkan/vulkan.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"
#include "volumetric_kit/recon/mesh/device_mesh.hpp"
#include "volumetric_kit/recon/mesh/marching_cubes.hpp"
#include "volumetric_kit/recon/mesh/mesh_exchange.hpp"
#include "volumetric_kit/recon/sensor/rgbd_frame.hpp"
#include "volumetric_kit/recon/sensor/rgbd_sensor.hpp"
#include "volumetric_kit/recon/sensor/utils/gpu_frame_prep.hpp"
#include "volumetric_kit/recon/texture/projective_texturer.hpp"
#include "volumetric_kit/recon/texture/texture_atlas.hpp"
#include "volumetric_kit/recon/tsdf/fuser.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"
#include "volumetric_kit/recon/volume/voxel_grid.hpp"
#include "volumetric_kit/recon/volume/voxel_hash_map.hpp"

#include "volumetric_kit/gfx/app/windowed_app.hpp"
#include "volumetric_kit/gfx/camera/camera.hpp"
#include "volumetric_kit/gfx/core/frame_metrics.hpp"
#include "volumetric_kit/gfx/core/profiler.hpp"
#include "volumetric_kit/gfx/core/render_target.hpp"
#include "volumetric_kit/gfx/pipelines/hybrid_mesh_pipeline.hpp"
#include "volumetric_kit/gfx/pipelines/live_mesh.hpp"
#include "volumetric_kit/gfx/ui/imgui_overlay.hpp"
#include "volumetric_kit/gfx/ui/metrics_panel.hpp"
#include "volumetric_kit/gfx/windowing/frame_loop.hpp"
#include "volumetric_kit/gfx/windowing/swapchain.hpp"

namespace vr = volumetric_kit::recon;
namespace vkc = volumetric_kit::core;
namespace vol = volumetric_kit::recon::volume;
namespace rtsdf = volumetric_kit::recon::tsdf;
namespace rmesh = volumetric_kit::recon::mesh;
namespace rtex = volumetric_kit::recon::texture;
namespace rsensor = volumetric_kit::recon::sensor;
namespace vg = volumetric_kit::gfx;
namespace vgp = volumetric_kit::gfx::pipelines;
namespace win = volumetric_kit::gfx::windowing;

namespace {

struct Options {
  vr_example::ReplicaFlags replica{400};
  vr_example::FusionFlags fusion{0.02f};
  fuse_viewer::WindowFlags window;
  int remesh_every = 1;  // re-extract + re-upload every N fused frames
  bool lit = true;
  bool texture = true;  // project each keyframe onto the growing mesh (uv0)
  // In-block vertex sharing (MarchingCubesConfig::share_vertices). Off here to
  // match the example's history, on in the iOS scanner, where the vertex arena
  // is the term that binds -- so this flag is what lets this window stand in
  // for that configuration rather than only for the desktop one.
  //
  // It is worth a flag specifically BECAUSE it runs beside --texture. Sharing
  // and projective texturing were mutually exclusive until the texture pass
  // moved to a per-vertex dispatch; this is where the two are exercised
  // together, live and growing, rather than in one still frame.
  bool share_vertices = false;
};

vkc::Result<Options> parse_args(int argc, char** argv) {
  Options o;
  vr_example::Cli cli("fuse_viewer");
  o.replica.add_to(cli);
  o.fusion.add_to(cli);
  cli.option("--remesh-every", "N", o.remesh_every, 1);
  o.window.add_to(cli);
  cli.flag("--unlit", o.lit, false)
      .flag("--no-texture", o.texture, false)
      .flag("--share-vertices", o.share_vertices);
  VKC_TRY(cli.parse(argc, argv));
  return o;
}

// What travels with a mesh through the exchange: the keyframe colour its uv0
// index into (empty for none), and the extract's timings, which the panel
// shows beside the mesh they describe.
struct MeshPayload {
  fuse_viewer::AtlasJob atlas;
  rmesh::ExtractTimings extract;
};

// Owns the WindowedApp (and the VkSurfaceKHR built from `window`) plus every
// device resource, so they all destruct BEFORE main destroys the window -- the
// gfx run()/main() split. Destroying a surface/swapchain after its window is a
// use-after-free, notably on MoltenVK where the surface wraps the window's
// CAMetalLayer.
int run(GLFWwindow* window, const Options& opt) {
  // --- One VkDevice, adopted by both libraries ------------------------------
  // The shared device is declared before every wrapper that borrows it, so it
  // outlives them: the gfx app and recon's device/allocator below hold raw
  // handles into it, and both must be gone before the instance and device are
  // destroyed. gfx's requirements are the app config's, which adopt verifies.
  vg::app::WindowedAppConfig config;
  config.app_name = "fuse_viewer";
  config.swapchain.extent = fuse_viewer::window_extent(window);
  config.swapchain.depth_format = VK_FORMAT_D32_SFLOAT;
  config.frames_in_flight = 2;
  fuse_viewer::SharedDeviceConfig shared_config;
  shared_config.enable_validation = opt.window.validation;
  shared_config.graphics = config.device;
  const std::unique_ptr<fuse_viewer::vkc::SharedDevice> shared =
      fuse_viewer::build_shared_device(window, shared_config);
  if (shared == nullptr) return 1;

  // The surface already exists -- picking a present-capable device required
  // one -- so the factory hands over the one the bootstrap made rather than
  // creating a second. Ownership transfers with it.
  auto app_r = vg::app::WindowedApp::adopt(
      shared->graphics_payload(), config,
      [&shared](VkInstance instance) -> vkc::Result<VkSurfaceKHR> {
        // adopt calls this with the instance from the payload, so this can only
        // trip if the two ever stop coming from the same SharedDevice -- at
        // which point the surface would belong to a different instance than the
        // swapchain built on it.
        if (instance != shared->instance().handle()) {
          return vkc::Status::invalid_argument(
              "surface factory: the app adopted a different VkInstance than "
              "the bootstrap created the surface on");
        }
        return shared->release_surface();
      });
  if (!app_r.ok()) {
    std::fprintf(stderr, "WindowedApp::adopt: %s\n",
                 app_r.status().message().c_str());
    return 1;
  }
  vg::app::WindowedApp app = std::move(app_r).value();

  // recon takes its share of the same device. It gets its own VMA allocator --
  // allocators are independent bookkeeping over one VkDevice's memory, so each
  // library manages its own even when the device is shared.
  auto recon_device_result =
      vkc::Device::adopt(shared->compute_payload(), vr::device_requirements());
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

  // The sequence arrives through the sensor interface: the frame cap and the
  // depth gate are the sensor's options, so every frame it hands out is
  // already gated, and the fuse thread below drives an IRgbdSensor& that
  // never learns it is reading a disk. Declared here, before the fuse thread
  // that drives it, so it outlives that thread.
  auto capture_result = opt.replica.open(opt.fusion);
  if (!capture_result) {
    std::fprintf(stderr, "capture: %s\n",
                 capture_result.status().message().c_str());
    return 1;
  }
  vr_example::ReplicaSensor replica = std::move(capture_result).value();
  const vr::camera::CameraModel& cam = *replica.info().color;
  // The far plane follows the depth gate the frames carry.
  const float max_depth = opt.fusion.max_depth.value_or(
      vr_example::ReplicaSensor::Options{}.max_depth);

  auto grid_result = vr_example::create_fusion_grid(
      rdevice, rallocator, opt.fusion.voxel, opt.fusion.trunc);
  if (!grid_result) {
    std::fprintf(stderr, "grid: %s\n", grid_result.status().message().c_str());
    return 1;
  }
  vol::VoxelBlockGrid volume = std::move(grid_result).value();
  auto fuser_result = rtsdf::Fuser::create(rdevice, rallocator);
  if (!fuser_result) {
    std::fprintf(stderr, "fuser: %s\n",
                 fuser_result.status().message().c_str());
    return 1;
  }
  rtsdf::Fuser fuser = std::move(fuser_result).value();
  // The frame prep, like the fuser, used only on the fuse thread below. Its
  // colour is what gfx copies into the atlas, so it is shared with gfx's
  // family too, as the mesh buffers below are.
  rsensor::GpuFramePrepConfig prep_config;
  prep_config.color_queue_families[0] = shared->compute_family();
  prep_config.color_queue_families[1] = shared->graphics_family();
  prep_config.color_queue_family_count = 2;
  auto prep_result =
      rsensor::GpuFramePrep::create(rdevice, rallocator, prep_config);
  if (!prep_result) {
    std::fprintf(stderr, "frame prep: %s\n",
                 prep_result.status().message().c_str());
    return 1;
  }
  rsensor::GpuFramePrep prep = std::move(prep_result).value();
  // The extractor's output buffers are what gfx draws, so this file -- the one
  // place that knows both siblings -- states what the renderer needs of them.
  // recon names none of it: the mesh tier is deliberately not compiled against
  // gfx, the same shape as the create/adopt device seam.
  rmesh::MarchingCubesConfig mc_config;
  // Beyond the STORAGE_BUFFER the kernel writes through. Usage is a union, not
  // a choice -- a buffer created with only the draw bits could not be bound to
  // recon's own descriptors. INDIRECT_BUFFER is already unconditional on the
  // command, so there is nothing to add for the indirect draw.
  mc_config.extra_vertex_usage = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
  mc_config.extra_index_usage = VK_BUFFER_USAGE_INDEX_BUFFER_BIT;
  // Both families, unconditionally. recon reduces them to their distinct
  // entries and picks EXCLUSIVE where they collapse to one, so this needs no
  // branch on the queue plan -- and passing only recon's would leave the
  // buffers EXCLUSIVE under kTwoFamilies (what MoltenVK actually gives), where
  // gfx reading them from the family that does not own them is undefined with
  // nothing to report it.
  mc_config.queue_families[0] = shared->compute_family();
  mc_config.queue_families[1] = shared->graphics_family();
  mc_config.queue_family_count = 2;
  // One slot per frame in flight, plus one: the frames still in flight each
  // hold a generation, and one more is being extracted. Derived from the value
  // fed to WindowedAppConfig rather than written as a literal, since that is
  // the whole correctness argument -- a slot must not be reused while any frame
  // that could still be reading it is alive. Each slot is a full vertex arena
  // that never shrinks, so this is not free; deeper buys nothing.
  mc_config.slot_count = config.frames_in_flight + 1;
  // In-block vertex sharing, which the texture pass no longer excludes: it
  // dispatches per vertex, so a vertex belonging to several triangles has one
  // writer rather than several disagreeing ones. See --share-vertices.
  mc_config.share_vertices = opt.share_vertices;
  auto extractor_result =
      rmesh::MarchingCubes::create(rdevice, rallocator, mc_config);
  if (!extractor_result) {
    std::fprintf(stderr, "marching cubes: %s\n",
                 extractor_result.status().message().c_str());
    return 1;
  }
  rmesh::MarchingCubes extractor = std::move(extractor_result).value();
  // Projective texturer (recon device; used, like the fuser and extractor,
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

  const std::size_t frame_count = replica.frame_count();
  const float vfov =
      2.0f *
      std::atan(static_cast<float>(cam.size.height) /
                (2.0f * std::max(1.0f, static_cast<float>(cam.intrinsics.fy))));

  // --- gfx: pipeline + the keyframe atlas -----------------------------------
  auto pipeline_result = vgp::HybridMeshPipeline::create(
      app.device(), app.allocator(), app.swapchain().layout());
  if (!pipeline_result.ok()) {
    std::fprintf(stderr, "pipeline: %s\n",
                 pipeline_result.status().message().c_str());
    return 1;
  }
  vgp::HybridMeshPipeline pipeline = std::move(pipeline_result).value();
  // The keyframe's whole colour image is the atlas: one tile, the colour
  // camera's size. A keyframe of another size is not textured.
  const rtex::AtlasTile keyframe_tile{0, 0, cam.size.width, cam.size.height};
  std::unique_ptr<fuse_viewer::LiveAtlas> atlas;
  if (opt.texture) {
    auto atlas_result = fuse_viewer::LiveAtlas::create(
        pipeline, app.allocator(), app.frame_loop().timeline(),
        {keyframe_tile.width, keyframe_tile.height}, config.frames_in_flight);
    if (!atlas_result.ok()) {
      std::fprintf(stderr, "atlas: %s\n",
                   atlas_result.status().message().c_str());
      return 1;
    }
    atlas = std::move(atlas_result).value();
  }

  // --- gfx profiler: the renderer's own per-frame CPU/GPU timings ------------
  // The render side gets real GPU spans (this device reports 64
  // timestampValidBits through MoltenVK) plus fps and whole-frame CPU time;
  // attaching it to the app makes the frame loop drive begin_frame/end_frame.
  // recon's stages are measured separately, as wall-clock rows unless a
  // vkc::GpuTimer timed them (StageRow::has_gpu says which): every recon
  // dispatch blocks on its fence, so a host span covers the device work too.
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
  // Performance panel reports the mesh/atlas/swapchain footprint. recon runs
  // its own VMA allocator over the same device (independent bookkeeping, not a
  // second device), reported separately by the Reconstruction panel below, so
  // the two figures partition the shared device's memory rather than
  // double-counting it. `app` outlives the profiler (declared before it),
  // which set_memory_source requires.
  profiler.set_memory_source(&app.allocator());
  app.set_profiler(&profiler);
  const fuse_viewer::ProfilerDetach profiler_guard{app};

  // --- gfx ui: the Dear ImGui performance overlay ----------------------------
  // Optional: --no-overlay skips both the context and the platform backend, so
  // the disabled path costs nothing and stays a clean A/B. The pipeline bakes
  // the swapchain layout, which survives a resize, so the overlay is built
  // once.
  std::optional<vg::ui::ImGuiOverlay> overlay;
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
  const fuse_viewer::ImGuiGlfwShutdown imgui_glfw_guard{opt.window.overlay};

  // --- Background fuse thread: load + decode + fuse + extract off the render
  // thread (per-frame JPEG/PNG decode is CPU-heavy and would otherwise gate the
  // loop, starving the GPU). It publishes the newest coloured mesh + the
  // trajectory; the render thread only uploads + draws, so the window stays at
  // full frame rate. Whether the two threads' *submits* also overlap is the
  // shared device's queue plan: they do under kTwoQueuesOneFamily and
  // kTwoFamilies (a queue each), and serialize under kSharedQueue (one queue
  // behind a mutex) -- which is why the bootstrap prefers a second family over
  // sharing a queue. recon's device wrapper is used solely on this thread.
  // -------------------------------------------
  // The mesh handoff: recon's buffers, borrowed -- handles and counts, not
  // bytes -- each with its MeshPayload.
  const bool cross_family =
      shared->graphics_family() != shared->compute_family();
  rmesh::MeshExchangeConfig exchange_config;
  exchange_config.frames_in_flight = config.frames_in_flight;
  exchange_config.cross_family = cross_family;
  rmesh::MeshExchange<MeshPayload> exchange(exchange_config);
  fuse_viewer::SharedView shared_view;  // the render camera, meshed
  std::mutex share_mtx;
  std::vector<glm::mat4> shared_poses;  // trajectory, grows as frames fuse
  std::atomic<std::size_t> fused_count{0};
  std::atomic<bool> fusing_done{false};
  std::atomic<bool> quit{false};
  // Newest fused frame's stage breakdown + the volume's footprint, published
  // for the overlay under share_mtx.
  std::vector<vkc::StageRow> shared_fuse_stages;
  double shared_fuse_ms = 0.0;
  vkc::MemoryStats shared_recon_memory;
  std::int32_t shared_map_buckets = 0;
  std::int32_t shared_map_blocks = 0;
  float shared_map_load_factor = 0.0f;
  std::uint64_t shared_preloaded_bytes = 0;

  std::thread fuse_thread([&]() {
    // Any throw escaping this thread function (e.g. a decode/allocation
    // bad_alloc) would call std::terminate; contain it so shutdown stays clean.
    try {
      // Scratch for this thread only; copied under share_mtx once per frame.
      vkc::StageMetrics fuse_stages;
      // The remesh-only rows -- extract and its breakdown, and texture --
      // measured here rather than straight into `fuse_stages`, and
      // merged in on every frame whether or not this one remeshed.
      //
      // The gate below fires only on the fused frames the renderer has kept up
      // with, and fusion routinely outruns it (a preloaded run fuses several
      // frames per presented frame), so measuring them into `fuse_stages`
      // directly left them reading 0.00 on most samples -- on the very
      // instrument this repo's history credits with catching the arena-alloc
      // and neighbour-table regressions, and against its own rule that a row
      // which is usually zero is worse than an absent one. Held, they describe
      // the newest remesh; the panel's `fuse ms/frame` therefore reads as the
      // cost of a fused frame that also remeshed.
      vkc::StageMetrics remesh_stages;
      // Texture `device_mesh` with one keyframe, then publish it with that
      // keyframe's colour, the atlas its uv0 index into, and the `timings` of
      // the extract that made it. On any texturing failure -- or when
      // --no-texture -- the atlas job stays empty and the mesh draws in its
      // fused voxel colour.
      //
      // The mesh and its atlas are ONE value, published and taken together.
      // uv0 is a normalized coordinate into the image of the camera that
      // textured it, and every texture() call rewrites every vertex's uv0
      // against the *current* frame -- so drawing a mesh against a later
      // frame's image samples the wrong place on every textured triangle.
      //
      // Nothing is copied to the host: what crosses is the DeviceMesh, five
      // words of handles and counts, and the keyframe's colour buffer, which
      // the render thread copies into the atlas on the device.
      //
      // `keyframe` is the frame to texture with, or null for none. A frame
      // without colour, or colour gfx cannot copy into the atlas, is not
      // textured: once uv0 index the atlas, the atlas must hold the image.
      bool uncopyable_said = false;
      auto publish = [&](const rmesh::DeviceMesh& device_mesh,
                         const rsensor::DeviceFrame* keyframe,
                         const rmesh::ExtractTimings& timings) {
        MeshPayload payload;
        payload.extract = timings;
        if (texturer && keyframe != nullptr && keyframe->has_color() &&
            !device_mesh.empty()) {
          if (keyframe->color_camera.width != keyframe_tile.width ||
              keyframe->color_camera.height != keyframe_tile.height ||
              !fuse_viewer::copyable(*keyframe->color, keyframe_tile,
                                     cross_family)) {
            // Said once: a frame's colour is fixed by the source.
            if (!uncopyable_said) {
              std::fprintf(stderr,
                           "fuse_viewer: the keyframe's colour cannot be "
                           "copied into the atlas (not the colour camera's "
                           "size, not a copy source, or EXCLUSIVE across two "
                           "queue families); drawing fused colour\n");
              uncopyable_said = true;
            }
          } else {
            // Textures the extractor's buffers in place -- no upload, no
            // readback; the geometry has not left the device since it was
            // meshed.
            //
            // The tier opens its own "texture" row, so there is no
            // StageScope here: rows accumulate by name, and wrapping the call
            // as well would count the host span twice while adding nothing.
            // What the tier's row has that a wrapper's cannot is the device
            // half.
            //
            // Through the colour camera, the coverage read off the colour's
            // high byte: the frame as the GPU pass left it, as for any camera.
            rtex::TextureView view;
            view.cam = keyframe->depth_camera;
            view.depth_buffer = keyframe->depth;
            view.color_camera = keyframe->color_camera;
            view.coverage = keyframe->color;
            const vkc::Status texture_status =
                texturer->texture(device_mesh, view, 0.02f, &remesh_stages);
            if (texture_status.ok()) {
              payload.atlas.tiles.push_back(
                  {keyframe->color, keyframe_tile, 0});
            } else {
              std::fprintf(stderr, "fuse_viewer: texture: %s\n",
                           texture_status.message().c_str());
            }
          }
        }
        exchange.publish(device_mesh, std::move(payload));
      };
      // Extract, break the extract row down, and publish the mesh textured
      // with `keyframe`.
      auto remesh = [&](const rsensor::DeviceFrame* keyframe) {
        remesh_stages.clear();
        rmesh::ExtractTimings extract_timings;
        vkc::Result<rmesh::DeviceMesh> extracted = [&]() {
          vkc::StageScope scope(remesh_stages, "extract");
          return shared_view.extract(extractor, volume, &extract_timings);
        }();
        // Break the extract row down in place. The phases sum to the
        // `extract` row above rather than adding to it, so they carry
        // StageMetrics::kBreakdownPrefix -- which is what makes the table
        // read as a hierarchy *and* keeps total_cpu_ms from counting the
        // extract twice.
        remesh_stages.add_cpu("  ..compact", extract_timings.compact_ms);
        remesh_stages.add_cpu("  ..arena alloc",
                              extract_timings.arena_alloc_ms);
        remesh_stages.add_cpu("  ..descriptors", extract_timings.descriptor_ms);
        remesh_stages.add_cpu("  ..dispatch", extract_timings.dispatch_ms);
        remesh_stages.add_cpu("  ..readback", extract_timings.readback_ms);
        // Published even when it meshed nothing: it holds a ring slot all the
        // same (see MeshExchange::publish).
        if (extracted) {
          publish(extracted.value(), keyframe, extract_timings);
        } else {
          // Every other stage in this loop reports its failure; this one used
          // to be silent, which under seam B reads as a frozen mesh with a
          // healthy frame counter beside it.
          std::fprintf(stderr, "fuse_viewer: extract: %s\n",
                       extracted.status().message().c_str());
        }
      };
      // Decode the whole sequence up front when asked, so the loop below is
      // gated by fusion rather than by JPEG/PNG decode (~75% of a streaming
      // loop). Done here, on the fuse thread, so the window is already up and
      // responsive while it works.
      if (opt.replica.preload) {
        // `quit` stops the decode at the next frame boundary, so closing the
        // window mid-preload does not leave the join at shutdown waiting out
        // the whole sequence -- the same reason the final extract below is
        // skipped once the user has quit.
        auto cached_frames = vr_example::preload_frames(replica, &quit);
        if (!cached_frames) {
          // poll() still decodes on demand, so a failed preload costs speed,
          // not the run.
          std::fprintf(stderr, "fuse_viewer: preload: %s (streaming instead)\n",
                       cached_frames.status().message().c_str());
        }
        // Sampled once, here: preloaded_bytes() walks the cache, and the cache
        // is only immutable now that the decode has finished.
        const std::uint64_t cache_bytes = replica.preloaded_bytes();
        std::lock_guard<std::mutex> lock(share_mtx);
        shared_preloaded_bytes = cache_bytes;
      }
      // From here on the source is the interface, not the dataset: the loop
      // polls an IRgbdSensor& and a live driver slots in at the open above.
      rsensor::IRgbdSensor& capture = replica;
      const vkc::Status started = capture.start();
      if (!started.ok()) {
        std::fprintf(stderr, "fuse_viewer: capture start: %s\n",
                     started.message().c_str());
      }
      // The newest successfully fused frame, kept through the next prepare
      // and fuse attempt. A failure must leave its pixels and cameras intact
      // for the final extract and later remeshes on a view change.
      std::optional<rsensor::DeviceFrame> last_frame;
      // `i` counts frames handed out, so it advances at the bottom of the
      // body rather than in the loop header: an empty poll from a source that
      // is not exhausted retries without consuming a frame index.
      for (std::size_t i = 0; started.ok() && !quit.load();) {
        // Stage spans are per fused frame: the overlay shows the newest frame's
        // breakdown, not a running total. Seed every row this frame *could*
        // fill, in display order, so the remesh-only stages report 0 between
        // remeshes instead of dropping out and shuffling the table.
        fuse_stages.clear();
        for (const char* stage :
             {"frame", "frame prep", "allocate", "resize", "integrate",
              "  ..active set", "extract", "  ..compact", "  ..arena alloc",
              "  ..descriptors", "  ..dispatch", "  ..readback", "texture"}) {
          fuse_stages.seed(stage);
        }
        // A preload cache hit, else a disk read + JPEG/PNG decode (the CPU
        // cost the preload exists to hoist out of this loop). Timed either way,
        // so --preload's effect is visible as this row collapsing to ~0.
        auto polled = [&]() {
          vkc::StageScope scope(fuse_stages, "frame");
          return capture.poll();
        }();
        if (!polled) {
          // Every other failure in this loop already prints (fuse / extract),
          // so swallowing a decode error here was the one way to stop early
          // with nothing on stderr -- the panel just froze at "fused N / M",
          // indistinguishable from a normal finish.
          std::fprintf(stderr, "frame %zu failed to load: %s\n", i,
                       polled.status().message().c_str());
          break;
        }
        // An empty poll is "nothing this tick", which a replay and an idle
        // live device report alike; only the source knows whether that is the
        // end.
        if (!polled.value()) {
          if (capture.exhausted()) {
            break;
          }
          // A live sensor polled faster than it runs: re-mesh if the view
          // moved, else yield, and ask again.
          if (shared_view.moved() &&
              exchange.release_and_may_publish(extractor)) {
            remesh(last_frame ? &*last_frame : nullptr);
          } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
          }
          continue;
        }
        // Prepare, then fuse (grow ahead, allocate the band, integrate). Each
        // tier fills its own stage rows. Commit the new keyframe only after
        // both succeed; a failure still remeshes with the previous frame's
        // texture. The error names the stage that stopped fusion.
        const vkc::Status fuse_status = vr_example::fuse_keyframe(
            fuser, volume, prep, *polled.value(), last_frame,
            opt.fusion.max_weight, &fuse_stages);
        if (!fuse_status.ok()) {
          std::fprintf(stderr, "fuse_viewer: frame %zu: %s\n", i,
                       fuse_status.message().c_str());
          break;
        }
        const rsensor::DeviceFrame& frame = *last_frame;
        {
          std::lock_guard<std::mutex> lock(share_mtx);
          shared_poses.push_back(frame.depth_camera.cam_to_world);
        }
        fused_count.store(i + 1);
        // Do not publish over a mesh the renderer has not collected: the
        // extract is not run, saving the dispatch. Fusion routinely outruns
        // the render loop here (a preloaded run remeshes several times per
        // presented frame), so this is the common path, not a corner.
        if ((i % static_cast<std::size_t>(opt.remesh_every)) == 0 &&
            exchange.release_and_may_publish(extractor)) {
          remesh(&frame);
        }
        // Merge the newest remesh's rows in, on every frame -- see
        // remesh_stages. merge() matches by name, so they land in the slots the
        // seed loop above reserved and the table keeps its order, and it
        // carries both halves: a hand-written add_cpu loop here would drop the
        // device column the moment these stages gain one.
        fuse_stages.merge(remesh_stages);
        // Publish this frame's stage breakdown + the volume's device memory for
        // the overlay. Sampled here (not in the render thread) because the
        // recon allocator belongs to the fuse thread's device; VMA's own
        // synchronisation makes the read safe either way.
        {
          const vkc::MemoryStats recon_memory = rallocator.memory_stats();
          // Read out here beside memory_stats and for the same reason: it is a
          // host field behind a Result whose Status carries a string, and the
          // render thread is waiting on this lock. Constant-time, so
          // sampling it every fused frame is affordable (2026-08-08); it fails
          // only on a moved-from map, and the negative it publishes then is not
          // defensiveness for its own sake: a gauge whose whole claim is "you
          // will not have to already know the threshold" cannot answer an
          // unknown with the last good fraction, which is the one reading that
          // looks exactly like a healthy map.
          const vkc::Result<float> lf = volume.map().load_factor();
          if (!lf) {
            std::fprintf(stderr, "fuse_viewer: load_factor (frame %zu): %s\n",
                         i, lf.status().message().c_str());
          }
          const float load_factor = lf ? lf.value() : -1.0f;
          std::lock_guard<std::mutex> lock(share_mtx);
          shared_fuse_stages = fuse_stages.rows();
          // Fusion cost, so the dataset read is excluded: it is dataloading,
          // not fusion, and while streaming it dwarfs the rest (~10 ms of
          // JPEG/PNG decode). It stays visible as its own `frame` row.
          shared_fuse_ms = fuse_stages.total_cpu_ms(/*exclude=*/"frame");
          shared_recon_memory = recon_memory;
          shared_map_buckets = volume.grid().num_buckets;
          shared_map_blocks = volume.grid().num_blocks;
          shared_map_load_factor = load_factor;
        }
        ++i;
      }
      // Skip the final extract when the user has already quit, so the join at
      // shutdown does not stall on a marching-cubes pass.
      if (!quit.load()) {
        // Unlike a remesh inside the loop, this one is worth waiting for: it
        // meshes the last frames fused, and only a view change supersedes it.
        // So rather than skip on an uncollected publish, give the render thread
        // a moment to take it -- it collects on every iteration, so this is
        // normally one frame. Bounded, because a window the compositor has
        // stopped scheduling would otherwise hold the shutdown join open.
        exchange.wait_collected(std::chrono::seconds(1), quit);
      }
      // Re-checked after the wait, which can span a whole second: `quit` is
      // what says the window is gone, and running a marching-cubes pass into a
      // closed window is exactly what the guard above exists to avoid.
      // The two used to be adjacent statements, so the gap did not exist.
      if (!quit.load()) {
        // Apply the release the render thread reported, then extract *whether
        // or not* the last in-loop publish was collected. This one supersedes
        // that mesh, so overwriting it is the intended outcome; and if the ring
        // genuinely has no free slot recon refuses with a Status, which is
        // reported below. Skipping the whole final extract on an uncollected
        // publish -- which is what a minimized window produces, since
        // begin_frame returns no frame and the render loop never reaches its
        // take -- lost the complete surface with nothing on stderr.
        if (!exchange.release_and_may_publish(extractor)) {
          std::fprintf(stderr,
                       "fuse_viewer: the renderer never collected the last "
                       "mesh (window hidden, or drawing stopped); extracting "
                       "the final mesh anyway\n");
        }
        // Textured with the last keyframe, or untextured if no frame ever
        // fused.
        remesh(last_frame ? &*last_frame : nullptr);
      }
      // Fusion is over, but the view still moves -- the replay walks the
      // trajectory -- so re-mesh what it sees whenever it changes.
      fusing_done.store(true);
      std::printf("fuse thread: done (%zu frames)\n", fused_count.load());
      while (!quit.load()) {
        if (shared_view.moved() &&
            exchange.release_and_may_publish(extractor)) {
          remesh(last_frame ? &*last_frame : nullptr);
        } else {
          std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
      }
    } catch (const std::exception& e) {
      std::fprintf(stderr, "fuse_viewer: fuse thread aborted: %s\n", e.what());
    }
    fusing_done.store(true);
  });
  fuse_viewer::QuitJoin fuse_guard{fuse_thread, quit};

  // --- Render thread (main): pick up the newest mesh + trajectory, copy its
  // atlas, draw following the capture path.
  // ------------------------------------------------
  std::vector<glm::mat4> poses;
  std::size_t view_frame = 0;
  // The fuse thread's newest published stage rows + counters, copied out under
  // share_mtx each frame so the panels read a consistent snapshot.
  std::vector<vkc::StageRow> fuse_stages_snapshot;
  fuse_viewer::ReconstructionPanel recon_panel;

  std::printf(
      "fuse_viewer: %zu frames, fusing on a background thread; close the "
      "window to quit\n",
      frame_count);
  int tick = 0;
  int exit_code = 0;
  while (glfwWindowShouldClose(window) == GLFW_FALSE) {
    glfwPollEvents();

    auto frame = app.begin_frame(fuse_viewer::window_extent(window));
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

    // Retire this slot's last frame and take the newest mesh (begin_frame
    // waited for the slot's last frame), then commit it with its payload: its
    // atlas, the keyframe's colour copied in this command buffer before the
    // frame's rendering begins, and the panel's extract rows. A refused copy
    // keeps the mesh parked and the previous mesh, atlas and rows shown, so
    // all three always come from the same extract. The commit runs while the
    // fuse thread may already be extracting the next mesh, which is why the
    // rows come in the payload. An empty mesh skips the callback, so the rows
    // stay those of the last mesh that drew anything.
    if (atlas) atlas->poll();
    {
      // Scoped over the whole step, so the row reports ~0 on a frame with no
      // new mesh instead of vanishing from the table.
      vg::Profiler::Scope copy_scope =
          profiler.gpu_scope(render_frame.cmd, "atlas copy");
      const rmesh::ExchangeOutcome outcome = exchange.begin_frame(
          render_frame.slot,
          [&](const rmesh::DeviceMesh&, const MeshPayload& payload) {
            if (atlas) {
              const vkc::Status committed = atlas->commit(
                  render_frame.cmd, render_frame.number, payload.atlas);
              if (!committed.ok()) {
                std::fprintf(stderr, "fuse_viewer: atlas: %s\n",
                             committed.message().c_str());
                return false;
              }
            }
            recon_panel.extract = payload.extract;
            return true;
          });
      if (outcome == rmesh::ExchangeOutcome::kRefused) {
        std::fprintf(stderr,
                     "fuse_viewer: the extracted mesh cannot be bound as "
                     "geometry (%s); drawing stops here\n",
                     exchange.refused());
      }
    }
    const rmesh::DeviceMesh& live_view = exchange.live();
    // The fuse thread's newest counters: they describe the volume and the
    // fuse loop, not the drawn mesh, whose rows the commit above set.
    {
      std::lock_guard<std::mutex> lock(share_mtx);
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
      recon_panel.map_load_factor = shared_map_load_factor;
      recon_panel.preloaded_bytes = shared_preloaded_bytes;
    }
    const std::size_t done_frames = fused_count.load();
    const bool done = fusing_done.load();
    recon_panel.fused_frames = done_frames;
    recon_panel.total_frames = frame_count;

    // What is on screen, not what has merely been taken.
    recon_panel.vertices = live_view.vertex_count;
    recon_panel.triangles = live_view.triangle_count;
    recon_panel.mesh_version = live_view.generation;

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
      view_proj = vg::camera::Camera::look_at_perspective(
                      eye, eye + fwd, up, vfov, aspect, 0.05f, 2.0f * max_depth)
                      .view_proj();
      shared_view.publish(view_proj);  // for the fuse thread to mesh
    }

    const bool has_mesh = live_view.valid() && !live_view.empty();
    if (tick % 120 == 0)
      std::printf("render tick %d: fused %zu/%zu, mesh v%llu, drawing=%d\n",
                  tick, done_frames, frame_count,
                  (unsigned long long)live_view.generation, has_mesh ? 1 : 0);

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
      fuse_viewer::draw_reconstruction_panel(recon_panel);
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
        // recon's buffers, named rather than copied. LiveMesh owns nothing and
        // reads the index count GPU-side out of the indirect command recon's
        // marching-cubes kernel wrote, so the count does not cross the CPU
        // either.
        //
        // No barrier is recorded here, and the reason is the fence + submit
        // chain, not the barrier scope: recon's extract blocks on its own fence
        // inside submit_single_time -- an availability operation covering every
        // device write it made -- before the mesh is published at all, and this
        // thread's vkQueueSubmit then makes those writes visible to the draw.
        // recon's dispatch barrier does also name DRAW_INDIRECT, but it names
        // VERTEX_INPUT only where its queue family advertises graphics (Vulkan
        // forbids that stage on a compute-only family, and the bootstrap
        // matches recon's on VK_QUEUE_COMPUTE_BIT alone), so it is not
        // something this seam can rest on off Apple.
        //
        // The arena and index run are device-local, so the vertex-input stage
        // fetches the mesh from VRAM on every presented frame (~64 MiB at
        // room0's 991 k vertices). MarchingCubesConfig::share_vertices cuts
        // that fetch ~4x by emitting ~4x fewer vertices, and
        // `--share-vertices` now takes it here. It was
        // unavailable to this example while the texture tier refused a shared
        // mesh -- it decided visibility per triangle and wrote uv0 per vertex
        // -- and the per-vertex dispatch removed that refusal, so the ~4x is
        // measurable on the running window rather than only in `fuse_replica`.
        // The texture tier's several-view atlas still refuses a shared mesh
        // -- it chooses per triangle, and waits on a per-primitive tile id --
        // but this example textures from one camera.
        const vgp::HybridMeshDraw draw{fuse_viewer::to_live_mesh(live_view)};
        vgp::HybridMeshFrame hybrid_frame;
        hybrid_frame.extent = extent;
        hybrid_frame.view_proj = view_proj;
        hybrid_frame.light_dir = glm::vec3(0.4f, 0.9f, 0.5f);
        hybrid_frame.flags = opt.lit ? vgp::kHybridMeshLit : 0u;
        hybrid_frame.atlas =
            atlas ? atlas->use(render_frame.number) : VK_NULL_HANDLE;
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

    const vkc::Status present = app.end_frame(render_frame);
    if (!present.ok() && !win::swapchain_stale(present)) {
      std::fprintf(stderr, "end_frame: %s\n", present.message().c_str());
      exit_code = 1;
      break;
    }
    ++tick;
  }

  quit.store(true);  // stop the fuse thread promptly; QuitJoin joins it on exit
  if (const vkc::Status idle = app.wait_idle(); !idle) {
    std::fprintf(stderr, "wait_idle: %s\n", idle.message().c_str());
    exit_code = 1;
  }
  return exit_code;
}

}  // namespace

int main(int argc, char** argv) {
  const vkc::Result<Options> parsed = parse_args(argc, argv);
  if (!parsed) {
    std::fprintf(stderr, "%s\n", parsed.status().message().c_str());
    return 2;
  }
  const Options& opt = parsed.value();

  if (glfwInit() != GLFW_TRUE) {
    std::fprintf(stderr, "glfwInit failed\n");
    return 1;
  }
  glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
  GLFWwindow* window = glfwCreateWindow(opt.window.width, opt.window.height,
                                        "fuse_viewer", nullptr, nullptr);
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
