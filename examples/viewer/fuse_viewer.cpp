// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// fuse_viewer: the live recon -> gfx interop demo. Opens a window and fuses a
// posed Replica RGB-D sequence into a sparse TSDF+colour volume frame by frame
// (volumetric_kit_recon), periodically re-extracting a marching-cubes mesh, and
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
//   * lifetime -- recon rings its output slots and this file releases a
//     generation once every frame that drew it has retired. begin_frame's
//     per-slot fence wait is the only completion signal gfx exposes, and no
//     semaphore may cross this seam (a cross-library GPU wait deadlocks
//     against a swapchain rebuild).
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
// sensor resolution, the rest fall back to fused voxel colour. --no-texture
// disables it (an A/B against the pure vertex-colour path).
//
// Two Dear ImGui panels (gfx's ui tier) show where the time and the memory go:
// Performance (the renderer's fps + GPU spans, with recon's per-frame fuse
// stages appended) and Reconstruction (mesh/volume counters + recon's device
// memory). --no-overlay turns both off.
//
//   fuse_viewer <scene_dir> [--voxel 0.02] [--trunc m] [--max-frames N]
//               [--remesh-every N] [--width 1280] [--height 720] [--unlit]
//               [--no-texture] [--preload] [--no-overlay] [--validation]

#include <algorithm>
#include <atomic>
#include <chrono>
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
// Not for to_gfx_mesh -- seam B deleted this file's only call to it. Kept for
// the vertex-layout static_asserts it carries, which matter MORE without the
// host copy that used to justify them: gfx now reads recon's arena in place
// through its own attribute offsets, so a divergence between recon's
// mesh::Vertex and gfx's assets::Vertex is no longer a mis-sized memcpy but
// every attribute silently read from the wrong bytes. (fuse_render.cpp
// includes the same header and does still call the converter, so the
// assertions fire in that TU too; they are kept here because this is the TU
// that binds the arena, which is where a divergence is silent. What they pin
// is the two *structs* -- that gfx's vertex-input description reads those
// offsets with that stride is asserted nowhere, and cannot be from here.)
#include "recon_gfx_bridge.hpp"
#include "shared_device.hpp"
#include "stage_metrics.hpp"  // fuse_viewer::to_sections

#include "volumetric_kit/recon/core/allocator.hpp"
#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/mesh/device_mesh.hpp"
#include "volumetric_kit/recon/mesh/marching_cubes.hpp"
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
#include "volumetric_kit/gfx/pipelines/hybrid_mesh_pipeline.hpp"
#include "volumetric_kit/gfx/pipelines/live_mesh.hpp"
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
  // Derived from `voxel` when left at 0 (see parse_args): a truncation band
  // fixed in metres is a band whose width *in voxels* changes with --voxel,
  // which silently degrades the reconstruction in both directions.
  float trunc = 0.0f;
  float max_depth = 8.0f;
  int max_frames = 400;
  int remesh_every = 1;  // re-extract + re-upload every N fused frames
  int width = 1280;
  int height = 720;
  bool lit = true;
  bool texture = true;      // project each keyframe onto the growing mesh (uv0)
  bool preload = false;     // decode every frame up front (RAM for decode time)
  bool overlay = true;      // Dear ImGui performance + reconstruction panels
  bool validation = false;  // Vulkan validation layer on the shared device
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
    } else if (a == "--trunc") {
      const char* x = v();
      if (!x) return false;
      o.trunc = std::strtof(x, nullptr);
    } else if (a == "--max-depth") {
      const char* x = v();
      if (!x) return false;
      o.max_depth = std::strtof(x, nullptr);
    } else if (a == "--max-frames") {
      const char* x = v();
      if (!x) return false;
      o.max_frames = std::atoi(x);
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
    } else if (a == "--share-vertices") {
      o.share_vertices = true;
    } else if (a == "--preload") {
      o.preload = true;
    } else if (a == "--no-overlay") {
      o.overlay = false;
    } else if (a == "--validation") {
      o.validation = true;
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
                 "usage: fuse_viewer <scene_dir> [--voxel m] [--trunc m] "
                 "[--max-frames n] "
                 "[--remesh-every n] [--unlit] "
                 "[--no-texture] [--share-vertices] [--preload] [--no-overlay] "
                 "[--validation]\n");
    return false;
  }
  // strtof parses "nan"/"inf" without error, and a non-finite knob slips the
  // downstream guards (NaN compares false to every bound) to reach the grid
  // params and the GPU -- a silent, degenerate reconstruction. Reject up front.
  if (!std::isfinite(o.voxel) || o.voxel <= 0.0f) {
    std::fprintf(stderr, "--voxel must be finite and > 0\n");
    return false;
  }
  // Default the band to 4 voxels, as fuse_replica does -- see the same note in
  // fuse_render: a band fixed in metres makes --voxel silently change the
  // reconstruction's quality, and makes the same flag value mean different
  // things across these examples.
  if (o.trunc <= 0.0f) o.trunc = 4.0f * o.voxel;
  if (!std::isfinite(o.trunc) || o.trunc <= 0.0f) {
    std::fprintf(stderr, "--trunc must be finite and > 0\n");
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
// metrics. gfx's FrameMetrics carries one memory pair (its own allocator's),
// so recon's device memory + the volume/mesh counters live in their own panel
// rather than being squeezed into that contract.
struct ReconstructionPanel {
  std::size_t fused_frames = 0;
  std::size_t total_frames = 0;
  std::size_t vertices = 0;
  std::size_t triangles = 0;
  std::uint64_t mesh_version = 0;
  std::int32_t map_buckets = 0;
  std::int32_t map_blocks = 0;
  /// Fraction of the block heap in use, from VoxelHashMap::load_factor -- a
  /// 4-byte read of the mapped heap counter, not the diagnostics scan.
  /// **Negative when that read failed**, which the panel draws as `unavailable`
  /// rather than as a low fraction: this is the figure a reader checks to
  /// decide whether the scan is still taking geometry in, so the one answer it
  /// must never give is a calm-looking number it does not have.
  float map_load_factor = 0.0f;
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

// A filled bar carrying its own ceiling.
//
// The point of a gauge over the two numbers it replaces: a threshold a reader
// has to know about is a threshold they will miss. A scan on this pipeline can
// sit deep in the band where allocation has stopped with every other figure
// looking healthy -- which is exactly what happened on an iPad, for half a
// scan, behind an `errors 0` banner. `warn_at` is what makes that state read as
// a state rather than as one more number.
void gauge(const char* label, double value, double capacity, double warn_at,
           const char* overlay) {
  const double fraction =
      capacity > 0.0 ? std::min(value / capacity, 1.0) : 0.0;
  const bool hot = fraction >= warn_at;
  if (hot) {
    // Semantic, not decorative: the bar changes colour only when the reader is
    // meant to act.
    ImGui::PushStyleColor(ImGuiCol_PlotHistogram,
                          ImVec4(0.72f, 0.39f, 0.18f, 1.0f));
  }
  ImGui::Text("%s", label);
  ImGui::SameLine(110.0f);
  ImGui::ProgressBar(static_cast<float>(fraction), ImVec2(-1.0f, 0.0f),
                     overlay);
  if (hot) {
    ImGui::PopStyleColor();
  }
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
  // The two fitted output buffers' occupancy, each against its OWN capacity.
  // They are reported separately because MarchingCubesConfig::share_vertices
  // decouples them -- pairing the vertex arena's bytes with the index run's
  // fill ratio would describe a buffer it is not measuring -- and because the
  // arena is the one that dominates the MiB.
  //
  // The MiB is the whole ring (both buffers, every slot), so at
  // slot_count = 3 it is roughly three times the buffers those fractions are
  // over. The dispatch count matters on its own: a run that keeps reporting 2
  // is one whose planner is not tracking the surface, and the ..dispatch row
  // cannot say so because it sums both attempts into one span.
  {
    // 0.9 because past it the next growth step is a refit: the arenas are
    // grow-only, so a full one costs a second dispatch, which is the `dispatch`
    // count below going to 2.
    char overlay[64];
    std::snprintf(overlay, sizeof(overlay), "%u / %u verts",
                  panel.extract.emitted_vertices,
                  panel.extract.vertex_capacity);
    gauge("  arena", panel.extract.emitted_vertices,
          panel.extract.vertex_capacity, 0.9, overlay);
    // Its own bar, against its own capacity, for the reason above: with
    // share_vertices these two fractions come apart, and the one that forces
    // the refit is whichever fills first.
    std::snprintf(overlay, sizeof(overlay), "%u / %u tris",
                  panel.extract.emitted_triangles,
                  panel.extract.triangle_capacity);
    gauge("  indices", panel.extract.emitted_triangles,
          panel.extract.triangle_capacity, 0.9, overlay);
  }
  ImGui::Text("  output %.0f MiB (ring), %u dispatch%s",
              to_mebibytes(panel.extract.arena_bytes), panel.extract.dispatches,
              panel.extract.dispatches == 1 ? "" : "es");
  {
    // Occupancy against the block heap num_blocks sizes -- the figure that says
    // a scan has stopped taking in new geometry, which the bare capacity this
    // row used to print cannot. VoxelHashMap::load_factor is a 4-byte read of
    // the host-mapped heap counter, added on 2026-08-08 as the constant-time
    // reading a per-frame caller can afford; the diagnostics scan it is often
    // confused with walks every slot on the host and cannot run per frame.
    //
    // The threshold is the map's own (see kGrowThreshold), read once for both
    // the colour and the words: two literals here drifted apart from the
    // library's guidance and from each other.
    const double warn = vol::VoxelHashMap::kGrowThreshold;
    char overlay[80];
    if (panel.map_load_factor < 0.0f) {
      // Negative means the read failed (the fuse thread said why on stderr).
      // Drawn as full rather than as a fraction: an unknown occupancy is not a
      // low one, and this bar is read to decide whether to keep scanning.
      std::snprintf(overlay, sizeof(overlay), "unavailable, %d blocks",
                    panel.map_blocks);
      gauge("map", 1.0, 1.0, warn, overlay);
    } else {
      std::snprintf(overlay, sizeof(overlay), "%.1f%% of %d blocks%s",
                    100.0 * panel.map_load_factor, panel.map_blocks,
                    panel.map_load_factor >= warn ? "  -- grow now" : "");
      gauge("map", panel.map_load_factor, 1.0, warn, overlay);
    }
    // num_blocks is bucket_size * num_buckets, so the bucket count is what a
    // resize doubles -- the memory story behind the fraction above.
    ImGui::Text("  %d buckets", panel.map_buckets);
  }
  ImGui::Separator();
  // recon's device memory: its own VMA allocator's share of the device. On a
  // shared/adopted device each library allocates separately, so this is recon's
  // footprint, not the process total.
  for (std::uint32_t heap = 0; heap < panel.recon_memory.heap_count; ++heap) {
    const vr::HeapStats& stats = panel.recon_memory.heaps[heap];
    if (stats.budget_bytes == 0 && stats.usage_bytes == 0) continue;
    char overlay[64];
    std::snprintf(overlay, sizeof(overlay), "%.0f / %.0f MiB",
                  to_mebibytes(stats.usage_bytes),
                  to_mebibytes(stats.budget_bytes));
    char label[32];
    std::snprintf(label, sizeof(label), "recon heap %u", heap);
    gauge(label, static_cast<double>(stats.usage_bytes),
          static_cast<double>(stats.budget_bytes), 0.9, overlay);
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
  // --- One VkDevice, adopted by both libraries ------------------------------
  // Declared first so it outlives every wrapper that borrows it: the gfx app
  // and recon's device/allocator below hold raw handles into this, and both
  // must be gone before the instance and device are destroyed.
  fuse_viewer::SharedDeviceConfig shared_config;
  shared_config.enable_validation = opt.validation;
  fuse_viewer::SharedDevice shared;
  if (!fuse_viewer::build_shared_device(window, shared_config, shared)) {
    return 1;
  }

  vg::app::WindowedAppConfig config;
  config.app_name = "fuse_viewer";
  config.swapchain.extent = window_extent(window);
  config.swapchain.depth_format = VK_FORMAT_D32_SFLOAT;
  config.frames_in_flight = 2;
  // The surface already exists -- picking a present-capable device required
  // one -- so the factory hands over the one the bootstrap made rather than
  // creating a second. Ownership transfers with it.
  auto app_r = vg::app::WindowedApp::adopt(
      fuse_viewer::gfx_adopt_payload(shared), config,
      [&shared](VkInstance instance) -> vg::Result<VkSurfaceKHR> {
        // adopt calls this with the instance from the payload, so this can only
        // trip if the two ever stop coming from the same SharedDevice -- at
        // which point the surface would belong to a different instance than the
        // swapchain built on it.
        if (instance != shared.instance) {
          return vg::Status::invalid_argument(
              "surface factory: the app adopted a different VkInstance than "
              "the bootstrap created the surface on");
        }
        return shared.release_surface();
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
      vr::Device::adopt(fuse_viewer::recon_adopt_payload(shared), {});
  if (!recon_device_result) {
    std::fprintf(stderr, "recon Device::adopt: %s\n",
                 recon_device_result.status().message().c_str());
    return 1;
  }
  auto recon_allocator_result =
      vr::Allocator::create(shared.instance, recon_device_result.value());
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
  mc_config.queue_families[0] = shared.compute_family;
  mc_config.queue_families[1] = shared.graphics_family;
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
  // Performance panel reports the mesh/atlas/swapchain footprint. recon runs
  // its own VMA allocator over the same device (independent bookkeeping, not a
  // second device), reported separately by the Reconstruction panel below, so
  // the two figures partition the shared device's memory rather than
  // double-counting it. `app` outlives the profiler (declared before it),
  // which set_memory_source requires.
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
    // _SRGB: the atlas holds canonical-encoded 8-bit camera pixels, so the
    // sampler decodes and filters in linear for free -- and the swapchain
    // (already _SRGB) applies the one encode at the end. See the 2026-08-02
    // color-space decision.
    upload_desc.format = VK_FORMAT_R8G8B8A8_SRGB;
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
  // loop, starving the GPU). It publishes the newest coloured mesh + the
  // trajectory; the render thread only uploads + draws, so the window stays at
  // full frame rate. Whether the two threads' *submits* also overlap is the
  // shared device's queue plan: they do under kTwoQueuesOneFamily and
  // kTwoFamilies (a queue each), and serialize under kSharedQueue (one queue
  // behind a mutex) -- which is why the bootstrap prefers a second family over
  // sharing a queue. recon's device wrapper is used solely on this thread
  // (submit_single_time is not thread-safe: it owns one command pool).
  // -------------------------------------------
  std::mutex share_mtx;
  // The extractor's own buffers, borrowed -- handles and counts, not bytes.
  // Its presence IS the "uncollected" flag the fuse thread tests before
  // extracting again: a published mesh nobody took still holds a ring slot, and
  // the fuse thread cannot free that slot itself (release_through is the
  // consumer's monotonic high-water mark, so releasing this generation would
  // retire every older one with it -- including the ones in-flight frames are
  // drawing out of).
  std::optional<rmesh::DeviceMesh> pending_mesh;
  std::vector<std::uint8_t> pending_atlas;  // its keyframe RGBA8 (empty = none)
  std::uint64_t published_version = 0;
  // The render thread's release mark, applied to the extractor BY THE FUSE
  // THREAD at the top of its next remesh. MarchingCubes::release_through is not
  // atomic and its header makes serializing it against the extracting thread
  // the caller's job, so calling it from the render thread would race
  // extract_device; deferring costs at most one remesh of latency and keeps
  // recon's extractor touched by exactly one thread.
  std::uint64_t shared_released_through = 0;
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
  float shared_map_load_factor = 0.0f;
  std::uint64_t shared_preloaded_bytes = 0;
  rmesh::ExtractTimings shared_extract;

  std::thread fuse_thread([&]() {
    // Any throw escaping this thread function (e.g. a decode/allocation
    // bad_alloc) would call std::terminate; contain it so shutdown stays clean.
    try {
      // Scratch for this thread only; copied under share_mtx once per frame.
      vr::StageMetrics fuse_stages;
      // The remesh-only rows -- extract and its breakdown, texture, atlas pack
      // -- measured here rather than straight into `fuse_stages`, and merged in
      // on every frame whether or not this one remeshed.
      //
      // The gate below fires only on the fused frames the renderer has kept up
      // with, and fusion routinely outruns it (a preloaded run fuses several
      // frames per presented frame), so measuring them into `fuse_stages`
      // directly left them reading 0.00 on most samples -- on the very
      // instrument this repo's history credits with catching the arena-alloc
      // and neighbour-table regressions, and against its own rule that a row
      // which is usually zero is worse than an absent one. Held, they describe
      // the newest remesh, exactly as `extract_stats` beside them does; the
      // panel's `fuse ms/frame` therefore reads as the cost of a fused frame
      // that also remeshed.
      vr::StageMetrics remesh_stages;
      // Held across frames so the panel keeps showing the newest remesh's
      // sizes between remeshes, rather than blanking to zero.
      rmesh::ExtractTimings extract_stats;
      // Texture `device_mesh` with one keyframe, then publish it plus that
      // keyframe's colour image as the atlas its uv0 index into. On any
      // texturing failure -- or when --no-texture -- the atlas stays empty and
      // the render thread binds the white dummy (every triangle falls back to
      // fused voxel colour).
      //
      // The mesh and its atlas are ONE value, published and taken together.
      // uv0 is a normalized coordinate into the image of the camera that
      // textured it, and every texture() call rewrites every vertex's uv0
      // against the *current* frame -- so drawing a mesh against a later
      // frame's image samples the wrong place on every textured triangle.
      //
      // Nothing is copied to the host: what crosses is the DeviceMesh, five
      // words of handles and counts. That is the whole of seam B on this side.
      auto publish = [&](const rmesh::DeviceMesh& device_mesh,
                         const float* depth,
                         const vr::DepthCameraParams& depth_camera,
                         const std::vector<std::uint32_t>& color) {
        std::vector<std::uint8_t> atlas;
        if (texturer && depth != nullptr && !device_mesh.empty()) {
          // Textures the extractor's buffers in place -- no upload, no
          // readback; the geometry has not left the device since it was meshed.
          //
          // The tier opens its own "texture" row, so there is no StageScope
          // here: rows accumulate by name, and wrapping the call as well would
          // count the host span twice while adding nothing. What the tier's row
          // has that a wrapper's cannot is the device half.
          const vr::Status texture_status = texturer->texture(
              device_mesh, depth, depth_camera, 0.02f, &remesh_stages);
          if (texture_status.ok()) {
            // Its own row, not folded into "texture": repacking a full sensor
            // frame to RGBA8 is host work of the same order as the texturing
            // dispatch, so charging it to the GPU pass would misattribute it.
            vr::StageScope scope(remesh_stages, "atlas pack");
            atlas = vr_example::pack_color_rgba8(color);
          } else {
            std::fprintf(stderr, "fuse_viewer: texture: %s\n",
                         texture_status.message().c_str());
          }
        }
        std::lock_guard<std::mutex> lock(share_mtx);
        pending_mesh = device_mesh;
        pending_atlas = std::move(atlas);
        ++published_version;
      };
      // Hand the render thread's release mark to recon -- on THIS thread, for
      // the reason given on shared_released_through -- and report whether this
      // thread may publish again.
      //
      // The release runs BEFORE the extract it makes room for. After it, the
      // ring would sit permanently one slot shallower than its depth, and an
      // extract that failed for want of a slot would skip the very release that
      // would have supplied one.
      auto release_and_may_publish = [&]() {
        std::uint64_t mark = 0;
        bool uncollected = false;
        {
          std::lock_guard<std::mutex> lock(share_mtx);
          mark = shared_released_through;
          uncollected = pending_mesh.has_value();
        }
        if (mark != 0) extractor.release_through(mark);
        return !uncollected;
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
             {"frame", "allocate", "resize", "integrate", "  ..active set",
              "extract", "  ..compact", "  ..inputs", "  ..arena alloc",
              "  ..descriptors", "  ..dispatch", "  ..readback", "texture",
              "atlas pack"}) {
          fuse_stages.seed(stage);
        }
        // A preload cache hit, else a disk read + JPEG/PNG decode (the CPU
        // cost the preload exists to hoist out of this loop). Timed either way,
        // so --preload's effect is visible as this row collapsing to ~0.
        auto frame_result = [&]() {
          vr::StageScope scope(fuse_stages, "frame");
          return dataset.frame(i);
        }();
        if (!frame_result) {
          // Only NotFound means "ran past the frames on disk". Every other
          // failure in this loop already prints (allocate / resize /
          // integrate), so swallowing a decode error here was the one way to
          // stop early with nothing on stderr -- the panel just froze at
          // "fused N / M", indistinguishable from a normal finish.
          if (frame_result.status().domain() != vr::Status::Code::NotFound) {
            std::fprintf(stderr, "frame %zu failed to load: %s\n", i,
                         frame_result.status().message().c_str());
          }
          break;
        }
        vr_example::FrameView view = std::move(frame_result).value();
        const vr_example::RgbdFrame& frame = *view;
        {
          std::lock_guard<std::mutex> lock(share_mtx);
          shared_poses.push_back(frame.cam_to_world);
        }
        const vr::DepthCameraParams depth_camera =
            vr_example::make_depth_camera(cam, frame.cam_to_world,
                                          opt.max_depth);
        vr::ColorCameraParams color_camera{};
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
          // The tier fills "allocate" itself -- every retry round, host and
          // device, under the one name. So no StageScope around the loop: rows
          // accumulate by name, and one here would add each round's host span a
          // second time (up to 5x) and inflate "fuse ms/frame" with it. What
          // the loop adds beyond the tier is the resize, which gets its own row
          // below rather than being folded into a stage whose device share it
          // would sink on exactly the frames that overflow.
          for (int attempt = 0; attempt < 5; ++attempt) {
            vol::AllocFailures failures;
            auto failed = volume.map().allocate_from_depth(
                frame.depth.data(), depth_camera, &failures, &fuse_stages);
            if (!failed) {
              std::fprintf(stderr, "fuse_viewer: allocate (frame %zu): %s\n", i,
                           failed.status().message().c_str());
              break;
            }
            if (failed.value() == 0) {
              allocated = true;
              break;
            }
            // Retry, don't grow, when the residue is only lost bucket-lock
            // races: depth allocation is the map's most contended entry point
            // and can leave failures on a table that is nowhere near full,
            // where doubling every attribute array is a large and pointless
            // cost -- and here it would also stall the live window.
            if (!failures.capacity_limited()) continue;
            vr::StageScope resize_span(fuse_stages, "resize");
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
        // Again the tier's own row rather than a wrapper's, and this one also
        // decomposes: the active-set compaction is a second dispatch inside the
        // stage and reports itself as "  ..active set" beneath it.
        const vr::Status integrate_status = integrator.integrate(
            volume, frame.depth.data(), depth_camera, 20.0f,
            rtsdf::IntegrationMode::Classic, &color_frame, &fuse_stages);
        if (!integrate_status.ok()) {
          std::fprintf(stderr, "fuse_viewer: integrate (frame %zu): %s\n", i,
                       integrate_status.message().c_str());
          break;
        }
        fused_count.store(i + 1);
        // Do not publish over a mesh the renderer has not collected: it still
        // holds a ring slot, and this thread cannot free that slot (see
        // pending_mesh). The extract is simply not run -- its result would have
        // been discarded anyway, so the skip costs nothing and saves the
        // dispatch. Fusion routinely outruns the render loop here (a preloaded
        // run remeshes several times per presented frame), so this is the
        // common path, not a corner.
        if ((i % static_cast<std::size_t>(opt.remesh_every)) == 0 &&
            release_and_may_publish()) {
          remesh_stages.clear();
          rmesh::ExtractTimings extract_timings;
          vr::Result<rmesh::DeviceMesh> extracted = [&]() {
            vr::StageScope scope(remesh_stages, "extract");
            return extractor.extract_device(volume, 0.0f, &extract_timings);
          }();
          // Break the extract row down in place. The phases sum to the
          // `extract` row above rather than adding to it, so they carry
          // StageMetrics::kBreakdownPrefix -- which is what makes the table
          // read as a hierarchy *and* keeps total_cpu_ms from counting the
          // extract twice.
          remesh_stages.add_cpu("  ..compact", extract_timings.compact_ms);
          remesh_stages.add_cpu("  ..inputs", extract_timings.input_upload_ms);
          remesh_stages.add_cpu("  ..arena alloc",
                                extract_timings.arena_alloc_ms);
          remesh_stages.add_cpu("  ..descriptors",
                                extract_timings.descriptor_ms);
          remesh_stages.add_cpu("  ..dispatch", extract_timings.dispatch_ms);
          remesh_stages.add_cpu("  ..readback", extract_timings.readback_ms);
          extract_stats = extract_timings;
          // Published even when it meshed nothing, which the host-mesh path
          // did not need to do. An empty extract still claims and stamps a ring
          // slot, so a mesh that never reaches a consumer is a slot nothing can
          // ever release -- slot_count of those and every later extract is
          // refused, permanently. It draws nothing either way: recon resets the
          // command, so indexCount is 0.
          if (extracted) {
            publish(extracted.value(), frame.depth.data(), depth_camera,
                    frame.color);
          } else {
            // Every other stage in this loop reports its failure; this one used
            // to be silent, which under seam B reads as a frozen mesh with a
            // healthy frame counter beside it.
            std::fprintf(stderr, "fuse_viewer: extract (frame %zu): %s\n", i,
                         extracted.status().message().c_str());
          }
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
          const vr::MemoryStats recon_memory = rallocator.memory_stats();
          // Read out here beside memory_stats and for the same reason: it is a
          // mapped-memory read behind a Result whose Status carries a string,
          // and the render thread is waiting on this lock. Constant-time, so
          // sampling it every fused frame is affordable (2026-08-08); it fails
          // only on a moved-from map, and the negative it publishes then is not
          // defensiveness for its own sake: a gauge whose whole claim is "you
          // will not have to already know the threshold" cannot answer an
          // unknown with the last good fraction, which is the one reading that
          // looks exactly like a healthy map.
          const vr::Result<float> lf = volume.map().load_factor();
          if (!lf) {
            std::fprintf(stderr, "fuse_viewer: load_factor (frame %zu): %s\n",
                         i, lf.status().message().c_str());
          }
          const float load_factor = lf ? lf.value() : -1.0f;
          std::lock_guard<std::mutex> lock(share_mtx);
          shared_fuse_stages = fuse_viewer::to_sections(fuse_stages);
          // Fusion cost, so the dataset read is excluded: it is dataloading,
          // not fusion, and while streaming it dwarfs the rest (~10 ms of
          // JPEG/PNG decode). It stays visible as its own `frame` row.
          shared_fuse_ms = fuse_stages.total_cpu_ms(/*exclude=*/"frame");
          shared_recon_memory = recon_memory;
          shared_map_buckets = volume.grid().num_buckets;
          shared_map_blocks = volume.grid().num_blocks;
          shared_map_load_factor = load_factor;
          shared_extract = extract_stats;
        }
        // Retain this frame (the newest keyframe) for the final extract below.
        last_frame = std::move(view);
      }
      // Skip the full-volume final extract when the user has already quit, so
      // the join at shutdown does not stall on a whole marching-cubes pass.
      if (!quit.load()) {
        // Unlike a remesh inside the loop, this one is worth waiting for: it is
        // the complete surface, and there is no later extract to supersede it.
        // So rather than skip on an uncollected publish, give the render thread
        // a moment to take it -- it collects on every iteration, so this is
        // normally one frame. Bounded, because a window the compositor has
        // stopped scheduling would otherwise hold the shutdown join open.
        for (int wait = 0; wait < 500 && !quit.load(); ++wait) {
          {
            std::lock_guard<std::mutex> lock(share_mtx);
            if (!pending_mesh) break;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
      }
      // Re-checked after the wait, which can span a whole second: `quit` is
      // what says the window is gone, and running a full marching-cubes pass
      // into a closed window is exactly what the guard above exists to avoid.
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
        if (!release_and_may_publish()) {
          std::fprintf(stderr,
                       "fuse_viewer: the renderer never collected the last "
                       "mesh (window hidden, or drawing stopped); extracting "
                       "the final mesh anyway\n");
        }
        // Measured like any other extract, and *published* like one below.
        // Without this the panel's arena row keeps describing the last in-loop
        // extract while the mesh row describes this one -- and they genuinely
        // differ, since fusing the remaining frames refines the field and the
        // zero-crossing set is not monotonic (400-frame room0: 330 394
        // triangles at the last remesh, 330 389 here). Two halves of one
        // read-out taken from different extracts, frozen that way for the whole
        // replay, is exactly the trap the ExtractTimings rows exist to avoid.
        rmesh::ExtractTimings final_timings;
        auto m = extractor.extract_device(volume, 0.0f, &final_timings);
        if (m) {
          {
            std::lock_guard<std::mutex> lock(share_mtx);
            shared_extract = final_timings;
          }
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
            publish(m.value(), nullptr, vr::DepthCameraParams{}, kNoColor);
          }
        } else {
          std::fprintf(stderr, "fuse_viewer: final extract: %s\n",
                       m.status().message().c_str());
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
  std::vector<std::shared_ptr<AtlasVersion>> slot_atlas(
      config.frames_in_flight);
  // What the draw binds: recon's own buffers, borrowed. Committed in lockstep
  // with `current_atlas` below, since the mesh's uv0 index into that image.
  // Its `generation` is also the mesh version the panel reports: recon numbers
  // extracts from 1, so a live_view that has never been committed reads 0, and
  // a second hand-maintained counter beside it could only drift.
  rmesh::DeviceMesh live_view;
  // A taken mesh + its keyframe pixels, held here until BOTH can be committed.
  // Declared outside the loop on purpose: an atlas upload that fails must be
  // RETRIED, not dropped. A generation this thread took but never committed
  // keeps its ring slot until a *newer committed* generation lets the release
  // mark sweep past it (the mark is monotone, so it cannot skip one), and the
  // commit that would produce one is exactly what just failed -- so two dropped
  // takes fill the ring and recon refuses every later extract, permanently.
  // Retrying bounds the uncommitted set at one, and the take below is gated on
  // this being empty so a second cannot start.
  rmesh::DeviceMesh taken;
  std::vector<std::uint8_t> taken_atlas_px;
  std::uint64_t taken_version = 0;
  // The recon generation each in-flight frame drew, read as a SET: what may be
  // released is everything older than the *oldest* entry, not the entry
  // belonging to the frame that just retired. One generation is normally drawn
  // by several consecutive frames (the mesh only changes when fusion publishes
  // a new one), so the retired frame's generation is often still being read by
  // a newer frame -- releasing on that would hand recon a slot a live
  // vkCmdDrawIndexedIndirect is reading, and a grow frees its buffers outright.
  // begin_frame's per-slot fence wait is the only completion signal gfx gives,
  // and it says a *frame* finished, not a generation.
  std::vector<std::uint64_t> frame_generations(config.frames_in_flight, 0);
  // The newest generation taken, drawn or not: every one this thread accepts
  // becomes its to release, or the ring drains with nothing able to refill it.
  // The fallback when no frame in flight holds a generation at all.
  std::uint64_t newest_taken_generation = 0;
  // Latched when a published mesh cannot be bound as geometry. That is a
  // configuration fault, not a transient -- the usage bits and queue families
  // come from mc_config above -- so collecting the ones that follow would only
  // walk the ring to exhaustion one undrawable generation at a time.
  bool mesh_unusable = false;
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

    // --- Retire, then take: both under ONE lock, retire first ----------------
    //
    // begin_frame fence-waited this slot, so the frame that last used it has
    // completed and its entry may be dropped. What remains in the array is
    // exactly the generations frames still in flight are reading, so everything
    // strictly below their minimum is finished everywhere.
    //
    // The order and the single lock are the correctness argument, not tidiness.
    // Taking is what frees the fuse thread to extract again, and the fuse
    // thread reads the release mark in the same breath as it tests for an
    // uncollected mesh -- so a take published *ahead* of the mark for the same
    // frame lets it run on a mark one iteration stale, ask for a slot beyond
    // the ring's depth, and be refused. That is not hypothetical: it cost 25
    // refused extracts in a 200-frame preloaded run, and it is invisible
    // whenever fusion is slower than the render loop.
    //
    // The panel snapshot rides in the same section, and that is not tidiness
    // either: the arena/dispatch rows describe the extract that produced the
    // mesh beside them, so reading them before begin_frame -- whose per-slot
    // fence wait can span a whole frame, during which the fuse thread routinely
    // publishes -- printed one generation's counts above another's arena.
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
      recon_panel.extract = shared_extract;

      frame_generations[render_frame.slot] = 0;
      std::uint64_t oldest_in_flight = 0;
      for (const std::uint64_t g : frame_generations) {
        if (g != 0 && (oldest_in_flight == 0 || g < oldest_in_flight))
          oldest_in_flight = g;
      }
      // No other frame in flight holds one, so the floor is what *this* frame
      // is about to draw -- releasing that would hand recon the slot under a
      // live draw. Only when nothing has been committed at all (generation 0,
      // since recon numbers extracts from 1) does everything taken so far
      // become releasable, which is the path that drains the ring when takes
      // are accepted but never drawn.
      if (oldest_in_flight == 0) oldest_in_flight = live_view.generation;
      // Generations count from 1, so there is nothing below the first.
      shared_released_through =
          oldest_in_flight > 0 ? oldest_in_flight - 1 : newest_taken_generation;
      // Taking frees the fuse thread whether or not the mesh proves drawable
      // below -- except once latched, where declining to take is also what
      // stops the extracts that would follow, and while one is still awaiting
      // its atlas (see `taken`).
      if (pending_mesh && !mesh_unusable && taken_version == 0) {
        taken = *pending_mesh;
        pending_mesh.reset();
        taken_atlas_px = std::move(pending_atlas);
        pending_atlas.clear();  // moved-from vector -> defined empty state
        taken_version = published_version;
        // An accepted generation is this thread's to release whether or not it
        // is ever drawn, so this is recorded before anything can reject it.
        newest_taken_generation = taken.generation;
      }
    }
    const std::size_t done_frames = fused_count.load();
    const bool done = fusing_done.load();
    recon_panel.fused_frames = done_frames;
    recon_panel.total_frames = frame_count;

    // Commit the newly taken mesh + its atlas. The mesh itself needs no upload
    // -- it is already in device buffers gfx binds directly -- so what remains
    // is the keyframe image, which genuinely is host pixels.
    {
      // Scoped over the whole check, not just the upload, so the row reports
      // ~0 on a frame with no new mesh instead of vanishing from the table.
      vg::Profiler::Scope upload_scope = profiler.cpu_scope("atlas upload");
      if (taken_version != 0 && taken.empty()) {
        // An empty extract draws nothing, so there is nothing to bind and
        // nothing to sample -- and, crucially, nothing here is a fault. recon
        // publishes one for slot hygiene and names its buffers as it found
        // them, so a slot that was never sized carries NULL HANDLES beside
        // empty() being true; that is the documented "draw nothing" case. It
        // must therefore be committed without going through the bindable check
        // below, which folds valid() in with the usage bits and latches: recon
        // reaching its own legal empty path (a first extract on an empty map,
        // which --max-frames 0 or a frame-0 load failure produces) would
        // otherwise stop this viewer drawing and extracting for good.
        //
        // Committed, not held back: `live_view` is what parks a generation for
        // release below, so keeping the previous mesh here would strand this
        // slot until some later generation swept past it. The window blanking
        // is the honest read -- recon meshed no surface. `current_atlas` is
        // left alone rather than reset to white; with no geometry, nothing
        // samples it, and the coherence rule binds only what is drawn.
        live_view = taken;
        taken = rmesh::DeviceMesh{};
        taken_atlas_px.clear();
        taken_version = 0;
      } else if (taken_version != 0) {
        // Verified, not assumed. recon reports the usage its buffers were
        // created with -- and their sharing mode -- precisely because Vulkan
        // cannot be asked, and binding one that lacks a usage bit is a
        // validation-layer-only diagnostic: undefined behaviour with layers
        // off, which is the shipping configuration.
        //
        // The sharing mode is the term that can actually vary, and the one with
        // most at stake: reading an EXCLUSIVE buffer from a family that does
        // not own it is undefined outright, and on Apple -- where Metal has no
        // queue-ownership concept -- undefined in the way that appears to work.
        // Checked only where the families really differ, since recon collapses
        // the pair to EXCLUSIVE when they are one family and that is correct.
        const bool cross_family =
            shared.graphics_family != shared.compute_family;
        const bool sharing_ok =
            !cross_family || taken.sharing_mode == VK_SHARING_MODE_CONCURRENT;
        const bool bindable =
            taken.valid() && sharing_ok &&
            (taken.vertex_usage & VK_BUFFER_USAGE_VERTEX_BUFFER_BIT) != 0 &&
            (taken.index_usage & VK_BUFFER_USAGE_INDEX_BUFFER_BIT) != 0 &&
            (taken.indirect_usage & VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT) != 0;
        if (!bindable) {
          std::fprintf(
              stderr, "fuse_viewer: %s\n",
              sharing_ok
                  ? "extracted mesh is not bindable as geometry (usage bits or "
                    "handles missing); drawing stops here"
                  : "extracted mesh buffers are VK_SHARING_MODE_EXCLUSIVE but "
                    "recon and gfx are on different queue families; binding "
                    "them would be undefined. Drawing stops here");
          mesh_unusable = true;
        } else {
          // Build this version's atlas (its keyframe image, else the white
          // dummy) and commit it with the mesh, or commit neither -- so the
          // drawn mesh and the atlas its uv0 index into always come from the
          // SAME version. A transient upload failure leaves the previous
          // coherent pair in place rather than binding a new mesh against a
          // stale atlas, and this pair stays in `taken` to be retried on the
          // next frame: dropping it would strand its ring slot (see `taken`).
          std::shared_ptr<AtlasVersion> next =
              taken_atlas_px.empty()
                  ? white_atlas
                  : build_atlas(taken_atlas_px.data(), cam.width, cam.height);
          if (next) {
            live_view = taken;
            current_atlas = std::move(next);
            taken = rmesh::DeviceMesh{};
            taken_atlas_px.clear();
            taken_version = 0;
          }
        }
      }
    }
    // What is on screen, not what has merely been taken: a mesh whose atlas
    // failed to upload is held back above, and reporting it here would show
    // counts for geometry no frame is drawing.
    recon_panel.vertices = live_view.vertex_count;
    recon_panel.triangles = live_view.triangle_count;
    recon_panel.mesh_version = live_view.generation;
    // This slot adopts the current atlas, releasing whatever it bound last
    // frame (safe: begin_frame fence-waited this slot). Holding it per slot
    // keeps a version an in-flight frame bound alive past its replacement --
    // the mesh's own lifetime is the ring below, which recon owns.
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
    // Park the generation this frame reads, for the *next* frame that lands on
    // this slot to retire. Any committed view, drawn or not: an empty mesh
    // still occupies a ring slot, and this is what stops it being released
    // early. Outside the draw branch on purpose -- the release this thread owes
    // recon does not depend on whether a frame drew, and gating it on drawing
    // is how a ring drains to exhaustion with nothing able to refill it.
    frame_generations[render_frame.slot] = live_view.generation;

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
        // TODO(examples): the arena and index run are host-visible mapped
        // memory -- core::storage_buffer allocates every recon buffer that way
        // -- so the vertex-input stage fetches the whole mesh from system RAM
        // on every presented frame (~64 MiB at room0's 991 k vertices).
        // Free on Apple's unified memory, which is what this was measured on,
        // and a per-frame PCIe fetch on a discrete GPU, where it would invert
        // the win this seam exists for. Same shape as the host-visible indirect
        // command's TODO(mesh), and it waits on the same thing: a discrete-GPU
        // consumer to measure a device-local arena + staging against it.
        //
        // MarchingCubesConfig::share_vertices cuts that fetch ~4x by emitting
        // ~4x fewer vertices, and `--share-vertices` now takes it here. It was
        // unavailable to this example while the texture tier refused a shared
        // mesh -- it decided visibility per triangle and wrote uv0 per vertex
        // -- and the per-vertex dispatch removed that refusal, so the ~4x is
        // measurable on the running window rather than only in `fuse_replica`.
        // What still waits on a per-primitive camera id is the packed
        // multi-camera atlas, which is a different problem.
        vgp::LiveMesh live;
        live.vertices = live_view.vertices;
        live.indices = live_view.indices;
        live.indirect = live_view.indirect;
        const vgp::HybridMeshDraw draw{live};
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
