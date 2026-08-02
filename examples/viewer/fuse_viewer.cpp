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
//   fuse_viewer <scene_dir> [--voxel 0.02] [--max-frames N] [--fuse-per-tick K]
//               [--remesh-every N] [--width 1280] [--height 720] [--unlit]
//               [--no-texture]

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
#include <glm/glm.hpp>

#include "dataset.hpp"
#include "example_camera.hpp"  // vr_example::make_depth_camera
#include "image_io.hpp"        // vr_example::pack_color_rgba8
#include "recon_gfx_bridge.hpp"

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
#include "volumetric_kit/gfx/core/render_target.hpp"
#include "volumetric_kit/gfx/core/result.hpp"
#include "volumetric_kit/gfx/core/sampler.hpp"
#include "volumetric_kit/gfx/core/texture_upload.hpp"
#include "volumetric_kit/gfx/core/vulkan.hpp"
#include "volumetric_kit/gfx/pipelines/gpu_mesh.hpp"
#include "volumetric_kit/gfx/pipelines/hybrid_mesh_pipeline.hpp"
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
                 "[--no-texture] [--preload]\n");
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

VkExtent2D window_extent(GLFWwindow* w) {
  int fw = 0, fh = 0;
  glfwGetFramebufferSize(w, &fw, &fh);
  return {static_cast<std::uint32_t>(std::max(1, fw)),
          static_cast<std::uint32_t>(std::max(1, fh))};
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
  auto ri = vr::Instance::create({});
  if (!ri) {
    std::fprintf(stderr, "recon instance: %s\n", ri.status().message().c_str());
    return 1;
  }
  auto rgpu = ri.value().select_physical_device();
  if (!rgpu) {
    std::fprintf(stderr, "recon gpu: %s\n", rgpu.status().message().c_str());
    return 1;
  }
  auto rdev = vr::Device::create(ri.value().handle(), rgpu.value(), {});
  if (!rdev) {
    std::fprintf(stderr, "recon device: %s\n", rdev.status().message().c_str());
    return 1;
  }
  auto ralloc = vr::Allocator::create(ri.value().handle(), rdev.value());
  if (!ralloc) {
    std::fprintf(stderr, "recon allocator: %s\n",
                 ralloc.status().message().c_str());
    return 1;
  }
  vr::Device& rdevice = rdev.value();
  vr::Allocator& rallocator = ralloc.value();

  auto ds = vr_example::ReplicaDataset::open(opt.scene_dir, opt.cam_params);
  if (!ds) {
    std::fprintf(stderr, "dataset: %s\n", ds.status().message().c_str());
    return 1;
  }
  vr_example::ReplicaDataset dataset = std::move(ds).value();
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
  auto vbg_r = vol::VoxelBlockGrid::create(rdevice, rallocator, grid, attrs, 3);
  if (!vbg_r) {
    std::fprintf(stderr, "grid: %s\n", vbg_r.status().message().c_str());
    return 1;
  }
  vol::VoxelBlockGrid volume = std::move(vbg_r).value();
  auto integ_r = rtsdf::TsdfIntegrator::create(rdevice, rallocator);
  if (!integ_r) {
    std::fprintf(stderr, "integrator: %s\n",
                 integ_r.status().message().c_str());
    return 1;
  }
  rtsdf::TsdfIntegrator integrator = std::move(integ_r).value();
  auto mc_r = rmesh::MarchingCubes::create(rdevice, rallocator);
  if (!mc_r) {
    std::fprintf(stderr, "marching cubes: %s\n",
                 mc_r.status().message().c_str());
    return 1;
  }
  rmesh::MarchingCubes extractor = std::move(mc_r).value();
  // Projective texturer (recon device; used, like the integrator/extractor,
  // only on the fuse thread below). Cheap to keep even when --no-texture, but
  // build it only when texturing so the disabled path stays a pure A/B.
  std::optional<rtex::ProjectiveTexturer> texturer;
  if (opt.texture) {
    auto tex_r = rtex::ProjectiveTexturer::create(rdevice, rallocator);
    if (!tex_r) {
      std::fprintf(stderr, "texturer: %s\n", tex_r.status().message().c_str());
      return 1;
    }
    texturer = std::move(tex_r).value();
  }

  const std::size_t frame_count = std::min<std::size_t>(
      dataset.frame_count(),
      static_cast<std::size_t>(std::max(0, opt.max_frames)));
  const float vfov = 2.0f * std::atan(static_cast<float>(cam.height) /
                                      (2.0f * std::max(1.0f, cam.fy)));

  // --- gfx: pipeline + 1x1 white atlas set ----------------------------------
  auto pipe_r = vgp::HybridMeshPipeline::create(app.device().handle(),
                                                app.swapchain().layout());
  if (!pipe_r.ok()) {
    std::fprintf(stderr, "pipeline: %s\n", pipe_r.status().message().c_str());
    return 1;
  }
  vgp::HybridMeshPipeline pipeline = std::move(pipe_r).value();

  // One sampler shared by every atlas version (immutable; outlives them all).
  auto sampler_r = vg::Sampler::create(app.device().handle());
  if (!sampler_r.ok()) {
    std::fprintf(stderr, "sampler: %s\n", sampler_r.status().message().c_str());
    return 1;
  }
  vg::Sampler sampler = std::move(sampler_r).value();

  // Build one atlas bundle (texture + its own pool + a combined-image-sampler
  // set bound to `sampler`) from RGBA8 pixels. Returns nullptr on failure so a
  // transient upload error keeps the previous atlas rather than crashing.
  auto build_atlas = [&](const std::uint8_t* pixels, std::uint32_t w,
                         std::uint32_t h) -> std::shared_ptr<AtlasVersion> {
    vg::ImageUploadDesc adesc;
    adesc.extent = {w, h};
    adesc.format = VK_FORMAT_R8G8B8A8_UNORM;
    adesc.pixels = pixels;
    adesc.size = static_cast<std::size_t>(w) * h * 4;
    auto tex_r = vg::upload_texture(app.device(), app.allocator(), adesc);
    if (!tex_r.ok()) {
      std::fprintf(stderr, "atlas upload: %s\n",
                   tex_r.status().message().c_str());
      return nullptr;
    }
    const VkDescriptorPoolSize ps{VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1};
    auto pool_r = vg::DescriptorPool::create(app.device().handle(), &ps, 1, 1);
    if (!pool_r.ok()) {
      std::fprintf(stderr, "atlas pool: %s\n",
                   pool_r.status().message().c_str());
      return nullptr;
    }
    vg::DescriptorPool apool = std::move(pool_r).value();
    auto set_r = apool.allocate(pipeline.descriptor_set_layout(0));
    if (!set_r.ok()) {
      std::fprintf(stderr, "atlas set: %s\n", set_r.status().message().c_str());
      return nullptr;
    }
    auto av = std::make_shared<AtlasVersion>();
    av->tex = std::move(tex_r).value();
    av->pool = std::move(apool);
    av->set = std::move(set_r).value();
    av->set.write_combined_image_sampler(
        0, av->tex.view(), sampler.handle(),
        VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    return av;
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

  std::thread fuse_thread([&]() {
    // Any throw escaping this thread function (e.g. a decode/allocation
    // bad_alloc) would call std::terminate; contain it so shutdown stays clean.
    try {
      // Texture `rm` with one keyframe, then publish it plus that keyframe's
      // colour image as the atlas its uv0 index into. On any texturing failure
      // -- or when --no-texture -- the atlas stays empty and the render thread
      // binds the white dummy (every triangle falls back to fused voxel
      // colour). uv0 must be filled BEFORE to_gfx_mesh, which copies it across.
      auto publish = [&](rmesh::Mesh&& rm, const float* depth,
                         const vol::DepthCameraParams& dcam,
                         const std::vector<std::uint32_t>& color) {
        std::vector<std::uint8_t> atlas;
        if (texturer && depth != nullptr && !rm.vertices.empty()) {
          const vr::Status ts = texturer->texture(rm, depth, dcam);
          if (ts.ok()) {
            atlas = vr_example::pack_color_rgba8(color);
          } else {
            std::fprintf(stderr, "fuse_viewer: texture: %s\n",
                         ts.message().c_str());
          }
        }
        vg::assets::Mesh gm = fuse_viewer::to_gfx_mesh(rm);
        std::lock_guard<std::mutex> lk(share_mtx);
        pending_mesh = std::move(gm);
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
      }
      // The newest fused frame, retained so the final extract (after the loop)
      // textures with the last keyframe rather than losing its texture.
      std::optional<vr_example::FrameView> last_frame;
      for (std::size_t i = 0; i < frame_count && !quit.load(); ++i) {
        // A preload cache hit, else a disk read + JPEG/PNG decode (the CPU
        // cost the preload exists to hoist out of this loop).
        auto frame_result = dataset.frame(i);
        if (!frame_result) break;
        vr_example::FrameView view = std::move(frame_result).value();
        const vr_example::RgbdFrame& frame = *view;
        {
          std::lock_guard<std::mutex> lk(share_mtx);
          shared_poses.push_back(frame.cam_to_world);
        }
        const vol::DepthCameraParams dcam = vr_example::make_depth_camera(
            cam, frame.cam_to_world, opt.max_depth);
        rtsdf::ColorCameraParams ccam{};
        ccam.fx = cam.fx;
        ccam.fy = cam.fy;
        ccam.cx = cam.cx;
        ccam.cy = cam.cy;
        ccam.width = cam.width;
        ccam.height = cam.height;
        ccam.cam_to_world = frame.cam_to_world;
        const rtsdf::ColorFrame color_frame{frame.color.data(), ccam};

        // Grow the map to fit this frame's surface band; surface any hard
        // failure instead of silently integrating a partially-allocated frame.
        bool allocated = false;
        for (int attempt = 0; attempt < 5; ++attempt) {
          auto failed =
              volume.map().allocate_from_depth(frame.depth.data(), dcam);
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
        if (!allocated) {
          std::fprintf(
              stderr, "fuse_viewer: map overflow at frame %zu; stopping fuse\n",
              i);
          break;
        }
        const vr::Status ist =
            integrator.integrate(volume, frame.depth.data(), dcam, 20.0f,
                                 rtsdf::IntegrationMode::Classic, &color_frame);
        if (!ist.ok()) {
          std::fprintf(stderr, "fuse_viewer: integrate (frame %zu): %s\n", i,
                       ist.message().c_str());
          break;
        }
        fused_count.store(i + 1);
        if ((i % static_cast<std::size_t>(opt.remesh_every)) == 0) {
          auto m = extractor.extract(volume);
          if (m && !m.value().vertices.empty())
            publish(std::move(m).value(), frame.depth.data(), dcam,
                    frame.color);
        }
        // Retain this frame (the newest keyframe) for the final extract below.
        last_frame = std::move(view);
      }
      // Skip the full-volume final extract when the user has already quit, so
      // the join at shutdown does not stall on a whole marching-cubes pass.
      if (!quit.load()) {
        auto m = extractor.extract(volume);  // final mesh
        if (m && !m.value().vertices.empty()) {
          // Texture the final mesh with the last keyframe (its depth camera
          // rebuilt from the retained frame), or leave it untextured if no
          // frame ever fused.
          static const std::vector<std::uint32_t> kNoColor;
          if (last_frame) {
            const vr_example::RgbdFrame& keyframe = **last_frame;
            publish(std::move(m).value(), keyframe.depth.data(),
                    vr_example::make_depth_camera(cam, keyframe.cam_to_world,
                                                  opt.max_depth),
                    keyframe.color);
          } else {
            publish(std::move(m).value(), nullptr, vol::DepthCameraParams{},
                    kNoColor);
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
      std::lock_guard<std::mutex> lk(share_mtx);
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
    }
    const std::size_t done_frames = fused_count.load();
    const bool done = fusing_done.load();

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
    const win::Frame& f = *frame.value();

    // Upload the newest host mesh once when its version changes (not once per
    // ring slot): a shared_ptr keeps a GpuMesh alive until every slot that drew
    // it has cycled (each release gated by begin_frame's per-slot fence wait),
    // so one upload safely feeds all slots.
    if (current_version != 0 && uploaded_version != current_version) {
      // Build this version's atlas first (its keyframe image, else the white
      // dummy), then upload the mesh, and commit both together -- so the drawn
      // mesh and the atlas its uv0 index into always come from the SAME
      // version. A transient atlas- or mesh-upload failure leaves the previous
      // coherent pair in place and retries next frame (the viewer keeps
      // drawing), rather than binding a new mesh against a stale atlas.
      std::shared_ptr<AtlasVersion> next =
          current_atlas_px.empty()
              ? white_atlas
              : build_atlas(current_atlas_px.data(), cam.width, cam.height);
      if (next) {
        auto gpu =
            vgp::upload_mesh(app.device(), app.allocator(), current_mesh);
        if (gpu.ok()) {
          current_gpu = std::make_shared<vgp::GpuMesh>(std::move(gpu).value());
          current_atlas = std::move(next);
          uploaded_version = current_version;
        }
      }
    }
    // This slot adopts the latest mesh + its atlas, releasing whatever it drew
    // last frame (safe: begin_frame fence-waited this slot). Holding the atlas
    // per slot keeps a version bound by an in-flight frame alive past its swap.
    slot_mesh[f.slot] = current_gpu;
    slot_atlas[f.slot] = current_atlas;

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
    const bool has_mesh = slot_mesh[f.slot] && slot_mesh[f.slot]->valid();
    if (tick % 120 == 0)
      std::printf("render tick %d: fused %zu/%zu, mesh v%llu, drawing=%d\n",
                  tick, done_frames, frame_count,
                  (unsigned long long)current_version, has_mesh ? 1 : 0);

    vg::RenderTargetBeginInfo bi;
    bi.clear_color.float32[0] = 0.05f;
    bi.clear_color.float32[1] = 0.05f;
    bi.clear_color.float32[2] = 0.07f;
    bi.clear_color.float32[3] = 1.0f;
    f.target->begin(f.cmd, bi);
    if (has_mesh) {
      const vgp::HybridMeshDraw draw{slot_mesh[f.slot].get()};
      vgp::HybridMeshFrame hframe;
      hframe.extent = extent;
      hframe.view_proj = view_proj;
      hframe.light_dir = glm::vec3(0.4f, 0.9f, 0.5f);
      hframe.flags = opt.lit ? vgp::kHybridMeshLit : 0u;
      hframe.atlas = slot_atlas[f.slot]->set.handle();
      hframe.draws = &draw;
      hframe.draw_count = 1;
      pipeline.submit(f.cmd, hframe);
    }
    f.target->end(f.cmd);

    const vg::Status present = app.end_frame(f);
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
