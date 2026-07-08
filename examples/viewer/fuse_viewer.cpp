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
//   fuse_viewer <scene_dir> [--voxel 0.02] [--max-frames N] [--fuse-per-tick K]
//               [--remesh-every N] [--width 1280] [--height 720] [--unlit]

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
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
#include "recon_gfx_bridge.hpp"

#include "volumetric_kit/recon/core/allocator.hpp"
#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/instance.hpp"
#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/mesh/marching_cubes.hpp"
#include "volumetric_kit/recon/mesh/mesh.hpp"
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
                 "[--fuse-per-tick k] [--remesh-every n] [--unlit]\n");
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

}  // namespace

int main(int argc, char** argv) {
  Options opt;
  if (!parse_args(argc, argv, opt)) return 2;

  // --- Window ---------------------------------------------------------------
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

  const std::uint8_t white[4] = {255, 255, 255, 255};
  vg::ImageUploadDesc adesc;
  adesc.extent = {1, 1};
  adesc.format = VK_FORMAT_R8G8B8A8_UNORM;
  adesc.pixels = white;
  adesc.size = sizeof(white);
  auto atlas_tex_r = vg::upload_texture(app.device(), app.allocator(), adesc);
  if (!atlas_tex_r.ok()) {
    std::fprintf(stderr, "atlas: %s\n", atlas_tex_r.status().message().c_str());
    return 1;
  }
  vg::Texture atlas_tex = std::move(atlas_tex_r).value();
  auto sampler_r = vg::Sampler::create(app.device().handle());
  if (!sampler_r.ok()) {
    std::fprintf(stderr, "sampler: %s\n", sampler_r.status().message().c_str());
    return 1;
  }
  vg::Sampler sampler = std::move(sampler_r).value();
  const VkDescriptorPoolSize pool_size{
      VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1};
  auto pool_r =
      vg::DescriptorPool::create(app.device().handle(), &pool_size, 1, 1);
  if (!pool_r.ok()) {
    std::fprintf(stderr, "pool: %s\n", pool_r.status().message().c_str());
    return 1;
  }
  vg::DescriptorPool pool = std::move(pool_r).value();
  auto set_r = pool.allocate(pipeline.descriptor_set_layout(0));
  if (!set_r.ok()) {
    std::fprintf(stderr, "atlas set: %s\n", set_r.status().message().c_str());
    return 1;
  }
  vg::DescriptorSet atlas_set = std::move(set_r).value();
  atlas_set.write_combined_image_sampler(
      0, atlas_tex.view(), sampler.handle(),
      VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

  // --- Background fuse thread: load + decode + fuse + extract off the render
  // thread (per-frame JPEG/PNG decode is CPU-heavy and would otherwise gate the
  // loop, starving both GPUs). It publishes the newest coloured mesh + the
  // trajectory; the render thread only uploads + draws, so the window stays at
  // full frame rate and the two devices' queues run concurrently. recon's
  // device is used solely on this thread.
  // -------------------------------------------
  std::mutex share_mtx;
  std::optional<vg::assets::Mesh> pending_mesh;  // newest mesh awaiting upload
  std::uint64_t published_version = 0;
  std::vector<glm::mat4> shared_poses;  // trajectory, grows as frames fuse
  std::atomic<std::size_t> fused_count{0};
  std::atomic<bool> fusing_done{false};
  std::atomic<bool> quit{false};

  std::thread fuse_thread([&]() {
    auto publish = [&](rmesh::Mesh&& rm) {
      vg::assets::Mesh gm = fuse_viewer::to_gfx_mesh(rm);
      std::lock_guard<std::mutex> lk(share_mtx);
      pending_mesh = std::move(gm);
      ++published_version;
    };
    for (std::size_t i = 0; i < frame_count && !quit.load(); ++i) {
      auto fr = dataset.load(i);  // disk read + JPEG/PNG decode (the CPU cost)
      if (!fr) break;
      const vr_example::RgbdFrame frame = std::move(fr).value();
      {
        std::lock_guard<std::mutex> lk(share_mtx);
        shared_poses.push_back(frame.cam_to_world);
      }
      vol::DepthCameraParams dcam{};
      dcam.fx = cam.fx;
      dcam.fy = cam.fy;
      dcam.cx = cam.cx;
      dcam.cy = cam.cy;
      dcam.min_depth = 0.1f;
      dcam.max_depth = opt.max_depth;
      dcam.width = cam.width;
      dcam.height = cam.height;
      dcam.cam_to_world = frame.cam_to_world;
      rtsdf::ColorCameraParams ccam{};
      ccam.fx = cam.fx;
      ccam.fy = cam.fy;
      ccam.cx = cam.cx;
      ccam.cy = cam.cy;
      ccam.width = cam.width;
      ccam.height = cam.height;
      ccam.cam_to_world = frame.cam_to_world;
      const rtsdf::ColorFrame color_frame{frame.color.data(), ccam};
      bool ok = true;
      for (int attempt = 0; attempt < 5; ++attempt) {
        auto failed =
            volume.map().allocate_from_depth(frame.depth.data(), dcam);
        if (!failed) {
          ok = false;
          break;
        }
        if (failed.value() == 0) break;
        if (!volume.resize(volume.grid().num_buckets * 2).ok()) {
          ok = false;
          break;
        }
      }
      if (ok)
        integrator.integrate(volume, frame.depth.data(), dcam, 20.0f,
                             rtsdf::IntegrationMode::Classic, &color_frame);
      fused_count.store(i + 1);
      if ((i % static_cast<std::size_t>(opt.remesh_every)) == 0) {
        auto m = extractor.extract(volume);
        if (m && !m.value().vertices.empty()) publish(std::move(m).value());
      }
    }
    auto m = extractor.extract(volume);  // final mesh
    if (m && !m.value().vertices.empty()) publish(std::move(m).value());
    fusing_done.store(true);
    std::printf("fuse thread: done (%zu frames)\n", fused_count.load());
  });

  // --- Render thread (main): pick up the newest mesh + trajectory, upload,
  // draw following the capture path.
  // ------------------------------------------------
  std::vector<vgp::GpuMesh> mesh_ring(config.frames_in_flight);
  std::vector<std::uint64_t> slot_version(config.frames_in_flight, 0);
  vg::assets::Mesh current_mesh;
  std::uint64_t current_version = 0;
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
        current_version = published_version;
      }
      if (poses.size() != shared_poses.size()) poses = shared_poses;
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

    // Upload the latest mesh into this slot if behind (begin_frame fence-waited
    // this slot, so overwriting its buffers is safe).
    if (current_version != 0 && slot_version[f.slot] != current_version) {
      auto gpu = vgp::upload_mesh(app.device(), app.allocator(), current_mesh);
      if (gpu.ok()) {
        mesh_ring[f.slot] = std::move(gpu).value();
        slot_version[f.slot] = current_version;
      }
    }

    // Follow the trajectory: the frontier (latest fused pose) while fusing,
    // then replay the path in a loop once done.
    const VkExtent2D extent = app.swapchain().extent();
    const float aspect = static_cast<float>(extent.width) /
                         static_cast<float>(std::max(1u, extent.height));
    if (!poses.empty()) {
      if (!done)
        view_frame = std::min(done_frames == 0 ? std::size_t{1} : done_frames,
                              poses.size()) -
                     1;
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
    if (tick % 120 == 0)
      std::printf("render tick %d: fused %zu/%zu, mesh v%llu, drawing=%d\n",
                  tick, done_frames, frame_count,
                  (unsigned long long)current_version,
                  mesh_ring[f.slot].valid() ? 1 : 0);

    vg::RenderTargetBeginInfo bi;
    bi.clear_color.float32[0] = 0.05f;
    bi.clear_color.float32[1] = 0.05f;
    bi.clear_color.float32[2] = 0.07f;
    bi.clear_color.float32[3] = 1.0f;
    f.target->begin(f.cmd, bi);
    if (mesh_ring[f.slot].valid()) {
      const vgp::HybridMeshDraw draw{&mesh_ring[f.slot]};
      vgp::HybridMeshFrame hframe;
      hframe.extent = extent;
      hframe.view_proj = view_proj;
      hframe.light_dir = glm::vec3(0.4f, 0.9f, 0.5f);
      hframe.flags = opt.lit ? vgp::kHybridMeshLit : 0u;
      hframe.atlas = atlas_set.handle();
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

  quit.store(true);
  if (fuse_thread.joinable()) fuse_thread.join();
  app.wait_idle();
  glfwDestroyWindow(window);
  glfwTerminate();
  return exit_code;
}
