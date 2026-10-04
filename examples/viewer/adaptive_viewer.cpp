// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// adaptive_viewer: fuse_viewer with resolution chosen per block. A Replica
// sequence is fused into uniform grids at halving voxel sizes (2 cm, 1 cm and
// 5 mm by default; adaptive_levels.hpp). The coarsest covers everything; a
// block refines when the depth does not line up with its iso-surface, and the
// finer grid then allocates there. Each level meshes the blocks it owns
// through the existing culled extract, is textured with the same keyframe,
// and gfx draws all of them in one HybridMeshFrame.
//
// The device, ring and release protocol are fuse_viewer's (see that file for
// why each step is safe), applied to one extractor per level: a remesh
// publishes every level's mesh together with their atlas, and the render
// thread releases each extractor's ring by its own in-flight generations.
//
// "Show levels" in the panel paints each level a flat colour instead of
// texturing it: green 2 cm, yellow 1 cm, red 5 mm.
//
// Live, in a build with the Orbbec driver: --orbbec [--serial SN] fuses one
// camera, --rig sync.json a synced rig (one re-mesh per set), both posed by
// --calibration. Raw frames are undistorted on the GPU (sensor::GpuFramePrep,
// H.265 colour decoded there), and each set is fused with one allocation and
// one integration per level. The mesh is textured from the set's first
// camera, its colour read back once per re-mesh for the atlas. A block refines
// where the depth is systematically off its surface by more than --eps-mm
// (the panel's slider), averaged over frames so the sensor noise drops out.
//
//   adaptive_viewer <scene_dir> [--cam-params path] [--base-voxel 0.02]
//                   [--levels 3] [--eps-mm 1]
//                   [--max-view-deg 0] [--check-every 5] [--max-frames N]
//                   [--min-depth m] [--max-depth m] [--width 1280]
//                   [--height 720] [--unlit] [--no-texture] [--show-levels]
//                   [--preload] [--no-overlay] [--validation]
//   adaptive_viewer --orbbec [--serial SN] | --rig sync.json [--apply-sync]
//                   [--calibration calib.json] [--dynamic | --static]
//                   [--max-weight 20] [--color WxH] [--fps N] [...as above]

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

#include "adaptive_levels.hpp"
#include "recon_gfx_bridge.hpp"  // vertex-layout static_asserts
#include "replica_capture.hpp"
#include "rgbd_frame.hpp"
#include "shared_device.hpp"
#include "stage_metrics.hpp"
#include "viewer_common.hpp"

#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"
#include "volumetric_kit/recon/mesh/device_mesh.hpp"
#include "volumetric_kit/recon/mesh/marching_cubes.hpp"
#include "volumetric_kit/recon/sensor/camera_capture.hpp"
#include "volumetric_kit/recon/sensor/color_conventions.hpp"
#ifdef VR_ADAPTIVE_ORBBEC
#include "fuse_device_frame.hpp"  // device_color
#include "volumetric_kit/recon/sensor/orbbec/orbbec_capture.hpp"
#include "volumetric_kit/recon/sensor/orbbec/orbbec_rig.hpp"
#include "volumetric_kit/recon/sensor/orbbec/orbbec_sync_config.hpp"
#include "volumetric_kit/recon/sensor/rig_calibration.hpp"
#endif
#include "volumetric_kit/recon/texture/projective_texturer.hpp"

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
namespace vkc = volumetric_kit::core;
namespace vol = volumetric_kit::recon::volume;
namespace rmesh = volumetric_kit::recon::mesh;
namespace rtex = volumetric_kit::recon::texture;
namespace rsensor = volumetric_kit::recon::sensor;
namespace vg = volumetric_kit::gfx;
namespace vgp = volumetric_kit::gfx::pipelines;
namespace win = volumetric_kit::gfx::windowing;

namespace {

struct Options {
  std::string scene_dir;
  std::string cam_params;
  adaptive::Config adaptive;
  // Unset: 0.1-8 m for a sequence, the driver's range (0.25-5 m) live.
  std::optional<float> min_depth;
  std::optional<float> max_depth;
  int max_frames = -1;  // -1: 400 for a sequence, unlimited live
  bool orbbec = false;  // one live camera
  std::string serial;
  std::string rig;          // a sync configuration: a live rig
  std::string calibration;  // camera poses by serial
  bool apply_sync = false;
  std::uint32_t color_width = 0;  // 0: the driver's 1280 x 720
  std::uint32_t color_height = 0;
  int fps = 0;  // 0: 30, or 25 for 4K colour (H.265's limit there)
  // Dynamic clears surfaces that moved away; unset, it is on live (as in
  // rig_viewer) and off for a recorded sequence.
  std::optional<bool> dynamic;
  int width = 1280;
  int height = 720;
  bool lit = true;
  bool texture = true;
  bool show_levels = false;
  bool preload = false;
  bool overlay = true;
  bool validation = false;
};

bool parse_args(int argc, char** argv, Options& o) {
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto v = [&]() -> const char* {
      return (i + 1 < argc) ? argv[++i] : nullptr;
    };
    auto number = [&](float& out) {
      const char* x = v();
      if (x == nullptr) return false;
      out = std::strtof(x, nullptr);
      return std::isfinite(out);
    };
    auto integer = [&](int& out) {
      const char* x = v();
      if (x == nullptr) return false;
      out = std::atoi(x);
      return true;
    };
    bool ok = true;
    if (a == "--cam-params") {
      const char* x = v();
      ok = x != nullptr;
      if (ok) o.cam_params = x;
    } else if (a == "--base-voxel") {
      ok = number(o.adaptive.base_voxel);
    } else if (a == "--levels") {
      ok = integer(o.adaptive.levels);
    } else if (a == "--eps-mm") {
      ok = number(o.adaptive.eps_mm);
    } else if (a == "--max-view-deg") {
      ok = number(o.adaptive.max_view_deg);
    } else if (a == "--check-every") {
      ok = integer(o.adaptive.check_every);
    } else if (a == "--max-frames") {
      ok = integer(o.max_frames);
    } else if (a == "--min-depth") {
      float x = 0.0f;
      ok = number(x);
      o.min_depth = x;
    } else if (a == "--max-depth") {
      float x = 0.0f;
      ok = number(x);
      o.max_depth = x;
    } else if (a == "--width") {
      ok = integer(o.width);
    } else if (a == "--height") {
      ok = integer(o.height);
    } else if (a == "--orbbec") {
      o.orbbec = true;
    } else if (a == "--serial") {
      const char* x = v();
      ok = x != nullptr;
      if (ok) o.serial = x;
      o.orbbec = true;
    } else if (a == "--rig") {
      const char* x = v();
      ok = x != nullptr;
      if (ok) o.rig = x;
    } else if (a == "--calibration") {
      const char* x = v();
      ok = x != nullptr;
      if (ok) o.calibration = x;
    } else if (a == "--color") {
      const char* x = v();
      unsigned w = 0, h = 0;
      ok = x != nullptr && std::sscanf(x, "%ux%u", &w, &h) == 2 && w > 0 &&
           h > 0;
      o.color_width = w;
      o.color_height = h;
    } else if (a == "--fps") {
      ok = integer(o.fps) && o.fps > 0;
    } else if (a == "--apply-sync") {
      o.apply_sync = true;
    } else if (a == "--dynamic" || a == "--static") {
      o.dynamic = a == "--dynamic";
    } else if (a == "--max-weight") {
      ok = number(o.adaptive.max_weight) && o.adaptive.max_weight > 0.0f;
    } else if (a == "--unlit") {
      o.lit = false;
    } else if (a == "--no-texture") {
      o.texture = false;
    } else if (a == "--show-levels") {
      o.show_levels = true;
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
    if (!ok) {
      std::fprintf(stderr, "bad value for %s\n", a.c_str());
      return false;
    }
  }
  const bool live = o.orbbec || !o.rig.empty();
  if (o.scene_dir.empty() == !live || (o.orbbec && !o.rig.empty())) {
    std::fprintf(stderr,
                 "usage: adaptive_viewer <scene_dir> | --orbbec [--serial SN] "
                 "| --rig sync.json [--apply-sync] [--calibration calib.json] "
                 "[--dynamic | --static] [--max-weight w] [--color WxH] "
                 "[--fps n] [--cam-params path] "
                 "[--base-voxel m] [--levels 2..4] [--eps-mm e] "
                 "[--max-view-deg d] [--check-every n] "
                 "[--max-frames n] [--min-depth m] [--max-depth m] "
                 "[--unlit] [--no-texture] [--show-levels] [--preload] "
                 "[--no-overlay] [--validation]\n");
    return false;
  }
  if (o.min_depth && o.max_depth &&
      (!(*o.min_depth >= 0.0f) || !(*o.max_depth > *o.min_depth))) {
    std::fprintf(stderr, "--min-depth must be in [0, --max-depth)\n");
    return false;
  }
#ifndef VR_ADAPTIVE_ORBBEC
  if (live) {
    std::fprintf(stderr, "this build has no Orbbec driver (VR_WITH_ORBBEC)\n");
    return false;
  }
#endif
  if (o.max_frames < 0) o.max_frames = live ? 0 : 400;
  o.adaptive.mode = o.dynamic.value_or(live)
                        ? vr::tsdf::IntegrationMode::Dynamic
                        : vr::tsdf::IntegrationMode::Classic;
  if (o.cam_params.empty()) o.cam_params = o.scene_dir + "/../cam_params.json";
  return true;
}

// One atlas version and the set binding it; see fuse_viewer.
struct AtlasVersion {
  vg::Texture tex;
  vg::DescriptorPool pool;
  vg::DescriptorSet set;
};

// A keyframe's colour in canonical RGBA8.
struct AtlasPixels {
  std::vector<std::uint32_t> pixels;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  bool empty() const noexcept { return pixels.empty(); }
};

// One remesh: each level's extract (empty when it failed, and the previous
// one stays drawn) and the keyframe their uv0 index into.
struct Bundle {
  std::vector<std::optional<rmesh::DeviceMesh>> meshes;
  AtlasPixels atlas;
};

// One poll's frames, ready to fuse: host frames from a sequence, or a live
// camera's frames prepared on the GPU. They borrow the source until the next
// poll; the first is the keyframe a remesh textures from.
struct PolledSet {
  std::vector<rsensor::CapturedFrame> host;
#ifdef VR_ADAPTIVE_ORBBEC
  std::vector<rsensor::DeviceFrame> device;
#endif
  void clear() {
    host.clear();
#ifdef VR_ADAPTIVE_ORBBEC
    device.clear();
#endif
  }
};

// What a remesh textures from, and the colour its atlas holds.
struct Keyframe {
  rtex::TextureView view;
  const std::uint32_t* host_color = nullptr;
  std::shared_ptr<const vkc::Buffer> device_color;
  vr::ColorCameraParams color_camera{};
  vr::ColorEncoding encoding{};
  float occlusion = 0.02f;  // how far depth may disagree and still texture
  bool has_color() const { return host_color != nullptr || device_color; }
};

Keyframe keyframe_of(const rsensor::CapturedFrame& f) {
  Keyframe k;
  k.view.depth = f.depth;
  k.view.cam = f.depth_camera;
  k.host_color = f.color;
  k.color_camera = f.color_camera;
  k.encoding = f.color_encoding;
  return k;
}

#ifdef VR_ADAPTIVE_ORBBEC
Keyframe keyframe_of(const rsensor::DeviceFrame& f) {
  Keyframe k;
  k.view.cam = f.depth_camera;
  k.view.depth_buffer = f.depth;
  k.view.color_camera = f.color_camera;
  k.view.coverage = f.color;
  k.device_color = f.color;
  k.color_camera = f.color_camera;
  k.encoding = f.color_encoding;
  k.occlusion = 0.05f;  // rig_viewer's: real depth disagrees more
  return k;
}
#endif

// Where frames come from: a Replica sequence, one Orbbec camera, or a rig.
struct Source {
  std::optional<vr_example::ReplicaCapture> replica;
#ifdef VR_ADAPTIVE_ORBBEC
  std::optional<rsensor::OrbbecCapture> camera;
  std::optional<rsensor::OrbbecRig> rig;
  std::vector<rsensor::GpuFramePrep> preps;  // one per camera
#endif
  vr::ColorCameraParams frame_camera{};  // for the view's field of view
  std::size_t frame_count = 0;           // 0: live, until the window closes

  rsensor::ICameraCapture& capture() {
#ifdef VR_ADAPTIVE_ORBBEC
    if (rig) return *rig;
    if (camera) return *camera;
#endif
    return *replica;
  }
  bool live() const noexcept { return !replica.has_value(); }

  // The next frames to fuse together: one, or a rig set's present cameras.
  vkc::Result<bool> poll(PolledSet& out) {
    out.clear();
#ifdef VR_ADAPTIVE_ORBBEC
    if (rig) {
      VKC_ASSIGN(const std::optional<rsensor::OrbbecRigRawSet> set,
                 rig->poll_raw_set());
      if (!set || set->count() == 0) return false;
      VKC_ASSIGN(const std::vector<std::optional<rsensor::DeviceFrame>> frames,
                 rsensor::prepare_set(preps, set->frames));
      for (const std::optional<rsensor::DeviceFrame>& f : frames) {
        if (f) out.device.push_back(*f);
      }
      return !out.device.empty();
    }
    if (camera) {
      VKC_ASSIGN(const std::optional<rsensor::RawFrame> raw,
                 camera->poll_raw());
      if (!raw) return false;
      VKC_ASSIGN(rsensor::DeviceFrame frame, preps.front().prepare(*raw));
      out.device.push_back(std::move(frame));
      return true;
    }
#endif
    VKC_ASSIGN(const std::optional<rsensor::CapturedFrame> frame,
               capture().poll());
    if (frame) out.host.push_back(*frame);
    return frame.has_value();
  }
};

#ifdef VR_ADAPTIVE_ORBBEC
// Raw frames, H.265 colour decoded onto `device`, at the asked colour mode.
void apply_streams(const Options& opt, vkc::Device& device,
                   rsensor::OrbbecStreamOptions& streams) {
  if (opt.min_depth) streams.min_depth = *opt.min_depth;
  if (opt.max_depth) streams.max_depth = *opt.max_depth;
  streams.raw = true;
  streams.device = &device;
  streams.color_codec = rsensor::OrbbecColorCodec::Hevc;
  if (opt.color_width != 0) {
    streams.color_width = opt.color_width;
    streams.color_height = opt.color_height;
  }
  if (opt.fps != 0) {
    streams.fps = std::uint32_t(opt.fps);
  } else if (streams.color_width >= 3840) {
    streams.fps = 25;
  }
}

// The GPU pass per camera. Depth is kept only where the colour camera saw,
// as rig_viewer keeps it, so fused surfaces can be coloured and textured.
vkc::Status add_preps(Source& source, vkc::Device& device,
                      vkc::Allocator& allocator, std::size_t cameras) {
  rsensor::GpuFramePrepConfig config;
  config.depth_within_color = true;
  for (std::size_t c = 0; c < cameras; ++c) {
    VKC_ASSIGN(rsensor::GpuFramePrep prep,
               rsensor::GpuFramePrep::create(device, allocator, config));
    source.preps.push_back(std::move(prep));
  }
  return {};
}
#endif

vkc::Result<Source> open_source(const Options& opt,
                                [[maybe_unused]] vkc::Device& device,
                                [[maybe_unused]] vkc::Allocator& allocator) {
  Source source;
#ifdef VR_ADAPTIVE_ORBBEC
  std::vector<rsensor::RigCameraCalibration> calibration;
  if (!opt.calibration.empty()) {
    VKC_ASSIGN(calibration, rsensor::read_rig_calibration(opt.calibration));
  }
  if (!opt.rig.empty()) {
    rsensor::OrbbecRig::Options rig_options;
    VKC_ASSIGN(rig_options.sync, rsensor::read_orbbec_sync_config(opt.rig));
    rig_options.calibration = calibration;
    rig_options.apply_sync_config = opt.apply_sync;
    apply_streams(opt, device, rig_options);
    VKC_ASSIGN(rsensor::OrbbecRig rig, rsensor::OrbbecRig::open(rig_options));
    VKC_TRY(add_preps(source, device, allocator, rig.camera_count()));
    source.frame_camera = rig.color_camera(0);
    std::printf(
        "rig: %zu cameras%s\n", rig.camera_count(),
        calibration.empty() ? ", NO calibration: all at the origin" : "");
    source.rig.emplace(std::move(rig));
    return source;
  }
  if (opt.orbbec) {
    rsensor::OrbbecCapture::Options options;
    options.serial = opt.serial;
    apply_streams(opt, device, options);
    VKC_TRY(add_preps(source, device, allocator, 1));
    for (const rsensor::RigCameraCalibration& c : calibration) {
      if (c.serial == opt.serial || calibration.size() == 1) {
        options.serial = c.serial;
        options.cam_to_world = c.cam_to_world;
      }
    }
    VKC_ASSIGN(rsensor::OrbbecCapture camera,
               rsensor::OrbbecCapture::open(options));
    source.frame_camera = camera.color_camera();
    std::printf("camera %s\n", camera.device_info().serial.c_str());
    source.camera.emplace(std::move(camera));
    return source;
  }
#endif
  vr_example::ReplicaCapture::Options options;
  options.frame_limit = static_cast<std::size_t>(opt.max_frames);
  options.min_depth = opt.min_depth.value_or(0.1f);
  options.max_depth = opt.max_depth.value_or(8.0f);
  VKC_ASSIGN(
      vr_example::ReplicaCapture replica,
      vr_example::ReplicaCapture::open(opt.scene_dir, opt.cam_params, options));
  source.frame_camera = replica.color_camera();
  source.frame_count = replica.frame_count();
  source.replica.emplace(std::move(replica));
  return source;
}

// A turntable around `target` about the first camera's image-up axis (the
// world follows OpenCV, +Y down), as rig_viewer's.
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

struct Panel {
  std::size_t fused = 0;
  std::size_t total = 0;
  double fuse_ms = 0.0;       // the newest frame
  double fuse_mean_ms = 0.0;  // over every fused frame
  std::vector<adaptive::LevelStats> levels;
  std::vector<std::uint32_t> triangles;
};

void draw_adaptive_panel(const Panel& panel, std::atomic<bool>& show_levels,
                         std::atomic<float>& eps_mm, bool& follow,
                         const adaptive::Config& config) {
  if (!ImGui::Begin("Adaptive")) {
    ImGui::End();
    return;
  }
  ImGui::Text(
      "fused  %zu / %zu frames, %s TSDF, weight cap %.0f", panel.fused,
      panel.total,
      config.mode == vr::tsdf::IntegrationMode::Dynamic ? "dynamic" : "classic",
      double(config.max_weight));
  ImGui::Text("fuse   %.2f ms/frame, mean %.2f (incl. remesh)", panel.fuse_ms,
              panel.fuse_mean_ms);
  float eps = eps_mm.load();
  if (ImGui::SliderFloat("eps mm", &eps, 0.25f, 8.0f, "%.2f",
                         ImGuiSliderFlags_Logarithmic)) {
    eps_mm.store(eps);
  }
  const double floor_mm = panel.levels.empty() ? 0.0 : panel.levels[0].floor_mm;
  ImGui::Text("wall baseline %.2f mm: refine blocks off by > %.2f mm", floor_mm,
              floor_mm + double(eps));
  bool show = show_levels.load();
  if (ImGui::Checkbox("show levels (green / yellow / red)", &show)) {
    show_levels.store(show);
  }
  ImGui::Checkbox("follow the capture camera", &follow);
  ImGui::TextDisabled("drag: orbit, right-drag: pan, scroll: zoom");
  ImGui::Separator();
  ImGui::Text(
      "level  voxel   blocks refined   owned  triangles  offset  noise");
  for (std::size_t l = 0; l < panel.levels.size(); ++l) {
    const adaptive::LevelStats& s = panel.levels[l];
    ImGui::Text("%5zu  %4.1fmm %7zu %7zu %7zu %10u  %4.2fmm %4.2fmm", l,
                1000.0 * double(s.voxel), s.blocks, s.refined, s.owned,
                l < panel.triangles.size() ? panel.triangles[l] : 0u,
                s.median_offset_mm, s.median_noise_mm);
  }
  ImGui::TextDisabled("offset / noise: the median block's, at that level");
  ImGui::End();
}

int run(GLFWwindow* window, const Options& opt) {
  // --- One VkDevice, adopted by both libraries (as fuse_viewer) -------------
  fuse_viewer::SharedDeviceConfig shared_config;
  shared_config.enable_validation = opt.validation;
  const std::unique_ptr<fuse_viewer::vkc::SharedDevice> shared =
      fuse_viewer::build_shared_device(window, shared_config);
  if (shared == nullptr) return 1;
  vg::app::WindowedAppConfig config;
  config.app_name = "adaptive_viewer";
  config.swapchain.extent = fuse_viewer::window_extent(window);
  config.swapchain.depth_format = VK_FORMAT_D32_SFLOAT;
  config.frames_in_flight = 2;
  auto app_r = vg::app::WindowedApp::adopt(
      fuse_viewer::gfx_adopt_payload(*shared), config,
      [&shared](VkInstance instance) -> vg::Result<VkSurfaceKHR> {
        if (instance != shared->instance().handle()) {
          return vg::Status::invalid_argument(
              "surface factory: instance mismatch");
        }
        return shared->release_surface();
      });
  if (!app_r.ok()) {
    std::fprintf(stderr, "WindowedApp::adopt: %s\n",
                 app_r.status().message().c_str());
    return 1;
  }
  vg::app::WindowedApp app = std::move(app_r).value();
  auto rdevice_r =
      vkc::Device::adopt(shared->compute_payload(), vr::device_requirements());
  if (!rdevice_r) {
    std::fprintf(stderr, "recon Device::adopt: %s\n",
                 rdevice_r.status().message().c_str());
    return 1;
  }
  vkc::Device& rdevice = rdevice_r.value();
  auto rallocator_r =
      vkc::Allocator::create(shared->instance().handle(), rdevice);
  if (!rallocator_r) {
    std::fprintf(stderr, "recon allocator: %s\n",
                 rallocator_r.status().message().c_str());
    return 1;
  }
  vkc::Allocator& rallocator = rallocator_r.value();

  // --- Capture, levels, one extractor per level -----------------------------
  auto source_r = open_source(opt, rdevice, rallocator);
  if (!source_r) {
    std::fprintf(stderr, "source: %s\n", source_r.status().message().c_str());
    return 1;
  }
  Source source = std::move(source_r).value();
  const vr::ColorCameraParams cam = source.frame_camera;

  auto levels_r = adaptive::Levels::create(rdevice, rallocator, opt.adaptive);
  if (!levels_r) {
    std::fprintf(stderr, "adaptive: %s\n", levels_r.status().message().c_str());
    return 1;
  }
  adaptive::Levels& levels = *levels_r.value();
  const std::size_t level_count = levels.count();

  rmesh::MarchingCubesConfig mc_config;
  mc_config.extra_vertex_usage = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
  mc_config.extra_index_usage = VK_BUFFER_USAGE_INDEX_BUFFER_BIT;
  mc_config.queue_families[0] = shared->compute_family();
  mc_config.queue_families[1] = shared->graphics_family();
  mc_config.queue_family_count = 2;
  mc_config.slot_count = config.frames_in_flight + 1;
  std::vector<rmesh::MarchingCubes> extractors;
  for (std::size_t l = 0; l < level_count; ++l) {
    auto mc_r = rmesh::MarchingCubes::create(rdevice, rallocator, mc_config);
    if (!mc_r) {
      std::fprintf(stderr, "marching cubes: %s\n",
                   mc_r.status().message().c_str());
      return 1;
    }
    extractors.push_back(std::move(mc_r).value());
  }
  std::optional<rtex::ProjectiveTexturer> texturer;
  if (opt.texture) {
    auto texture_r = rtex::ProjectiveTexturer::create(rdevice, rallocator);
    if (!texture_r) {
      std::fprintf(stderr, "texturer: %s\n",
                   texture_r.status().message().c_str());
      return 1;
    }
    texturer = std::move(texture_r).value();
  }
  const std::size_t frame_count = source.frame_count;
  const float vfov = 2.0f * std::atan(static_cast<float>(cam.height) /
                                      (2.0f * std::max(1.0f, cam.fy)));

  // --- gfx: pipeline, profiler, overlay, atlas ------------------------------
  auto pipeline_r = vgp::HybridMeshPipeline::create(app.device().handle(),
                                                    app.swapchain().layout());
  if (!pipeline_r.ok()) {
    std::fprintf(stderr, "pipeline: %s\n",
                 pipeline_r.status().message().c_str());
    return 1;
  }
  vgp::HybridMeshPipeline pipeline = std::move(pipeline_r).value();
  vg::ProfilerConfig profiler_config;
  profiler_config.frames_in_flight = config.frames_in_flight;
  auto profiler_r = vg::Profiler::create(app.device(), profiler_config);
  if (!profiler_r.ok()) {
    std::fprintf(stderr, "profiler: %s\n",
                 profiler_r.status().message().c_str());
    return 1;
  }
  vg::Profiler profiler = std::move(profiler_r).value();
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
  if (opt.overlay) {
    vg::ui::ImGuiOverlayConfig overlay_config;
    overlay_config.layout = app.swapchain().layout();
    overlay_config.min_image_count = app.swapchain().image_count();
    overlay_config.image_count = app.swapchain().image_count();
    auto overlay_r = vg::ui::ImGuiOverlay::create(
        app.device(), app.instance_handle(), overlay_config);
    if (!overlay_r.ok()) {
      std::fprintf(stderr, "overlay: %s\n",
                   overlay_r.status().message().c_str());
      return 1;
    }
    overlay = std::move(overlay_r).value();
    ImGui::SetCurrentContext(overlay->context());
    if (!ImGui_ImplGlfw_InitForVulkan(window, true)) {
      std::fprintf(stderr, "ImGui_ImplGlfw_InitForVulkan failed\n");
      return 1;
    }
  }
  const fuse_viewer::ImGuiGlfwShutdown imgui_glfw_guard{opt.overlay};
  auto sampler_r = vg::Sampler::create(app.device().handle());
  if (!sampler_r.ok()) {
    std::fprintf(stderr, "sampler: %s\n", sampler_r.status().message().c_str());
    return 1;
  }
  vg::Sampler sampler = std::move(sampler_r).value();
  auto build_atlas =
      [&](const void* pixels, std::uint32_t width,
          std::uint32_t height) -> std::shared_ptr<AtlasVersion> {
    vg::ImageUploadDesc upload_desc;
    upload_desc.extent = {width, height};
    upload_desc.format = VK_FORMAT_R8G8B8A8_SRGB;  // canonical-encoded pixels
    upload_desc.pixels = pixels;
    upload_desc.size = static_cast<std::size_t>(width) * height * 4;
    auto texture_r =
        vg::upload_texture(app.device(), app.allocator(), upload_desc);
    if (!texture_r.ok()) return nullptr;
    const VkDescriptorPoolSize pool_size{
        VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1};
    auto pool_r =
        vg::DescriptorPool::create(app.device().handle(), &pool_size, 1, 1);
    if (!pool_r.ok()) return nullptr;
    vg::DescriptorPool atlas_pool = std::move(pool_r).value();
    auto set_r = atlas_pool.allocate(pipeline.descriptor_set_layout(0));
    if (!set_r.ok()) return nullptr;
    auto version = std::make_shared<AtlasVersion>();
    version->tex = std::move(texture_r).value();
    version->pool = std::move(atlas_pool);
    version->set = std::move(set_r).value();
    version->set.write_combined_image_sampler(
        0, version->tex.view(), sampler.handle(),
        VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    return version;
  };
  const std::uint8_t white[4] = {255, 255, 255, 255};
  std::shared_ptr<AtlasVersion> white_atlas = build_atlas(white, 1, 1);
  if (!white_atlas) return 1;
  std::shared_ptr<AtlasVersion> current_atlas = white_atlas;

  // --- Fuse thread
  // ------------------------------------------------------------
  std::mutex share_mtx;
  std::optional<Bundle> pending;  // its presence is the "uncollected" flag
  std::vector<std::uint64_t> shared_released(level_count, 0);
  std::vector<glm::mat4> shared_poses;
  std::vector<vg::FrameMetrics::Section> shared_stages;
  Panel shared_panel;
  std::atomic<std::size_t> fused_count{0};
  std::atomic<bool> fusing_done{false};
  std::atomic<bool> quit{false};
  std::atomic<bool> show_levels{opt.show_levels};
  std::atomic<float> eps_mm{opt.adaptive.eps_mm};

  std::thread fuse_thread([&]() {
    try {
      vkc::StageMetrics stages;
      vkc::StageMetrics remesh_stages;  // held so the rows read the last remesh
      bool last_tint = false;           // what the last remesh painted
      vkc::StageMetrics totals;         // every frame's work, for the summary
      std::size_t remeshes = 0;
      double fuse_total_ms = 0.0;
      // Release each extractor's ring by the render thread's marks, on this
      // thread, before an extract; report whether a new bundle may go out.
      auto release_and_may_publish = [&]() {
        std::vector<std::uint64_t> marks;
        bool uncollected = false;
        {
          std::lock_guard<std::mutex> lock(share_mtx);
          marks = shared_released;
          uncollected = pending.has_value();
        }
        for (std::size_t l = 0; l < level_count; ++l) {
          if (marks[l] != 0) extractors[l].release_through(marks[l]);
        }
        return !uncollected;
      };
      // Mesh each level's own blocks, texture (or tint) them, publish.
      auto remesh = [&](const Keyframe* keyframe) {
        remesh_stages.clear();
        std::vector<std::vector<vol::BlockIndex>> owned;
        {
          vkc::StageScope scope(remesh_stages, "ownership");
          const vkc::Status s = levels.owned(owned);
          if (!s.ok()) {
            std::fprintf(stderr, "adaptive_viewer: ownership: %s\n",
                         s.message().c_str());
            return;
          }
        }
        const bool tint = show_levels.load();
        last_tint = tint;
        const bool textured =
            !tint && texturer && keyframe != nullptr && keyframe->has_color();
        Bundle bundle;
        bundle.meshes.resize(level_count);
        for (std::size_t l = 0; l < level_count; ++l) {
          const vol::BlockList list = levels.grid(l).block_list(owned[l]);
          vkc::Result<rmesh::DeviceMesh> m = [&]() {
            vkc::StageScope scope(remesh_stages, "extract");
            return extractors[l].extract_device(levels.grid(l), 0.0f, list);
          }();
          if (!m) {
            std::fprintf(stderr, "adaptive_viewer: extract level %zu: %s\n", l,
                         m.status().message().c_str());
            continue;
          }
          vkc::Status painted;
          if (tint) {
            vkc::StageScope scope(remesh_stages, "tint");
            painted = levels.tint(m.value(), l);
          } else if (textured && !m.value().empty()) {
            painted = texturer->texture(m.value(), keyframe->view,
                                        keyframe->occlusion, &remesh_stages);
          }
          if (!painted.ok()) {
            std::fprintf(stderr, "adaptive_viewer: paint level %zu: %s\n", l,
                         painted.message().c_str());
          }
          bundle.meshes[l] = m.value();
        }
        if (textured) {
          vkc::StageScope scope(remesh_stages, "atlas pack");
          const std::size_t pixels = std::size_t(keyframe->color_camera.width) *
                                     keyframe->color_camera.height;
          // A device frame's colour comes back once a remesh; its high byte
          // is coverage, which the atlas does not need.
          std::vector<std::uint32_t> device_pixels;
          const std::uint32_t* src = keyframe->host_color;
          if (keyframe->device_color) {
            device_pixels.resize(pixels);
            vkc::CommandBatch batch(rdevice, rallocator);
            vkc::Status read = batch.readback(*keyframe->device_color, 0,
                                              pixels * 4, device_pixels.data());
            if (read.ok()) read = batch.submit();
            if (read.ok()) src = device_pixels.data();
          }
          bundle.atlas.pixels.resize(pixels);
          if (src != nullptr &&
              rsensor::to_canonical(src, pixels, keyframe->encoding,
                                    bundle.atlas.pixels.data())
                  .ok()) {
            for (std::uint32_t& px : bundle.atlas.pixels) px |= 0xFF000000u;
            bundle.atlas.width = keyframe->color_camera.width;
            bundle.atlas.height = keyframe->color_camera.height;
          } else {
            bundle.atlas.pixels.clear();
          }
        }
        std::lock_guard<std::mutex> lock(share_mtx);
        pending = std::move(bundle);
      };

      if (opt.preload && source.replica) {
        std::printf(
            "preloading %.0f MB...\n",
            double(source.replica->preload_bytes_projected()) / (1024 * 1024));
        auto cached = source.replica->preload(&quit);
        if (!cached) {
          std::fprintf(stderr, "adaptive_viewer: preload: %s (streaming)\n",
                       cached.status().message().c_str());
        }
      }
      rsensor::ICameraCapture& capture = source.capture();
      const vkc::Status started = capture.start();
      if (!started.ok()) {
        std::fprintf(stderr, "adaptive_viewer: capture start: %s\n",
                     started.message().c_str());
      }
      // The newest keyframe, kept for the remesh after the loop: a host
      // frame copied (its storage is the source's), a device frame held.
      vr_example::RgbdFrame last_host;
      std::optional<Keyframe> last_key;
      PolledSet frames;
      std::vector<adaptive::Input> inputs;
      std::size_t sets = 0;
      bool warned_silent = false;
      auto last_arrival = std::chrono::steady_clock::now();
      for (std::size_t i = 0; started.ok() && !quit.load();) {
        stages.clear();
        for (const char* stage :
             {"frame", "adaptive upload", "adaptive check", "adaptive residual",
              "allocate", "resize", "integrate", "  ..active set", "ownership",
              "extract", "tint", "texture", "atlas pack"}) {
          stages.seed(stage);
        }
        auto polled = [&]() {
          vkc::StageScope scope(stages, "frame");
          return source.poll(frames);
        }();
        if (!polled) {
          std::fprintf(stderr, "frame %zu failed to load: %s\n", i,
                       polled.status().message().c_str());
          break;
        }
        if (!polled.value()) {
          if (capture.exhausted()) break;
          if (source.live() && !warned_silent &&
              std::chrono::steady_clock::now() - last_arrival >
                  std::chrono::seconds(10)) {
            warned_silent = true;
            std::fprintf(stderr,
                         "adaptive_viewer: no frames for 10 s. On macOS, allow "
                         "this binary local-network access; a sync secondary "
                         "streams only while its primary does.\n");
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
          continue;
        }
        last_arrival = std::chrono::steady_clock::now();
        // The set as the levels take it, and its first camera as keyframe.
        inputs.clear();
        vkc::Status fused;
        Keyframe key;
        for (std::size_t c = 0; c < frames.host.size() && fused.ok(); ++c) {
          vkc::Result<adaptive::Input> in = levels.upload(frames.host[c], c);
          if (in) {
            inputs.push_back(std::move(in).value());
          } else {
            fused = in.status();
          }
        }
        if (!frames.host.empty()) key = keyframe_of(frames.host.front());
#ifdef VR_ADAPTIVE_ORBBEC
        for (const rsensor::DeviceFrame& f : frames.device) {
          adaptive::Input in{f.depth, f.depth_camera, std::nullopt};
          if (f.has_color()) in.color = vr_example::device_color(f);
          inputs.push_back(std::move(in));
        }
        if (!frames.device.empty()) key = keyframe_of(frames.device.front());
#endif
        {
          std::lock_guard<std::mutex> lock(share_mtx);
          shared_poses.push_back(key.view.cam.cam_to_world);
        }
        levels.set_eps_mm(eps_mm.load());
        if (fused.ok()) fused = levels.fuse(inputs, &stages);
        if (!fused.ok()) {
          std::fprintf(stderr, "adaptive_viewer: fuse (frame %zu): %s\n", i,
                       fused.message().c_str());
          break;
        }
        fused_count.store(i + inputs.size());
        totals.merge(stages);
        if (release_and_may_publish()) {
          remesh(&key);
          totals.merge(remesh_stages);
          ++remeshes;
        }
        stages.merge(remesh_stages);
        {
          std::lock_guard<std::mutex> lock(share_mtx);
          shared_stages = fuse_viewer::to_sections(stages);
          shared_panel.fuse_ms = stages.total_cpu_ms(/*exclude=*/"frame");
          fuse_total_ms += shared_panel.fuse_ms;
          shared_panel.fuse_mean_ms = fuse_total_ms / double(++sets);
          shared_panel.levels = levels.stats();
        }
        if (!frames.host.empty()) {
          last_host.assign(frames.host.front());
          last_key = keyframe_of(last_host.view());
        } else {
          last_key = key;
        }
        i += inputs.size();
        if (opt.max_frames > 0 && i >= std::size_t(opt.max_frames)) break;
      }
      // The final surface: wait (bounded) for the last bundle to be taken.
      for (int wait = 0; wait < 500 && !quit.load(); ++wait) {
        {
          std::lock_guard<std::mutex> lock(share_mtx);
          if (!pending) break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
      if (!quit.load() && last_key && release_and_may_publish()) {
        remesh(&*last_key);
      }
      fusing_done.store(true);
      const std::size_t n = std::max<std::size_t>(1, sets);
      std::printf(
          "fuse thread: done (%zu frames in %zu sets, %zu remeshes); "
          "mean per set, host / device ms:\n",
          fused_count.load(), sets, remeshes);
      for (const vkc::StageRow& row : totals.rows()) {
        std::printf(
            "  %-16s %7.3f / %s\n", row.name, row.cpu_ms / double(n),
            row.has_gpu ? std::to_string(row.gpu_ms / double(n)).c_str() : "-");
      }
      std::printf("  %-16s %7.3f (excluding frame decode)\n", "total",
                  totals.total_cpu_ms(/*exclude=*/"frame") / double(n));
      // Fusion is over, but the paint mode can still change: re-mesh then.
      while (!quit.load() && last_key) {
        if (show_levels.load() != last_tint && release_and_may_publish()) {
          remesh(&*last_key);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
    } catch (const std::exception& e) {
      std::fprintf(stderr, "adaptive_viewer: fuse thread aborted: %s\n",
                   e.what());
    }
    fusing_done.store(true);
  });
  fuse_viewer::QuitJoin fuse_guard{fuse_thread, quit};

  // --- Render thread
  // ----------------------------------------------------------
  std::vector<std::shared_ptr<AtlasVersion>> slot_atlas(
      config.frames_in_flight);
  std::vector<rmesh::DeviceMesh> live(level_count);
  std::vector<std::vector<std::uint64_t>> frame_generations(
      level_count, std::vector<std::uint64_t>(config.frames_in_flight, 0));
  std::vector<std::uint64_t> newest_taken(level_count, 0);
  std::optional<Bundle> taken;  // held until its atlas uploads
  bool mesh_unusable = false;
  const bool cross_family =
      shared->graphics_family() != shared->compute_family();
  std::vector<glm::mat4> poses;
  std::size_t view_frame = 0;
  // A replay follows its camera until the mouse takes over; live, it orbits.
  bool follow = !source.live();
  std::optional<OrbitView> view;
  double last_x = 0.0, last_y = 0.0;
  bool have_last = false;
  std::vector<vg::FrameMetrics::Section> stages_snapshot;
  Panel panel;
  std::printf(
      "adaptive_viewer: %s, %zu levels, %s TSDF, weight cap %.0f; "
      "close the window to quit\n",
      source.live() ? "live" : "replay", level_count,
      opt.adaptive.mode == vr::tsdf::IntegrationMode::Dynamic ? "dynamic"
                                                              : "classic",
      double(opt.adaptive.max_weight));
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

    // Retire then take, under one lock (fuse_viewer explains the order).
    {
      std::lock_guard<std::mutex> lock(share_mtx);
      if (poses.size() < shared_poses.size()) {
        poses.insert(poses.end(), shared_poses.begin() + poses.size(),
                     shared_poses.end());
      }
      stages_snapshot = shared_stages;
      panel.fuse_ms = shared_panel.fuse_ms;
      panel.fuse_mean_ms = shared_panel.fuse_mean_ms;
      panel.levels = shared_panel.levels;
      for (std::size_t l = 0; l < level_count; ++l) {
        shared_released[l] = fuse_viewer::retire_and_release_mark(
            frame_generations[l], render_frame.slot, live[l].generation,
            newest_taken[l]);
      }
      if (pending && !taken && !mesh_unusable) {
        taken = std::move(*pending);
        pending.reset();
        for (std::size_t l = 0; l < level_count; ++l) {
          if (taken->meshes[l]) newest_taken[l] = taken->meshes[l]->generation;
        }
      }
    }
    const bool done = fusing_done.load();
    panel.fused = fused_count.load();
    panel.total = frame_count;

    // Commit every level and their atlas together, or nothing.
    if (taken) {
      for (std::size_t l = 0; l < level_count && !mesh_unusable; ++l) {
        const std::optional<rmesh::DeviceMesh>& m = taken->meshes[l];
        if (!m || m->empty()) continue;
        if (const char* why =
                fuse_viewer::unbindable_reason(*m, cross_family)) {
          std::fprintf(stderr,
                       "adaptive_viewer: level %zu mesh cannot be "
                       "bound (%s); drawing stops here\n",
                       l, why);
          mesh_unusable = true;
        }
      }
      if (mesh_unusable) {
        taken.reset();
      } else {
        std::shared_ptr<AtlasVersion> next =
            taken->atlas.empty()
                ? white_atlas
                : build_atlas(taken->atlas.pixels.data(), taken->atlas.width,
                              taken->atlas.height);
        if (next) {
          for (std::size_t l = 0; l < level_count; ++l) {
            if (taken->meshes[l]) live[l] = *taken->meshes[l];
          }
          current_atlas = std::move(next);
          taken.reset();
        }
      }
    }
    slot_atlas[render_frame.slot] = current_atlas;
    panel.triangles.assign(level_count, 0);
    std::vector<vgp::HybridMeshDraw> draws;
    for (std::size_t l = 0; l < level_count; ++l) {
      frame_generations[l][render_frame.slot] = live[l].generation;
      panel.triangles[l] = live[l].triangle_count;
      if (live[l].valid() && !live[l].empty()) {
        vgp::LiveMesh mesh;
        mesh.vertices = live[l].vertices;
        mesh.indices = live[l].indices;
        mesh.indirect = live[l].indirect;
        draws.push_back(vgp::HybridMeshDraw{mesh});
      }
    }

    // A status line every 120 frames.
    if (tick % 120 == 0) {
      const double fps = profiler.metrics().fps;
      std::printf(
          "tick %d: %.1f fps, fused %zu/%zu, fuse %.2f ms (mean %.2f), "
          "floor %.2f mm |",
          tick, fps, panel.fused, panel.total, panel.fuse_ms,
          panel.fuse_mean_ms,
          panel.levels.empty() ? 0.0 : panel.levels[0].floor_mm);
      for (std::size_t l = 0; l < level_count; ++l) {
        const adaptive::LevelStats s =
            l < panel.levels.size() ? panel.levels[l] : adaptive::LevelStats{};
        std::printf(" L%zu %zu blk %zu ref %zu own %u tri off %.2f noise %.2f;",
                    l, s.blocks, s.refined, s.owned, panel.triangles[l],
                    s.median_offset_mm, s.median_noise_mm);
      }
      std::printf("\n");
    }

    // Follow the capture, then replay the path once fusion is done.
    const VkExtent2D extent = app.swapchain().extent();
    const float aspect =
        float(extent.width) / float(std::max(1u, extent.height));
    if (!poses.empty()) {
      if (!done) {
        view_frame = poses.size() - 1;
      } else if (tick % 2 == 0) {
        view_frame = (view_frame + 1) % poses.size();
      }
      if (!view) view = OrbitView::from_pose(poses.front(), 2.0f);
    }
    // Orbit, pan and zoom, unless a panel has the mouse. Taking over from the
    // followed camera starts from where it is looking.
    {
      const bool ui_mouse = overlay && ImGui::GetIO().WantCaptureMouse;
      double x = 0.0, y = 0.0;
      glfwGetCursorPos(window, &x, &y);
      const float dx = have_last ? float(x - last_x) : 0.0f;
      const float dy = have_last ? float(y - last_y) : 0.0f;
      last_x = x;
      last_y = y;
      have_last = true;
      const bool left =
          glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
      const bool right =
          glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
      if (view && !ui_mouse && (left || right || scroll.pending != 0.0)) {
        if (follow) {
          view = OrbitView::from_pose(
              poses[std::min(view_frame, poses.size() - 1)], 2.0f);
          follow = false;
        }
        const glm::vec3 look = glm::normalize(view->target - view->eye());
        const glm::vec3 side = glm::normalize(glm::cross(look, view->up));
        const glm::vec3 lift = glm::cross(side, look);
        if (left) {
          view->azimuth -= dx * 0.005f;
          view->elevation =
              std::clamp(view->elevation + dy * 0.005f, -1.55f, 1.55f);
        }
        if (right) {
          view->target += (-dx * side + dy * lift) * view->distance * 0.0015f;
        }
        if (scroll.pending != 0.0) {
          view->distance = std::max(
              0.05f, view->distance * std::pow(0.9f, float(scroll.pending)));
        }
      }
      scroll.pending = 0.0;
    }
    glm::mat4 view_proj(1.0f);
    if (!follow && view) {
      view_proj =
          vg::camera::Camera::look_at_perspective(
              view->eye(), view->target, view->up, vfov, aspect, 0.05f, 16.0f)
              .view_proj();
    } else if (!poses.empty()) {
      const glm::mat4& c2w = poses[std::min(view_frame, poses.size() - 1)];
      const glm::vec3 eye(c2w[3]);
      const glm::vec3 fwd(c2w[2]);
      const glm::vec3 up(-glm::vec3(c2w[1]));
      view_proj = vg::camera::Camera::look_at_perspective(
                      eye, eye + fwd, up, vfov, aspect, 0.05f, 16.0f)
                      .view_proj();
    }

    if (overlay) {
      ImGui_ImplGlfw_NewFrame();
      overlay->new_frame();
      ImGui::SetNextWindowPos(ImVec2(16.0f, 16.0f), ImGuiCond_FirstUseEver);
      ImGui::SetNextWindowSize(ImVec2(400.0f, 420.0f), ImGuiCond_FirstUseEver);
      vg::FrameMetrics metrics = profiler.metrics();
      metrics.sections.insert(metrics.sections.end(), stages_snapshot.begin(),
                              stages_snapshot.end());
      vg::ui::draw_metrics_panel(metrics, "Performance");
      ImGui::SetNextWindowPos(ImVec2(16.0f, 448.0f), ImGuiCond_FirstUseEver);
      ImGui::SetNextWindowSize(ImVec2(420.0f, 190.0f), ImGuiCond_FirstUseEver);
      draw_adaptive_panel(panel, show_levels, eps_mm, follow, opt.adaptive);
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
      if (!draws.empty()) {
        vgp::HybridMeshFrame hybrid_frame;
        hybrid_frame.extent = extent;
        hybrid_frame.view_proj = view_proj;
        hybrid_frame.light_dir = glm::vec3(0.4f, 0.9f, 0.5f);
        hybrid_frame.flags = opt.lit ? vgp::kHybridMeshLit : 0u;
        hybrid_frame.atlas = slot_atlas[render_frame.slot]->set.handle();
        hybrid_frame.draws = draws.data();
        hybrid_frame.draw_count = static_cast<std::uint32_t>(draws.size());
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
  quit.store(true);
  app.wait_idle();
  return exit_code;
}

}  // namespace

int main(int argc, char** argv) {
  // Line-buffered, so the status lines reach a redirected log as they print.
  std::setvbuf(stdout, nullptr, _IOLBF, 0);
  Options opt;
  if (!parse_args(argc, argv, opt)) return 2;
  if (glfwInit() != GLFW_TRUE) {
    std::fprintf(stderr, "glfwInit failed\n");
    return 1;
  }
  glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
  GLFWwindow* window = glfwCreateWindow(opt.width, opt.height,
                                        "adaptive_viewer", nullptr, nullptr);
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
