// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// fuse_render: the recon -> gfx interop demo, headless. Fuse a posed Replica
// RGB-D sequence -- polled through the sensor tier's IRgbdSensor interface and
// prepared on the GPU (sensor::GpuFramePrep) as a camera's frames are -- into
// a sparse TSDF+colour volume with volumetric_kit_recon,
// extract a marching-cubes mesh, hand it across the interop seam (a host mesh:
// recon extracts on its device, gfx uploads on its own), and render the
// coloured reconstruction to a PNG through volumetric_kit_gfx's
// HybridMeshPipeline (the per-vertex-colour renderer path). No window -- this
// proves the full recon->gfx colour handoff to a file the same rendering drives
// live later.
//
//   fuse_render <scene_dir> [-o out.png] [--voxel 0.02] [--trunc m]
//               [--min-depth m] [--max-depth m] [--max-frames N]
//               [--width 1280] [--height 720] [--yaw 45] [--pitch 30] [--lit]

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

#include "fuse_frame.hpp"  // vr_example::fuse_set
#include "recon_gfx_bridge.hpp"
#include "replica_sensor.hpp"  // vr_example::ReplicaSensor (examples/common)
#include "viewer_atlas.hpp"

// core and recon tiers
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/command_batch.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/core/vulkan/instance.hpp"
#include "volumetric_kit/core/vulkan/vulkan.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"
#include "volumetric_kit/recon/io/image_io.hpp"
#include "volumetric_kit/recon/mesh/marching_cubes.hpp"
#include "volumetric_kit/recon/mesh/mesh.hpp"
#include "volumetric_kit/recon/sensor/rgbd_frame.hpp"
#include "volumetric_kit/recon/sensor/rgbd_sensor.hpp"
#include "volumetric_kit/recon/sensor/utils/gpu_frame_prep.hpp"
#include "volumetric_kit/recon/texture/projective_texturer.hpp"
#include "volumetric_kit/recon/tsdf/fuser.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"
#include "volumetric_kit/recon/volume/voxel_grid.hpp"
#include "volumetric_kit/recon/volume/voxel_hash_map.hpp"

// gfx
#include "volumetric_kit/gfx/app/headless_app.hpp"
#include "volumetric_kit/gfx/camera/camera.hpp"
#include "volumetric_kit/gfx/core/offscreen_target.hpp"
#include "volumetric_kit/gfx/core/render_target.hpp"
#include "volumetric_kit/gfx/core/sampler.hpp"
#include "volumetric_kit/gfx/pipelines/gpu_mesh.hpp"
#include "volumetric_kit/gfx/pipelines/hybrid_mesh_pipeline.hpp"

namespace vr = volumetric_kit::recon;
namespace vkc = volumetric_kit::core;
namespace vol = volumetric_kit::recon::volume;
namespace rtsdf = volumetric_kit::recon::tsdf;
namespace rmesh = volumetric_kit::recon::mesh;
namespace rtex = volumetric_kit::recon::texture;
namespace rsensor = volumetric_kit::recon::sensor;
namespace vg = volumetric_kit::gfx;
namespace vgp = volumetric_kit::gfx::pipelines;

namespace {

struct Options {
  std::string scene_dir;
  std::string cam_params;
  std::string out = "fuse_render.png";
  float voxel = 0.02f;
  // Derived from `voxel` when left at 0 (see parse_args): a truncation band
  // fixed in metres is a band whose width *in voxels* changes with --voxel,
  // which silently degrades the reconstruction in both directions.
  float trunc = 0.0f;
  // The depth gate, both ends. Exposed as a pair because the capture takes a
  // pair: leaving the near plane implicit meant a --max-depth at or under the
  // default 0.1 m was refused with a message naming a knob this example never
  // offered.
  float min_depth = 0.1f;
  float max_depth = 8.0f;
  int max_frames = 400;
  int width = 1280;
  int height = 720;
  float yaw = 45.0f;    // degrees, around the up axis
  float pitch = 30.0f;  // degrees, above the horizon
  bool lit = false;     // flat raw colour by default (the reconstruction's own)
  int follow = -1;      // >=0: render from this trajectory frame's sensor pose
  bool texture = true;  // project the keyframe image onto the mesh (uv0 atlas)
  bool preload = false;  // decode every frame up front (RAM for decode time)
  // In-block vertex sharing (MarchingCubesConfig::share_vertices), which
  // `fuse_replica` already exposes and this example did not. It belongs here
  // because this is the only example that renders, and sharing is what a
  // memory-bound consumer turns on: an iOS scanner runs with it because the
  // vertex arena is the term that binds there.
  //
  // It is worth a flag specifically BECAUSE it runs beside --texture. The two
  // were mutually exclusive until the texture pass moved to a per-vertex
  // dispatch -- the texturer used to decide visibility per triangle and write
  // uv0 per vertex, so a shared vertex its triangles disagreed about was
  // written by whichever thread ran last, and it refused the pair outright.
  // One thread per vertex leaves one writer per vertex, and this is the
  // example that can show the result: rendering both combinations to a PNG
  // turns "byte-identical" from a claim into a diff.
  bool share_vertices = false;
};

bool parse_args(int argc, char** argv, Options& o) {
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto val = [&](const char* n) -> const char* {
      return (i + 1 < argc)
                 ? argv[++i]
                 : (std::fprintf(stderr, "%s needs a value\n", n), nullptr);
    };
    if (a == "-o" || a == "--out") {
      const char* v = val("-o");
      if (!v) return false;
      o.out = v;
    } else if (a == "--cam-params") {
      const char* v = val(a.c_str());
      if (!v) return false;
      o.cam_params = v;
    } else if (a == "--voxel") {
      const char* v = val(a.c_str());
      if (!v) return false;
      o.voxel = std::strtof(v, nullptr);
    } else if (a == "--trunc") {
      const char* v = val(a.c_str());
      if (!v) return false;
      o.trunc = std::strtof(v, nullptr);
    } else if (a == "--min-depth") {
      const char* v = val(a.c_str());
      if (!v) return false;
      o.min_depth = std::strtof(v, nullptr);
    } else if (a == "--max-depth") {
      const char* v = val(a.c_str());
      if (!v) return false;
      o.max_depth = std::strtof(v, nullptr);
    } else if (a == "--max-frames") {
      const char* v = val(a.c_str());
      if (!v) return false;
      o.max_frames = std::atoi(v);
    } else if (a == "--width") {
      const char* v = val(a.c_str());
      if (!v) return false;
      o.width = std::atoi(v);
    } else if (a == "--height") {
      const char* v = val(a.c_str());
      if (!v) return false;
      o.height = std::atoi(v);
    } else if (a == "--yaw") {
      const char* v = val(a.c_str());
      if (!v) return false;
      o.yaw = std::strtof(v, nullptr);
    } else if (a == "--pitch") {
      const char* v = val(a.c_str());
      if (!v) return false;
      o.pitch = std::strtof(v, nullptr);
    } else if (a == "--follow") {
      const char* v = val(a.c_str());
      if (!v) return false;
      o.follow = std::atoi(v);
    } else if (a == "--lit") {
      o.lit = true;
    } else if (a == "--no-texture") {
      o.texture = false;
    } else if (a == "--share-vertices") {
      o.share_vertices = true;
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
                 "usage: fuse_render <scene_dir> [-o out.png] [--voxel m] "
                 "[--trunc m] [--min-depth m] [--max-depth m] [--max-frames n] "
                 "[--yaw d] [--pitch d] [--lit] [--no-texture] "
                 "[--share-vertices] [--preload]\n");
    return false;
  }
  // strtof parses "nan"/"inf" without error, and a non-finite knob slips the
  // downstream guards (NaN compares false to every bound) to reach the grid
  // params, the GPU, or the camera math -- a silent, degenerate render. Reject
  // up front.
  if (!std::isfinite(o.voxel) || o.voxel <= 0.0f) {
    std::fprintf(stderr, "--voxel must be finite and > 0\n");
    return false;
  }
  // Default the band to 4 voxels, as fuse_replica does. Hardcoding it in metres
  // meant --voxel alone changed the band's width in voxels: at --voxel 0.05 the
  // old 0.08 is a 1.6-voxel band, leaving marching cubes a thin, hole-prone
  // zero set, and at --voxel 0.005 it is 16 voxels, which pushes the
  // allocation dilation from 27 blocks per surface block to 125. Neither is an
  // error, and the same flag reconstructed differently here than in
  // fuse_replica -- which defeats the A/B these examples exist for.
  if (o.trunc <= 0.0f) o.trunc = 4.0f * o.voxel;
  if (!std::isfinite(o.trunc) || o.trunc <= 0.0f) {
    std::fprintf(stderr, "--trunc must be finite and > 0\n");
    return false;
  }
  if (!std::isfinite(o.max_depth) || o.max_depth <= 0.0f) {
    std::fprintf(stderr, "--max-depth must be finite and > 0\n");
    return false;
  }
  // The same range rule the sensor applies, checked here where the flags
  // still have their names -- as fuse_replica does.
  if (!(o.min_depth > 0.0f) || o.min_depth >= o.max_depth) {
    std::fprintf(stderr, "--min-depth must be in (0, --max-depth)\n");
    return false;
  }
  if (!std::isfinite(o.yaw) || !std::isfinite(o.pitch)) {
    std::fprintf(stderr, "--yaw/--pitch must be finite\n");
    return false;
  }
  if (o.cam_params.empty()) o.cam_params = o.scene_dir + "/../cam_params.json";
  return true;
}

// --- Fuse a Replica sequence into a coloured host mesh (recon side). ---------

// The reconstruction handed to the renderer: the textured mesh plus the RGBA8
// atlas its uv0 index into (the keyframe image projected onto it). `atlas` is
// empty when texturing is off, and the caller binds a 1x1 white dummy instead.
struct Reconstruction {
  rmesh::Mesh mesh;
  // Canonical packed pixels (R | G<<8 | B<<16, the coverage byte on top: 0xFF
  // wherever the lens saw), atlas_w * atlas_h -- the bytes of an RGBA8 upload
  // on a little-endian host, which every Vulkan platform this repo targets
  // is.
  std::vector<std::uint32_t> atlas;
  std::uint32_t atlas_w = 0;
  std::uint32_t atlas_h = 0;
  /// The sensor's vertical field of view (radians), from its own intrinsics.
  /// Carried out of fuse() because --follow renders through the sensor and must
  /// match it; deriving it in main() from constants is how it came to use 360
  /// as the half-height of a 680-tall image.
  float sensor_vfov = 0.0f;
};

vkc::Result<Reconstruction> fuse(const Options& opt,
                                 std::vector<glm::mat4>& poses) {
  VKC_ASSIGN(vkc::Instance instance, vkc::Instance::create({}));
  VKC_ASSIGN(vkc::PhysicalDeviceInfo gpu,
             instance.select_physical_device(vr::device_requirements()));
  VKC_ASSIGN(vkc::Device device,
             vkc::Device::create(instance, gpu, vr::device_requirements()));
  VKC_ASSIGN(vkc::Allocator allocator,
             vkc::Allocator::create(instance.handle(), device));

  // The sequence arrives through the sensor interface; only this construction
  // knows it is a disk. The frame cap and the depth gate are the sensor's
  // options, so every frame it hands out is already gated.
  vr_example::ReplicaSensor::Options capture_options;
  capture_options.frame_limit =
      static_cast<std::size_t>(std::max(0, opt.max_frames));
  capture_options.min_depth = opt.min_depth;
  capture_options.max_depth = opt.max_depth;
  VKC_ASSIGN(vr_example::ReplicaSensor replica,
             vr_example::ReplicaSensor::open(opt.scene_dir, opt.cam_params,
                                             capture_options));
  const vr::camera::CameraModel& cam = *replica.info().color;
  const float cy = static_cast<float>(cam.intrinsics.cy);
  const float fy = static_cast<float>(cam.intrinsics.fy);
  // Split about the principal point rather than assuming it is centred: cy is
  // 339.5 on Replica, not height/2.
  const float sensor_vfov =
      std::atan(cy / fy) +
      std::atan((static_cast<float>(cam.size.height) - cy) / fy);

  VKC_ASSIGN(
      vol::VoxelBlockGrid volume,
      vr_example::create_fusion_grid(device, allocator, opt.voxel, opt.trunc));
  VKC_ASSIGN(rtsdf::Fuser fuser, rtsdf::Fuser::create(device, allocator));
  VKC_ASSIGN(rsensor::GpuFramePrep prep,
             rsensor::GpuFramePrep::create(device, allocator));
  rmesh::MarchingCubesConfig mc_config;
  mc_config.share_vertices = opt.share_vertices;
  VKC_ASSIGN(rmesh::MarchingCubes extractor,
             rmesh::MarchingCubes::create(device, allocator, mc_config));

  // Decode the sequence up front when asked, so the fuse loop below runs at
  // GPU speed instead of at JPEG/PNG decode speed (~75% of a streaming loop).
  // Costs ~4.9 MB per frame of RAM, announced before it is spent.
  if (opt.preload) {
    std::printf(
        "preloading %.0f MB...\n",
        static_cast<double>(replica.preload_bytes_projected()) / (1024 * 1024));
    VKC_ASSIGN(const std::size_t cached_frames, replica.preload());
    std::printf("preloaded %zu frames (%.0f MB)\n", cached_frames,
                static_cast<double>(replica.preloaded_bytes()) / (1024 * 1024));
  }

  // The keyframe the mesh is textured with, kept as prepared as it goes by:
  // the --follow frame if given, else the middle of the sequence. It holds its
  // buffers on the device, so the next prepare writes to new ones -- exactly
  // what a live consumer does with a keyframe, and why this never reaches
  // back into the dataset for it once fusion is done.
  const std::size_t keyframe_index =
      (opt.follow >= 0 &&
       static_cast<std::size_t>(opt.follow) < replica.frame_count())
          ? static_cast<std::size_t>(opt.follow)
          : replica.frame_count() / 2;
  std::optional<rsensor::DeviceFrame> keyframe;

  // From here on the source is the interface, not the dataset.
  rsensor::IRgbdSensor& capture = replica;
  VKC_TRY(capture.start());
  std::size_t fused = 0;
  for (;;) {
    // An empty poll is "nothing this tick", which a replay and an idle live
    // device report alike; only the source knows whether that is the end. A
    // decode failure -- a truncated JPEG, a wrong-size depth PNG -- is a real
    // failure and ends the run, where treating it as the end of the sequence
    // once made this leg write a partial-room PNG and exit 0.
    VKC_ASSIGN(const std::optional<rsensor::RgbdFrame> polled, capture.poll());
    if (!polled) {
      if (capture.exhausted()) {
        break;
      }
      // A live sensor polled faster than it runs: yield and ask again.
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    VKC_ASSIGN(rsensor::DeviceFrame frame, prep.prepare(*polled));
    poses.push_back(frame.depth_camera.cam_to_world);
    // Grow the map ahead of need, allocate the band and integrate depth +
    // colour, as fuse_replica does; an incomplete band is an error.
    VKC_TRY(vr_example::fuse_set(fuser, volume, {frame}, 20.0f, nullptr));
    if (opt.texture && fused == keyframe_index && frame.has_color()) {
      keyframe = std::move(frame);
    }
    ++fused;
  }
  std::printf("fused %zu frames\n", fused);

  Reconstruction recon;
  recon.sensor_vfov = sensor_vfov;
  VKC_ASSIGN(recon.mesh, extractor.extract_host(volume));

  // Project the retained keyframe onto the mesh (the live single-camera
  // texturing slice). Its uv0 mark the triangles that keyframe saw unoccluded;
  // the rest keep the sentinel and render with fused voxel colour. The atlas
  // the uv0 index into is that frame's own colour image (below), so texturing
  // keeps full sensor resolution where the camera had line of sight.
  if (keyframe && !recon.mesh.vertices.empty()) {
    const rsensor::DeviceFrame& kf = *keyframe;
    VKC_ASSIGN(rtex::ProjectiveTexturer texturer,
               rtex::ProjectiveTexturer::create(device, allocator));
    // Through the colour camera, the coverage read off the colour's high
    // byte: the frame as the GPU pass left it, as for any camera.
    rtex::TextureView view;
    view.cam = kf.depth_camera;
    view.depth_buffer = kf.depth;
    view.color_camera = kf.color_camera;
    view.coverage = kf.color;
    VKC_TRY(texturer.texture(recon.mesh, view));

    // Atlas = the keyframe's colour image at full resolution -- exactly what
    // uv0 = (pixel + 0.5)/size index -- read back from the device. The pass
    // refuses colour it cannot make canonical, so these are canonical bytes,
    // R | G<<8 | B<<16 and the coverage byte, opaque wherever the lens saw.
    // The atlas is uploaded as _SRGB below, which assumes exactly that, onto
    // gfx's own device: hence the host copy (seam A).
    const std::size_t pixels = static_cast<std::size_t>(kf.color_camera.width) *
                               kf.color_camera.height;
    recon.atlas.resize(pixels);
    vkc::CommandBatch batch(device, allocator);
    VKC_TRY(batch.readback(*kf.color, 0, pixels * sizeof(std::uint32_t),
                           recon.atlas.data()));
    VKC_TRY(batch.submit());
    recon.atlas_w = kf.color_camera.width;
    recon.atlas_h = kf.color_camera.height;
    std::printf("textured with frame %zu (%ux%u atlas)\n", keyframe_index,
                recon.atlas_w, recon.atlas_h);
  }
  return recon;
}

}  // namespace

int main(int argc, char** argv) {
  Options opt;
  if (!parse_args(argc, argv, opt)) return 2;

  // 1. Fuse + extract the reconstruction (recon device).
  std::vector<glm::mat4> poses;
  vkc::Result<Reconstruction> recon_result = fuse(opt, poses);
  if (!recon_result) {
    std::fprintf(stderr, "fuse failed: %s\n",
                 recon_result.status().message().c_str());
    return 1;
  }
  const Reconstruction recon = std::move(recon_result).value();
  const rmesh::Mesh& mesh = recon.mesh;
  if (mesh.vertices.empty()) {
    std::fprintf(stderr, "empty reconstruction\n");
    return 1;
  }

  // Bounding box -> camera framing.
  glm::vec3 lo(1e30f), hi(-1e30f);
  for (const rmesh::Vertex& v : mesh.vertices) {
    lo = glm::min(lo, glm::vec3(v.position));
    hi = glm::max(hi, glm::vec3(v.position));
  }
  const glm::vec3 center = 0.5f * (lo + hi);
  // Floor the radius so the orbit near/far planes below stay ordered
  // (z_near = max(0.01, r*0.05) < z_far = r*6) even for a tiny/degenerate
  // reconstruction; the --follow path uses fixed planes and is unaffected.
  const float radius = std::max(0.5f * glm::length(hi - lo), 0.05f);
  const float aspect =
      static_cast<float>(opt.width) / static_cast<float>(opt.height);
  glm::mat4 view_proj;
  if (opt.follow >= 0 && static_cast<std::size_t>(opt.follow) < poses.size()) {
    // Render from the sensor's own pose (OpenCV: +Z forward, +Y down) -- the
    // same follow-camera math the live viewer uses.
    const glm::mat4& c2w = poses[static_cast<std::size_t>(opt.follow)];
    const glm::vec3 eye(c2w[3]);
    // The sensor's own vertical FOV, carried out of fuse(). It used to be
    // 2 * atan(360 / 600), wrong twice over: the half-height of a 680-tall
    // sensor is 340, not 360, and hardcoding 600 for fy silently ignored
    // --cam-params.
    const float vfov = recon.sensor_vfov;
    view_proj = vg::camera::Camera::look_at_perspective(
                    eye, eye + glm::vec3(c2w[2]), -glm::vec3(c2w[1]), vfov,
                    aspect, 0.05f, 16.0f)
                    .view_proj();
    std::printf("follow frame %d: eye=(%.2f,%.2f,%.2f)\n", opt.follow, eye.x,
                eye.y, eye.z);
  } else {
    const float yaw = glm::radians(opt.yaw);
    const float pitch = glm::radians(opt.pitch);
    const glm::vec3 dir(std::cos(pitch) * std::cos(yaw), std::sin(pitch),
                        std::cos(pitch) * std::sin(yaw));
    const glm::vec3 eye = center + dir * (radius * 2.4f);
    view_proj =
        vg::camera::Camera::look_at_perspective(
            eye, center, glm::vec3(0.0f, 1.0f, 0.0f), glm::radians(50.0f),
            aspect, std::max(0.01f, radius * 0.05f), radius * 6.0f)
            .view_proj();
  }

  // 2. gfx device (its own), offscreen target, hybrid-mesh pipeline.
  vg::app::HeadlessAppConfig app_config;
  app_config.app_name = "fuse_render";
  auto app_r = vg::app::HeadlessApp::create(app_config);
  if (!app_r.ok()) {
    std::fprintf(stderr, "HeadlessApp: %s\n", app_r.status().message().c_str());
    return 1;
  }
  vg::app::HeadlessApp app = std::move(app_r).value();

  vg::OffscreenTargetDesc td;
  td.extent = {static_cast<std::uint32_t>(opt.width),
               static_cast<std::uint32_t>(opt.height)};
  // _SRGB, not _UNORM: this is a presentation site, and the only encode on the
  // headless path. The mesh's vertex colors and the decoded atlas are LINEAR
  // (the 2026-08-02 color-space decision), and the readback goes straight into
  // a PNG, which every viewer reads as sRGB -- so nothing else would encode and
  // the image would come out dark. The hardware does it here for free.
  td.color_format = VK_FORMAT_R8G8B8A8_SRGB;
  td.depth_format = VK_FORMAT_D32_SFLOAT;
  auto target_r = vg::OffscreenTarget::create(app.allocator(), td);
  if (!target_r.ok()) {
    std::fprintf(stderr, "OffscreenTarget: %s\n",
                 target_r.status().message().c_str());
    return 1;
  }
  vg::OffscreenTarget target = std::move(target_r).value();

  auto pipeline_result =
      vgp::HybridMeshPipeline::create(app.device().handle(), target.layout());
  if (!pipeline_result.ok()) {
    std::fprintf(stderr, "HybridMeshPipeline: %s\n",
                 pipeline_result.status().message().c_str());
    return 1;
  }
  vgp::HybridMeshPipeline pipeline = std::move(pipeline_result).value();

  // 3. Upload the mesh (recon -> gfx, via the host bridge).
  const vg::assets::Mesh gfx_mesh = fuse_viewer::to_gfx_mesh(mesh);
  auto gpu_r = vgp::upload_mesh(app.device(), app.allocator(), gfx_mesh);
  if (!gpu_r.ok()) {
    std::fprintf(stderr, "upload_mesh: %s\n", gpu_r.status().message().c_str());
    return 1;
  }
  vgp::GpuMesh gpu_mesh = std::move(gpu_r).value();

  // 4. Atlas: the keyframe's colour image where texturing ran (uv0 index into
  // it), else the white dummy.
  auto sampler_r = vg::Sampler::create(app.device().handle());
  if (!sampler_r.ok()) {
    std::fprintf(stderr, "sampler: %s\n", sampler_r.status().message().c_str());
    return 1;
  }
  vg::Sampler sampler = std::move(sampler_r).value();
  auto atlas_r = recon.atlas.empty()
                     ? fuse_viewer::white_atlas(
                           app.device(), app.allocator(),
                           pipeline.descriptor_set_layout(0), sampler.handle())
                     : fuse_viewer::upload_atlas(
                           app.device(), app.allocator(),
                           pipeline.descriptor_set_layout(0), sampler.handle(),
                           recon.atlas.data(), recon.atlas_w, recon.atlas_h);
  if (!atlas_r.ok()) {
    std::fprintf(stderr, "atlas: %s\n", atlas_r.status().message().c_str());
    return 1;
  }
  const std::shared_ptr<fuse_viewer::Atlas> atlas = std::move(atlas_r).value();

  // 5. Render one frame to the offscreen target, then read it back.
  const vgp::HybridMeshDraw draw{&gpu_mesh};
  vgp::HybridMeshFrame frame;
  frame.extent = td.extent;
  frame.view_proj = view_proj;
  frame.light_dir = glm::vec3(0.4f, 0.9f, 0.5f);
  frame.flags = opt.lit ? vgp::kHybridMeshLit : 0u;
  frame.atlas = atlas->set.handle();
  frame.draws = &draw;
  frame.draw_count = 1;

  const vkc::Status rendered =
      app.device().submit_single_time([&](VkCommandBuffer cmd) {
        target.prepare(cmd);
        const vg::RenderTarget rt = target.target();
        vg::RenderTargetBeginInfo begin_info;
        // A float colour, so under the one colour rule it is LINEAR, and the
        // _SRGB target encodes it on clear -- these land at codes ~63/63/75 in
        // the PNG rather than the ~13/13/18 the same literals gave against the
        // old _UNORM target. Kept as written rather than re-darkened: it is the
        // background fuse_viewer has always drawn (its swapchain was already
        // _SRGB with these very values), so the two examples now agree instead
        // of differing by an encode.
        begin_info.clear_color.float32[0] = 0.05f;
        begin_info.clear_color.float32[1] = 0.05f;
        begin_info.clear_color.float32[2] = 0.07f;
        begin_info.clear_color.float32[3] = 1.0f;
        rt.begin(cmd, begin_info);
        pipeline.submit(cmd, frame);
        rt.end(cmd);
        target.record_readback(cmd);
      });
  if (!rendered.ok()) {
    std::fprintf(stderr, "render: %s\n", rendered.message().c_str());
    return 1;
  }

  const auto* pixels = static_cast<const std::uint8_t*>(target.pixels());
  if (pixels == nullptr) {
    std::fprintf(stderr, "no readback pixels\n");
    return 1;
  }
  // gfx sizes the readback buffer to the target's extent at the RGBA8 texel
  // size chosen above; it exposes no byte count of its own.
  const VkExtent2D extent = target.extent();
  const vkc::Status written = vr::io::write_png_rgba8(
      opt.out, pixels,
      static_cast<std::size_t>(extent.width) * extent.height * 4, extent.width,
      extent.height);
  if (!written) {
    std::fprintf(stderr, "write PNG: %s\n", written.message().c_str());
    return 1;
  }
  // Vertices as well as triangles, and the ratio between them: that ratio is
  // the whole observable effect of --share-vertices (3.00 when off, since the
  // mesh tier emits independent triangles; well under it when on), and it is
  // what prices the vertex arena a memory-bound consumer is trading against.
  // Printing only the triangle count made the flag look like a no-op -- sharing
  // does not change how many triangles there are.
  std::printf(
      "rendered %zu triangles, %zu vertices (%.2f v/tri) -> %s (%dx%d), "
      "center=(%.2f,%.2f,%.2f)\n",
      mesh.triangle_count(), mesh.vertices.size(),
      mesh.triangle_count() > 0 ? static_cast<double>(mesh.vertices.size()) /
                                      static_cast<double>(mesh.triangle_count())
                                : 0.0,
      opt.out.c_str(), opt.width, opt.height, center.x, center.y, center.z);
  return 0;
}
