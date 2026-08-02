// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// fuse_render: the recon -> gfx interop demo, headless. Fuse a posed Replica
// RGB-D sequence into a sparse TSDF+colour volume with volumetric_kit_recon,
// extract a marching-cubes mesh, hand it across the interop seam (a host mesh:
// recon extracts on its device, gfx uploads on its own), and render the
// coloured reconstruction to a PNG through volumetric_kit_gfx's
// HybridMeshPipeline (the per-vertex-colour renderer path). No window -- this
// proves the full recon->gfx colour handoff to a file the same rendering drives
// live later.
//
//   fuse_render <scene_dir> [-o out.png] [--voxel 0.02] [--max-frames N]
//               [--width 1280] [--height 720] [--yaw 45] [--pitch 30] [--lit]

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

#include "dataset.hpp"  // vr_example::ReplicaDataset (recon examples/common)
#include "example_camera.hpp"  // vr_example::make_depth_camera
#include "image_io.hpp"        // vr_example::pack_color_rgba8
#include "recon_gfx_bridge.hpp"

// recon tiers
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

// gfx
#include "volumetric_kit/gfx/app/headless_app.hpp"
#include "volumetric_kit/gfx/camera/camera.hpp"
#include "volumetric_kit/gfx/core/descriptor.hpp"
#include "volumetric_kit/gfx/core/offscreen_target.hpp"
#include "volumetric_kit/gfx/core/render_target.hpp"
#include "volumetric_kit/gfx/core/sampler.hpp"
#include "volumetric_kit/gfx/core/texture_upload.hpp"
#include "volumetric_kit/gfx/core/vulkan.hpp"
#include "volumetric_kit/gfx/pipelines/gpu_mesh.hpp"
#include "volumetric_kit/gfx/pipelines/hybrid_mesh_pipeline.hpp"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

namespace vr = volumetric_kit::recon;
namespace vol = volumetric_kit::recon::volume;
namespace rtsdf = volumetric_kit::recon::tsdf;
namespace rmesh = volumetric_kit::recon::mesh;
namespace rtex = volumetric_kit::recon::texture;
namespace vg = volumetric_kit::gfx;
namespace vgp = volumetric_kit::gfx::pipelines;

namespace {

struct Options {
  std::string scene_dir;
  std::string cam_params;
  std::string out = "fuse_render.png";
  float voxel = 0.02f;
  float trunc = 0.08f;
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
                 "[--max-frames n] [--yaw d] [--pitch d] [--lit] "
                 "[--preload]\n");
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
  if (!std::isfinite(o.max_depth) || o.max_depth <= 0.0f) {
    std::fprintf(stderr, "--max-depth must be finite and > 0\n");
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
  std::vector<std::uint8_t> atlas;  // RGBA8, atlas_w * atlas_h * 4
  std::uint32_t atlas_w = 0;
  std::uint32_t atlas_h = 0;
};

vr::Result<Reconstruction> fuse(const Options& opt,
                                std::vector<glm::mat4>& poses) {
  VR_ASSIGN(vr::Instance instance, vr::Instance::create({}));
  VR_ASSIGN(VkPhysicalDevice gpu, instance.select_physical_device());
  VR_ASSIGN(vr::Device device, vr::Device::create(instance.handle(), gpu, {}));
  VR_ASSIGN(vr::Allocator allocator,
            vr::Allocator::create(instance.handle(), device));

  VR_ASSIGN(vr_example::ReplicaDataset dataset,
            vr_example::ReplicaDataset::open(opt.scene_dir, opt.cam_params));
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
  VR_ASSIGN(vol::VoxelBlockGrid volume,
            vol::VoxelBlockGrid::create(device, allocator, grid, attrs, 3));
  VR_ASSIGN(rtsdf::TsdfIntegrator integrator,
            rtsdf::TsdfIntegrator::create(device, allocator));
  VR_ASSIGN(rmesh::MarchingCubes extractor,
            rmesh::MarchingCubes::create(device, allocator));

  const auto last = std::min<std::size_t>(
      dataset.frame_count(),
      static_cast<std::size_t>(std::max(0, opt.max_frames)));

  // Decode the sequence up front when asked, so the fuse loop below runs at
  // GPU speed instead of at JPEG/PNG decode speed (~75% of a streaming loop).
  // Costs ~6 MB per frame of RAM, announced before it is spent.
  if (opt.preload) {
    std::printf("preloading %.0f MB...\n",
                static_cast<double>(dataset.preload_bytes_projected(last)) /
                    (1024 * 1024));
    VR_ASSIGN(const std::size_t cached_frames, dataset.preload(last));
    std::printf("preloaded %zu frames (%.0f MB)\n", cached_frames,
                static_cast<double>(dataset.preloaded_bytes()) / (1024 * 1024));
  }

  std::size_t fused = 0;
  for (std::size_t i = 0; i < last; ++i) {
    vr::Result<vr_example::FrameView> frame_result = dataset.frame(i);
    if (!frame_result) break;  // ran past on-disk frames
    const vr_example::FrameView view = std::move(frame_result).value();
    const vr_example::RgbdFrame& frame = *view;
    poses.push_back(frame.cam_to_world);

    const vol::DepthCameraParams depth_camera =
        vr_example::make_depth_camera(cam, frame.cam_to_world, opt.max_depth);
    rtsdf::ColorCameraParams color_camera{};
    color_camera.fx = cam.fx;
    color_camera.fy = cam.fy;
    color_camera.cx = cam.cx;
    color_camera.cy = cam.cy;
    color_camera.width = cam.width;
    color_camera.height = cam.height;
    color_camera.cam_to_world = frame.cam_to_world;
    const rtsdf::ColorFrame color_frame{frame.color.data(), color_camera};

    bool allocated = false;
    for (int attempt = 0; attempt < 5; ++attempt) {
      VR_ASSIGN(std::uint32_t failed, volume.map().allocate_from_depth(
                                          frame.depth.data(), depth_camera));
      if (failed == 0) {
        allocated = true;
        break;
      }
      VR_TRY(volume.resize(volume.grid().num_buckets * 2));
    }
    // Don't integrate a frame whose surface band never fully allocated (silent
    // holes); report the overflow cleanly instead, as fuse_replica does.
    if (!allocated) {
      return vr::Status::out_of_memory(
          "fuse_render: allocation kept overflowing after resize");
    }
    VR_TRY(integrator.integrate(volume, frame.depth.data(), depth_camera, 20.0f,
                                rtsdf::IntegrationMode::Classic, &color_frame));
    ++fused;
  }
  std::printf("fused %zu frames\n", fused);

  Reconstruction recon;
  VR_ASSIGN(recon.mesh, extractor.extract(volume));

  // Project one keyframe onto the mesh (the live single-camera texturing
  // slice): the --follow frame if given, else the middle fused frame. Its uv0
  // mark the triangles that keyframe saw unoccluded; the rest keep the sentinel
  // and render with fused voxel colour. The atlas the uv0 index into is that
  // frame's own colour image (below), so texturing keeps full sensor resolution
  // where the camera had line of sight.
  if (opt.texture && fused > 0 && !recon.mesh.vertices.empty()) {
    VR_ASSIGN(rtex::ProjectiveTexturer texturer,
              rtex::ProjectiveTexturer::create(device, allocator));
    const int tex_idx =
        (opt.follow >= 0 && static_cast<std::size_t>(opt.follow) < fused)
            ? opt.follow
            : static_cast<int>(fused / 2);
    VR_ASSIGN(vr_example::FrameView keyframe,
              dataset.frame(static_cast<std::size_t>(tex_idx)));
    const vol::DepthCameraParams keyframe_camera =
        vr_example::make_depth_camera(cam, keyframe->cam_to_world,
                                      opt.max_depth);
    VR_TRY(
        texturer.texture(recon.mesh, keyframe->depth.data(), keyframe_camera));

    // Atlas = the keyframe's colour image as RGBA8 at full resolution --
    // exactly what uv0 = (pixel + 0.5)/size index.
    recon.atlas = vr_example::pack_color_rgba8(keyframe->color);
    recon.atlas_w = cam.width;
    recon.atlas_h = cam.height;
    std::printf("textured with frame %d (%ux%u atlas)\n", tex_idx, cam.width,
                cam.height);
  }
  return recon;
}

}  // namespace

int main(int argc, char** argv) {
  Options opt;
  if (!parse_args(argc, argv, opt)) return 2;

  // 1. Fuse + extract the reconstruction (recon device).
  std::vector<glm::mat4> poses;
  vr::Result<Reconstruction> recon_result = fuse(opt, poses);
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
    const float vfov = 2.0f * std::atan(360.0f / 600.0f);  // ~Replica vfov
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
  auto app_r = vg::app::HeadlessApp::create({/*app_name=*/"fuse_render"});
  if (!app_r.ok()) {
    std::fprintf(stderr, "HeadlessApp: %s\n", app_r.status().message().c_str());
    return 1;
  }
  vg::app::HeadlessApp app = std::move(app_r).value();

  vg::OffscreenTargetDesc td;
  td.extent = {static_cast<std::uint32_t>(opt.width),
               static_cast<std::uint32_t>(opt.height)};
  td.color_format = VK_FORMAT_R8G8B8A8_UNORM;
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
  // it), else a 1x1 white dummy (the binding is required; the shader takes the
  // vertex-colour path wherever uv0 is the sentinel).
  const std::uint8_t white[4] = {255, 255, 255, 255};
  const bool has_atlas = !recon.atlas.empty();
  vg::ImageUploadDesc upload_desc;
  upload_desc.extent =
      has_atlas ? VkExtent2D{recon.atlas_w, recon.atlas_h} : VkExtent2D{1, 1};
  upload_desc.format = VK_FORMAT_R8G8B8A8_UNORM;
  upload_desc.pixels = has_atlas ? recon.atlas.data() : white;
  upload_desc.size = has_atlas ? recon.atlas.size() : sizeof(white);
  auto atlas_tex_r =
      vg::upload_texture(app.device(), app.allocator(), upload_desc);
  if (!atlas_tex_r.ok()) {
    std::fprintf(stderr, "atlas upload: %s\n",
                 atlas_tex_r.status().message().c_str());
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
  auto pool_result =
      vg::DescriptorPool::create(app.device().handle(), &pool_size, 1, 1);
  if (!pool_result.ok()) {
    std::fprintf(stderr, "descriptor pool: %s\n",
                 pool_result.status().message().c_str());
    return 1;
  }
  vg::DescriptorPool pool = std::move(pool_result).value();
  auto set_result = pool.allocate(pipeline.descriptor_set_layout(0));
  if (!set_result.ok()) {
    std::fprintf(stderr, "atlas set: %s\n",
                 set_result.status().message().c_str());
    return 1;
  }
  vg::DescriptorSet atlas_set = std::move(set_result).value();
  atlas_set.write_combined_image_sampler(
      0, atlas_tex.view(), sampler.handle(),
      VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

  // 5. Render one frame to the offscreen target, then read it back.
  const vgp::HybridMeshDraw draw{&gpu_mesh};
  vgp::HybridMeshFrame frame;
  frame.extent = td.extent;
  frame.view_proj = view_proj;
  frame.light_dir = glm::vec3(0.4f, 0.9f, 0.5f);
  frame.flags = opt.lit ? vgp::kHybridMeshLit : 0u;
  frame.atlas = atlas_set.handle();
  frame.draws = &draw;
  frame.draw_count = 1;

  const vg::Status rendered =
      app.device().submit_single_time([&](VkCommandBuffer cmd) {
        target.prepare(cmd);
        const vg::RenderTarget rt = target.target();
        vg::RenderTargetBeginInfo begin_info;
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
  if (stbi_write_png(opt.out.c_str(), opt.width, opt.height, 4, pixels,
                     opt.width * 4) == 0) {
    std::fprintf(stderr, "stbi_write_png failed for %s\n", opt.out.c_str());
    return 1;
  }
  std::printf("rendered %zu triangles -> %s (%dx%d), center=(%.2f,%.2f,%.2f)\n",
              mesh.triangle_count(), opt.out.c_str(), opt.width, opt.height,
              center.x, center.y, center.z);
  return 0;
}
