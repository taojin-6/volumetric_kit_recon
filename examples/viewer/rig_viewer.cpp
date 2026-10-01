// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// rig_viewer: live reconstruction from a synced rig of Orbbec cameras, drawn
// as it grows and textured from every camera at once. fuse_viewer's sibling
// for a live rig rather than a recorded sequence: the same shared device (the
// neutral bootstrap in shared_device.hpp), the same fuse thread and mesh ring,
// and the same HybridMeshPipeline drawing recon's buffers in place.
//
// Nothing the cameras produce visits the host after it arrives. The rig hands
// out raw sets whose colour the hardware decoder left on the GPU;
// sensor::prepare_set undistorts and converts each camera's frame there; the
// frames fuse through the device-input overloads; the marching-cubes mesh is
// textured from every camera of the set (texture::ProjectiveTexturer, each
// view's depth on the device, its colour camera its own, and its colour the
// coverage that keeps the lens's black corners off the mesh); and the atlas is
// filled by copying each camera's colour buffer into its tile of a gfx image,
// recorded in the renderer's own frame before the draw. The only copies are
// device to device.
//
// What makes the atlas copy safe, each checked or derived rather than assumed:
//   * sharing -- the frame prep's colour is made CONCURRENT across recon's
//     and gfx's queue families (GpuFramePrepConfig::color_queue_families),
//     since MoltenVK hands the two libraries different families, where reading
//     an EXCLUSIVE buffer is undefined in the way that appears to work.
//     Verified per buffer before its camera textures the mesh, so a buffer gfx
//     cannot copy costs that camera's triangles rather than the mesh's texture.
//   * lifetime -- the colour buffers a copy reads are held by the frame slot
//     that recorded it until begin_frame fence-waits that slot again, and an
//     atlas image is reused only once no frame in flight and no committed
//     mesh holds it.
//   * visibility -- as for the mesh: the frame prep fence-waits its batch
//     before prepare_set returns, and gfx's later vkQueueSubmit makes those
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
//   rig_viewer --rig sync.json [--calibration calib.json] [--apply-sync]
//              [--hevc | --mjpeg] [--color WxH] [--fps N]
//              [--voxel 0.01] [--trunc m] [--min-depth m] [--max-depth m]
//              [--max-weight 20] [--dynamic | --static] [--occlusion 0.05]
//              [--remesh-every 1] [--hold-ms 2000] [--sets N] [--frames N]
//              [--width 1280] [--height 720] [--lit | --normals]
//              [--no-texture] [--show-sources] [--texture-stats] [--all-depth]
//              [--no-overlay] [--validation]
//
// Without --calibration every camera sits at the world origin, which fuses a
// rig of several into nonsense; the viewer says so and runs anyway. Drag with
// the left button to orbit, the right to pan, and scroll to zoom; the View
// panel switches shading, texturing and the viewpoint, and tunes the fusion
// and texturing knobs below live. Closing the window, or Ctrl+C in the
// terminal, stops the rig before the process ends.

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
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

#include "fuse_device_frame.hpp"  // vr_example::fuse_set
#include "fuse_frame.hpp"         // vr_example::create_fusion_grid
// For the vertex-layout static_asserts it carries: gfx reads recon's arena in
// place through its own attribute offsets (see fuse_viewer.cpp).
#include "recon_gfx_bridge.hpp"
#include "shared_device.hpp"
#include "stage_metrics.hpp"  // fuse_viewer::to_sections
#include "viewer_common.hpp"

#include "volumetric_kit/recon/core/allocator.hpp"
#include "volumetric_kit/recon/core/buffer.hpp"
#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/core/stage_metrics.hpp"
#include "volumetric_kit/recon/mesh/device_mesh.hpp"
#include "volumetric_kit/recon/mesh/marching_cubes.hpp"
#include "volumetric_kit/recon/sensor/orbbec/orbbec_capture.hpp"
#include "volumetric_kit/recon/sensor/orbbec/orbbec_rig.hpp"
#include "volumetric_kit/recon/sensor/orbbec/orbbec_sync_config.hpp"
#include "volumetric_kit/recon/sensor/rig_calibration.hpp"
#include "volumetric_kit/recon/sensor/utils/gpu_frame_prep.hpp"
#include "volumetric_kit/recon/texture/projective_texturer.hpp"
#include "volumetric_kit/recon/texture/texture_atlas.hpp"
#include "volumetric_kit/recon/tsdf/tsdf_integrator.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"
#include "volumetric_kit/recon/volume/voxel_hash_map.hpp"

#include "volumetric_kit/gfx/app/windowed_app.hpp"
#include "volumetric_kit/gfx/camera/camera.hpp"
#include "volumetric_kit/gfx/core/allocator.hpp"
#include "volumetric_kit/gfx/core/descriptor.hpp"
#include "volumetric_kit/gfx/core/frame_metrics.hpp"
#include "volumetric_kit/gfx/core/image_barrier.hpp"
#include "volumetric_kit/gfx/core/profiler.hpp"
#include "volumetric_kit/gfx/core/render_target.hpp"
#include "volumetric_kit/gfx/core/result.hpp"
#include "volumetric_kit/gfx/core/sampler.hpp"
#include "volumetric_kit/gfx/core/texture.hpp"
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
namespace rsensor = volumetric_kit::recon::sensor;
namespace vg = volumetric_kit::gfx;
namespace vgp = volumetric_kit::gfx::pipelines;
namespace win = volumetric_kit::gfx::windowing;

namespace {

// Ctrl+C (or a SIGTERM) in the terminal closes the window rather than ending
// the process there: the render loop then leaves as it does on a click, the
// fuse thread stops the rig, and the cameras are released. Killed outright, the
// process would leave them to notice the dropped connection. The handler
// restores the default, so a second Ctrl+C still ends a run that has hung.
// Lock-free, as a signal handler may only touch such an atomic.
std::atomic<bool> g_interrupted{false};
static_assert(std::atomic<bool>::is_always_lock_free,
              "the interrupt flag must be safe to set from a signal handler");
extern "C" void on_interrupt(int signal) {
  g_interrupted.store(true);
  std::signal(signal, SIG_DFL);
}

// How long the fuse thread waits for a set before saying the rig is silent. It
// keeps waiting after saying so: a window is not a batch run, and a camera
// coming back is worth drawing.
constexpr auto kSilenceLimit = std::chrono::seconds(10);

// The three ways the hybrid pipeline shades, as the View panel offers them.
enum class Shading { kLit, kUnlit, kNormals };

std::uint32_t shading_flags(Shading shading) {
  switch (shading) {
    case Shading::kLit:
      return vgp::kHybridMeshLit;
    case Shading::kUnlit:
      return 0u;
    case Shading::kNormals:
      return vgp::kHybridMeshNormals;
  }
  return vgp::kHybridMeshLit;
}

struct Options {
  std::string rig;          // the sync configuration; required
  std::string calibration;  // poses by serial; empty: all at the origin
  bool apply_sync = false;  // write the sync configuration where it differs
  // H.265 colour unless --mjpeg: raw MJPEG takes about nine times the
  // bandwidth (185 against 21 Mbit/s a camera at 4K).
  bool hevc = true;
  std::uint32_t color_width = 0;  // 0 keeps the driver's default mode
  std::uint32_t color_height = 0;
  std::uint32_t fps = 0;
  // 1 cm: a Femto Mega's depth pixel covers about 4 mm at 1.5 m, so the rig
  // resolves finer than fuse_viewer's 2 cm default for a phone-scale scan.
  float voxel = 0.01f;             // metres
  float trunc = 0.0f;              // 0 => 4 * voxel, as the other examples
  std::optional<float> min_depth;  // unset keeps the driver's defaults
  std::optional<float> max_depth;
  float max_weight = 20.0f;
  // Dynamic integration by default: a voxel a camera now sees as free space
  // is cleared, so a surface that moves away is gone on the next set rather
  // than fading over max_weight frames, and one arriving there forms at once.
  // --static fuses Classic, keeping it.
  bool dynamic = true;
  // Fuse depth only where the camera's colour camera recorded it
  // (GpuFramePrepConfig::depth_within_color): the Femto Mega's depth sees
  // about 65 degrees vertically to its 16:9 colour's 51, and depth past the
  // colour fuses into surfaces no camera can colour or texture, which draw
  // white. --all-depth fuses all of it.
  bool depth_within_color = true;
  // How far, in metres, a camera's depth may disagree with the fused surface
  // for it to texture a vertex (ProjectiveTexturer's occlusion_threshold).
  // 5 cm, not the tier's 2 cm: on the lab rig, calibrated, 2 cm left 28% of
  // the mesh untextured and 5 cm 17% with all depth fused, 8.6% with
  // depth_within_color (see the 2026-09-29 decision's amendments).
  float occlusion = 0.05f;
  // Sets between re-extracts: every set, so the mesh follows the cameras. A
  // set with a remesh cost about 21.6 ms on the M5 Max at 1 cm (four cameras
  // fused, extracted and textured), inside the 33 ms a 30 fps rig leaves.
  int remesh_every = 1;
  // How old, on the rig's clock, a camera's last frame may be and still
  // texture the mesh when a set lacks the camera, whose triangles would
  // otherwise flicker to fused colour. The SDK drops a secondary's frame now
  // and then, and a lost H.265 frame costs every frame to the next key frame
  // (30 at the camera's default, 1.2 s at 25 fps). The held frame is a
  // fallback view (TextureView::fallback) with its own depth, so it takes
  // only triangles no present camera does, where that depth still agrees
  // with the mesh within the occlusion threshold -- a geometric test, which
  // a surface that slid along itself passes. Holding costs the frame prep
  // its buffer reuse: 0.1 ms a set for four 4K cameras on an RTX 5090. 0
  // turns it off.
  int hold_ms = 2000;
  int sets = 0;  // stop fusing after N sets; 0 fuses until the window closes
  // Draw N frames, then exit as a closed window does -- for a scripted run;
  // 0 draws until the window is closed.
  int frames = 0;
  int width = 1280;
  int height = 720;
  // Unlit by default: the cameras' own colour, as they saw it, with no
  // light to darken the faces turned from it.
  Shading shading = Shading::kUnlit;
  bool texture = true;  // texture from the cameras; off: fused colour
  // Each camera's tile filled with a colour of its own rather than its image,
  // so the window shows which camera textured each triangle.
  bool show_sources = false;
  // Now and then read the mesh back and print how much of it each camera
  // textured. Costs a readback of the whole mesh, so off by default.
  bool texture_stats = false;
  bool overlay = true;      // Dear ImGui panels
  bool validation = false;  // Vulkan validation layer on the shared device
};

const char* kUsage =
    "usage: rig_viewer --rig sync.json [--calibration calib.json] "
    "[--apply-sync] [--hevc | --mjpeg] [--color WxH] [--fps N] [--voxel m] "
    "[--trunc m] [--min-depth m] [--max-depth m] [--max-weight w] "
    "[--dynamic | --static] [--occlusion m] "
    "[--remesh-every N] [--hold-ms N] [--sets N] [--frames N] "
    "[--width W] [--height H] "
    "[--lit | --normals] [--no-texture] [--show-sources] [--texture-stats] "
    "[--all-depth] "
    "[--no-overlay] [--validation]\n";

bool parse_args(int argc, char** argv, Options& o) {
  bool codec_given = false;
  bool shading_given = false;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto value = [&]() -> const char* {
      return (i + 1 < argc) ? argv[++i] : nullptr;
    };
    auto number = [&](float& dst) -> bool {
      const char* x = value();
      if (x == nullptr) return false;
      dst = std::strtof(x, nullptr);
      return true;
    };
    if (a == "--rig" || a == "--calibration") {
      const char* x = value();
      if (x == nullptr) return false;
      (a == "--rig" ? o.rig : o.calibration) = x;
    } else if (a == "--apply-sync") {
      o.apply_sync = true;
    } else if (a == "--hevc" || a == "--mjpeg") {
      const bool hevc = a == "--hevc";
      if (codec_given && o.hevc != hevc) {
        std::fprintf(stderr, "--hevc or --mjpeg, not both\n");
        return false;
      }
      codec_given = true;
      o.hevc = hevc;
    } else if (a == "--color") {
      const char* x = value();
      unsigned w = 0, h = 0;
      if (x == nullptr || std::sscanf(x, "%ux%u", &w, &h) != 2 || w == 0 ||
          h == 0) {
        std::fprintf(stderr, "--color needs WxH, e.g. 1920x1080\n");
        return false;
      }
      o.color_width = w;
      o.color_height = h;
    } else if (a == "--fps") {
      const char* x = value();
      if (x == nullptr || std::atoi(x) < 1) {
        std::fprintf(stderr, "--fps needs N >= 1\n");
        return false;
      }
      o.fps = static_cast<std::uint32_t>(std::atoi(x));
    } else if (a == "--voxel") {
      if (!number(o.voxel)) return false;
    } else if (a == "--trunc") {
      if (!number(o.trunc)) return false;
    } else if (a == "--min-depth" || a == "--max-depth") {
      float d = 0.0f;
      if (!number(d)) return false;
      (a == "--min-depth" ? o.min_depth : o.max_depth) = d;
    } else if (a == "--max-weight") {
      if (!number(o.max_weight)) return false;
    } else if (a == "--remesh-every") {
      const char* x = value();
      if (x == nullptr) return false;
      o.remesh_every = std::max(1, std::atoi(x));
    } else if (a == "--hold-ms") {
      const char* x = value();
      if (x == nullptr) return false;
      o.hold_ms = std::max(0, std::atoi(x));
    } else if (a == "--sets" || a == "--frames") {
      const char* x = value();
      if (x == nullptr) return false;
      (a == "--sets" ? o.sets : o.frames) = std::max(0, std::atoi(x));
    } else if (a == "--width" || a == "--height") {
      const char* x = value();
      if (x == nullptr) return false;
      (a == "--width" ? o.width : o.height) = std::max(1, std::atoi(x));
    } else if (a == "--lit" || a == "--normals") {
      const Shading shading = a == "--lit" ? Shading::kLit : Shading::kNormals;
      if (shading_given && o.shading != shading) {
        std::fprintf(stderr, "--lit or --normals, not both\n");
        return false;
      }
      shading_given = true;
      o.shading = shading;
    } else if (a == "--no-texture") {
      o.texture = false;
    } else if (a == "--dynamic" || a == "--static") {
      o.dynamic = a == "--dynamic";
    } else if (a == "--occlusion") {
      if (!number(o.occlusion)) return false;
    } else if (a == "--all-depth") {
      o.depth_within_color = false;
    } else if (a == "--show-sources") {
      o.show_sources = true;
    } else if (a == "--texture-stats") {
      o.texture_stats = true;
    } else if (a == "--no-overlay") {
      o.overlay = false;
    } else if (a == "--validation") {
      o.validation = true;
    } else {
      std::fprintf(stderr, "unknown argument %s\n", a.c_str());
      return false;
    }
  }
  if (o.rig.empty()) {
    std::fprintf(stderr, "--rig is required\n");
    return false;
  }
  // strtof takes "nan" and "inf" without complaint, and a NaN knob passes
  // every bound below by comparing false.
  for (const float knob : {o.voxel, o.trunc, o.max_weight, o.occlusion}) {
    if (!std::isfinite(knob)) {
      std::fprintf(stderr,
                   "--voxel, --trunc, --max-weight and --occlusion must be "
                   "finite\n");
      return false;
    }
  }
  if (!(o.voxel > 0.0f) || !(o.max_weight > 0.0f) || !(o.occlusion > 0.0f)) {
    std::fprintf(stderr, "--voxel, --max-weight and --occlusion must be > 0\n");
    return false;
  }
  if (o.trunc <= 0.0f) o.trunc = 4.0f * o.voxel;
  return true;
}

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

// One atlas image: the gfx texture a mesh version's uv0 index into, and the
// descriptor set binding it. Reused once nothing holds it but the pool (see
// acquire), so a live rig does not allocate an atlas per remesh.
struct AtlasImage {
  vg::Texture tex;
  vg::DescriptorPool pool;
  vg::DescriptorSet set;
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
// `hold_ns` before `set_ns` on the rig's clock (Options::hold_ms).
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

// One camera's share of an atlas: its colour buffer, as the frame prep left
// it (R, G, B and coverage in each word -- the bytes of R8G8B8A8), and the
// tile it goes to.
struct AtlasTileSource {
  std::shared_ptr<const vr::Buffer> color;
  rtex::AtlasTile tile;
  std::size_t camera = 0;  // which camera, for the colour-by-camera view
};

// What a mesh version's atlas is made from: the cameras that textured it,
// each copied into its tile. Empty for a mesh textured by none.
struct AtlasJob {
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  std::vector<AtlasTileSource> tiles;
  bool empty() const noexcept { return tiles.empty(); }
};

// Record `job` into `image`, a whole atlas: every tile copied from its
// camera's buffer, then the image made ready for the fragment shader. The
// previous contents are discarded (UNDEFINED), since a version samples only the
// tiles it wrote -- a camera missing from the set left no triangle pointing at
// its tile. With `solid`, each tile is copied from that camera's solid-colour
// buffer instead (at least a tile's size), so every triangle shows the camera
// that textured it.
void record_atlas_copy(VkCommandBuffer cmd, VkImage image, const AtlasJob& job,
                       const std::vector<VkBuffer>* solid = nullptr) {
  vg::ImageBarrierDesc to_copy;
  to_copy.image = image;
  // Nothing to wait for: acquire hands out only an image no frame in flight
  // binds, and every frame that bound it has been fence-waited.
  to_copy.src_stage = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
  to_copy.dst_stage = VK_PIPELINE_STAGE_TRANSFER_BIT;
  to_copy.dst_access = VK_ACCESS_TRANSFER_WRITE_BIT;
  to_copy.old_layout = VK_IMAGE_LAYOUT_UNDEFINED;
  to_copy.new_layout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
  vg::cmd_image_barrier(cmd, to_copy);
  for (const AtlasTileSource& source : job.tiles) {
    VkBufferImageCopy region{};
    region.bufferRowLength = source.tile.width;
    region.bufferImageHeight = source.tile.height;
    region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    region.imageSubresource.layerCount = 1;
    region.imageOffset = {static_cast<std::int32_t>(source.tile.x),
                          static_cast<std::int32_t>(source.tile.y), 0};
    region.imageExtent = {source.tile.width, source.tile.height, 1};
    const VkBuffer from =
        solid != nullptr ? (*solid)[source.camera] : source.color->handle();
    vkCmdCopyBufferToImage(cmd, from, image,
                           VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);
  }
  vg::ImageBarrierDesc to_sample;
  to_sample.image = image;
  to_sample.src_stage = VK_PIPELINE_STAGE_TRANSFER_BIT;
  to_sample.src_access = VK_ACCESS_TRANSFER_WRITE_BIT;
  to_sample.dst_stage = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
  to_sample.dst_access = VK_ACCESS_SHADER_READ_BIT;
  to_sample.old_layout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
  to_sample.new_layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
  vg::cmd_image_barrier(cmd, to_sample);
}

// Why a mesh went out without an atlas. Such a mesh draws in fused vertex
// colour until the next remesh, so a run that often publishes one flickers
// between the cameras' texture and the fused colour; the Rig panel counts each.
enum Untextured : std::size_t {
  kEmptyMesh,   // the extract had no triangles, so there was nothing to texture
  kNoColour,    // no camera of the set carried colour gfx can copy
  kTextureOff,  // texturing switched off in the View panel
  kTextureFailed,  // the texture pass refused or failed (said on stderr)
  kUntexturedReasons,
};
constexpr const char* kUntexturedNames[kUntexturedReasons] = {
    "empty extract", "no colour", "texturing off", "texture failed"};

// What the rig side reports beside the renderer's metrics, sampled on the
// fuse thread (which owns the rig) and copied out by the render thread every
// frame, so it holds only what changes.
struct RigPanel {
  std::uint64_t sets_fused = 0;
  std::uint64_t frames_fused = 0;
  std::size_t cameras_textured = 0;  // in the newest textured set
  std::size_t cameras_held = 0;      // of those, from a held frame
  // Over the run: views the texture pass was given from a held frame, and
  // remeshes it was given fewer cameras than the rig has -- each one a
  // flicker, where the missing cameras' triangles fell to fused colour. Both
  // count views given, not triangles taken: a held view whose depth no longer
  // agrees with the mesh takes none, which --texture-stats shows.
  std::uint64_t held_views = 0;
  std::uint64_t short_remeshes = 0;
  std::array<std::uint64_t, kUntexturedReasons> untextured{};
  rsensor::OrbbecRigStats stats;
  std::size_t vertices = 0;
  std::size_t triangles = 0;
  std::uint64_t mesh_version = 0;
  float map_load_factor = 0.0f;  // negative: the read failed
  std::int32_t map_blocks = 0;
  double fuse_ms = 0.0;    // the newest set's prep, allocate and integrate
  double remesh_ms = 0.0;  // the newest remesh's extract and texture
  vr::MemoryStats recon_memory;
  bool silent = false;  // no set within kSilenceLimit
  // Meshes the window committed per second, over the last second: how often
  // what is drawn changes, the viewer's live rate. Filled on the render
  // thread, as the mesh counts are.
  double mesh_rate = 0.0;
};

double to_mebibytes(std::uint64_t bytes) {
  return static_cast<double>(bytes) / (1024.0 * 1024.0);
}

void draw_rig_panel(const RigPanel& panel,
                    const std::vector<std::string>& serials) {
  if (!ImGui::Begin("Rig")) {
    ImGui::End();
    return;
  }
  ImGui::Text("sets     %llu fused (%llu frames), %llu missing a camera",
              static_cast<unsigned long long>(panel.sets_fused),
              static_cast<unsigned long long>(panel.frames_fused),
              static_cast<unsigned long long>(panel.stats.incomplete));
  if (panel.silent) {
    ImGui::TextColored(ImVec4(0.9f, 0.5f, 0.2f, 1.0f),
                       "no set from the rig in %lld s",
                       static_cast<long long>(kSilenceLimit.count()));
  }
  ImGui::Text("fuse     %.2f ms/set", panel.fuse_ms);
  ImGui::Text("remesh   %.2f ms, the newest", panel.remesh_ms);
  ImGui::Text("mesh     %.1f updates/s", panel.mesh_rate);
  ImGui::Text("textured from %zu camera%s, %zu held", panel.cameras_textured,
              panel.cameras_textured == 1 ? "" : "s", panel.cameras_held);
  ImGui::Text("  so far  %llu held views, %llu remeshes short of a camera",
              static_cast<unsigned long long>(panel.held_views),
              static_cast<unsigned long long>(panel.short_remeshes));
  for (std::size_t r = 0; r < kUntexturedReasons; ++r) {
    if (panel.untextured[r] == 0) continue;
    ImGui::Text("  untextured meshes, %s: %llu", kUntexturedNames[r],
                static_cast<unsigned long long>(panel.untextured[r]));
  }
  ImGui::Separator();
  for (std::size_t i = 0; i < panel.stats.cameras.size(); ++i) {
    const rsensor::OrbbecCaptureStats& st = panel.stats.cameras[i];
    ImGui::Text("%s  %llu in, %llu dropped, %llu failed, %llu lost",
                i < serials.size() ? serials[i].c_str() : "?",
                static_cast<unsigned long long>(st.received),
                static_cast<unsigned long long>(st.dropped),
                static_cast<unsigned long long>(st.failed),
                static_cast<unsigned long long>(st.lost));
    // Colour that should have stayed on the GPU and did not: on a discrete
    // GPU every such frame crossed the bus (OrbbecCaptureStats::host_pictures).
    if (st.host_pictures != 0) {
      ImGui::TextColored(ImVec4(0.9f, 0.5f, 0.2f, 1.0f),
                         "  %llu with colour on the host",
                         static_cast<unsigned long long>(st.host_pictures));
    }
  }
  ImGui::Separator();
  ImGui::Text("mesh v%llu  %zu vertices / %zu triangles",
              static_cast<unsigned long long>(panel.mesh_version),
              panel.vertices, panel.triangles);
  if (panel.map_load_factor < 0.0f) {
    ImGui::Text("map      unavailable, %d blocks", panel.map_blocks);
  } else {
    ImGui::Text("map      %.1f%% of %d blocks%s", 100.0 * panel.map_load_factor,
                panel.map_blocks,
                panel.map_load_factor >= vol::VoxelHashMap::kGrowThreshold
                    ? "  -- grow now"
                    : "");
  }
  for (std::uint32_t heap = 0; heap < panel.recon_memory.heap_count; ++heap) {
    const vr::HeapStats& stats = panel.recon_memory.heaps[heap];
    if (stats.budget_bytes == 0 && stats.usage_bytes == 0) continue;
    ImGui::Text("recon heap %u  %.0f / %.0f MiB", heap,
                to_mebibytes(stats.usage_bytes),
                to_mebibytes(stats.budget_bytes));
  }
  ImGui::End();
}

// Owns the WindowedApp and every device resource, so they are gone before
// main destroys the window (see fuse_viewer's run()).
int run(GLFWwindow* window, const Options& opt) {
  // --- One VkDevice, adopted by both libraries (as fuse_viewer) ------------
  fuse_viewer::SharedDeviceConfig shared_config;
  shared_config.enable_validation = opt.validation;
  shared_config.app_name = "rig_viewer";
  fuse_viewer::SharedDevice shared;
  if (!fuse_viewer::build_shared_device(window, shared_config, shared)) {
    return 1;
  }

  vg::app::WindowedAppConfig config;
  config.app_name = "rig_viewer";
  config.swapchain.extent = fuse_viewer::window_extent(window);
  config.swapchain.depth_format = VK_FORMAT_D32_SFLOAT;
  config.frames_in_flight = 2;
  auto app_r = vg::app::WindowedApp::adopt(
      fuse_viewer::gfx_adopt_payload(shared), config,
      [&shared](VkInstance instance) -> vg::Result<VkSurfaceKHR> {
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

  // --- The rig, opened raw onto the shared device --------------------------
  // After the device, which it decodes onto and must outlive it. Opened here,
  // on the main thread, so a camera that does not answer ends the run with its
  // reason before the window starts drawing; the fuse thread starts it.
  rsensor::OrbbecRig::Options rig_options;
  {
    auto sync = rsensor::read_orbbec_sync_config(opt.rig);
    if (!sync) {
      std::fprintf(stderr, "%s: %s\n", opt.rig.c_str(),
                   sync.status().message().c_str());
      return 1;
    }
    rig_options.sync = std::move(sync).value();
  }
  if (!opt.calibration.empty()) {
    auto calibration = rsensor::read_rig_calibration(opt.calibration);
    if (!calibration) {
      std::fprintf(stderr, "%s: %s\n", opt.calibration.c_str(),
                   calibration.status().message().c_str());
      return 1;
    }
    rig_options.calibration = std::move(calibration).value();
  }
  rig_options.apply_sync_config = opt.apply_sync;
  rig_options.raw = true;
  rig_options.device = &rdevice;
  if (opt.hevc) rig_options.color_codec = rsensor::OrbbecColorCodec::Hevc;
  if (opt.color_width != 0) {
    rig_options.color_width = opt.color_width;
    rig_options.color_height = opt.color_height;
  }
  if (opt.fps != 0) rig_options.fps = opt.fps;
  if (opt.min_depth) rig_options.min_depth = *opt.min_depth;
  if (opt.max_depth) rig_options.max_depth = *opt.max_depth;
  auto rig_result = rsensor::OrbbecRig::open(rig_options);
  if (!rig_result) {
    std::fprintf(stderr, "rig: %s\n", rig_result.status().message().c_str());
    return 1;
  }
  rsensor::OrbbecRig rig = std::move(rig_result).value();
  const std::size_t cameras = rig.camera_count();
  std::vector<std::string> serials;
  std::vector<glm::mat4> camera_poses;
  for (std::size_t i = 0; i < cameras; ++i) {
    const vr::ColorCameraParams& cam = rig.color_camera(i);
    serials.push_back(rig.device_info(i).serial);
    camera_poses.push_back(cam.cam_to_world);
    std::printf("camera %zu: %s%s, colour %ux%u, at (%.3f, %.3f, %.3f) m\n", i,
                serials.back().c_str(), i == rig.primary() ? " (primary)" : "",
                cam.width, cam.height, cam.cam_to_world[3].x,
                cam.cam_to_world[3].y, cam.cam_to_world[3].z);
  }
  if (opt.calibration.empty() && cameras > 1) {
    std::fprintf(stderr,
                 "rig_viewer: no --calibration, so every camera sits at the "
                 "origin and the %zu cameras fuse over one another\n",
                 cameras);
  }

  // --- recon: volume, integrator, extractor, texturer, frame prep ---------
  auto grid_result =
      vr_example::create_fusion_grid(rdevice, rallocator, opt.voxel, opt.trunc);
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
  // What gfx needs of the mesh buffers, as fuse_viewer states it: the draw's
  // usage bits, both queue families, and a ring a slot deeper than the frames
  // in flight. Unshared, for the several-view texturing.
  rmesh::MarchingCubesConfig mc_config;
  mc_config.extra_vertex_usage = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
  mc_config.extra_index_usage = VK_BUFFER_USAGE_INDEX_BUFFER_BIT;
  mc_config.queue_families[0] = shared.compute_family;
  mc_config.queue_families[1] = shared.graphics_family;
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
  // The colour buffers are what gfx copies into its atlas, so the passes
  // share them with gfx's family too -- the same reasoning as the mesh buffers
  // above, and the same unconditional pair. Depth only recon reads.
  rsensor::GpuFramePrepConfig prep_config;
  prep_config.color_queue_families[0] = shared.compute_family;
  prep_config.color_queue_families[1] = shared.graphics_family;
  prep_config.color_queue_family_count = 2;
  prep_config.depth_within_color = opt.depth_within_color;
  std::vector<rsensor::GpuFramePrep> preps;
  for (std::size_t i = 0; i < cameras; ++i) {
    auto prep = rsensor::GpuFramePrep::create(rdevice, rallocator, prep_config);
    if (!prep) {
      std::fprintf(stderr, "frame prep: %s\n", prep.status().message().c_str());
      return 1;
    }
    preps.push_back(std::move(prep).value());
  }

  // The atlas: one tile per camera, laid out once from the rig's colour
  // cameras. A set textures from the cameras it has, into their own tiles.
  rtex::AtlasLayout layout;
  {
    std::vector<rtex::TextureView> sizes(cameras);
    for (std::size_t i = 0; i < cameras; ++i) {
      sizes[i].image_width = rig.color_camera(i).width;
      sizes[i].image_height = rig.color_camera(i).height;
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

  // --- gfx: pipeline, profiler, overlay, sampler ---------------------------
  auto pipeline_result = vgp::HybridMeshPipeline::create(
      app.device().handle(), app.swapchain().layout());
  if (!pipeline_result.ok()) {
    std::fprintf(stderr, "pipeline: %s\n",
                 pipeline_result.status().message().c_str());
    return 1;
  }
  vgp::HybridMeshPipeline pipeline = std::move(pipeline_result).value();

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
    ImGui::SetCurrentContext(overlay->context());
    if (!ImGui_ImplGlfw_InitForVulkan(window, true)) {
      std::fprintf(stderr, "ImGui_ImplGlfw_InitForVulkan failed\n");
      return 1;
    }
  }
  const fuse_viewer::ImGuiGlfwShutdown imgui_glfw_guard{opt.overlay};

  auto sampler_result = vg::Sampler::create(app.device().handle());
  if (!sampler_result.ok()) {
    std::fprintf(stderr, "sampler: %s\n",
                 sampler_result.status().message().c_str());
    return 1;
  }
  vg::Sampler sampler = std::move(sampler_result).value();

  // A texture + its own pool + a set binding it, as fuse_viewer's bundle.
  using AtlasResult = vg::Result<std::shared_ptr<AtlasImage>>;
  auto bind_atlas = [&](vg::Texture texture) -> AtlasResult {
    const VkDescriptorPoolSize pool_size{
        VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1};
    auto pool_result =
        vg::DescriptorPool::create(app.device().handle(), &pool_size, 1, 1);
    if (!pool_result.ok()) return pool_result.status();
    vg::DescriptorPool atlas_pool = std::move(pool_result).value();
    auto set_result = atlas_pool.allocate(pipeline.descriptor_set_layout(0));
    if (!set_result.ok()) return set_result.status();
    auto atlas = std::make_shared<AtlasImage>();
    atlas->tex = std::move(texture);
    atlas->pool = std::move(atlas_pool);
    atlas->set = std::move(set_result).value();
    atlas->set.write_combined_image_sampler(
        0, atlas->tex.view(), sampler.handle(),
        VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
    return atlas;
  };

  // The fallback atlas, bound for a mesh no camera textured: a 1x1 white
  // texel its sentinel uv0 never select.
  std::shared_ptr<AtlasImage> white_atlas;
  {
    const std::uint8_t white[4] = {255, 255, 255, 255};
    vg::ImageUploadDesc upload_desc;
    upload_desc.extent = {1, 1};
    upload_desc.format = VK_FORMAT_R8G8B8A8_SRGB;
    upload_desc.pixels = white;
    upload_desc.size = sizeof(white);
    auto uploaded =
        vg::upload_texture(app.device(), app.allocator(), upload_desc);
    if (!uploaded.ok()) {
      std::fprintf(stderr, "white atlas: %s\n",
                   uploaded.status().message().c_str());
      return 1;
    }
    AtlasResult bound = bind_atlas(std::move(uploaded).value());
    if (!bound.ok()) {
      std::fprintf(stderr, "white atlas: %s\n",
                   bound.status().message().c_str());
      return 1;
    }
    white_atlas = std::move(bound).value();
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
  std::vector<vg::Buffer> solid_buffers;
  std::vector<VkBuffer> solid_handles;
  auto ensure_solid = [&](VkCommandBuffer cmd) -> bool {
    if (solid_handles.size() == cameras) return true;
    solid_buffers.clear();
    solid_handles.clear();
    for (std::size_t c = 0; c < cameras; ++c) {
      const rtex::AtlasTile& tile = layout.tiles[c];
      vg::BufferDesc desc;
      desc.size = VkDeviceSize(tile.width) * tile.height * 4u;
      desc.usage =
          VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
      desc.memory = vg::MemoryUsage::DeviceLocal;
      auto buffer = app.allocator().create_buffer(desc);
      if (!buffer.ok()) {
        std::fprintf(stderr, "rig_viewer: colour-by-camera buffer: %s\n",
                     buffer.status().message().c_str());
        solid_buffers.clear();
        solid_handles.clear();
        return false;
      }
      solid_handles.push_back(buffer.value().handle());
      solid_buffers.push_back(std::move(buffer).value());
    }
    // Filled only once every buffer exists, so a failed allocation above
    // frees none that a recorded fill names.
    for (std::size_t c = 0; c < cameras; ++c) {
      const auto& rgb = kCameraColours[c % kCameraColours.size()];
      const std::uint32_t word = std::uint32_t{rgb[0]} |
                                 (std::uint32_t{rgb[1]} << 8) |
                                 (std::uint32_t{rgb[2]} << 16) | 0xFF000000u;
      vkCmdFillBuffer(cmd, solid_handles[c], 0, VK_WHOLE_SIZE, word);
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

  // Atlas images the copies fill, reused once only the pool holds one: the
  // committed version holds its image, and so does every frame slot that
  // bound it, until begin_frame fence-waits that slot. _SRGB, since the frame
  // prep's colour is canonical-encoded 8-bit and the sampler then filters in
  // linear (the 2026-08-02 colour-space decision, as fuse_viewer's upload).
  std::vector<std::shared_ptr<AtlasImage>> atlas_pool;
  auto acquire_atlas = [&](std::uint32_t width,
                           std::uint32_t height) -> AtlasResult {
    for (const std::shared_ptr<AtlasImage>& atlas : atlas_pool) {
      if (atlas.use_count() == 1 && atlas->tex.extent().width == width &&
          atlas->tex.extent().height == height) {
        return atlas;
      }
    }
    vg::TextureDesc desc;
    desc.extent = {width, height};
    desc.format = VK_FORMAT_R8G8B8A8_SRGB;
    desc.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
    auto image = app.allocator().create_image(desc);
    if (!image.ok()) return image.status();
    AtlasResult atlas = bind_atlas(std::move(image).value());
    if (atlas.ok()) atlas_pool.push_back(atlas.value());
    return atlas;
  };

  // --- The view -------------------------------------------------------------
  // Around where the cameras look, from the primary's side.
  OrbitView home;
  float vfov = 1.0f;
  {
    const glm::mat4& c2w = camera_poses[rig.primary()];
    const glm::vec3 eye(c2w[3]);
    home.forward = glm::normalize(glm::vec3(c2w[2]));
    home.up = -glm::normalize(glm::vec3(c2w[1]));  // image up is -cameraY
    home.right = glm::normalize(glm::cross(home.forward, home.up));
    const float depth_mid =
        0.5f * (rig_options.min_depth + rig_options.max_depth);
    home.target = axes_meet(camera_poses, eye + home.forward * depth_mid);
    home.distance = std::max(0.3f, glm::length(home.target - eye));
    const vr::ColorCameraParams& cam = rig.color_camera(rig.primary());
    vfov = 2.0f * std::atan(static_cast<float>(cam.height) /
                            (2.0f * std::max(1.0f, cam.fy)));
  }
  OrbitView view = home;
  // Looking out of camera i, set by the View panel, until a drag orbits away.
  std::optional<std::size_t> from_camera;
  Shading shading = opt.shading;
  std::atomic<bool> texture_on{opt.texture};
  // The fusion and texturing knobs the View panel tunes while the rig runs,
  // read by the fuse thread at each set and each remesh.
  std::atomic<bool> dynamic_on{opt.dynamic};
  std::atomic<float> max_weight{opt.max_weight};
  std::atomic<float> occlusion{opt.occlusion};
  // Render-thread state: whether the next atlas copy fills each tile with its
  // camera's colour rather than its image.
  bool show_sources = opt.show_sources;

  // --- Fuse thread ----------------------------------------------------------
  // The rig, the frame prep, fusion, extraction and texturing all run here;
  // the render thread only copies the atlas and draws. Shared state is under
  // share_mtx, as in fuse_viewer, and so is the ring's release mark.
  std::mutex share_mtx;
  std::optional<rmesh::DeviceMesh> pending_mesh;
  AtlasJob pending_job;
  std::uint64_t published_version = 0;
  std::uint64_t shared_released_through = 0;
  std::vector<vg::FrameMetrics::Section> shared_fuse_stages;
  RigPanel shared_panel;
  std::atomic<bool> fusing_done{false};
  // Set by whatever ended fusion early, so a scripted run exits non-zero.
  std::atomic<bool> fuse_failed{false};
  std::atomic<bool> quit{false};
  const bool cross_family = shared.graphics_family != shared.compute_family;

  std::thread fuse_thread([&]() {
    try {
      vr::StageMetrics fuse_stages;
      // The remesh rows, held between remeshes (see fuse_viewer).
      vr::StageMetrics remesh_stages;
      std::size_t cameras_textured = 0;
      std::size_t cameras_held = 0;
      std::uint64_t held_views = 0;
      std::uint64_t short_remeshes = 0;
      std::array<std::uint64_t, kUntexturedReasons> untextured{};
      bool texture_error_reported = false;
      // Each camera's newest frame with colour, which the remeshes texture
      // from (texture_sources).
      std::vector<std::optional<NewestFrame>> newest(cameras);
      const std::uint64_t hold_ns =
          static_cast<std::uint64_t>(opt.hold_ms) * 1000000u;
      std::vector<bool> uncopyable_reported(cameras, false);

      // Whether gfx can copy `color` into `tile`: a copy source, as large as
      // the tile, and CONCURRENT where gfx is on another family, since a copy
      // from a buffer EXCLUSIVE to recon's is undefined with nothing to report
      // it. Checked before the camera textures the mesh: once uv0 point into
      // its tile, the tile must be filled.
      auto copyable = [&](const vr::Buffer& color,
                          const rtex::AtlasTile& tile) {
        return color.valid() &&
               (color.usage() & VK_BUFFER_USAGE_TRANSFER_SRC_BIT) != 0 &&
               (!cross_family ||
                color.sharing_mode() == VK_SHARING_MODE_CONCURRENT) &&
               color.size() >= VkDeviceSize(tile.width) * tile.height * 4u;
      };

      // Texture `mesh` from each camera's source (texture_sources: the set's
      // frame, or the camera's held one) and publish the two together: uv0
      // index into those images, so the mesh and its atlas are one value.
      auto publish = [&](const rmesh::DeviceMesh& mesh,
                         const std::vector<TextureSource>& sources) {
        AtlasJob job;
        std::size_t views_held = 0;
        std::size_t why = kTextureOff;
        if (!texture_on.load()) {
          why = kTextureOff;
        } else if (mesh.empty()) {
          why = kEmptyMesh;
        } else {
          why = kNoColour;
          std::vector<rtex::TextureView> views;
          rtex::AtlasLayout present{layout.width, layout.height, {}};
          job.width = layout.width;
          job.height = layout.height;
          for (std::size_t c = 0; c < sources.size() && c < cameras; ++c) {
            if (sources[c].frame == nullptr) continue;
            const rsensor::DeviceFrame& f = *sources[c].frame;
            if (!copyable(*f.color, layout.tiles[c])) {
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
            const vr::Status textured = texturer.texture(
                mesh, views, present, occlusion.load(), &remesh_stages);
            if (textured.ok()) {
              cameras_textured = views.size();
              cameras_held = views_held;
              held_views += views_held;
              short_remeshes += views.size() < cameras ? 1 : 0;
            } else {
              why = kTextureFailed;
              // Said once: a refusal here is a configuration fault (a frame
              // whose colour is not its tile's size, say), and every remesh
              // would repeat it.
              if (!texture_error_reported) {
                std::fprintf(stderr, "rig_viewer: texture: %s\n",
                             textured.message().c_str());
                texture_error_reported = true;
              }
              job = AtlasJob{};
            }
          } else {
            job = AtlasJob{};
          }
        }
        if (job.empty()) {
          cameras_textured = 0;
          cameras_held = 0;
          ++untextured[why];
        }
        std::lock_guard<std::mutex> lock(share_mtx);
        pending_mesh = mesh;
        pending_job = std::move(job);
        ++published_version;
      };
      // The release mark, applied on this thread, and whether it may publish
      // (see fuse_viewer).
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
      // --texture-stats: read `mesh` back and count, per camera, the triangles
      // it textured (uv0 inside its tile; a triangle's three share one) and
      // those no camera did, marking the cameras textured from a held frame.
      // A readback of the whole mesh, hence a flag.
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
        const std::vector<rmesh::Vertex>& v = host.value().vertices;
        std::vector<std::size_t> per_camera(cameras, 0);
        std::size_t none = 0;
        const std::size_t triangles = v.size() / 3;
        for (std::size_t t = 0; t < triangles; ++t) {
          const vr::Vec2f uv = v[3 * t].uv0;
          if (!(uv.x >= 0.0f)) {
            ++none;
            continue;
          }
          const float px = uv.x * static_cast<float>(layout.width);
          const float py = uv.y * static_cast<float>(layout.height);
          for (std::size_t c = 0; c < cameras; ++c) {
            const rtex::AtlasTile& tile = layout.tiles[c];
            if (px >= tile.x && px < tile.x + tile.width && py >= tile.y &&
                py < tile.y + tile.height) {
              ++per_camera[c];
              break;
            }
          }
        }
        const double ms = std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now() - start)
                              .count();
        const double n = triangles > 0 ? static_cast<double>(triangles) : 1.0;
        std::printf("texture stats: %zu triangles, %.1f%% untextured;",
                    triangles, 100.0 * static_cast<double>(none) / n);
        for (std::size_t c = 0; c < cameras; ++c) {
          std::printf(" camera %zu %.1f%%%s", c,
                      100.0 * static_cast<double>(per_camera[c]) / n,
                      c < from.size() && from[c].held ? " (held)" : "");
        }
        std::printf(" (occlusion %.1f cm, %s; read back in %.0f ms)\n",
                    100.0 * occlusion.load(),
                    dynamic_on.load() ? "dynamic" : "static", ms);
      };
      auto remesh = [&](const std::vector<TextureSource>& sources) {
        remesh_stages.clear();
        vr::Result<rmesh::DeviceMesh> extracted = [&]() {
          vr::StageScope scope(remesh_stages, "extract");
          return extractor.extract_device(volume, 0.0f);
        }();
        // Published even when empty: an extract claims a ring slot either
        // way (see fuse_viewer).
        if (extracted) {
          publish(extracted.value(), sources);
          // Before the next extract, which retires this mesh.
          if (opt.texture_stats && remeshes++ % 30 == 0) {
            print_texture_stats(extracted.value(), sources);
          }
        } else {
          std::fprintf(stderr, "rig_viewer: extract: %s\n",
                       extracted.status().message().c_str());
        }
      };

      const vr::Status started = rig.start();
      if (!started.ok()) {
        std::fprintf(stderr, "rig_viewer: rig start: %s\n",
                     started.message().c_str());
        fuse_failed.store(true);
      }
      std::uint64_t sets = 0;
      std::uint64_t frames_fused = 0;
      auto last_set = std::chrono::steady_clock::now();
      bool said_silent = false;
      while (started.ok() && !quit.load()) {
        const auto poll_start = std::chrono::steady_clock::now();
        auto polled = rig.poll_raw_set();
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
        if (!polled.value() || polled.value()->count() == 0) {
          // Published only when it changes: this path runs every millisecond.
          const bool silent =
              std::chrono::steady_clock::now() - last_set > kSilenceLimit;
          if (silent != said_silent) {
            said_silent = silent;
            std::lock_guard<std::mutex> lock(share_mtx);
            shared_panel.silent = silent;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
          continue;
        }
        last_set = std::chrono::steady_clock::now();
        said_silent = false;
        // The table shows the newest set's rows, seeded in display order so the
        // remesh rows read 0 between remeshes rather than drop out.
        fuse_stages.clear();
        for (const char* stage :
             {"poll", "frame prep", "allocate", "resize", "integrate",
              "  ..active set", "extract", "texture"}) {
          fuse_stages.seed(stage);
        }
        fuse_stages.add_cpu("poll", poll_ms);
        const rsensor::OrbbecRigRawSet& set = *polled.value();
        const std::uint64_t set_ns = set.timestamp_ns;
        auto prepared = [&]() {
          // One row for the set: the cameras prepare at once, so their sum
          // would overstate it.
          vr::StageScope scope(fuse_stages, "frame prep");
          return rsensor::prepare_set(preps, set.frames);
        }();
        if (!prepared) {
          std::fprintf(stderr, "rig_viewer: frame prep: %s\n",
                       prepared.status().message().c_str());
          fuse_failed.store(true);
          break;
        }
        const std::vector<std::optional<rsensor::DeviceFrame>>& frames =
            prepared.value();
        const vr::Status fused = vr_example::fuse_set(
            volume, integrator, frames, max_weight.load(), &fuse_stages,
            dynamic_on.load() ? rtsdf::IntegrationMode::Dynamic
                              : rtsdf::IntegrationMode::Classic);
        if (!fused.ok()) {
          std::fprintf(stderr, "rig_viewer: fuse: %s\n",
                       fused.message().c_str());
          fuse_failed.store(true);
          break;
        }
        for (const std::optional<rsensor::DeviceFrame>& frame : frames) {
          frames_fused += frame ? 1 : 0;
        }
        ++sets;
        for (std::size_t c = 0; c < frames.size() && c < cameras; ++c) {
          if (frames[c] && frames[c]->has_color()) {
            newest[c] = NewestFrame{*frames[c], sets};
          }
        }

        // The last set of a --sets run is always meshed: it is the complete
        // surface, and nothing later will supersede it. So it waits a moment
        // for the renderer to collect the previous mesh, then extracts whether
        // or not it did, as fuse_viewer's final extract does -- a minimized
        // window never collects, and skipping lost the surface with nothing
        // said. Not once the window has closed, where a full extract and
        // texture would only stall the join.
        const bool last =
            opt.sets > 0 && sets >= static_cast<std::uint64_t>(opt.sets);
        if (last) {
          for (int wait = 0; wait < 500 && !quit.load(); ++wait) {
            {
              std::lock_guard<std::mutex> lock(share_mtx);
              if (!pending_mesh) break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
          }
        }
        if (!quit.load() &&
            (last ||
             sets % static_cast<std::uint64_t>(opt.remesh_every) == 0)) {
          if (release_and_may_publish()) {
            remesh(texture_sources(newest, sets, set_ns, hold_ns));
          } else if (last) {
            std::fprintf(stderr,
                         "rig_viewer: the renderer never collected the last "
                         "mesh (window hidden, or drawing stopped); meshing "
                         "the last set anyway\n");
            remesh(texture_sources(newest, sets, set_ns, hold_ns));
          }
        }
        // Kept past this set only while a later remesh can texture from them:
        // a kept frame stops its camera's frame prep reusing the buffers.
        if (hold_ns == 0 || !texture_on.load()) {
          newest.assign(cameras, std::nullopt);
        }
        // Fusion alone, before the remesh rows go in: those describe the
        // newest remesh, not this set, and are reported beside it.
        const double fuse_ms = fuse_stages.total_cpu_ms(/*exclude=*/"poll");
        fuse_stages.merge(remesh_stages);
        {
          const vr::MemoryStats memory = rallocator.memory_stats();
          const vr::Result<float> lf = volume.map().load_factor();
          const rsensor::OrbbecRigStats stats = rig.stats();
          std::lock_guard<std::mutex> lock(share_mtx);
          shared_fuse_stages = fuse_viewer::to_sections(fuse_stages);
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
    } catch (const std::exception& e) {
      std::fprintf(stderr, "rig_viewer: fuse thread aborted: %s\n", e.what());
      fuse_failed.store(true);
    }
    // Outside the try, so an exception stops the cameras too.
    rig.stop();
    fusing_done.store(true);
  });
  fuse_viewer::QuitJoin fuse_guard{fuse_thread, quit};

  // --- Render thread (main) -------------------------------------------------
  // The mesh ring's bookkeeping is fuse_viewer's, and its two decisions are
  // viewer_common.hpp's: retire, then take, under one lock; release everything
  // older than the oldest generation a frame in flight draws; retry a take
  // that could not be committed rather than drop it. What is new is the atlas:
  // a taken version's job is recorded into this frame's command buffer, and the
  // colour buffers it reads stay with this frame's slot until the slot comes
  // round again.
  std::vector<std::shared_ptr<AtlasImage>> slot_atlas(config.frames_in_flight);
  std::vector<std::vector<std::shared_ptr<const vr::Buffer>>> slot_sources(
      config.frames_in_flight);
  std::shared_ptr<AtlasImage> current_atlas = white_atlas;
  rmesh::DeviceMesh live_view;
  rmesh::DeviceMesh taken;
  AtlasJob taken_job;
  std::uint64_t taken_version = 0;
  std::vector<std::uint64_t> frame_generations(config.frames_in_flight, 0);
  std::uint64_t newest_taken_generation = 0;
  bool mesh_unusable = false;
  bool atlas_error_said = false;  // until an atlas image is acquired again
  std::vector<vg::FrameMetrics::Section> fuse_stages_snapshot;
  RigPanel panel;
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
  while (glfwWindowShouldClose(window) == GLFW_FALSE &&
         (opt.frames == 0 || drawn < opt.frames)) {
    glfwPollEvents();
    if (g_interrupted.load()) {
      glfwSetWindowShouldClose(window, GLFW_TRUE);
      continue;
    }

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
    // begin_frame fence-waited this slot: what its last frame bound or copied
    // from is free.
    slot_atlas[render_frame.slot].reset();
    slot_sources[render_frame.slot].clear();

    {
      std::lock_guard<std::mutex> lock(share_mtx);
      fuse_stages_snapshot = shared_fuse_stages;
      // The mesh rows are filled below, from what this frame draws.
      panel = shared_panel;

      shared_released_through = fuse_viewer::retire_and_release_mark(
          frame_generations, render_frame.slot, live_view.generation,
          newest_taken_generation);
      if (pending_mesh && !mesh_unusable && taken_version == 0) {
        taken = *pending_mesh;
        pending_mesh.reset();
        taken_job = std::move(pending_job);
        pending_job = AtlasJob{};
        taken_version = published_version;
        newest_taken_generation = taken.generation;
      }
    }

    // Commit a taken mesh with its atlas, the copy recorded here -- before the
    // frame's rendering begins, since a copy may not sit inside it.
    {
      vg::Profiler::Scope copy_scope =
          profiler.gpu_scope(render_frame.cmd, "atlas copy");
      if (taken_version != 0 && taken.empty()) {
        // An empty extract draws nothing; committed for the ring (see
        // fuse_viewer).
        live_view = taken;
        taken = rmesh::DeviceMesh{};
        taken_job = AtlasJob{};
        taken_version = 0;
      } else if (taken_version != 0) {
        if (const char* why =
                fuse_viewer::unbindable_reason(taken, cross_family)) {
          std::fprintf(stderr,
                       "rig_viewer: the extracted mesh cannot be bound as "
                       "geometry (%s); drawing stops here\n",
                       why);
          mesh_unusable = true;
          // Dropped, so this is said once (see fuse_viewer).
          taken = rmesh::DeviceMesh{};
          taken_job = AtlasJob{};
          taken_version = 0;
        } else {
          // Every tile is one the fuse thread found copyable, since it left
          // out any camera whose colour is not.
          std::shared_ptr<AtlasImage> next = white_atlas;
          if (!taken_job.empty()) {
            AtlasResult acquired =
                acquire_atlas(taken_job.width, taken_job.height);
            if (acquired.ok()) {
              next = std::move(acquired).value();
              atlas_error_said = false;
              record_atlas_copy(render_frame.cmd, next->tex.image(), taken_job,
                                show_sources && ensure_solid(render_frame.cmd)
                                    ? &solid_handles
                                    : nullptr);
              ++atlas_copies;
              for (AtlasTileSource& source : taken_job.tiles) {
                slot_sources[render_frame.slot].push_back(
                    std::move(source.color));
              }
            } else {
              next = nullptr;
              // Retried every frame, below, so said once until it succeeds.
              if (!atlas_error_said) {
                std::fprintf(stderr, "rig_viewer: atlas image: %s\n",
                             acquired.status().message().c_str());
                atlas_error_said = true;
              }
            }
          }
          // No image to copy into keeps the pair for the next frame, as a
          // failed upload does in fuse_viewer; anything else commits.
          if (next) {
            live_view = taken;
            current_atlas = std::move(next);
            taken = rmesh::DeviceMesh{};
            taken_job = AtlasJob{};
            taken_version = 0;
          }
        }
      }
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
    }
    slot_atlas[render_frame.slot] = current_atlas;
    frame_generations[render_frame.slot] = live_view.generation;

    // --- Input: orbit, pan, zoom, unless a panel has the mouse -------------
    {
      const bool ui_mouse = overlay && ImGui::GetIO().WantCaptureMouse;
      double x = 0.0, y = 0.0;
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
        2.0f * (view.distance + std::max(rig_options.max_depth, 1.0f));
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

    if (overlay) {
      ImGui_ImplGlfw_NewFrame();
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
      draw_rig_panel(panel, serials);
      ImGui::SetNextWindowPos(ImVec2(432.0f, 16.0f), ImGuiCond_FirstUseEver);
      if (ImGui::Begin("View")) {
        int mode = static_cast<int>(shading);
        ImGui::RadioButton("unlit", &mode, static_cast<int>(Shading::kUnlit));
        ImGui::SameLine();
        ImGui::RadioButton("lit", &mode, static_cast<int>(Shading::kLit));
        ImGui::SameLine();
        ImGui::RadioButton("normals", &mode,
                           static_cast<int>(Shading::kNormals));
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
        if (ImGui::Checkbox("fusion clears free space (dynamic)", &dynamic)) {
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

    vg::RenderTargetBeginInfo begin_info;
    begin_info.clear_color.float32[0] = 0.05f;
    begin_info.clear_color.float32[1] = 0.05f;
    begin_info.clear_color.float32[2] = 0.07f;
    begin_info.clear_color.float32[3] = 1.0f;
    render_frame.target->begin(render_frame.cmd, begin_info);
    {
      vg::Profiler::Scope draw_scope =
          profiler.gpu_scope(render_frame.cmd, "mesh draw");
      if (live_view.valid() && !live_view.empty()) {
        // recon's buffers, named rather than copied, as in fuse_viewer.
        vgp::LiveMesh live;
        live.vertices = live_view.vertices;
        live.indices = live_view.indices;
        live.indirect = live_view.indirect;
        const vgp::HybridMeshDraw draw{live};
        vgp::HybridMeshFrame hybrid_frame;
        hybrid_frame.extent = extent;
        hybrid_frame.view_proj = view_proj;
        hybrid_frame.light_dir = -view.up + 0.5f * view.right - view.forward;
        hybrid_frame.flags = shading_flags(shading);
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
    if (drawn % 120 == 0) {
      std::printf(
          "frame %d: %llu sets fused (%.1f ms/set, remesh %.1f ms), mesh "
          "v%llu (%zu triangles, %.1f updates/s), textured from %zu "
          "camera%s (%zu held; %llu held views, %llu remeshes short so far), "
          "%llu atlas copies",
          drawn, static_cast<unsigned long long>(panel.sets_fused),
          panel.fuse_ms, panel.remesh_ms,
          static_cast<unsigned long long>(panel.mesh_version), panel.triangles,
          panel.mesh_rate, panel.cameras_textured,
          panel.cameras_textured == 1 ? "" : "s", panel.cameras_held,
          static_cast<unsigned long long>(panel.held_views),
          static_cast<unsigned long long>(panel.short_remeshes),
          static_cast<unsigned long long>(atlas_copies));
      for (std::size_t r = 0; r < kUntexturedReasons; ++r) {
        if (panel.untextured[r] != 0) {
          std::printf(", %llu untextured (%s)",
                      static_cast<unsigned long long>(panel.untextured[r]),
                      kUntexturedNames[r]);
        }
      }
      std::printf("\n");
    }
    ++drawn;
  }

  // Joined here rather than by fuse_guard, so fuse_failed is final when read.
  quit.store(true);
  fuse_thread.join();
  app.wait_idle();
  // The fuse thread said why on stderr; the exit code is for a script.
  if (fuse_failed.load()) exit_code = 1;
  std::printf(
      "rig_viewer: drew %d frames; %llu sets (%llu frames) fused, "
      "mesh v%llu with %zu triangles, %llu atlas copies\n",
      drawn, static_cast<unsigned long long>(panel.sets_fused),
      static_cast<unsigned long long>(panel.frames_fused),
      static_cast<unsigned long long>(panel.mesh_version), panel.triangles,
      static_cast<unsigned long long>(atlas_copies));
  return exit_code;
}

}  // namespace

int main(int argc, char** argv) {
  Options opt;
  if (!parse_args(argc, argv, opt)) {
    std::fputs(kUsage, stderr);
    return 2;
  }
  std::signal(SIGINT, on_interrupt);
  std::signal(SIGTERM, on_interrupt);
  if (glfwInit() != GLFW_TRUE) {
    std::fprintf(stderr, "glfwInit failed\n");
    return 1;
  }
  glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
  GLFWwindow* window =
      glfwCreateWindow(opt.width, opt.height, "rig_viewer", nullptr, nullptr);
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
