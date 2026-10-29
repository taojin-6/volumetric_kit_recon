// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// rig_viewer where there is no display (examples/viewer/viewer_offscreen.hpp):
// a BMP is the readback's bytes behind a 54-byte header; on the device both
// libraries adopt with no surface, gfx's offscreen frame loop draws what
// rig_viewer draws -- a live atlas committed from a colour buffer recon
// filled, under the atlas-copy scope, and a mesh textured from it through the
// viewers' render pass -- with the profiler's rows a window's loop gives, less
// the present; FrameSaver saves the first frame with a mesh, the frame
// `every` later and the last, each written when its slot's next frame begins
// or after the device idles, never one it was not told was ended, and a path
// it cannot write is said once while the frames go on. The draw runs on a
// device, which the test skips without.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rig_report.hpp"
#include "viewer_atlas.hpp"
#include "viewer_common.hpp"
#include "viewer_offscreen.hpp"
#include "volumetric_kit/core/testing/scratch_dir.hpp"
#include "volumetric_kit/core/testing/test_check.hpp"
#include "volumetric_kit/core/testing/vulkan_policy.hpp"

#include "volumetric_kit/core/base/stage_metrics.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/buffer.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/core/vulkan/shared_device.hpp"
#include "volumetric_kit/core/vulkan/vulkan.hpp"
#include "volumetric_kit/gfx/app/windowed_app.hpp"
#include "volumetric_kit/gfx/assets/mesh.hpp"
#include "volumetric_kit/gfx/core/frame_metrics.hpp"
#include "volumetric_kit/gfx/core/profiler.hpp"
#include "volumetric_kit/gfx/pipelines/gpu_mesh.hpp"
#include "volumetric_kit/gfx/pipelines/hybrid_mesh_pipeline.hpp"
#include "volumetric_kit/gfx/windowing/frame_loop.hpp"
#include "volumetric_kit/recon/texture/texture_atlas.hpp"

namespace {

namespace vkc = volumetric_kit::core;
namespace vg = volumetric_kit::gfx;
namespace vgp = volumetric_kit::gfx::pipelines;
namespace rtex = volumetric_kit::recon::texture;
namespace win = volumetric_kit::gfx::windowing;
using Clock = std::chrono::steady_clock;

constexpr VkExtent2D kExtent{64, 48};
// The atlas's one tile, filled with one colour: sRGB bytes, as the frame
// prep leaves a camera's colour.
constexpr rtex::AtlasTile kTile{0, 0, 8, 8};
constexpr std::uint8_t kRed = 200, kGreen = 40, kBlue = 90;

std::uint32_t u32(const std::vector<std::uint8_t>& b, std::size_t at) {
  return std::uint32_t{b[at]} | (std::uint32_t{b[at + 1]} << 8) |
         (std::uint32_t{b[at + 2]} << 16) | (std::uint32_t{b[at + 3]} << 24);
}
std::uint32_t u16(const std::vector<std::uint8_t>& b, std::size_t at) {
  return std::uint32_t{b[at]} | (std::uint32_t{b[at + 1]} << 8);
}

// A linear value as an sRGB byte: what an _SRGB target stores.
int srgb_byte(double linear) {
  const double encoded = linear <= 0.0031308
                             ? 12.92 * linear
                             : 1.055 * std::pow(linear, 1.0 / 2.4) - 0.055;
  return static_cast<int>(std::lround(encoded * 255.0));
}

// Every header field of a top-down 32-bit BI_RGB BMP of `extent`, and the
// file's size.
int check_bmp_header(const std::vector<std::uint8_t>& file, VkExtent2D extent) {
  const std::uint32_t pixel_bytes = extent.width * extent.height * 4u;
  CHECK(file.size() == 54u + pixel_bytes);
  CHECK(file[0] == 'B' && file[1] == 'M');
  CHECK(u32(file, 2) == 54u + pixel_bytes);  // the file's size
  CHECK(u32(file, 6) == 0u);                 // reserved
  CHECK(u32(file, 10) == 54u);               // the pixels' offset
  CHECK(u32(file, 14) == 40u);               // BITMAPINFOHEADER
  CHECK(u32(file, 18) == extent.width);
  CHECK(static_cast<std::int32_t>(u32(file, 22)) ==
        -static_cast<std::int32_t>(extent.height));  // top-down
  CHECK(u16(file, 26) == 1u);                        // planes
  CHECK(u16(file, 28) == 32u);                       // bits per pixel
  CHECK(u32(file, 30) == 0u);                        // BI_RGB
  CHECK(u32(file, 34) == pixel_bytes);
  return 0;
}

// The pixel at (x, y) of a BMP check_bmp_header accepted: B, G, R, unused.
const std::uint8_t* bmp_pixel(const std::vector<std::uint8_t>& file,
                              VkExtent2D extent, std::uint32_t x,
                              std::uint32_t y) {
  return &file[54u + (std::size_t{y} * extent.width + x) * 4u];
}

bool near(int a, int b) { return std::abs(a - b) <= 2; }

// Host only: the bytes, the refusals, the write, and the names.
int bmp_is_the_bytes_behind_a_header(const std::filesystem::path& dir) {
  const VkExtent2D extent{3, 2};
  std::vector<std::uint8_t> pixels(3u * 2u * 4u);
  for (std::size_t i = 0; i < pixels.size(); ++i) {
    pixels[i] = static_cast<std::uint8_t>(i * 7u + 1u);
  }
  const vkc::Result<std::string> bmp =
      fuse_viewer::bmp_bytes(pixels.data(), extent);
  CHECK(bmp.ok());
  const std::vector<std::uint8_t> file(bmp.value().begin(), bmp.value().end());
  if (const int rc = check_bmp_header(file, extent)) return rc;
  CHECK(std::equal(pixels.begin(), pixels.end(), file.begin() + 54));

  // Nothing to write, and an image a BMP's sizes cannot hold: refused before
  // any byte is read.
  CHECK(fuse_viewer::bmp_bytes(nullptr, extent).status().domain() ==
        vkc::Status::Code::InvalidArgument);
  CHECK(fuse_viewer::bmp_bytes(pixels.data(), {0, 2}).status().domain() ==
        vkc::Status::Code::InvalidArgument);
  CHECK(
      fuse_viewer::bmp_bytes(pixels.data(), {65536, 65536}).status().domain() ==
      vkc::Status::Code::InvalidArgument);

  // Written as the bytes say, replacing a file there; not into a directory
  // that does not exist.
  const std::string path = (dir / "host.bmp").string();
  CHECK(fuse_viewer::write_bmp(path, pixels.data(), {1, 1}).ok());
  CHECK(fuse_viewer::write_bmp(path, pixels.data(), extent).ok());
  CHECK(vkc::test::read_bytes(path) == file);
  CHECK(!fuse_viewer::write_bmp((dir / "missing" / "host.bmp").string(),
                                pixels.data(), extent)
             .ok());

  // Frame N goes beside the name, numbered; no name saves nothing, and a
  // window's frame, with no offscreen target, is never saved.
  const fuse_viewer::FrameSaver saver((dir / "rig.bmp").string(),
                                      std::chrono::seconds(30), 2);
  CHECK(saver.enabled());
  CHECK(saver.name(12) == (dir / "rig_12.bmp").string());
  fuse_viewer::FrameSaver none("", std::chrono::seconds(30), 2);
  CHECK(!none.enabled());
  fuse_viewer::FrameSaver windowed((dir / "rig.bmp").string(),
                                   std::chrono::seconds(30), 2);
  win::Frame window_frame;
  window_frame.number = 1;
  CHECK(!windowed.record(window_frame, true, true, Clock::now()));
  return 0;
}

// What rig_viewer records into an offscreen frame: the slot's previous frame
// collected, an atlas job committed under the atlas-copy scope when given,
// the viewers' render pass drawing `mesh` (or nothing) with the atlas, the
// readback of a frame to save, and the frame ended.
struct FrameDraw {
  win::FrameLoop& loop;
  vg::Profiler& profiler;
  fuse_viewer::LiveAtlas& atlas;
  const vgp::HybridMeshPipeline& pipeline;
  const vgp::GpuMesh& mesh;
  fuse_viewer::AtlasCopyTimes& copy_times;

  // Returns 0, or 1 on a failed check; `saved` says whether it was saved.
  // Without `tell_ended` the saver is not told the frame was ended, as when
  // rig_viewer's end_frame fails.
  int operator()(fuse_viewer::FrameSaver& saver,
                 const fuse_viewer::AtlasJob* job, bool draws_mesh, bool last,
                 Clock::time_point now, std::uint64_t number, bool* saved,
                 bool tell_ended = true) {
    auto begun = loop.begin_frame(kExtent);  // never skips offscreen
    CHECK(begun.ok() && begun.value().has_value());
    const win::Frame& frame = *begun.value();
    CHECK(frame.number == number);
    CHECK(frame.offscreen != nullptr);
    CHECK(frame.target->extent().width == kExtent.width &&
          frame.target->extent().height == kExtent.height);
    saver.collect(frame);
    copy_times.begun(frame.slot, profiler.metrics().sections);
    atlas.poll();
    bool copied = false;
    {
      const vg::Profiler::Scope copy_scope =
          profiler.gpu_scope(frame.cmd, fuse_viewer::kAtlasCopyRow);
      if (job != nullptr) {
        CHECK(atlas.commit(frame.cmd, frame.number, *job).ok());
        copied = true;
      }
    }
    fuse_viewer::record_view_pass(
        profiler, frame.cmd, *frame.target,
        [&](VkCommandBuffer cmd) {
          if (!draws_mesh) return;
          const vgp::HybridMeshDraw draw{&mesh};
          vgp::HybridMeshFrame hybrid;
          hybrid.extent = kExtent;
          hybrid.view_proj = glm::mat4(1.0f);  // the quad is in clip space
          hybrid.flags = 0;                    // unlit: the atlas as it is
          hybrid.atlas = atlas.use(frame.number);
          hybrid.draws = &draw;
          hybrid.draw_count = 1;
          pipeline.submit(cmd, hybrid);
        },
        nullptr);
    *saved = saver.record(frame, draws_mesh, last, now);
    CHECK(loop.end_frame(frame).ok());
    if (copied) copy_times.copied(frame.slot);
    if (tell_ended) saver.ended(frame);
    return 0;
  }
};

const vkc::StageRow* find_row(const vg::FrameMetrics& metrics,
                              const char* name) {
  for (const vkc::StageRow& row : metrics.sections) {
    if (std::strcmp(row.name, name) == 0) return &row;
  }
  return nullptr;
}

int draws_and_saves_offscreen(const std::filesystem::path& dir) {
  vg::app::WindowedAppConfig config;
  config.app_name = "recon_example_viewer_offscreen_test";
  config.swapchain.extent = kExtent;
  config.swapchain.depth_format = VK_FORMAT_D32_SFLOAT;
  config.frames_in_flight = 2;

  // What the loop draws into: the window's format and depth, read back, at
  // a display's pace.
  const win::OffscreenLoopDesc desc =
      fuse_viewer::offscreen_frames(config.swapchain);
  CHECK(desc.target.extent.width == kExtent.width &&
        desc.target.extent.height == kExtent.height);
  CHECK(desc.target.color_format == VK_FORMAT_B8G8R8A8_SRGB);
  CHECK(desc.target.depth_format == VK_FORMAT_D32_SFLOAT);
  CHECK(desc.target.readback);
  CHECK(desc.frame_interval == std::chrono::nanoseconds(1000000000 / 60));

  auto shared_r = fuse_viewer::make_offscreen_viewer_device(
      config, vkc::test::instance_config().enable_validation);
  if (!shared_r.ok()) {
    return vkc::test::no_device_exit_code("no Vulkan device (" +
                                          shared_r.status().message() + ")");
  }
  const std::unique_ptr<vkc::SharedDevice> shared = std::move(shared_r).value();
  CHECK(shared->surface() == VK_NULL_HANDLE);
  if (const vkc::Status loaded =
          vkc::test::check_layer_loaded(shared->instance());
      !loaded.ok()) {
    std::fprintf(stderr, "%s\n", loaded.message().c_str());
    return 1;
  }

  // recon's and the sensor package's half, as rig_viewer adopts it, and gfx's.
  auto rdevice = fuse_viewer::adopt_fuse_device(*shared);
  CHECK(rdevice.ok());
  auto rallocator =
      vkc::Allocator::create(shared->instance().handle(), rdevice.value());
  CHECK(rallocator.ok());
  auto renderer_r = fuse_viewer::OffscreenRenderer::adopt(*shared, config);
  if (!renderer_r.ok()) {
    std::fprintf(stderr, "OffscreenRenderer::adopt: %s\n",
                 renderer_r.status().message().c_str());
    return 1;
  }
  fuse_viewer::OffscreenRenderer& renderer = *renderer_r.value();
  win::FrameLoop& loop = renderer.frame_loop();
  CHECK(loop.frames_in_flight() == config.frames_in_flight);
  const vg::RenderTargetLayout layout = loop.layout();
  CHECK(layout.color_count == 1);
  CHECK(layout.color_formats[0] == VK_FORMAT_B8G8R8A8_SRGB);
  CHECK(layout.depth_format == VK_FORMAT_D32_SFLOAT);

  // A camera's colour as the frame prep leaves it: a device buffer recon
  // fills, shared with gfx's family, which copies it into the atlas.
  const std::uint32_t families[2] = {shared->compute_family(),
                                     shared->graphics_family()};
  vkc::BufferDesc color_desc;
  color_desc.size = VkDeviceSize{kTile.width} * kTile.height * 4u;
  color_desc.usage =
      VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
  color_desc.memory = vkc::MemoryUsage::DeviceOnly;
  color_desc.queue_families = families;
  color_desc.queue_family_count = 2;
  auto color_r = rallocator.value().create_buffer(color_desc);
  CHECK(color_r.ok());
  const auto color =
      std::make_shared<const vkc::Buffer>(std::move(color_r).value());
  const std::uint32_t word = std::uint32_t{kRed} |
                             (std::uint32_t{kGreen} << 8) |
                             (std::uint32_t{kBlue} << 16) | 0xFF000000u;
  CHECK(rdevice.value()
            .submit_single_time([&](VkCommandBuffer cmd) {
              vkCmdFillBuffer(cmd, color->handle(), 0, VK_WHOLE_SIZE, word);
            })
            .ok());
  CHECK(fuse_viewer::copyable(
      *color, kTile, shared->graphics_family() != shared->compute_family()));
  fuse_viewer::AtlasJob job;
  job.tiles.push_back({color, kTile, 0});

  // A quad in clip space, every vertex sampling the tile's middle; its
  // vertex colour, black, shows if the atlas is not bound.
  vg::assets::Mesh quad;
  for (const auto& [x, y] : {std::pair{-0.5f, -0.5f}, std::pair{0.5f, -0.5f},
                             std::pair{0.5f, 0.5f}, std::pair{-0.5f, 0.5f}}) {
    vg::assets::Vertex v;
    v.position = glm::vec3(x, y, 0.5f);
    v.uv0 = glm::vec2(0.5f, 0.5f);
    v.color = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
    quad.vertices.push_back(v);
  }
  quad.indices = {0, 1, 2, 0, 2, 3};
  auto mesh = vgp::upload_mesh(renderer.device(), renderer.allocator(), quad);
  CHECK(mesh.ok());

  auto pipeline = vgp::HybridMeshPipeline::create(
      renderer.device(), renderer.allocator(), loop.layout());
  CHECK(pipeline.ok());
  fuse_viewer::LiveAtlas atlas(pipeline.value(), renderer.allocator(),
                               loop.timeline(), {kTile.width, kTile.height},
                               config.frames_in_flight);

  vg::ProfilerConfig profiler_config;
  profiler_config.frames_in_flight = config.frames_in_flight;
  auto profiler = vg::Profiler::create(renderer.device(), profiler_config);
  CHECK(profiler.ok());
  profiler.value().set_memory_source(&renderer.allocator());
  loop.set_profiler(&profiler.value());
  const fuse_viewer::ProfilerDetach profiler_guard{loop};
  fuse_viewer::AtlasCopyTimes copy_times(config.frames_in_flight, 0);
  FrameDraw draw{loop,         profiler.value(), atlas, pipeline.value(),
                 mesh.value(), copy_times};

  // Seven frames, ten seconds apart on the saver's clock: the first draws no
  // mesh, the second commits the atlas and draws the mesh, as do the rest.
  // Saved: frame 2, the first with a mesh; frame 5, 30 s after it; frame 7,
  // the last, 20 s after that.
  fuse_viewer::FrameSaver saver((dir / "rig.bmp").string(),
                                std::chrono::seconds(30),
                                config.frames_in_flight);
  const Clock::time_point t0 = Clock::now();
  // Written once each slot's next frame has begun: frame 2 by frame 4, frame
  // 5 by frame 7, frame 7 at the flush.
  const std::size_t written_by[8] = {0, 0, 0, 0, 1, 1, 1, 2};
  for (std::uint64_t n = 1; n <= 7; ++n) {
    bool saved = false;
    if (const int rc =
            draw(saver, n == 2 ? &job : nullptr, n >= 2, n == 7,
                 t0 + std::chrono::seconds(10 * (n - 1)), n, &saved)) {
      return rc;
    }
    CHECK(saved == (n == 2 || n == 5 || n == 7));
    CHECK(saver.saved().size() == written_by[n]);
  }
  CHECK(renderer.device().wait_idle().ok());
  saver.flush();
  CHECK(saver.failures() == 0);
  CHECK(saver.saved() == (std::vector<std::string>{saver.name(2), saver.name(5),
                                                   saver.name(7)}));

  // Each file: the header, the quad's middle in the atlas's colour, and a
  // corner in the pass's clear, (0.05, 0.05, 0.07) linear.
  for (const std::string& path : saver.saved()) {
    const std::vector<std::uint8_t> file = vkc::test::read_bytes(path);
    if (const int rc = check_bmp_header(file, kExtent)) return rc;
    const std::uint8_t* centre =
        bmp_pixel(file, kExtent, kExtent.width / 2, kExtent.height / 2);
    const std::uint8_t* corner = bmp_pixel(file, kExtent, 0, 0);
    std::printf("%s: centre B G R %d %d %d, corner %d %d %d\n", path.c_str(),
                centre[0], centre[1], centre[2], corner[0], corner[1],
                corner[2]);
    CHECK(near(centre[0], kBlue) && near(centre[1], kGreen) &&
          near(centre[2], kRed));
    CHECK(near(corner[0], srgb_byte(0.07)) &&
          near(corner[1], srgb_byte(0.05)) && near(corner[2], srgb_byte(0.05)));
  }

  // The rows a window's loop gives, less the present, beside the viewer's;
  // and the one copy's device time, read from frame 4's snapshot.
  const vg::FrameMetrics& metrics = profiler.value().metrics();
  CHECK(find_row(metrics, fuse_viewer::kAtlasCopyRow) != nullptr);
  CHECK(find_row(metrics, "render pass") != nullptr);
  CHECK(find_row(metrics, "frame wait") != nullptr);
  CHECK(find_row(metrics, "submit") != nullptr);
  CHECK(find_row(metrics, "present") == nullptr);
  const std::string copies = copy_times.report();
  std::fputs(copies.c_str(), stdout);
  CHECK((copies.find("over 1 timed") != std::string::npos &&
         copies.find("; 0 untimed") != std::string::npos) ||
        copies == "atlas copy, device ms: n/a; 1 untimed\n");

  // A saved frame the saver is not told was ended is never written: its
  // readback may not have run. Frame 8 is saved; frame 10 recurs its slot.
  fuse_viewer::FrameSaver unended((dir / "unended.bmp").string(),
                                  std::chrono::seconds(30),
                                  config.frames_in_flight);
  for (std::uint64_t n = 8; n <= 10; ++n) {
    bool saved = false;
    if (const int rc = draw(unended, nullptr, n == 8, false, Clock::now(), n,
                            &saved, n != 8)) {
      return rc;
    }
    CHECK(saved == (n == 8));
  }
  CHECK(renderer.device().wait_idle().ok());
  unended.flush();
  CHECK(unended.saved().empty() && unended.failures() == 0);

  // A path that cannot be written: every saved frame fails, counted and said
  // once, and the frames go on.
  std::FILE* says = std::tmpfile();
  CHECK(says != nullptr);
  fuse_viewer::FrameSaver unwritable((dir / "missing" / "rig.bmp").string(),
                                     std::chrono::nanoseconds(0),
                                     config.frames_in_flight, says);
  for (std::uint64_t n = 11; n <= 13; ++n) {
    bool saved = false;
    if (const int rc =
            draw(unwritable, nullptr, true, n == 13, Clock::now(), n, &saved)) {
      return rc;
    }
    CHECK(saved);
  }
  CHECK(renderer.device().wait_idle().ok());
  unwritable.flush();
  CHECK(unwritable.saved().empty());
  CHECK(unwritable.failures() == 3);
  std::rewind(says);
  std::string said;
  for (int c = std::fgetc(says); c != EOF; c = std::fgetc(says)) {
    said.push_back(static_cast<char>(c));
  }
  std::fclose(says);
  std::fputs(said.c_str(), stdout);
  CHECK(std::count(said.begin(), said.end(), '\n') == 1);
  CHECK(said.find("frame 11 not saved") != std::string::npos);
  return 0;
}

int run_offscreen_tests() {
  const vkc::test::Scratch scratch("recon_viewer_offscreen_");
  if (const int rc = bmp_is_the_bytes_behind_a_header(scratch.path)) {
    return rc;
  }
  if (const int rc = draws_and_saves_offscreen(scratch.path)) return rc;
  std::puts("viewer_offscreen: OK");
  return 0;
}

}  // namespace

int main() {
  const vkc::test::ValidationSession validation;
  const vkc::test::LogCapture log;
  return log.exit_code(run_offscreen_tests());
}
