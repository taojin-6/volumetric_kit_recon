// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The atlas the live viewers draw with (examples/viewer/viewer_atlas.hpp):
// each tile is copied from its camera's colour buffer, or its solid-colour
// one, rows of the tile's width placed at the tile; a buffer gfx cannot copy
// is refused before it textures anything; and LiveAtlas allocates its ring
// only for the first textured job, refusing the job (and holding nothing)
// when the ring cannot be made, holds a frame's colour buffers until the
// frame loop's timeline reaches that frame, binds nothing (vertex colour) for
// an untextured mesh, and leaves the atlas as it was when a copy is refused.
// Teardown keeps submitted copies' sources alive and releases the sources of
// a later frame that never reached the queue without waiting for its number.
// The copies run on a headless device, which the test skips without.

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <memory>
#include <optional>
#include <thread>
#include <utility>
#include <vector>

#include "test_check.hpp"
#include "viewer_atlas.hpp"
#include "volumetric_kit/core/testing/vulkan_policy.hpp"

#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/buffer.hpp"
#include "volumetric_kit/core/vulkan/command_pool.hpp"
#include "volumetric_kit/core/vulkan/sync.hpp"
#include "volumetric_kit/core/vulkan/vulkan.hpp"
#include "volumetric_kit/gfx/app/headless_app.hpp"
#include "volumetric_kit/gfx/core/render_target.hpp"
#include "volumetric_kit/gfx/pipelines/hybrid_mesh_pipeline.hpp"

namespace {

namespace vkc = volumetric_kit::core;
namespace vg = volumetric_kit::gfx;
namespace vgp = volumetric_kit::gfx::pipelines;
namespace rtex = volumetric_kit::recon::texture;
using fuse_viewer::AtlasJob;
using fuse_viewer::LiveAtlas;

// A rig of two cameras: 3 x 2 tiles side by side, then a third tile below
// the first, which the job leaves out.
int copies_place_each_tile() {
  const auto a = std::make_shared<const vkc::Buffer>();
  const auto b = std::make_shared<const vkc::Buffer>();
  AtlasJob job;
  job.tiles.push_back({a, rtex::AtlasTile{0, 0, 3, 2}, 0});
  job.tiles.push_back({b, rtex::AtlasTile{3, 0, 3, 2}, 1});

  const std::vector<vg::ImageCopy> copies = fuse_viewer::atlas_copies(job);
  CHECK(copies.size() == 2);
  CHECK(copies[0].source == a.get());
  CHECK(copies[1].source == b.get());
  for (const vg::ImageCopy& copy : copies) {
    CHECK(copy.region.bufferOffset == 0);
    CHECK(copy.region.bufferRowLength == 3);
    CHECK(copy.region.bufferImageHeight == 2);
    CHECK(copy.region.imageSubresource.aspectMask == VK_IMAGE_ASPECT_COLOR_BIT);
    CHECK(copy.region.imageSubresource.mipLevel == 0);
    CHECK(copy.region.imageSubresource.layerCount == 1);
    CHECK(copy.region.imageOffset.y == 0 && copy.region.imageOffset.z == 0);
    CHECK(copy.region.imageExtent.width == 3);
    CHECK(copy.region.imageExtent.height == 2);
    CHECK(copy.region.imageExtent.depth == 1);
  }
  CHECK(copies[0].region.imageOffset.x == 0);
  CHECK(copies[1].region.imageOffset.x == 3);

  // The colour-by-camera view: each tile from its camera's solid buffer.
  const std::vector<vkc::Buffer> solid(2);
  const std::vector<vg::ImageCopy> sourced =
      fuse_viewer::atlas_copies(job, &solid);
  CHECK(sourced.size() == 2);
  CHECK(sourced[0].source == &solid[0]);
  CHECK(sourced[1].source == &solid[1]);

  CHECK(fuse_viewer::atlas_copies(AtlasJob{}).empty());
  return 0;
}

// A mapped colour buffer of `words`, with `usage`.
vkc::Result<std::shared_ptr<const vkc::Buffer>> make_color(
    vkc::Allocator& allocator, const std::vector<std::uint32_t>& words,
    VkBufferUsageFlags usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT) {
  vkc::BufferDesc desc;
  desc.size = words.size() * sizeof(std::uint32_t);
  desc.usage = usage;
  desc.memory = vkc::MemoryUsage::Staging;
  VKC_ASSIGN(vkc::Buffer buffer, allocator.create_buffer(desc));
  std::memcpy(buffer.mapped(), words.data(), desc.size);
  return std::make_shared<const vkc::Buffer>(std::move(buffer));
}

int copyable_checks_the_buffer(vkc::Allocator& allocator) {
  const rtex::AtlasTile tile{0, 0, 2, 2};
  auto fits = make_color(allocator, std::vector<std::uint32_t>(4, 0));
  CHECK(fits.ok());
  // EXCLUSIVE to one family: copyable on it, not from another family.
  CHECK(fuse_viewer::copyable(*fits.value(), tile, false));
  CHECK(!fuse_viewer::copyable(*fits.value(), tile, true));
  auto short_of_it = make_color(allocator, std::vector<std::uint32_t>(3, 0));
  CHECK(short_of_it.ok());
  CHECK(!fuse_viewer::copyable(*short_of_it.value(), tile, false));
  auto not_a_source = make_color(allocator, std::vector<std::uint32_t>(4, 0),
                                 VK_BUFFER_USAGE_TRANSFER_DST_BIT);
  CHECK(not_a_source.ok());
  CHECK(!fuse_viewer::copyable(*not_a_source.value(), tile, false));
  CHECK(!fuse_viewer::copyable(vkc::Buffer{}, tile, false));
  return 0;
}

int live_atlas_holds_a_frames_colour(vg::app::HeadlessApp& app) {
  vg::RenderTargetLayout target;
  target.color_formats[0] = VK_FORMAT_R8G8B8A8_SRGB;
  target.color_count = 1;
  target.depth_format = VK_FORMAT_D32_SFLOAT;
  auto pipeline =
      vgp::HybridMeshPipeline::create(app.device(), app.allocator(), target);
  CHECK(pipeline.ok());
  auto frames = vkc::TimelineSemaphore::create(app.device());
  CHECK(frames.ok());
  auto gate = vkc::TimelineSemaphore::create(app.device());
  CHECK(gate.ok());
  LiveAtlas atlas(pipeline.value(), app.allocator(), frames.value(), {4, 2}, 2);
  CHECK(atlas.use(1) == VK_NULL_HANDLE);  // nothing committed: vertex colour

  // Two cameras, a 2 x 2 tile each.
  auto left = make_color(app.allocator(), {1, 2, 3, 4});
  auto right = make_color(app.allocator(), {5, 6, 7, 8});
  CHECK(left.ok() && right.ok());
  AtlasJob job;
  job.tiles.push_back({left.value(), rtex::AtlasTile{0, 0, 2, 2}, 0});
  job.tiles.push_back({right.value(), rtex::AtlasTile{2, 0, 2, 2}, 1});

  // Frame 1 copies the job and is held at the gate, so it has not run.
  vkc::Status committed;
  VkDescriptorSet bound = VK_NULL_HANDLE;
  auto frame1 = app.device().submit_pending(
      [&](VkCommandBuffer cmd) {
        committed = atlas.commit(cmd, 1, job);
        bound = atlas.use(1);
      },
      {{&gate.value(), 1}}, {{&frames.value(), 1}});
  CHECK(frame1.ok());
  CHECK(committed.ok());
  CHECK(bound != VK_NULL_HANDLE);
  job.tiles.clear();  // the published job is gone; the atlas holds the colour
  atlas.poll();
  CHECK(left.value().use_count() == 2);
  CHECK(right.value().use_count() == 2);

  // Once frame 1 completes, the colour is let go.
  CHECK(gate.value().signal(1).ok());
  CHECK(frame1.value().wait().ok());
  atlas.poll();
  CHECK(left.value().use_count() == 1);
  CHECK(right.value().use_count() == 1);

  // A copy gfx refuses -- a tile past the atlas's edge -- holds nothing and
  // leaves frame 1's picture bound.
  AtlasJob outside;
  outside.tiles.push_back({left.value(), rtex::AtlasTile{3, 0, 2, 2}, 0});
  vkc::Status refused;
  auto frame2 = app.device().submit_pending(
      [&](VkCommandBuffer cmd) {
        refused = atlas.commit(cmd, 2, outside);
        bound = atlas.use(2);
      },
      {}, {{&frames.value(), 2}});
  CHECK(frame2.ok());
  CHECK(!refused.ok());
  CHECK(bound != VK_NULL_HANDLE);
  CHECK(frame2.value().wait().ok());
  outside.tiles.clear();
  CHECK(left.value().use_count() == 1);

  // A mesh no camera textured draws in vertex colour.
  auto frame3 = app.device().submit_pending(
      [&](VkCommandBuffer cmd) {
        committed = atlas.commit(cmd, 3, AtlasJob{});
        bound = atlas.use(3);
      },
      {}, {{&frames.value(), 3}});
  CHECK(frame3.ok());
  CHECK(committed.ok());
  CHECK(bound == VK_NULL_HANDLE);
  CHECK(frame3.value().wait().ok());
  return 0;
}

// A frame that failed before submission leaves the same state as an ended
// command buffer that is never queued. An earlier copy is still in flight:
// teardown must wait for that copy, then free both sources without waiting
// for the unsubmitted frame's number, which nothing will ever signal.
int live_atlas_teardown_after_an_unsubmitted_frame(vg::app::HeadlessApp& app) {
  vg::RenderTargetLayout target;
  target.color_formats[0] = VK_FORMAT_R8G8B8A8_SRGB;
  target.color_count = 1;
  target.depth_format = VK_FORMAT_D32_SFLOAT;
  auto pipeline =
      vgp::HybridMeshPipeline::create(app.device(), app.allocator(), target);
  CHECK(pipeline.ok());
  auto frames = vkc::TimelineSemaphore::create(app.device());
  CHECK(frames.ok());
  auto gate = vkc::TimelineSemaphore::create(app.device());
  CHECK(gate.ok());
  auto pool = vkc::CommandPool::create(app.device().handle(),
                                       app.device().queue_family());
  CHECK(pool.ok());
  auto cmd = pool.value().allocate_primary();
  CHECK(cmd.ok());
  auto first = make_color(app.allocator(), {1, 2, 3, 4});
  auto second = make_color(app.allocator(), {5, 6, 7, 8});
  CHECK(first.ok() && second.ok());
  const std::weak_ptr<const vkc::Buffer> first_source = first.value();
  const std::weak_ptr<const vkc::Buffer> second_source = second.value();
  auto atlas = std::make_unique<LiveAtlas>(pipeline.value(), app.allocator(),
                                           frames.value(), VkExtent2D{2, 2}, 2);

  AtlasJob job;
  job.tiles.push_back({first.value(), rtex::AtlasTile{0, 0, 2, 2}, 0});
  vkc::Status committed;
  auto frame1 = app.device().submit_pending(
      [&](VkCommandBuffer recording) {
        committed = atlas->commit(recording, 1, job);
      },
      {{&gate.value(), 1}}, {{&frames.value(), 1}});
  // Open the gate even if a later check fails, before PendingSubmit waits.
  struct OpenGate {
    vkc::TimelineSemaphore& gate;
    ~OpenGate() {
      const auto reached = gate.value();
      if (reached.ok() && reached.value() < 1) (void)gate.signal(1);
    }
  } open_gate{gate.value()};
  CHECK(frame1.ok() && committed.ok());

  job.tiles[0].color = second.value();
  CHECK(cmd.value().begin(VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT).ok());
  CHECK(atlas->commit(cmd.value().handle(), 2, job).ok());
  CHECK(cmd.value().end().ok());
  job.tiles.clear();
  first.value().reset();
  second.value().reset();

  bool sources_held = false;
  vkc::Status opened;
  std::thread release([&] {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    sources_held = !first_source.expired() && !second_source.expired();
    opened = gate.value().signal(1);
  });
  atlas.reset();
  release.join();
  CHECK(opened.ok() && sources_held);
  CHECK(frame1.value().wait().ok());
  const auto reached = frames.value().value();
  CHECK(reached.ok() && reached.value() == 1);  // frame 2 was never submitted
  CHECK(first_source.expired() && second_source.expired());
  return 0;
}

// The bytes of the allocator's live allocations, over every heap.
std::uint64_t allocated(const vkc::Allocator& allocator) {
  const vkc::MemoryStats stats = allocator.memory_stats();
  std::uint64_t bytes = 0;
  for (std::uint32_t h = 0; h < stats.heap_count; ++h) {
    bytes += stats.heaps[h].allocation_bytes;
  }
  return bytes;
}

// The ring is made by the first textured job, not before, and a ring that
// cannot be made refuses the job, which the next frame retries.
int live_atlas_allocates_for_its_first_textured_job(vg::app::HeadlessApp& app) {
  vg::RenderTargetLayout target;
  target.color_formats[0] = VK_FORMAT_R8G8B8A8_SRGB;
  target.color_count = 1;
  target.depth_format = VK_FORMAT_D32_SFLOAT;
  auto pipeline =
      vgp::HybridMeshPipeline::create(app.device(), app.allocator(), target);
  CHECK(pipeline.ok());
  auto frames = vkc::TimelineSemaphore::create(app.device());
  CHECK(frames.ok());
  auto color = make_color(app.allocator(), {1, 2, 3, 4});
  CHECK(color.ok());
  AtlasJob job;
  job.tiles.push_back({color.value(), rtex::AtlasTile{0, 0, 2, 2}, 0});

  // Made, then given an untextured mesh: nothing allocated.
  const std::uint64_t before = allocated(app.allocator());
  LiveAtlas atlas(pipeline.value(), app.allocator(), frames.value(), {4, 2}, 2);
  vkc::Status committed;
  VkDescriptorSet bound = VK_NULL_HANDLE;
  auto frame1 = app.device().submit_pending(
      [&](VkCommandBuffer cmd) {
        committed = atlas.commit(cmd, 1, AtlasJob{});
        bound = atlas.use(1);
      },
      {}, {{&frames.value(), 1}});
  CHECK(frame1.ok());
  CHECK(committed.ok());
  CHECK(bound == VK_NULL_HANDLE);
  CHECK(frame1.value().wait().ok());
  CHECK(allocated(app.allocator()) == before);

  // The first textured job makes the ring: three 4 x 2 RGBA8 images.
  auto frame2 = app.device().submit_pending(
      [&](VkCommandBuffer cmd) {
        committed = atlas.commit(cmd, 2, job);
        bound = atlas.use(2);
      },
      {}, {{&frames.value(), 2}});
  CHECK(frame2.ok());
  CHECK(committed.ok());
  CHECK(bound != VK_NULL_HANDLE);
  CHECK(frame2.value().wait().ok());
  CHECK(allocated(app.allocator()) >= before + 3u * 4u * 2u * 4u);
  atlas.poll();
  CHECK(color.value().use_count() == 2);  // `job`'s and this test's

  // An atlas wider than the device allows: made all the same, and each
  // textured job is refused, holding nothing and drawing in vertex colour.
  auto wide_frames = vkc::TimelineSemaphore::create(app.device());
  CHECK(wide_frames.ok());
  const std::uint32_t too_wide =
      app.device().caps().limits().maxImageDimension2D + 1u;
  LiveAtlas wide(pipeline.value(), app.allocator(), wide_frames.value(),
                 {too_wide, 2}, 2);
  for (std::uint64_t frame = 1; frame <= 2; ++frame) {
    auto submitted = app.device().submit_pending(
        [&](VkCommandBuffer cmd) {
          committed = wide.commit(cmd, frame, job);
          bound = wide.use(frame);
        },
        {}, {{&wide_frames.value(), frame}});
    CHECK(submitted.ok());
    CHECK(!committed.ok());
    CHECK(bound == VK_NULL_HANDLE);
    CHECK(submitted.value().wait().ok());
    wide.poll();
    CHECK(color.value().use_count() == 2);
  }
  return 0;
}

int run_atlas_tests() {
  if (const int rc = copies_place_each_tile()) return rc;

  vg::app::HeadlessAppConfig config;
  config.app_name = "recon_example_viewer_atlas_test";
  config.enable_validation = vkc::test::instance_config().enable_validation;
  auto app = vg::app::HeadlessApp::create(config);
  if (!app.ok()) {
    return vkc::test::no_device_exit_code("no Vulkan device (" +
                                          app.status().message() + ")");
  }
  if (const vkc::Status loaded =
          vkc::test::check_layer_loaded(app.value().instance());
      !loaded.ok()) {
    std::fprintf(stderr, "%s\n", loaded.message().c_str());
    return 1;
  }
  if (const int rc = copyable_checks_the_buffer(app.value().allocator())) {
    return rc;
  }
  if (const int rc =
          live_atlas_allocates_for_its_first_textured_job(app.value())) {
    return rc;
  }
  if (const int rc = live_atlas_holds_a_frames_colour(app.value())) return rc;
  if (const int rc =
          live_atlas_teardown_after_an_unsubmitted_frame(app.value())) {
    return rc;
  }
  std::puts("viewer_atlas: OK");
  return 0;
}

}  // namespace

int main() {
  const vkc::test::ValidationSession validation;
  const vkc::test::LogCapture log;
  return log.exit_code(run_atlas_tests());
}
