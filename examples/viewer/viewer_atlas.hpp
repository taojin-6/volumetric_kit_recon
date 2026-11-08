// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/viewer/viewer_atlas.hpp
/// @brief The atlas `fuse_viewer` and `rig_viewer` draw a live mesh with:
///        what a mesh's atlas is copied from, and gfx's `StreamedAtlas`
///        filled from it in the frame that first draws the mesh; the solid
///        tiles `rig_viewer`'s colour-by-camera view copies instead; and the
///        atlas description they and `fuse_render` share.

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "volumetric_kit/core/base/check.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/buffer.hpp"
#include "volumetric_kit/core/vulkan/sync.hpp"
#include "volumetric_kit/core/vulkan/vulkan.hpp"
#include "volumetric_kit/gfx/core/image_update.hpp"
#include "volumetric_kit/gfx/core/retire_queue.hpp"
#include "volumetric_kit/gfx/pipelines/hybrid_mesh_pipeline.hpp"
#include "volumetric_kit/gfx/pipelines/streamed_atlas.hpp"
#include "volumetric_kit/recon/texture/texture_atlas.hpp"

namespace fuse_viewer {

namespace vkc = volumetric_kit::core;
namespace vg = volumetric_kit::gfx;
namespace vgp = volumetric_kit::gfx::pipelines;
namespace rtex = volumetric_kit::recon::texture;

/// @brief One camera's share of an atlas: its colour buffer, as the frame
///        prep left it (R, G, B and coverage in each word -- the bytes of
///        R8G8B8A8), and the tile it goes to.
struct AtlasTileSource {
  std::shared_ptr<const vkc::Buffer> color;
  rtex::AtlasTile tile;
  std::size_t camera = 0;  ///< Which camera, for a colour-by-camera view.
};

/// @brief What a mesh's atlas is copied from: the cameras that textured it,
///        each into its tile. Empty for a mesh no camera textured, which draws
///        in its vertex colour.
struct AtlasJob {
  std::vector<AtlasTileSource> tiles;
  bool empty() const noexcept { return tiles.empty(); }
};

/// @brief Whether gfx can copy @p color into @p tile: a copy source holding
///        the tile's words, and `CONCURRENT` when gfx reads it from another
///        queue family than recon's (@p cross_family), since a copy from a
///        buffer `EXCLUSIVE` to recon's is undefined with nothing to report
///        it.
///
/// Check it before the camera textures a mesh: once its `uv0` point into the
/// tile, the tile must be filled.
inline bool copyable(const vkc::Buffer& color, const rtex::AtlasTile& tile,
                     bool cross_family) {
  return color.valid() &&
         (color.usage() & VK_BUFFER_USAGE_TRANSFER_SRC_BIT) != 0 &&
         (!cross_family ||
          color.sharing_mode() == VK_SHARING_MODE_CONCURRENT) &&
         color.size() >= VkDeviceSize{tile.width} * tile.height * 4u;
}

/// @brief The copies that fill @p job 's tiles: each from its camera's colour
///        buffer, rows of the tile's width, or with @p solid from that
///        camera's entry of @p solid (at least a tile's size).
inline std::vector<vg::ImageCopy> atlas_copies(
    const AtlasJob& job, const std::vector<vkc::Buffer>* solid = nullptr) {
  std::vector<vg::ImageCopy> copies;
  copies.reserve(job.tiles.size());
  for (const AtlasTileSource& source : job.tiles) {
    vg::ImageCopy copy;
    copy.source =
        solid != nullptr ? &(*solid)[source.camera] : source.color.get();
    copy.region.bufferRowLength = source.tile.width;
    copy.region.bufferImageHeight = source.tile.height;
    copy.region.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
    copy.region.imageOffset = {static_cast<std::int32_t>(source.tile.x),
                               static_cast<std::int32_t>(source.tile.y), 0};
    copy.region.imageExtent = {source.tile.width, source.tile.height, 1};
    copies.push_back(copy);
  }
  return copies;
}

/// Each camera's colour in the colour-by-camera view, as sRGB bytes (R, G,
/// B), the encoding the atlas holds; past the eighth, the colours repeat.
inline constexpr std::array<std::array<std::uint8_t, 3>, 8> kCameraColours = {{
    {230, 60, 60},
    {60, 200, 80},
    {60, 110, 240},
    {240, 210, 50},
    {210, 70, 210},
    {60, 210, 220},
    {245, 140, 40},
    {200, 200, 200},
}};

/// @return Camera @p camera 's @ref kCameraColours entry as an atlas word:
///         R, G and B in the low three bytes and full coverage in the high
///         one, the bytes of R8G8B8A8.
inline std::uint32_t camera_colour_word(std::size_t camera) {
  const std::array<std::uint8_t, 3>& rgb =
      kCameraColours[camera % kCameraColours.size()];
  return std::uint32_t{rgb[0]} | (std::uint32_t{rgb[1]} << 8) |
         (std::uint32_t{rgb[2]} << 16) | 0xFF000000u;
}

/// @brief The colour-by-camera view's sources: one device-local buffer a
///        tile, the tile's size, every word its camera's
///        @ref camera_colour_word, for @ref LiveAtlas::commit to copy instead
///        of the cameras' images.
///
/// Made the first time the view is switched on, since at 4K they are 33 MB a
/// camera, and filled on the device in that frame's command buffer:
/// host-visible, a discrete GPU would copy them across the bus on every
/// remesh.
///
/// @code
/// fuse_viewer::SolidTiles solid(allocator);
/// // in the frame that commits a textured job, before the commit:
/// VKC_TRY(solid.ensure(cmd, layout.tiles));
/// VKC_TRY(atlas.commit(cmd, frame, job, &solid.buffers()));
/// @endcode
class SolidTiles {
 public:
  /// @param allocator  Allocates the buffers; it must outlive this.
  explicit SolidTiles(vkc::Allocator& allocator) : allocator_(&allocator) {}

  SolidTiles(const SolidTiles&) = delete;
  SolidTiles& operator=(const SolidTiles&) = delete;
  SolidTiles(SolidTiles&&) = delete;
  SolidTiles& operator=(SolidTiles&&) = delete;

  /// @brief Make a buffer for each of @p tiles, in their order, unless they
  ///        are made, and record into @p cmd their fills and the barrier that
  ///        lands them before a later transfer in @p cmd reads them. With the
  ///        buffers already made, it records nothing.
  /// @param cmd    A command buffer in the recording state, which the atlas
  ///               copy is recorded into after this.
  /// @param tiles  One per camera, in camera order; the same on every call.
  ///               Made buffers are never made again, since a submitted
  ///               frame's copy may still read them and nothing retires them
  ///               by frame, so another count while they are held is a
  ///               programmer error (`VKC_CHECK`).
  /// @return OK; or the allocator's refusal, which leaves no buffer held and
  ///         records nothing, for a later call to try again.
  vkc::Status ensure(VkCommandBuffer cmd,
                     const std::vector<rtex::AtlasTile>& tiles) {
    if (buffers_.size() == tiles.size()) return vkc::Status{};
    VKC_CHECK(buffers_.empty(),
              "SolidTiles::ensure: the tile count changed after the buffers "
              "were made");
    for (const rtex::AtlasTile& tile : tiles) {
      vkc::BufferDesc desc;
      desc.size = VkDeviceSize(tile.width) * tile.height * 4u;
      desc.usage =
          VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT;
      desc.memory = vkc::MemoryUsage::DeviceOnly;
      vkc::Result<vkc::Buffer> buffer = allocator_->create_buffer(desc);
      if (!buffer.ok()) {
        buffers_.clear();
        return buffer.status();
      }
      buffers_.push_back(std::move(buffer).value());
    }
    // Filled only once every buffer exists, so a failed allocation above
    // frees none that a recorded fill names.
    for (std::size_t c = 0; c < buffers_.size(); ++c) {
      vkCmdFillBuffer(cmd, buffers_[c].handle(), 0, VK_WHOLE_SIZE,
                      camera_colour_word(c));
    }
    // The fills land before the atlas copy reads them, later in this buffer.
    VkMemoryBarrier filled{};
    filled.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER;
    filled.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    filled.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
    vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TRANSFER_BIT,
                         VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 1, &filled, 0,
                         nullptr, 0, nullptr);
    return vkc::Status{};
  }

  /// @return One buffer a tile, in the tiles' order; empty until
  ///         @ref ensure first succeeds. The `solid` @ref LiveAtlas::commit
  ///         takes.
  const std::vector<vkc::Buffer>& buffers() const noexcept { return buffers_; }

 private:
  vkc::Allocator* allocator_;
  std::vector<vkc::Buffer> buffers_;
};

/// @brief The atlas the viewers draw with: @p extent, in the frame prep's
///        canonical-encoded 8-bit colour, so the sampler decodes and filters
///        in linear (the 2026-08-02 colour-space decision), and a ring of
///        @p frames_in_flight plus one images.
///
/// With one copy a frame, that ring never waits for a frame, and it always
/// has the second image gfx needs to keep the picture a discard restores, so
/// a program drawing a single frame passes 1.
inline vgp::StreamedAtlasDesc atlas_desc(VkExtent2D extent,
                                         std::uint32_t frames_in_flight) {
  vgp::StreamedAtlasDesc desc;
  desc.extent = extent;
  desc.format = VK_FORMAT_R8G8B8A8_SRGB;
  desc.slots = frames_in_flight + 1;
  return desc;
}

/// @brief The atlas a live mesh is drawn with: gfx's `StreamedAtlas`,
///        rewritten from a mesh's @ref AtlasJob in the frame that first draws
///        the mesh.
///
/// The ring of images, the copy and its ordering are gfx's. What this adds is
/// when the ring is made and the colour buffers' lifetime. The ring, the
/// frames in flight plus one images of the whole atlas, is made by the first
/// textured job, so a viewer that never textures a mesh holds none. The copy
/// reads the colour buffers when the frame runs, so they are held until the
/// frame loop's timeline reaches the frame's number (gfx's `RetireQueue`),
/// and a frame prep reuses one only after that. A mesh no camera textured
/// draws in its vertex colour.
///
/// Commit a mesh's job in the frame that commits the mesh, so the mesh's
/// `uv0` and the picture they index into change together.
class LiveAtlas {
 public:
  /// @brief An atlas of @p extent for @p pipeline, its frames numbered on
  ///        @p frames. It allocates nothing until the first textured
  ///        @ref commit.
  /// @param pipeline          The pipeline that draws with it.
  /// @param allocator         Allocates the images.
  /// @param frames            The frame loop's timeline
  ///                          (`windowing::FrameLoop::timeline`).
  /// @param extent            The atlas's size; every tile lies inside it.
  /// @param frames_in_flight  The frame loop's (@ref atlas_desc).
  /// @warning @p pipeline, @p allocator and @p frames must outlive it.
  LiveAtlas(const vgp::HybridMeshPipeline& pipeline, vkc::Allocator& allocator,
            const vkc::TimelineSemaphore& frames, VkExtent2D extent,
            std::uint32_t frames_in_flight)
      : pipeline_(&pipeline),
        allocator_(&allocator),
        frames_(&frames),
        desc_(atlas_desc(extent, frames_in_flight)),
        held_(frames) {}

  LiveAtlas(const LiveAtlas&) = delete;
  LiveAtlas& operator=(const LiveAtlas&) = delete;
  LiveAtlas(LiveAtlas&&) = delete;
  LiveAtlas& operator=(LiveAtlas&&) = delete;
  ~LiveAtlas() {
    // StreamedAtlas waits for submitted frames, or drains the queues if the
    // newest frame never reached them. Keep the copy sources through that
    // wait, then reclaim them without waiting for an unsubmitted frame's
    // timeline value, which nothing will signal.
    atlas_ = {};
    held_.reclaim();
  }

  /// @brief Let go of the colour buffers of the frames that have completed.
  ///        Once a frame.
  void poll() { held_.poll(); }

  /// @brief Draw from frame @p frame on with @p job : record its copies into
  ///        frame @p frame 's command buffer @p cmd, before the rendering
  ///        scope, and hold its colour buffers until the frame completes. An
  ///        empty job records nothing, and the frames draw in vertex colour.
  ///        The first textured job makes the ring.
  /// @param solid  Each camera's solid-colour buffer to copy from instead of
  ///               its image, or null.
  /// @return OK; or why the job was not committed -- the ring could not be
  ///         made, or gfx refused the update -- which leaves the atlas as it
  ///         was, holding nothing new. The next textured job tries again.
  vkc::Status commit(VkCommandBuffer cmd, std::uint64_t frame,
                     const AtlasJob& job,
                     const std::vector<vkc::Buffer>* solid = nullptr) {
    if (job.empty()) {
      textured_ = false;
      return vkc::Status{};
    }
    if (!atlas_.valid()) {
      VKC_ASSIGN(atlas_, vgp::StreamedAtlas::create(*pipeline_, *allocator_,
                                                    *frames_, desc_));
    }
    const std::vector<vg::ImageCopy> copies = atlas_copies(job, solid);
    VKC_TRY(atlas_.record_update(cmd, frame, copies.data(),
                                 static_cast<std::uint32_t>(copies.size())));
    textured_ = true;
    std::vector<std::shared_ptr<const vkc::Buffer>> colors;
    for (const AtlasTileSource& source : job.tiles) {
      colors.push_back(source.color);
    }
    held_.push(frame,
               [colors = std::move(colors)]() mutable { colors.clear(); });
    return vkc::Status{};
  }

  /// @return The set frame @p frame binds: the newest picture's, or
  ///         `VK_NULL_HANDLE` -- every triangle in its vertex colour -- when
  ///         the committed mesh is untextured.
  VkDescriptorSet use(std::uint64_t frame) {
    return textured_ ? atlas_.use(frame) : VK_NULL_HANDLE;
  }

 private:
  const vgp::HybridMeshPipeline* pipeline_;
  vkc::Allocator* allocator_;
  const vkc::TimelineSemaphore* frames_;
  vgp::StreamedAtlasDesc desc_;
  vgp::StreamedAtlas atlas_;  // empty until the first textured job
  vg::RetireQueue held_;      // the colour buffers each frame's copy reads
  bool textured_ = false;
};

}  // namespace fuse_viewer
