// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/viewer/viewer_atlas.hpp
/// @brief The image a textured mesh's `uv0` index into, bound for gfx's
///        hybrid-mesh pipeline: the atlas `fuse_render`, `fuse_viewer` and
///        `rig_viewer` draw with.

#include <cstdint>
#include <memory>
#include <utility>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/descriptor.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/core/vulkan/image.hpp"
#include "volumetric_kit/core/vulkan/vulkan.hpp"
#include "volumetric_kit/gfx/core/texture_upload.hpp"

namespace fuse_viewer {

namespace vkc = volumetric_kit::core;
namespace vg = volumetric_kit::gfx;

/// @brief An atlas image, a descriptor pool of its own, and the set binding
///        the image at binding 0 as a combined image sampler.
///
/// The pool is its own because a set is freed only with its pool, so the
/// atlas frees whole when its last owner drops it. A viewer holds one per
/// frame slot that bound it, so an atlas a pending frame samples outlives its
/// replacement.
struct Atlas {
  vkc::Image tex;
  vkc::DescriptorPool pool;
  vkc::DescriptorSet set;
};

/// @brief Bind @p image as an atlas, sampled through @p sampler in
///        `SHADER_READ_ONLY_OPTIMAL`.
/// @param device   The device @p image and @p layout belong to.
/// @param layout   A set layout whose binding 0 is one combined image
///                 sampler, as `HybridMeshPipeline::descriptor_set_layout(0)`.
/// @param sampler  Outlives the atlas.
/// @param image    Adopted; in `SHADER_READ_ONLY_OPTIMAL` by the time a draw
///                 samples it.
inline vkc::Result<std::shared_ptr<Atlas>> bind_atlas(
    VkDevice device, VkDescriptorSetLayout layout, VkSampler sampler,
    vkc::Image image) {
  const VkDescriptorPoolSize pool_size{
      VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1};
  VKC_ASSIGN(vkc::DescriptorPool pool,
             vkc::DescriptorPool::create(device, &pool_size, 1, 1));
  VKC_ASSIGN(vkc::DescriptorSet set, pool.allocate(layout));
  auto atlas = std::make_shared<Atlas>();
  atlas->tex = std::move(image);
  atlas->pool = std::move(pool);
  atlas->set = std::move(set);
  atlas->set.write_combined_image_sampler(
      0, atlas->tex.view(), sampler, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
  return atlas;
}

/// @brief Upload @p width x @p height canonical-encoded RGBA8 pixels and bind
///        them as an atlas (@ref bind_atlas).
///
/// The image is `_SRGB`, so the sampler decodes and filters in linear, and the
/// presentation target applies the one encode (the 2026-08-02 colour-space
/// decision).
/// @param pixels  `width * height * 4` bytes, R first.
inline vkc::Result<std::shared_ptr<Atlas>> upload_atlas(
    const vkc::Device& device, vkc::Allocator& allocator,
    VkDescriptorSetLayout layout, VkSampler sampler, const void* pixels,
    std::uint32_t width, std::uint32_t height) {
  vg::ImageUploadDesc desc;
  desc.extent = {width, height};
  desc.format = VK_FORMAT_R8G8B8A8_SRGB;
  desc.pixels = pixels;
  desc.size = VkDeviceSize{width} * height * 4;
  VKC_ASSIGN(vkc::Image image, vg::upload_texture(device, allocator, desc));
  return bind_atlas(device.handle(), layout, sampler, std::move(image));
}

/// @brief A 1 x 1 white atlas, bound for a mesh no camera textured.
///
/// The pipeline needs an atlas bound all the same; the shader takes the
/// vertex colour wherever `uv0` is the sentinel, so the texel is never read.
inline vkc::Result<std::shared_ptr<Atlas>> white_atlas(
    const vkc::Device& device, vkc::Allocator& allocator,
    VkDescriptorSetLayout layout, VkSampler sampler) {
  static constexpr std::uint8_t kWhite[4] = {255, 255, 255, 255};
  return upload_atlas(device, allocator, layout, sampler, kWhite, 1, 1);
}

}  // namespace fuse_viewer
