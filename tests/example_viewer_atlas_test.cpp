// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The atlas the viewers draw with (examples/viewer/viewer_atlas.hpp): host
// pixels are uploaded as an _SRGB image of their extent, ready to sample, and
// bound through a pool and set of their own; the white dummy is one texel; an
// image made elsewhere, as rig_viewer's device-copied atlas, is adopted as
// is; and an empty upload is refused. Needs a device, so it skips (exit 0)
// where none is present.

#include <cstdint>
#include <cstdio>
#include <memory>
#include <utility>
#include <vector>

#include "no_device.hpp"
#include "viewer_atlas.hpp"

#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/descriptor.hpp"
#include "volumetric_kit/core/vulkan/vulkan.hpp"
#include "volumetric_kit/gfx/app/headless_app.hpp"
#include "volumetric_kit/gfx/core/sampler.hpp"

#define CHECK(cond)                                                        \
  do {                                                                     \
    if (!(cond)) {                                                         \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      return 1;                                                            \
    }                                                                      \
  } while (0)

namespace {

namespace vkc = volumetric_kit::core;
namespace vg = volumetric_kit::gfx;
using fuse_viewer::Atlas;

bool ready_srgb(const Atlas& atlas, std::uint32_t width, std::uint32_t height) {
  return atlas.tex.valid() && atlas.tex.width() == width &&
         atlas.tex.height() == height &&
         atlas.tex.format() == VK_FORMAT_R8G8B8A8_SRGB &&
         atlas.tex.layout() == VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL &&
         atlas.pool.handle() != VK_NULL_HANDLE && atlas.set.valid();
}

int run(vg::app::HeadlessApp& app) {
  auto sampler_result = vg::Sampler::create(app.device().handle());
  CHECK(sampler_result.ok());
  const vg::Sampler sampler = std::move(sampler_result).value();
  // As the hybrid-mesh pipeline's set 0: one combined image sampler.
  VkDescriptorSetLayoutBinding binding{};
  binding.binding = 0;
  binding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
  binding.descriptorCount = 1;
  binding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
  auto layout_result =
      vkc::DescriptorSetLayout::create(app.device().handle(), &binding, 1);
  CHECK(layout_result.ok());
  const vkc::DescriptorSetLayout layout = std::move(layout_result).value();

  // A 3 x 2 keyframe image: its extent, not a square or a fixed size.
  const std::vector<std::uint32_t> pixels = {
      0xFF0000FFu, 0xFF00FF00u, 0xFFFF0000u,
      0xFFFFFFFFu, 0x00000000u, 0xFF808080u,
  };
  auto uploaded =
      fuse_viewer::upload_atlas(app.device(), app.allocator(), layout.handle(),
                                sampler.handle(), pixels.data(), 3, 2);
  CHECK(uploaded.ok());
  const std::shared_ptr<Atlas> keyframe = std::move(uploaded).value();
  CHECK(ready_srgb(*keyframe, 3, 2));

  // Each atlas has a pool of its own, so one frees without the other.
  auto white = fuse_viewer::white_atlas(app.device(), app.allocator(),
                                        layout.handle(), sampler.handle());
  CHECK(white.ok());
  CHECK(ready_srgb(*white.value(), 1, 1));
  CHECK(white.value()->pool.handle() != keyframe->pool.handle());
  CHECK(white.value()->set.handle() != keyframe->set.handle());

  // An image made elsewhere, filled later by device copies.
  vkc::ImageDesc desc;
  desc.extent = {4, 4};
  desc.format = VK_FORMAT_R8G8B8A8_SRGB;
  desc.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
  auto image = app.allocator().create_image(desc);
  CHECK(image.ok());
  const VkImage handle = image.value().handle();
  const VkImageView view = image.value().view();
  auto bound =
      fuse_viewer::bind_atlas(app.device().handle(), layout.handle(),
                              sampler.handle(), std::move(image).value());
  CHECK(bound.ok());
  CHECK(bound.value()->tex.handle() == handle);
  CHECK(bound.value()->tex.view() == view);
  CHECK(bound.value()->set.valid());

  // Nothing to upload.
  CHECK(!fuse_viewer::upload_atlas(app.device(), app.allocator(),
                                   layout.handle(), sampler.handle(), nullptr,
                                   0, 0)
             .ok());
  return 0;
}

}  // namespace

int main() {
  vg::app::HeadlessAppConfig config;
  config.app_name = "recon_example_viewer_atlas_test";
  auto app = vg::app::HeadlessApp::create(config);
  if (!app.ok()) {
    return vr_test::no_device("no Vulkan device", app.status().message());
  }
  if (const int rc = run(app.value())) return rc;
  std::puts("viewer_atlas: OK");
  return 0;
}
