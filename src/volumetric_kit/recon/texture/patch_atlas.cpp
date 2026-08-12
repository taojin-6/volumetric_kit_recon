// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/recon/texture/patch_atlas.hpp"

#include <cstddef>
#include <cstring>

#include "texture_patch_fuse_comp.spv.hpp"
#include "volumetric_kit/recon/core/allocator.hpp"
#include "volumetric_kit/recon/core/compute_util.hpp"
#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/vulkan.hpp"

namespace volumetric_kit::recon::texture {
namespace {

// Matches `layout(local_size_x = 256)` in texture_patch_fuse.comp.
constexpr std::uint32_t kLocalSize = 256;

// The push block the fusion kernel reads; mirrors its `PushConstants` under
// scalar layout. All 4-byte scalars, so every offset is pinned below rather
// than trusted to declaration order -- a same-size reorder would keep sizeof
// unchanged and silently misread every field.
struct PushConstants {
  std::uint32_t num_triangles;
  std::uint32_t patch_leg;
  std::uint32_t texels_per_patch;
  std::uint32_t color_width;
  std::uint32_t color_height;
  float occlusion_threshold;
  float max_weight;
  float normal_epsilon;
  float depth_falloff;
  float near_depth;
  float far_depth;
};
static_assert(sizeof(PushConstants) == 44, "PushConstants must be 44 bytes");
static_assert(offsetof(PushConstants, num_triangles) == 0, "layout drift");
static_assert(offsetof(PushConstants, patch_leg) == 4, "layout drift");
static_assert(offsetof(PushConstants, texels_per_patch) == 8, "layout drift");
static_assert(offsetof(PushConstants, color_width) == 12, "layout drift");
static_assert(offsetof(PushConstants, color_height) == 16, "layout drift");
static_assert(offsetof(PushConstants, occlusion_threshold) == 20,
              "layout drift");
static_assert(offsetof(PushConstants, max_weight) == 24, "layout drift");
static_assert(offsetof(PushConstants, normal_epsilon) == 28, "layout drift");
static_assert(offsetof(PushConstants, depth_falloff) == 32, "layout drift");
static_assert(offsetof(PushConstants, near_depth) == 36, "layout drift");
static_assert(offsetof(PushConstants, far_depth) == 40, "layout drift");

// Below this a patch has no barycentric span: vrPatchBarycentric divides by
// leg - 1, and one texel cannot stand for a triangle.
constexpr std::uint32_t kMinPatchLeg = 2;

}  // namespace

Result<PatchAtlas> PatchAtlas::create(Device& device, Allocator& allocator,
                                      const PatchAtlasConfig& config) {
  if (config.patch_leg < kMinPatchLeg) {
    return Status::invalid_argument(
        "PatchAtlas::create: patch_leg must be at least " +
        std::to_string(kMinPatchLeg));
  }
  const VkDevice dev = device.handle();

  PatchAtlas atlas;
  atlas.device_ = &device;
  atlas.allocator_ = &allocator;
  atlas.config_ = config;

  // Six storage-buffer bindings: vertices, indices, patches, depth, colour,
  // camera. The count is a hand-passed literal and so is every
  // `layout(binding = N)` in the kernel, and nothing links the two -- the
  // compute core is explicit, not reflected (the 2026-07-05 decision) -- so the
  // GLSL and this `6` are one edit.
  VkPushConstantRange push_range{};
  push_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
  push_range.offset = 0;
  push_range.size = sizeof(PushConstants);
  KernelSetBuilder kb(dev);
  VR_TRY(kb.add(atlas.kernel_, vr_texture_patch_fuse_comp_spv,
                vr_texture_patch_fuse_comp_spv_size, 6, &push_range));
  VR_ASSIGN(atlas.pool_, kb.build());

  VkPhysicalDeviceProperties props{};
  vkGetPhysicalDeviceProperties(device.physical_device(), &props);
  atlas.max_workgroup_count_x_ = props.limits.maxComputeWorkGroupCount[0];
  atlas.max_storage_buffer_range_ = props.limits.maxStorageBufferRange;
  VR_ASSIGN(atlas.gpu_timer_, GpuTimer::create(device));

  // Fixed-size, so persist the SSBO and rewrite its contents each fuse, exactly
  // as the sibling texturer and the tsdf tier do.
  VR_ASSIGN(atlas.cam_buf_, storage_buffer(allocator, sizeof(DepthCameraParams),
                                           HostAccess::SequentialWrite));
  atlas.kernel_.set.write_storage_buffer(5, atlas.cam_buf_.handle(), 0,
                                         VK_WHOLE_SIZE);

  return atlas;
}

Status PatchAtlas::reserve(std::uint32_t triangles) {
  if (!valid()) {
    return Status::invalid_argument("PatchAtlas::reserve: moved-from");
  }
  if (triangles == 0) return {};
  return ensure_capacity(triangles);
}

void PatchAtlas::invalidate() noexcept {
  if (!patches_.valid()) return;
  // Zeroing sets every weight byte to 0, which is what makes the next
  // observation an assignment rather than a blend against black -- the same
  // property a never-touched texel has, reached the same way.
  std::memset(patches_.mapped(), 0, static_cast<std::size_t>(patches_.size()));
}

Status PatchAtlas::ensure_capacity(std::uint32_t triangles) {
  if (triangles <= triangle_capacity()) return {};

  const auto per_patch = static_cast<VkDeviceSize>(texels_per_patch());
  const VkDeviceSize bytes =
      static_cast<VkDeviceSize>(triangles) * per_patch * sizeof(std::uint32_t);
  if (bytes > max_storage_buffer_range_) {
    return Status::invalid_argument(
        "PatchAtlas::fuse: the atlas would exceed the device "
        "maxStorageBufferRange; lower PatchAtlasConfig::patch_leg or coarsen "
        "the voxel size");
  }
  VR_ASSIGN(Buffer grown,
            storage_buffer(*allocator_, bytes, HostAccess::Random));

  // Carry the existing patches forward and zero only the tail. A mesh that
  // outgrew the last one has not necessarily invalidated anything: an
  // incremental extract appends past the watermark and leaves every earlier
  // slot exactly where it was, so discarding them here would throw away the
  // accumulated colour of the whole scan on the frame it grew. The event that
  // DOES invalidate them is a full extract, and the caller answers that with
  // invalidate() -- see the class note.
  const auto old_bytes = static_cast<std::size_t>(patches_.size());
  auto* dst = static_cast<std::uint8_t*>(grown.mapped());
  if (old_bytes > 0) {
    std::memcpy(dst, patches_.mapped(), old_bytes);
  }
  std::memset(dst + old_bytes, 0, static_cast<std::size_t>(bytes) - old_bytes);
  patches_ = std::move(grown);
  return {};
}

Status PatchAtlas::fuse(const mesh::DeviceMesh& mesh, const float* depth,
                        const std::uint32_t* color, std::uint32_t color_width,
                        std::uint32_t color_height,
                        const DepthCameraParams& cam, float occlusion_threshold,
                        StageMetrics* metrics) {
  if (!valid()) {
    return Status::invalid_argument("PatchAtlas::fuse: moved-from");
  }
  GpuStageScope stage(metrics, gpu_timer_, "patch fuse");

  if (!mesh.valid()) {
    return Status::invalid_argument("PatchAtlas::fuse: mesh names no buffers");
  }
  // Checked rather than assumed, for the reason ProjectiveTexturer checks it:
  // binding a superseded view can be a use-after-free of a VkBuffer the
  // producer already destroyed.
  if (!mesh.is_current()) {
    return Status::invalid_argument(
        "PatchAtlas::fuse: the mesh has been superseded by a later extract");
  }
  if (mesh.empty()) {
    return {};  // nothing to texture; not an error
  }
  if (depth == nullptr) {
    return Status::invalid_argument("PatchAtlas::fuse: depth is null");
  }
  if (color == nullptr) {
    return Status::invalid_argument("PatchAtlas::fuse: color is null");
  }
  if (cam.width == 0 || cam.height == 0) {
    return Status::invalid_argument("PatchAtlas::fuse: camera extent is empty");
  }
  if (color_width == 0 || color_height == 0) {
    return Status::invalid_argument("PatchAtlas::fuse: color extent is empty");
  }

  VR_TRY(ensure_capacity(mesh.triangle_count));

  const auto depth_pixels = static_cast<std::size_t>(cam.width) *
                            static_cast<std::size_t>(cam.height);
  const VkDeviceSize depth_bytes =
      static_cast<VkDeviceSize>(depth_pixels) * sizeof(float);
  const auto color_pixels = static_cast<std::size_t>(color_width) *
                            static_cast<std::size_t>(color_height);
  const VkDeviceSize color_bytes =
      static_cast<VkDeviceSize>(color_pixels) * sizeof(std::uint32_t);
  if (depth_bytes > max_storage_buffer_range_ ||
      color_bytes > max_storage_buffer_range_) {
    return Status::invalid_argument(
        "PatchAtlas::fuse: a frame buffer exceeds the device "
        "maxStorageBufferRange");
  }

  // The two transfers this path makes. The geometry and the atlas never move.
  VR_ASSIGN(Buffer depth_buf,
            upload_storage_buffer(*allocator_, depth, depth_bytes));
  VR_ASSIGN(Buffer color_buf,
            upload_storage_buffer(*allocator_, color, color_bytes));
  std::memcpy(cam_buf_.mapped(), &cam, sizeof(DepthCameraParams));

  kernel_.set.write_storage_buffer(0, mesh.vertices, 0, VK_WHOLE_SIZE);
  kernel_.set.write_storage_buffer(1, mesh.indices, 0, VK_WHOLE_SIZE);
  kernel_.set.write_storage_buffer(2, patches_.handle(), 0, VK_WHOLE_SIZE);
  kernel_.set.write_storage_buffer(3, depth_buf.handle(), 0, VK_WHOLE_SIZE);
  kernel_.set.write_storage_buffer(4, color_buf.handle(), 0, VK_WHOLE_SIZE);

  // One thread per patch ROW, so the dispatch is triangles * patch_leg rather
  // than triangles * texels_per_patch -- eight times fewer at the default leg,
  // each doing eight times the work, with the triangle's vertices fetched once
  // for the row instead of once per texel.
  const std::uint64_t rows = static_cast<std::uint64_t>(mesh.triangle_count) *
                             static_cast<std::uint64_t>(config_.patch_leg);
  if (rows > 0xFFFFFFFFull) {
    return Status::invalid_argument(
        "PatchAtlas::fuse: the mesh needs more patch rows than a 32-bit "
        "dispatch index holds");
  }
  const PushConstants push{
      mesh.triangle_count, config_.patch_leg,      texels_per_patch(),
      color_width,         color_height,           occlusion_threshold,
      config_.max_weight,  config_.normal_epsilon, config_.depth_falloff,
      config_.near_depth,  config_.far_depth};
  return dispatch(*device_, kernel_, &push, sizeof(push),
                  group_count(static_cast<std::uint32_t>(rows), kLocalSize),
                  max_workgroup_count_x_, &stage);
}

}  // namespace volumetric_kit::recon::texture
