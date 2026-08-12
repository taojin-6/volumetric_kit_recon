// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file texture/patch_atlas.hpp
/// @brief A progressive texture atlas of per-triangle patches, keyed by the
///        mesh arena's triangle slot and fused from posed colour frames.

#include <cstdint>

#include "volumetric_kit/recon/core/buffer.hpp"
#include "volumetric_kit/recon/core/camera_params.hpp"
#include "volumetric_kit/recon/core/compute_kernel.hpp"
#include "volumetric_kit/recon/core/descriptor.hpp"
#include "volumetric_kit/recon/core/gpu_timer.hpp"
#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/core/stage_metrics.hpp"
#include "volumetric_kit/recon/mesh/device_mesh.hpp"
#include "volumetric_kit/recon/texture/export.hpp"

namespace volumetric_kit::recon {
class Device;
class Allocator;
}  // namespace volumetric_kit::recon

namespace volumetric_kit::recon::texture {

/// @brief Texels one patch holds, for a patch whose legs are @p leg texels.
///
/// A patch is a **right triangle**, not a square: rows `j = 0 .. leg-1`, with
/// row `j` holding `leg - j` texels. So it is the triangular number of @p leg,
/// and a square patch of the same edge would waste just under half its area --
/// which at room scale is the difference between a ~400 MB atlas and a ~800 MB
/// one. Mirrored by `vrPatchTexelCount` in `shaders/texture_patch.glsl`.
constexpr std::uint32_t patch_texel_count(std::uint32_t leg) noexcept {
  return leg * (leg + 1) / 2;
}

/// @brief Where texel `(i, j)` of a patch lands inside that patch.
///
/// Row-major over the triangle: row `j` starts at `j*leg - j*(j-1)/2` and runs
/// for `leg - j` texels, so `i + j` must be less than @p leg. A bijection onto
/// `[0, patch_texel_count(leg))`, which is what makes a patch addressable
/// without a lookup table on either side. Mirrored by `vrPatchTexelIndex` in
/// `shaders/texture_patch.glsl`.
constexpr std::uint32_t patch_texel_index(std::uint32_t leg, std::uint32_t i,
                                          std::uint32_t j) noexcept {
  return j * leg - j * (j - 1) / 2 + i;
}

/// @brief Fixed tuning for a @ref PatchAtlas, read at @ref PatchAtlas::create.
struct PatchAtlasConfig {
  /// @brief Texels along each leg of a patch.
  ///
  /// This is the whole memory/detail dial, and it is quadratic: the atlas costs
  /// `patch_texel_count(patch_leg) * 4` bytes per triangle slot, so 8 costs
  /// 144 B and 16 costs 544 B.
  ///
  /// Size it against the **working distance**, not against the camera's
  /// minimum. A voxel spans `fx * voxel_size / d` pixels at distance `d`, so a
  /// 1 cm voxel through a 1920-px ARKit frame (`fx` ~1500) covers ~7.5 px at
  /// 2 m and ~43 px at the 0.35 m the sensor can focus to. Sizing for the
  /// latter -- which is what the TextureMe paper's rule does, on a 24 GB
  /// desktop GPU -- needs ~11.8 GB at the 3.2 M triangles a 1 cm room scan
  /// reaches. 8 is the 2 m answer and the default.
  ///
  /// Note what does *not* change with voxel size: coarsening the voxel gives
  /// four times fewer triangles carrying four times the texels each, so the
  /// atlas costs a fixed amount per square metre of surface either way. Spend
  /// the budget here rather than on geometry.
  ///
  /// Must be at least 2 -- a single-texel patch has no barycentric span to
  /// interpolate across -- and is rejected by @ref PatchAtlas::create below it.
  std::uint32_t patch_leg = 8;

  /// @brief Accumulated observation weight a texel saturates at.
  ///
  /// The running average's cap, exactly as `tsdf::TsdfIntegratorConfig`'s is:
  /// past it a texel stops moving toward new observations, so a scan that
  /// lingers cannot wash out what it already resolved. The paper's value.
  ///
  /// It also sets the quantisation, because the stored weight is one byte
  /// holding `w / max_weight` -- so this is both the ceiling and the scale.
  float max_weight = 5.0f;

  /// @brief Floor on the `n . v` term, so a noisy normal cannot zero a texel's
  ///        weight outright and leave it unfilled forever.
  float normal_epsilon = 1e-4f;

  /// @brief How sharply an observation's weight falls off with range.
  ///
  /// `w_d = exp(-depth_falloff * dn^2)`, with `dn` the projected depth
  /// normalized into [@ref near_depth, @ref far_depth]. Depth noise and
  /// projective error both grow with range, so a far observation should not
  /// outvote a near one that saw the same surface.
  float depth_falloff = 3.0f;
  /// Depth at which @ref depth_falloff's normalized range starts (metres).
  float near_depth = 0.35f;
  /// Depth at which it ends; past this an observation carries the floor weight.
  float far_depth = 3.4f;
};

/// @brief A texture patch per triangle of a mesh arena, accumulated across
///        frames and addressed by the arena's own triangle slot.
///
/// The idea is TextureMe's (Kim et al., ACM TOG 41(3), 2022): give every
/// marching-cubes triangle a small fixed patch in a preallocated atlas and fuse
/// each frame's colour into it, so surface colour is resolved at the *camera's*
/// resolution rather than the voxel's -- and with no UV unwrapping, which no
/// incrementally-updated mesh could keep stable anyway.
///
/// **This atlas has no allocator, and that is the point.** The paper maintains
/// a GPU free list of patch indices, pushed and popped as voxels gain and lose
/// triangles. Here a patch *is* the arena triangle slot: `mesh::MarchingCubes`
/// already hands each block one contiguous range, reuses it in place when the
/// re-meshed count fits, and retires what it abandons. Since the 2026-08-12
/// scanned-offset decision a cell's triangles also land at a fixed offset
/// inside that range, so `slot = f(block base, cell, triangle-in-cell)` holds
/// across extracts -- which is exactly the durable name a patch needs. There is
/// no second lifetime to reason about: the atlas inherits the arena's.
///
/// **Linear, not a 2D image.** Patch `t` occupies
/// `[t * texels_per_patch, (t+1) * texels_per_patch)` of one storage buffer,
/// and a texel is one `uint32` of canonical-encoded R'G'B' plus a weight byte.
/// So there are no atlas dimensions, no packing grid, no `VK_MAX_IMAGE`
/// ceiling, and no gutter -- adjacent patches cannot bleed into each other
/// because nothing samples across a patch boundary. It also grows exactly as
/// the vertex arena does. What it gives up is hardware filtering and mips: a
/// consumer samples the buffer itself and does its own interpolation.
///
/// **Colour is blended in linear and stored encoded**, per the 2026-08-02
/// colour rule: an 8-bit value is encoded, and averaging is a linear operation,
/// so each observation is decoded through the exact piecewise sRGB curve,
/// mixed, and re-encoded. Storing 8 bits costs precision the running average
/// cannot recover; that is the price of 4 bytes a texel, and it is what makes
/// a room-scale atlas fit at all.
///
/// **Nothing here decides when a patch is stale.** A patch keyed by arena slot
/// is meaningful only while that slot still holds the triangle it was fused
/// against, and the one event that breaks every slot at once is a full extract
/// -- an arena grow, an overflow refit, a topology change. The caller sees that
/// as `mesh::ExtractTimings::incremental` coming back `false`, and answers it
/// with @ref invalidate. A finer answer (per block, or per triangle) is a later
/// slice; on Replica room0 only 2.2% of re-meshed blocks change triangulation
/// at all, so the coarse one costs little.
///
/// @warning The @ref Device and @ref Allocator passed to @ref create must
///          outlive this object; it stores references to them.
//
// TODO(texture): the observation weight has no blur term. TextureMe's `w_b`
// scores each frame with a perceptual blur metric and down-weights a smeared
// one, which matters most on a handheld sweep. A capture that already reports
// camera motion (ARKit does) can gate on that far more cheaply; neither is
// built.
// TODO(texture): sampling is nearest in the colour image. Bilinear would cost
// four fetches and a lerp and is probably worth it, but it wants a measurement
// against a real scan rather than a guess.
class VR_TEXTURE_API PatchAtlas {
 public:
  /// @brief Build the fusion pipeline + descriptors on @p device.
  /// @param device     The compute device (must outlive this object).
  /// @param allocator  The allocator its buffers come from (must outlive this).
  /// @param config     Patch size and weighting; see @ref PatchAtlasConfig.
  /// @return The atlas, or a non-OK @ref Status if @p config asks for a
  ///         @ref PatchAtlasConfig::patch_leg below 2, or a pipeline or
  ///         descriptor object fails to build.
  static Result<PatchAtlas> create(Device& device, Allocator& allocator,
                                   const PatchAtlasConfig& config = {});

  // Rule of zero: every owned Buffer / pipeline / pool self-frees and
  // self-resets on move; device_ / allocator_ are borrowed, so the defaulted
  // moves leave a moved-from atlas empty (valid() == false).
  ~PatchAtlas() = default;
  PatchAtlas(PatchAtlas&&) noexcept = default;
  PatchAtlas& operator=(PatchAtlas&&) noexcept = default;
  PatchAtlas(const PatchAtlas&) = delete;
  PatchAtlas& operator=(const PatchAtlas&) = delete;

  /// @brief Fuse one posed colour frame into the patches of @p mesh.
  ///
  /// One thread per patch **row**, so each texel has exactly one writer and
  /// nothing is atomic. Per texel: barycentrics give a world position on the
  /// triangle, that projects into @p cam, and the observation is kept when the
  /// point is in front, in frame, and **unoccluded** -- its projected depth
  /// agreeing with @p depth within @p occlusion_threshold, the same test
  /// @ref ProjectiveTexturer applies per vertex. A kept observation is weighted
  /// by `n . v` and by range (see @ref PatchAtlasConfig) and averaged into the
  /// patch; a rejected one leaves the texel exactly as it was.
  ///
  /// Grows the atlas to fit @p mesh if it has to, zeroing only what it adds --
  /// so a mesh that outgrew the last one keeps every patch it already had.
  ///
  /// @param mesh   A device-resident mesh whose producer has not extracted
  ///               again; checked through @ref mesh::DeviceMesh::is_current,
  ///               since binding a superseded view can be a use-after-free. An
  ///               empty mesh is a no-op.
  /// @param depth  Row-major depth in **metres**, `cam.width * cam.height`
  ///               long -- the occlusion reference.
  /// @param color  Row-major packed RGB, one `uint32` per pixel with R in the
  ///               low byte, in the canonical **encoded** 8-bit form
  ///               (`core/color_space.hpp`). Must be **registered** to @p cam:
  ///               the projection is normalized by @p cam's dimensions and
  ///               rescaled to these, so a different resolution is fine and a
  ///               different viewpoint is not.
  /// @param color_width   Pixels per row of @p color.
  /// @param color_height  Rows of @p color.
  /// @param cam    Intrinsics + camera->world pose + depth range.
  /// @param occlusion_threshold  Max metres between a texel's projected depth
  ///               and the sampled sensor depth for the observation to count.
  ///               Also the discontinuity bound the bilinear depth sampler
  ///               falls back to nearest across.
  /// @param metrics  Optional @ref StageMetrics collecting a `"patch fuse"`
  ///               host row and, from a timestamp span around the dispatch, its
  ///               device half. `nullptr` measures nothing.
  /// @return OK on success, or a non-OK @ref Status:
  ///         @ref Status::Code::InvalidArgument if this is moved-from, @p mesh
  ///         names no buffers or has been superseded, @p depth or @p color is
  ///         null, @p cam or the colour extent is empty, or the dispatch or a
  ///         buffer would exceed a device limit; otherwise a buffer or dispatch
  ///         failure.
  Status fuse(const mesh::DeviceMesh& mesh, const float* depth,
              const std::uint32_t* color, std::uint32_t color_width,
              std::uint32_t color_height, const DepthCameraParams& cam,
              float occlusion_threshold = 0.02f,
              StageMetrics* metrics = nullptr);

  /// @brief Grow the atlas to hold at least @p triangles of patches, now, so
  ///        no later @ref fuse has to.
  ///
  /// @ref fuse grows the buffer to fit whatever mesh it is given, carrying the
  /// existing patches forward -- which is correct, and still not enough for a
  /// consumer drawing the atlas while it accumulates. Growing **reallocates**,
  /// and the old buffer is freed with no fence wait, so a frame still reading
  /// it gets a use-after-free rather than a stale image. On a scan whose mesh
  /// is growing that is not a corner case; it is most frames.
  ///
  /// Reserving to the scan's expected ceiling makes the handle stable for the
  /// run, which is what lets a renderer bind it once per slot instead of
  /// re-deriving it every frame under a lock. Pair it with
  /// `mesh::MarchingCubes::reserve` on the same triangle count: the two
  /// buffers are indexed by the same slot, and reserving one without the other
  /// leaves the pair's weaker half deciding when a realloc happens.
  ///
  /// Exceeding it later is not an error -- the atlas grows as it always did.
  /// So this makes the event rare rather than impossible.
  ///
  /// @param triangles Triangle slots to hold patches for; 0 reserves nothing.
  /// @return OK, or @ref Status::Code::InvalidArgument on a moved-from atlas or
  ///         a reservation past the device `maxStorageBufferRange`; a backend
  ///         error if the allocation fails.
  Status reserve(std::uint32_t triangles);

  /// @brief Discard every patch, so the next @ref fuse starts from nothing.
  ///
  /// Call it when the arena's triangle slots stop meaning what they meant --
  /// which the caller sees as `mesh::ExtractTimings::incremental` reading
  /// `false`, since a full extract re-reserves every block's range through a
  /// global atomic and no slot survives it.
  ///
  /// Cheap and total: it zeroes the buffer, which sets every weight to zero, so
  /// the first observation after it assigns rather than blending into whatever
  /// the slot used to hold. It does not shrink or free -- the atlas is
  /// grow-only, like the arena it shadows.
  void invalidate() noexcept;

  /// @return The patch buffer, for a consumer that samples it. Null on a
  ///         moved-from atlas or before the first @ref fuse sized it.
  VkBuffer buffer() const noexcept { return patches_.handle(); }
  /// @return Texels along each leg of a patch (@ref PatchAtlasConfig).
  std::uint32_t patch_leg() const noexcept { return config_.patch_leg; }
  /// @return Texels one patch holds -- `patch_texel_count(patch_leg())`, and
  ///         the stride a consumer multiplies a triangle slot by.
  std::uint32_t texels_per_patch() const noexcept {
    return patch_texel_count(config_.patch_leg);
  }
  /// @return Triangle slots the atlas currently holds patches for. Derived from
  ///         the buffer rather than tracked beside it, so the two cannot
  ///         disagree -- including on a moved-from atlas, where both are zero.
  std::uint32_t triangle_capacity() const noexcept {
    const std::uint32_t per = texels_per_patch();
    return per == 0 ? 0
                    : static_cast<std::uint32_t>(patches_.size() /
                                                 (per * sizeof(std::uint32_t)));
  }
  /// @brief The patches as the host sees them, or `nullptr` before the first
  ///        @ref fuse sized the buffer.
  ///
  /// `triangle_capacity() * texels_per_patch()` entries, patch `t` occupying
  /// `[t * texels_per_patch(), (t+1) * texels_per_patch())` and each texel one
  /// `uint32` of canonical-encoded R'G'B' in the low three bytes with the
  /// accumulated weight -- scaled by @ref PatchAtlasConfig::max_weight -- in
  /// the high one. A texel no observation has reached is **entirely** zero, not
  /// merely zero-weighted, so a consumer that ignores the weight still reads
  /// black rather than whatever the allocator last held.
  ///
  /// Host-visible because every buffer this repo allocates is, so this is a
  /// mapped pointer and not a transfer -- but it is the atlas the *kernel*
  /// writes, so reading it while a @ref fuse is in flight on another thread is
  /// a data race like any other. Borrowed, and invalidated by the @ref fuse
  /// that next grows the buffer.
  const std::uint32_t* mapped() const noexcept {
    return static_cast<const std::uint32_t*>(patches_.mapped());
  }

  /// @return Bytes the atlas holds. Resident size, not this call's allocation:
  ///         it is grow-only, so a steady-state @ref fuse allocates nothing and
  ///         still reports them.
  std::uint64_t bytes() const noexcept { return patches_.size(); }

  /// @return `true` if this owns a live pipeline (`false` when moved-from).
  bool valid() const noexcept { return kernel_.valid(); }

 private:
  PatchAtlas() = default;

  // Grow patches_ to hold `triangles` slots, zeroing only the tail it adds.
  Status ensure_capacity(std::uint32_t triangles);

  // Borrowed (must outlive this).
  Device* device_ = nullptr;
  Allocator* allocator_ = nullptr;

  PatchAtlasConfig config_{};

  // Cached device limits the dispatch and the bindings are checked against.
  std::uint32_t max_workgroup_count_x_ = 0;
  std::uint32_t max_storage_buffer_range_ = 0;

  ComputeKernel kernel_;
  GpuTimer gpu_timer_;
  DescriptorPool pool_;
  // Fixed-size camera-params SSBO, rewritten each fuse, like the sibling
  // texturer's.
  Buffer cam_buf_;
  // The patches themselves: `triangle_capacity() * texels_per_patch()` uint32,
  // grow-only.
  Buffer patches_;
};

}  // namespace volumetric_kit::recon::texture
