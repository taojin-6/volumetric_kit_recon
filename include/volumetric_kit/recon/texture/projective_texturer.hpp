// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file texture/projective_texturer.hpp
/// @brief Projective texturing: fill a mesh's per-vertex `uv0` with the image
///        coordinates of a posed camera, for the vertices that camera sees
///        unoccluded (the rest fall back to per-vertex color).

#include <cstdint>
#include <vector>

#include "volumetric_kit/recon/core/buffer.hpp"
#include "volumetric_kit/recon/core/camera_params.hpp"
#include "volumetric_kit/recon/core/compute_kernel.hpp"
#include "volumetric_kit/recon/core/descriptor.hpp"
#include "volumetric_kit/recon/core/fwd.hpp"
#include "volumetric_kit/recon/core/gpu_timer.hpp"
#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/core/stage_metrics.hpp"
#include "volumetric_kit/recon/mesh/device_mesh.hpp"
#include "volumetric_kit/recon/mesh/mesh.hpp"
#include "volumetric_kit/recon/texture/export.hpp"
#include "volumetric_kit/recon/texture/texture_atlas.hpp"

namespace volumetric_kit::recon::texture {

/// @brief Assigns a posed camera's image coordinates to the mesh vertices it
///        sees, so the reconstruction is textured with that camera's image
///        where it had a clear line of sight and falls back to per-vertex color
///        everywhere else.
///
/// One GLSL dispatch runs a thread per **vertex**. It projects the vertex into
/// the camera (rigid `world -> camera` from the camera's `cam_to_world`,
/// pinhole intrinsics) and calls it visible when it is in front of the camera,
/// inside the image, and **unoccluded** -- its projected camera-space depth
/// agreeing with the frame's depth map at that pixel within
/// `occlusion_threshold`. That depth test is the "line of sight" check: a
/// vertex hidden behind nearer geometry projects onto a pixel whose sensor
/// depth is much closer, so it fails and stays untextured.
///
/// **Per vertex, not per triangle**, and that is what makes this pass safe on a
/// mesh built with `mesh::MarchingCubesConfig::share_vertices`. Every input to
/// the verdict above is a property of the vertex alone, so deciding it per
/// triangle and writing the answer to three vertices was only ever a choice --
/// and on a shared mesh a wrong one, since a vertex belonging to several
/// triangles that disagreed was written by whichever thread ran last. One
/// thread per vertex means one writer per vertex. It is also less work: a
/// shared vertex used to be projected once per referencing triangle.
///
/// The cost is the all-three-vertices gate. A triangle straddling the
/// visibility boundary is no longer refused whole; the renderer resolves albedo
/// from its provoking vertex, so it takes one class for its whole face and the
/// textured region grows by up to one triangle at an occlusion silhouette.
///
/// **`uv0` therefore has three outcomes, not two:**
/// - **visible** -- `uv0 = (pixel + 0.5) / (width, height)`, a half-texel-
///   centred normalized coordinate into the camera's image, clamped half a
///   texel inside the border to stop bilinear bleed at the atlas edge. The
///   caller binds that image as the renderer's atlas.
/// - **in frame but occluded** -- `uv0 = -uv - 1`: still negative, so
///   `HybridMeshPipeline` selects @ref mesh::Vertex::color exactly as before,
///   but the coordinate is **carried** rather than discarded. gfx decodes it
///   instead of substituting `(0, 0)`. Without this a triangle mixing the two
///   classes interpolates from a real coordinate toward the atlas origin and
///   smears the corner of the image across its face.
/// - **behind the camera** -- the `(-1, -1)` sentinel. The pinhole divide has
///   no meaning there, so no coordinate exists to carry; it decodes to
///   `(0, 0)`, which is what gfx already substituted. A vertex **in front but
///   outside the image** is not this case: its projection is perfectly well
///   defined, just past the border, so it carries the nearest edge coordinate
///   under the rule above and the frustum boundary renders as edge stretching.
///   Conflating the two drew the entire camera image inside one triangle, the
///   length of that boundary.
///
/// A consumer must therefore test the **sign** (`uv0.x < 0`), which is what
/// gfx's shader does, and never compare against `(-1, -1)` exactly.
///
/// Because every call **overwrites** every vertex's `uv0`, a vertex that leaves
/// the view on a later frame reverts rather than keeping a stale coordinate.
///
/// This is the live, single-camera slice: the caller passes the current frame's
/// depth + camera (and binds its image as the atlas) each frame, so the
/// textured region tracks the camera through the scene. The projection and the
/// occlusion depth test both use @p cam, and the UV is normalized by @p cam's
/// dimensions, so the bound atlas must be **registered** to the depth camera.
/// A registered (or synthetic, e.g. Replica) RGB-D frame satisfies this. An
/// unregistered colour stream with its own intrinsics and pose -- a
/// `sensor::GpuFramePrep` frame -- takes the @ref TextureView overloads, whose
/// view names that camera (below).
///
/// **Registered does not mean the same resolution**, and the difference matters
/// for any sensor whose colour image is larger than its depth map. Because the
/// UV is *normalized*, a colour image registered to @p cam addresses correctly
/// at any size, exactly -- provided the depth intrinsics were derived from the
/// colour ones with a pixel-centre-preserving rescale, which is what
/// `sensor::depth_from_registered_color` does. With `cx_d = (cx_c + 0.5)·s -
/// 0.5` the projections satisfy `u_d + 0.5 = s·(u_c + 0.5)`, so
/// `(u_d + 0.5)/W_d` is identically `(u_c + 0.5)/W_c`. An ARKit frame --
/// 256x192 depth registered to a 1920x1440 capture -- is therefore textured at
/// full colour resolution with no rescale and no correction here.
///
/// **Several views** (the @ref TextureView overloads) texture from an atlas of
/// their images in rows (`texture_atlas.hpp`), choosing a view per
/// **triangle**: a triangle whose vertices took different views would
/// interpolate across the atlas between two tiles, which no per-vertex
/// encoding can prevent. So that path needs an unshared mesh, where each
/// triangle owns its three vertices, and refuses a shared one -- the one
/// refusal @ref mesh::DeviceMesh::shares_vertices means to this class; see
/// those overloads for how a view is chosen. Each tile is its view's colour
/// image at that image's own resolution, by the normalized-coordinate argument
/// above.
///
/// **A colour camera of its own.** Where the colour image was taken by a
/// camera other than the depth one -- the separate colour camera the TSDF
/// tier also models (@ref ColorCameraParams) -- the vertex is projected into
/// the colour camera for its coordinate, and the three outcomes above are that
/// camera's. It is textured only where the colour camera saw it too, which the
/// depth map answers with both cameras' intrinsics and poses:
/// - it lands inside the colour image, on a pixel the image recorded
///   (@ref TextureView::coverage);
/// - both cameras see the same side of the surface there, by its normal;
/// - the depth camera sees it unoccluded, as above;
/// - and the colour camera's line of sight is clear. That line, from the
///   vertex back to the colour camera, projects into the depth image along the
///   vertex's epipolar line, and the pass walks it, a pixel at a time and at
///   most 64 samples, for a surface the map puts more than
///   `occlusion_threshold` in front of it. That is what catches the sliver
///   beside an occluding edge, about the cameras' parallax wide, where the
///   depth camera sees past the edge and the colour camera sees the occluder.
///   A surface the depth camera itself cannot see (behind the occluder, from
///   its side) is not in the map and cannot be tested.
///
/// A registered image is the case where the two cameras are one, and every
/// test above then passes wherever the depth camera's does.
///
/// @warning The @ref Device and @ref Allocator passed to @ref create must
///          outlive this object; it stores references to them.
class VR_TEXTURE_API ProjectiveTexturer {
 public:
  /// @brief Build the view-selection pipeline + descriptors on @p device.
  /// @param device     The compute device (must outlive this object).
  /// @param allocator  The allocator its transient buffers come from (must
  ///                   outlive this).
  /// @return The texturer, or a non-OK @ref Status if a pipeline or descriptor
  ///         object fails to build.
  static Result<ProjectiveTexturer> create(Device& device,
                                           Allocator& allocator);

  // Rule of zero: every owned pipeline / layout / pool self-frees and self-
  // resets on move; device_ / allocator_ are borrowed, so the defaulted moves
  // leave a moved-from texturer empty (valid() == false).
  ~ProjectiveTexturer() = default;
  ProjectiveTexturer(ProjectiveTexturer&&) noexcept = default;
  ProjectiveTexturer& operator=(ProjectiveTexturer&&) noexcept = default;
  ProjectiveTexturer(const ProjectiveTexturer&) = delete;
  ProjectiveTexturer& operator=(const ProjectiveTexturer&) = delete;

  /// @brief Texture @p mesh with one posed frame, rewriting every vertex's
  ///        `uv0` in place.
  /// @param mesh      The mesh to texture; **every** @ref mesh::Vertex::uv0 is
  ///                  overwritten, with one of the three values the class note
  ///                  above describes -- so read the **sign**, never `== (-1,
  ///                  -1)`. Positions/normals/colors are unchanged, and
  ///                  @ref mesh::Mesh::indices is neither read nor validated:
  ///                  the dispatch is per vertex, so a mesh carrying vertices
  ///                  and no indices is textured rather than refused.
  /// @param depth     Row-major depth image in **metres**, length
  ///                  `cam.width * cam.height` (the host applies any raw sensor
  ///                  depth-scale first) -- the occlusion reference.
  /// @param cam       Intrinsics + camera->world pose + depth range; the
  ///                  projected image coordinates index the image the caller
  ///                  binds as the atlas, which must be **registered** to @p
  ///                  cam (same intrinsics + pose, not merely the same
  ///                  resolution -- see the class note).
  /// @param occlusion_threshold  Max allowed metres between a vertex's
  /// projected
  ///                  depth and the sampled sensor depth for it to count as
  ///                  visible (the ported default is 0.02 m). Also the depth
  ///                  discontinuity bound the bilinear sampler falls back to
  ///                  nearest across, so it does not blend foreground and
  ///                  background depth at a surface edge.
  /// @param metrics  Optional @ref StageMetrics collecting a `"texture"` host
  ///                  row and, from a timestamp span around the dispatch, its
  ///                  device half. `nullptr` measures nothing. See
  ///                  @ref tsdf::TsdfIntegrator::integrate for why the two
  ///                  halves are worth separating.
  /// @return OK on success (including a mesh with no vertices, a no-op), or a
  ///         non-OK @ref Status: @ref Status::Code::InvalidArgument if the
  ///         texturer is moved-from, @p depth is null, @p cam is empty, the
  ///         vertex count exceeds a single 1-D dispatch, or a vertex / depth
  ///         buffer would exceed the device `maxStorageBufferRange`; otherwise
  ///         a buffer or dispatch failure.
  Status texture(mesh::Mesh& mesh, const float* depth,
                 const DepthCameraParams& cam,
                 float occlusion_threshold = 0.02f,
                 StageMetrics* metrics = nullptr);

  /// @brief Texture a mesh that is already on the device, in place.
  ///
  /// Same pass, same kernel, same result as the host overload -- but the
  /// geometry never moves. That overload has to upload every vertex and index
  /// and then read the vertices back (~45 MB each way on a ~940 k-vertex room
  /// scan) purely to hand the kernel bytes the producing pass had already
  /// written to the device. Given a @ref mesh::DeviceMesh straight from
  /// @ref mesh::MarchingCubes::extract_device, only the depth frame is
  /// uploaded and nothing is read back; `uv0` is rewritten where it already
  /// lives, ready for the next device consumer.
  ///
  /// @param mesh   A device-resident mesh whose producer has not extracted
  ///               again. Checked rather than assumed -- see
  ///               @ref mesh::DeviceMesh::is_current, which this calls: binding
  ///               a superseded view can be a use-after-free of a `VkBuffer`
  ///               the producer already destroyed, so this pass refuses instead
  ///               of relying on the caller. An empty mesh is a no-op.
  /// @param depth  As the host overload.
  /// @param cam    As the host overload.
  /// @param occlusion_threshold  As the host overload.
  /// @param metrics  Optional @ref StageMetrics collecting a `"texture"` host
  ///                  row and, from a timestamp span around the dispatch, its
  ///                  device half. `nullptr` measures nothing. See
  ///                  @ref tsdf::TsdfIntegrator::integrate for why the two
  ///                  halves are worth separating.
  /// @return OK on success, or the same failures as the host overload except
  ///         those about host arrays; @ref Status::Code::InvalidArgument if
  ///         @p mesh names no buffers or has been superseded.
  Status texture(const mesh::DeviceMesh& mesh, const float* depth,
                 const DepthCameraParams& cam,
                 float occlusion_threshold = 0.02f,
                 StageMetrics* metrics = nullptr);

  /// @brief @ref texture for a depth frame already on the device, bound in
  ///        place, so the depth never visits the host.
  ///
  /// The atlas must still be registered to @p cam, as for the host overload.
  /// A `sensor::GpuFramePrep` frame's is not -- its colour keeps a camera of
  /// its own -- so it takes the @ref TextureView overload below.
  /// @param depth  A storage buffer holding at least `cam.width * cam.height`
  ///               floats, row-major, in metres. The writer's dispatch must
  ///               have finished, which a dispatch on this device guarantees.
  /// @return As the host-depth overload; @ref Status::Code::InvalidArgument
  ///         also for a @p depth that is empty, not a storage buffer, or
  ///         smaller than the image.
  Status texture(const mesh::DeviceMesh& mesh, const Buffer& depth,
                 const DepthCameraParams& cam,
                 float occlusion_threshold = 0.02f,
                 StageMetrics* metrics = nullptr);

  /// @brief @ref texture from one @ref TextureView: its depth on the host or
  ///        the device, and its colour image registered to the depth camera
  ///        or taken by a colour camera of its own, whose coverage it may
  ///        mark. A `sensor::GpuFramePrep` frame textures without visiting
  ///        the host.
  ///
  /// With a @ref TextureView::color_camera, the coordinate is that camera's,
  /// under the class note's tests: a vertex behind it gets the `(-1, -1)`
  /// sentinel, and one in front gets its pixel centre in the colour image,
  /// normalized and clamped half a texel inside, carried (`-uv - 1`) unless
  /// the colour camera saw it. The atlas the caller binds is that camera's
  /// image. The view's `image_width` x `image_height` is the image's size,
  /// which matters only to @ref TextureView::coverage.
  /// @code
  /// const sensor::DeviceFrame& f = ...;  // from GpuFramePrep::prepare
  /// texture::TextureView view;
  /// view.cam = f.depth_camera;
  /// view.depth_buffer = f.depth;
  /// view.color_camera = f.color_camera;
  /// view.coverage = f.color;
  /// VR_TRY(texturer.texture(mesh, view));  // uv0 now addresses f.color
  /// @endcode
  /// @param mesh    As the other device overloads.
  /// @param view    The frame. A device depth or coverage is bound in place,
  ///                so each must be a storage buffer.
  /// @param occlusion_threshold  As the host overload; also how far in front
  ///                of the colour camera's line of sight a surface must be to
  ///                block it.
  /// @param metrics  As the host overload.
  /// @return As the overload above; @ref Status::Code::InvalidArgument also
  ///         for a view with no depth or with both a host and a device one, a
  ///         colour camera with no image, an image size that is one-sided or
  ///         not the colour camera's, or a coverage that is not a storage
  ///         buffer or is smaller than the image.
  Status texture(const mesh::DeviceMesh& mesh, const TextureView& view,
                 float occlusion_threshold = 0.02f,
                 StageMetrics* metrics = nullptr);

  /// @brief As the @ref TextureView overload above, for a host mesh, uploaded
  ///        and read back as the first host overload is.
  /// @return As that overload and the first host overload.
  Status texture(mesh::Mesh& mesh, const TextureView& view,
                 float occlusion_threshold = 0.02f,
                 StageMetrics* metrics = nullptr);

  /// @brief Texture @p mesh from several posed views, each triangle from the
  ///        one that sees it best, into an atlas of their images.
  ///
  /// One thread per triangle. A view qualifies when it sees the triangle's
  /// **front** -- its counter-clockwise side, the side marching cubes' outward
  /// normal leaves by and the one gfx draws -- and all three of its vertices
  /// are in front of it, inside its image and unoccluded (the same test as the
  /// single-camera pass). Among those, the one facing the triangle most
  /// squarely wins: the score is the cosine of the angle between the
  /// triangle's normal and the view's ray back from its centroid, less 0.01
  /// per metre of the smallest depth disagreement, as in the prior engine; the
  /// first view wins a tie, and a @ref TextureView::fallback view wins only
  /// where no other qualifies. A view behind the triangle does not qualify,
  /// however well its depth agrees: on thin geometry the back of a sheet sits
  /// within the threshold of the front the camera saw. All three vertices get
  /// the winner's image coordinates, scaled into its tile of @p layout and
  /// clamped half a texel inside it, so filtering never reaches a neighbouring
  /// view. A triangle no view sees whole from the front gets `(-1, -1)` on all
  /// three, the vertex colour, and so does one with no area. Every vertex's
  /// `uv0` is rewritten.
  ///
  /// A view with a @ref TextureView::color_camera qualifies only if that
  /// camera also sees the triangle's front, records all three vertices inside
  /// its image, on pixels its @ref TextureView::coverage marks, and has a
  /// clear line of sight to each (the class note's colour-camera tests), and
  /// the coordinates are its pixels. The score stays the depth camera's. A
  /// view's depth and coverage may be on the device, each copied device to
  /// device into the pass's buffer beside the others', so a rig of
  /// `sensor::GpuFramePrep` frames textures without visiting the host.
  ///
  /// @param mesh    An unshared mesh from the producer that has not extracted
  ///                again (`DeviceMesh::is_current`): vertices `3t..3t+2` are
  ///                triangle `t`'s. A `shares_vertices` mesh is refused.
  /// @param views   The views: each a depth map on the host or the device,
  ///                its camera, and its colour image's size and, when it is
  ///                not registered, camera (@ref TextureView).
  /// @param layout  Where each view's image sits (@ref side_by_side_atlas):
  ///                one tile per view, each its colour image's size, none
  ///                overlapping another.
  /// @param occlusion_threshold  As the single-camera overload.
  /// @param metrics  Optional; a `"texture"` row, as the single-camera
  ///                 overload.
  /// @return OK (an empty mesh is a no-op); @ref Status::Code::InvalidArgument
  ///         for a moved-from texturer, no views, a view with no depth or with
  ///         both a host and a device one, a device depth or coverage that is
  ///         smaller than its image or cannot be copied from, an empty camera
  ///         or image, a depth range with `min_depth >= max_depth` (under
  ///         which no sample would count), a layout that does not match the
  ///         views, overlaps itself or lies past @ref max_atlas_extent, a
  ///         shared, superseded or buffer-less mesh, or depth or coverage too
  ///         large for one binding; else a dispatch failure.
  Status texture(const mesh::DeviceMesh& mesh,
                 const std::vector<TextureView>& views,
                 const AtlasLayout& layout, float occlusion_threshold = 0.02f,
                 StageMetrics* metrics = nullptr);

  /// @brief As the device overload, for a host mesh, uploaded and read back.
  /// @param mesh  Unshared: `indices` must be `0, 1, 2, ...`, one per vertex.
  /// @return As the device overload; a mesh whose indices are not that run is
  ///         InvalidArgument.
  Status texture(mesh::Mesh& mesh, const std::vector<TextureView>& views,
                 const AtlasLayout& layout, float occlusion_threshold = 0.02f,
                 StageMetrics* metrics = nullptr);

  /// @return The largest atlas width or height this device samples
  ///         (`maxImageDimension2D`), to lay an atlas out within; 0 for a
  ///         moved-from texturer, which has no device.
  std::uint32_t max_atlas_extent() const noexcept {
    return valid() ? max_atlas_extent_ : 0;
  }

  /// @return `true` if this owns a live pipeline (`false` when moved-from).
  bool valid() const noexcept { return kernel_.valid(); }

 private:
  ProjectiveTexturer() = default;

  // Borrowed (must outlive this).
  Device* device_ = nullptr;
  Allocator* allocator_ = nullptr;

  // Cached maxComputeWorkGroupCount[0] -- the device cap on a 1-D dispatch's
  // groupCountX; texture() rejects a vertex count that would exceed it.
  std::uint32_t max_workgroup_count_x_ = 0;
  // Cached maxStorageBufferRange -- the device cap on a single storage-buffer
  // binding; texture() rejects a vertex or depth buffer larger than it. Not the
  // index buffer: the per-vertex dispatch does not bind one.
  std::uint32_t max_storage_buffer_range_ = 0;
  // Cached maxImageDimension2D, the largest atlas side the renderer can take.
  std::uint32_t max_atlas_extent_ = 0;

  // The view-selection kernel's bundled layout + pipeline + descriptor set, its
  // set allocated from pool_ (which must outlive it) by KernelSetBuilder.
  ComputeKernel kernel_;
  // The several-view kernel (texture_multiview.comp), from the same pool.
  ComputeKernel multiview_kernel_;
  // Device spans for the texturing dispatch; idle until a caller asks. See
  // tsdf::TsdfIntegrator's member of the same name.
  GpuTimer gpu_timer_;
  DescriptorPool pool_;
  // Fixed-size camera-params SSBO (the depth camera, then the colour camera
  // the single-camera kernel reads when it has one): bound once at create()
  // and rewritten inline in each texture()'s batch, like the tsdf tier's
  // camera SSBO. Device-local, as every buffer here is.
  Buffer cam_buf_;
  // A host depth frame's device copy for the single-camera pass. Grow-only,
  // like the views' buffers below, so a live pass texturing every remesh
  // allocates only its staging once the frame fits.
  Buffer depth_buf_;
  // The several-view pass's inputs: every view's depth end to end, every
  // marked image's coverage end to end, and the views. Grow-only and
  // rewritten each call, like cam_buf_, so a rig texturing every frame
  // allocates nothing once they fit.
  //
  // TODO(texture): keep a static keyframe set's depth and coverage here
  // between calls; every call stages or copies every view's afresh.
  Buffer view_depth_buf_;
  Buffer view_coverage_buf_;
  Buffer views_buf_;

  // Every single-camera DeviceMesh overload: `view` gives the cameras, the
  // image and its coverage, and `depth` the depth, the host array or device
  // buffer the caller passed (the view's own depth fields are not read).
  Status texture(const mesh::DeviceMesh& mesh, const TextureView& view,
                 const StorageInput& depth, float occlusion_threshold,
                 StageMetrics* metrics);
  // Both single-camera host-mesh overloads, as the one above.
  Status texture(mesh::Mesh& mesh, const TextureView& view,
                 const StorageInput& depth, float occlusion_threshold,
                 StageMetrics* metrics);
  // What every single-camera overload checks before anything is recorded.
  Status check_view(const TextureView& view, const StorageInput& depth) const;
  // Every single-camera overload, once the vertices are on the device: records
  // the depth, the cameras and the dispatch into `batch`, binding
  // `vertex_range` bytes of `vertices`. It may replace depth_buf_, as
  // texture_views may its buffers.
  Status texture_vertices(CommandBatch& batch, VkBuffer vertices,
                          VkDeviceSize vertex_range, std::uint32_t vertex_count,
                          const TextureView& view, const StorageInput& depth,
                          float occlusion_threshold, GpuStageScope* stage);
  // Both multi-view overloads, once the vertices are on the device: records
  // the views and the dispatch into `batch`.
  //
  // It may replace view_depth_buf_, view_coverage_buf_ and views_buf_, so
  // nothing already in `batch` may refer to them: the callers record only the
  // vertices first.
  Status texture_views(CommandBatch& batch, VkBuffer vertices,
                       std::uint32_t triangles,
                       const std::vector<TextureView>& views,
                       const AtlasLayout& layout, float occlusion_threshold,
                       GpuStageScope* stage);
  Status check_views(const std::vector<TextureView>& views,
                     const AtlasLayout& layout) const;
};

}  // namespace volumetric_kit::recon::texture
