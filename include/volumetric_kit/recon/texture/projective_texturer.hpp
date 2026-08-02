// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file texture/projective_texturer.hpp
/// @brief Projective texturing: fill a mesh's per-vertex `uv0` with the image
///        coordinates of a posed camera, for the triangles that camera sees
///        unoccluded (the rest fall back to per-vertex color).

#include <cstdint>

#include "volumetric_kit/recon/core/buffer.hpp"
#include "volumetric_kit/recon/core/compute_kernel.hpp"
#include "volumetric_kit/recon/core/descriptor.hpp"
#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/mesh/device_mesh.hpp"
#include "volumetric_kit/recon/mesh/mesh.hpp"
#include "volumetric_kit/recon/texture/export.hpp"
#include "volumetric_kit/recon/volume/voxel_hash_map.hpp"  // DepthCameraParams

namespace volumetric_kit::recon {
class Device;
class Allocator;
}  // namespace volumetric_kit::recon

namespace volumetric_kit::recon::texture {

/// @brief Assigns a posed camera's image coordinates to the mesh triangles it
///        sees, so the reconstruction is textured with that camera's image
///        where it had a clear line of sight and falls back to per-vertex color
///        everywhere else.
///
/// One GLSL dispatch runs a thread per triangle. It projects the triangle's
/// three world-space vertices into the camera (rigid `world -> camera` from the
/// camera's `cam_to_world`, pinhole intrinsics) and keeps the triangle only
/// when
/// **all three** vertices are: in front of the camera, inside the image, and
/// **unoccluded** -- their projected camera-space depth agrees with the frame's
/// depth map at that pixel within `occlusion_threshold`. That depth test is the
/// "line of sight" check: a triangle hidden behind nearer geometry projects
/// onto a pixel whose sensor depth is much closer, so it fails and stays
/// untextured.
///
/// A kept triangle's vertices receive `uv0 = (pixel + 0.5) / (width, height)`
/// -- a half-texel-centred normalized coordinate into the camera's image,
/// clamped half a texel inside the border to stop bilinear bleed at the atlas
/// edge -- which the caller binds as the renderer's atlas (`HybridMeshPipeline`
/// samples the atlas where `uv0` is valid). A rejected triangle's vertices are
/// written the `(-1, -1)` sentinel, so the shader falls back to the
/// @ref mesh::Vertex::color the TSDF tier fused -- and, because every call
/// **overwrites** `uv0`, a triangle that leaves the view on a later frame
/// reverts to that fallback rather than keeping a stale coordinate.
///
/// This is the live, single-camera slice: the caller passes the current frame's
/// depth + camera (and binds its image as the atlas) each frame, so the
/// textured region tracks the camera through the scene. The projection and the
/// occlusion depth test both use @p cam, and the UV is normalized by @p cam's
/// dimensions, so the bound atlas must be **registered** to the depth camera --
/// same intrinsics and pose, not merely the same resolution. A registered
/// (or synthetic, e.g. Replica) RGB-D frame satisfies this; an unregistered
/// colour stream with its own intrinsics would misaddress the atlas. The
/// separate-colour-camera path the TSDF tier models (`ColorCameraParams`) is a
/// later slice. Multi-keyframe selection (best of several views, a packed
/// atlas) is likewise a later slice; the winner-take-all vertex arbitration it
/// needs is unnecessary here because the mesh tier emits independent triangles
/// (no shared vertices), so each thread owns its three `uv0` writes outright.
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
  /// @param mesh      The mesh to texture; its @ref mesh::Vertex::uv0 fields
  /// are
  ///                  overwritten -- a normalized image coordinate for a
  ///                  triangle the camera sees unoccluded, the `(-1, -1)`
  ///                  sentinel otherwise. Positions/normals/colors are
  ///                  unchanged.
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
  /// @return OK on success (including an empty mesh, a no-op), or a non-OK
  ///         @ref Status: @ref Status::Code::InvalidArgument if the texturer is
  ///         moved-from, @p depth is null, @p cam is empty, the mesh's index
  ///         count is not a multiple of three, the triangle count exceeds a
  ///         single 1-D dispatch, or a vertex / index / depth buffer would
  ///         exceed the device `maxStorageBufferRange`; otherwise a buffer or
  ///         dispatch failure. An out-of-range triangle index is skipped
  ///         on-device (a malformed-mesh guard), not reported.
  Status texture(mesh::Mesh& mesh, const float* depth,
                 const volume::DepthCameraParams& cam,
                 float occlusion_threshold = 0.02f);

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
  /// @param mesh   A device-resident mesh, still valid (its producer has not
  ///               extracted again). An empty mesh is a no-op.
  /// @param depth  As the host overload.
  /// @param cam    As the host overload.
  /// @param occlusion_threshold  As the host overload.
  /// @return OK on success, or the same failures as the host overload except
  ///         those about host arrays; @ref Status::Code::InvalidArgument if
  ///         @p mesh names no buffers.
  Status texture(const mesh::DeviceMesh& mesh, const float* depth,
                 const volume::DepthCameraParams& cam,
                 float occlusion_threshold = 0.02f);

  /// @return `true` if this owns a live pipeline (`false` when moved-from).
  bool valid() const noexcept { return kernel_.valid(); }

 private:
  ProjectiveTexturer() = default;

  // Borrowed (must outlive this).
  Device* device_ = nullptr;
  Allocator* allocator_ = nullptr;

  // Cached maxComputeWorkGroupCount[0] -- the device cap on a 1-D dispatch's
  // groupCountX; texture() rejects a triangle count that would exceed it.
  std::uint32_t max_workgroup_count_x_ = 0;
  // Cached maxStorageBufferRange -- the device cap on a single storage-buffer
  // binding; texture() rejects a vertex / index / depth buffer larger than it.
  std::uint32_t max_storage_buffer_range_ = 0;

  // The view-selection kernel's bundled layout + pipeline + descriptor set, its
  // set allocated from pool_ (which must outlive it) by KernelSetBuilder.
  ComputeKernel kernel_;
  DescriptorPool pool_;
  // Fixed-size camera-params SSBO (volume::DepthCameraParams): bound once at
  // create() and rewritten each texture(), like the tsdf tier's camera SSBO.
  Buffer cam_buf_;
};

}  // namespace volumetric_kit::recon::texture
