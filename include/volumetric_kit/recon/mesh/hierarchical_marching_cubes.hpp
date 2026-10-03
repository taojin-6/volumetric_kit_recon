// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file mesh/hierarchical_marching_cubes.hpp
/// @brief Device extraction of a cell-centered hierarchical TSDF.

#include <memory>

#include "volumetric_kit/recon/mesh/marching_cubes.hpp"
#include "volumetric_kit/recon/volume/hierarchical_field_view.hpp"

namespace volumetric_kit::recon::mesh {

/// @brief Extract regular and cross-level dual cells on the device.
///
/// Uses the cell-centered AMR dual construction of Wald (2020). Adjacent
/// leaves share actual samples and canonical edge interpolation, without
/// materializing a finest-resolution volume. The existing node-centered
/// MarchingCubes remains independent. Every call meshes all active leaves.
/// Output is device-local and only the draw count/status return to the host.
///
/// The device and allocator must outlive this object. Calls and releases are
/// externally serialized. A single slot expires on the next extract; with
/// several slots the consumer releases completed generations before reuse,
/// on the same terms as MarchingCubesConfig::slot_count.
class VR_MESH_API HierarchicalMarchingCubes {
 public:
  /// @brief Create an extractor and its retained output ring.
  /// @param device Device shared by the field and output consumer.
  /// @param allocator Allocator on that device; borrowed.
  /// @param config Output usage, queue families and ring depth. Vertex sharing
  ///               and block spans are currently unsupported and refused.
  /// @return Extractor, or InvalidArgument/Unsupported for invalid settings,
  ///         or the underlying device allocation/pipeline failure.
  static Result<HierarchicalMarchingCubes> create(
      Device& device, Allocator& allocator,
      const MarchingCubesConfig& config = {});

  /// @brief Construct an empty extractor.
  HierarchicalMarchingCubes() noexcept;
  ~HierarchicalMarchingCubes();
  HierarchicalMarchingCubes(HierarchicalMarchingCubes&&) noexcept;
  HierarchicalMarchingCubes& operator=(HierarchicalMarchingCubes&&) noexcept;
  HierarchicalMarchingCubes(const HierarchicalMarchingCubes&) = delete;
  HierarchicalMarchingCubes& operator=(const HierarchicalMarchingCubes&) =
      delete;

  /// @return Whether device resources have been created.
  bool valid() const noexcept;

  /// @brief Extract the field's iso-surface into a released output slot.
  /// @param field Current borrowed field; remains quiescent during this call.
  /// @param iso Iso-value in metres; must be finite.
  /// @param timings Optional host phase/count metrics, reset on entry.
  /// @return Borrowed DeviceMesh, or an invalid/stale field, exhausted ring,
  ///         corrupt node, insufficient output capacity or device failure.
  /// @note A refusal before claiming a slot preserves outstanding views.
  Result<DeviceMesh> extract_device(const volume::HierarchicalFieldView& field,
                                    float iso = 0.0f,
                                    ExtractTimings* timings = nullptr);

  /// @brief Extract, download and retire only this call's output slot.
  /// @param field Current field, as for extract_device.
  /// @param iso Iso-value in metres.
  /// @param timings Optional extraction and readback metrics.
  /// @return Owned host mesh, or the extraction/download failure.
  Result<Mesh> extract_host(const volume::HierarchicalFieldView& field,
                            float iso = 0.0f,
                            ExtractTimings* timings = nullptr);

  /// @brief Download this object's newest output to an owned host mesh.
  /// @param mesh Current view from this extractor; another producer is refused.
  /// @return Owned mesh, or InvalidArgument for a stale/foreign view or a
  ///         readback failure. Does not release a renderer-owned generation.
  Result<Mesh> download(const DeviceMesh& mesh) const;

  /// @brief Declare all output generations through generation no longer used.
  /// @param generation Consumer's monotonically increasing completion mark.
  /// @note Producer-thread only. Future, unpublished generations are ignored.
  void release_through(std::uint64_t generation) noexcept;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace volumetric_kit::recon::mesh
