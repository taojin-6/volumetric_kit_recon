// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file tsdf/change_detector.hpp
/// @brief Per-camera change detection: each fixed camera keeps a history of
///        every pixel on the device and flags the pixels whose depth changed
///        for real, not by noise.

#include <cstdint>
#include <vector>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/buffer.hpp"
#include "volumetric_kit/core/vulkan/command_batch.hpp"
#include "volumetric_kit/core/vulkan/compute_kernel.hpp"
#include "volumetric_kit/core/vulkan/compute_util.hpp"
#include "volumetric_kit/core/vulkan/descriptor.hpp"
#include "volumetric_kit/core/vulkan/fwd.hpp"
#include "volumetric_kit/core/vulkan/gpu_timer.hpp"
#include "volumetric_kit/grid/volume/voxel_hash_map.hpp"
#include "volumetric_kit/recon/tsdf/export.hpp"
#include "volumetric_kit/sensor/frame/camera_params.hpp"

namespace volumetric_kit::recon::tsdf {

/// @brief A depth sensor's noise floor: the standard deviation of one sample
///        at depth z, `sigma(z) = a + b z + c z^2`, in metres for z in metres.
///
/// The only thing @ref ChangeDetector knows about a sensor. Time-of-flight
/// noise grows about linearly with depth, stereo noise as its square, a lidar
/// is about constant. The default is the Femto Mega's, fitted on a static
/// four-camera capture: 0.95 mm per metre of depth.
struct DepthNoise {
  float a = 0.0f;      ///< Constant term (m); at least 0.
  float b = 0.00095f;  ///< Linear term (m per m); at least 0.
  float c = 0.0f;      ///< Quadratic term (m per m^2); at least 0.
};

/// @brief What @ref ChangeDetector's mask says of a pixel in a frame: one
///        `uint32` a pixel.
enum class PixelChange : std::uint32_t {
  None = 0,     ///< No change: noise, an invalid sample, or still learning.
  Nearer = 1,   ///< The surface came nearer: something arrived in front.
  Farther = 2,  ///< The surface moved away: something left, or receded.
};

/// @brief How @ref ChangeDetector tests a pixel. The defaults are the
///        operating point measured on the Femto rig's static takes (one false
///        alarm per interior pixel in about 39 minutes; 4 cm steps flagged in
///        the frame they happen, 98%).
///
/// A pixel's residual is `r = (d - d_hist) / sigma_r`, with `d_hist` its
/// history and `sigma_r` its own temporal noise, floored by @ref noise. Two
/// cumulative sums, `S+ = max(0, S+ + r - k)` for farther and
/// `S- = max(0, S- - r - k)` for nearer, alarm past @ref h; one frame with
/// `|r|` past @ref one_frame alarms too.
struct ChangeDetectorConfig {
  /// The drift each frame's residual must exceed to accumulate, in sigma;
  /// finite, at least 0.
  float k = 0.5f;
  /// The sum at which a pixel alarms, in sigma; finite, in (0, 1e6].
  float h = 8.0f;
  /// One frame's residual that alarms alone, in sigma; positive (infinity
  /// turns the one-frame test off).
  float one_frame = 6.0f;
  /// The history's memory, in samples: a cumulative mean of the first
  /// `memory` samples, then an exponential average of weight 1/`memory`;
  /// the noise estimate follows the same schedule. In [1, 1023].
  std::uint32_t memory = 30;
  /// Samples a pixel learns from before it tests; at most @ref memory.
  std::uint32_t warm_up = 30;
  /// Samples after an alarm the noise estimate does not learn from, so a
  /// sustained motion cannot train it upward; at most 4095.
  std::uint32_t freeze = 30;
};

/// @brief Detects real change in each fixed camera's depth, a pixel at a
///        time, against that pixel's own past.
///
/// Every camera, named by a slot (@ref kMaxCameras of them), keeps a history
/// per pixel on the device: a running depth `d_hist` and the pixel's own
/// temporal noise `sigma_r` (sqrt(pi/2) times the running mean of
/// `|d - d_hist|`, each sample clipped at 3 `sigma_r`), floored by the
/// sensor's @ref DepthNoise at the sample's depth. Each frame, one pass a
/// camera (@ref record_detect) tests every valid sample against it
/// (@ref ChangeDetectorConfig) and writes the camera's mask
/// (@ref PixelChange). An alarm restarts the pixel's history from the sample,
/// clears both sums, and stops `sigma_r` learning for
/// @ref ChangeDetectorConfig::freeze samples; any other sample updates the
/// history. Comparing a camera with itself cancels its calibration error, and
/// a pixel that always flickers learns a large `sigma_r` and stays quiet, so
/// noise and the cameras' disagreement raise next to no alarm.
///
/// A sample is valid when it is positive and within the camera's
/// `[min_depth, max_depth]`, as fusion takes it. An invalid sample (a hole, a
/// dropout, `NaN`) changes nothing and flags nothing, and a camera missing
/// from a set is simply not detected: on return each pixel compares with its
/// own history. A camera whose image size changes starts a new history.
/// History lives in the camera's pixel grid, so the cameras must be fixed.
///
/// Device memory: 24 bytes a pixel per camera (the history and the mask);
/// 8.8 MB for a 640 x 576 camera.
///
/// @code
/// VKC_ASSIGN(tsdf::ChangeDetector detector,
///            tsdf::ChangeDetector::create(device, allocator, {}));
/// core::CommandBatch batch(device, allocator);
/// for (std::uint32_t c = 0; c < cameras; ++c) {
///   VKC_TRY(detector.record_detect(batch, c, core::StorageInput(depth[c]),
///                                  params[c], noise));
/// }
/// VKC_TRY(batch.submit());
/// const core::Buffer* changed = detector.mask(0);  // uint32 per pixel
/// @endcode
///
/// @warning The `Device` and `Allocator` passed to @ref create must outlive
///          this object; it stores references to them.
class VR_TSDF_API ChangeDetector {
 public:
  /// The camera slots a detector holds: 0 to `kMaxCameras - 1`.
  static constexpr std::uint32_t kMaxCameras = 64;

  /// @brief Build the detection kernel on @p device and check @p config.
  /// @param device     The compute device (must outlive this object).
  /// @param allocator  Its allocator (must outlive this object).
  /// @param config     How pixels are tested.
  /// @return The detector, holding no camera yet;
  ///         `Status::Code::InvalidArgument` for a @p config outside the
  ///         ranges @ref ChangeDetectorConfig gives; or a pipeline's or
  ///         descriptor pool's failure.
  static core::Result<ChangeDetector> create(
      core::Device& device, core::Allocator& allocator,
      const ChangeDetectorConfig& config = {});

  ~ChangeDetector();
  ChangeDetector(ChangeDetector&&) noexcept;
  ChangeDetector& operator=(ChangeDetector&&) = delete;
  ChangeDetector(const ChangeDetector&) = delete;
  ChangeDetector& operator=(const ChangeDetector&) = delete;

  /// @brief Record camera @p camera's test of one depth frame into @p batch:
  ///        its history updated and its mask written, for every pixel.
  ///
  /// The first frame of a camera, or one of a new size, allocates its
  /// history and mask and zeroes them first, in a submit of their own that
  /// it waits on.
  /// @param batch   Records the pass; the caller submits it.
  /// @param camera  The camera's slot, below @ref kMaxCameras: the same
  ///                camera every frame. At most one record a slot in a
  ///                batch.
  /// @param depth   Row-major depth in metres, `params.width *
  ///                params.height` samples: host bytes, staged in @p batch,
  ///                or a storage buffer, bound in place; unchanged until the
  ///                batch has run.
  /// @param params  The depth camera: its size, and the
  ///                `[min_depth, max_depth]` a valid sample lies in, both
  ///                finite. The rest is not read.
  /// @param noise   The camera's noise floor.
  /// @param gate    Optional word; 0 tests nothing and leaves the camera's
  ///                history and mask as they were (a set its allocation
  ///                left incomplete, fused again later). Must be a complete,
  ///                aligned word in a known device-local storage buffer.
  /// @param stage   Optional tag for the pass's device time.
  /// @return OK, with the pass recorded; or, with nothing recorded,
  ///         `Status::Code::InvalidArgument` for a moved-from detector, a
  ///         @p camera past the slots, an empty image, a @p depth that
  ///         cannot be bound as the image, a non-finite depth bound, a
  ///         @p noise with a negative, non-finite or all-zero term, or a
  ///         bad @p gate; or an allocation's or the upload's failure.
  /// @pre The batch of this camera's last record has resolved.
  core::Status record_detect(
      core::CommandBatch& batch, std::uint32_t camera,
      const core::StorageInput& depth, const sensor::DepthCameraParams& params,
      const DepthNoise& noise = {},
      const volumetric_kit::grid::volume::DeviceCount* gate = nullptr,
      core::GpuStageScope* stage = nullptr);

  /// @brief @ref record_detect in a batch of its own, submitted and waited
  ///        on.
  /// @return As @ref record_detect, or the submit's failure.
  core::Status detect(std::uint32_t camera, const core::StorageInput& depth,
                      const sensor::DepthCameraParams& params,
                      const DepthNoise& noise = {});

  /// @brief Camera @p camera's mask as its last pass left it.
  /// @return A device-local buffer of `width * height` `uint32`
  ///         (@ref PixelChange), row-major in the camera's image; null for a
  ///         slot with no history yet. Valid until the camera's next record
  ///         of another image size, or this detector's destruction; a reader
  ///         records after the pass in the same batch, or in a later one.
  const core::Buffer* mask(std::uint32_t camera) const noexcept;

  /// @brief Whether @ref record_detect takes @p noise: every term finite and
  ///        at least 0, not all 0.
  /// @param noise  The noise model.
  /// @return OK; or `Status::Code::InvalidArgument`.
  static core::Status check_noise(const DepthNoise& noise);

  /// @return The configuration this was created with.
  const ChangeDetectorConfig& config() const noexcept { return config_; }
  /// @return Whether this owns a live kernel; false when moved-from.
  bool valid() const noexcept { return kernel_.valid(); }

 private:
  // One camera's device state: the pixels' histories and the mask.
  struct Camera {
    core::Buffer history;
    core::Buffer mask;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
  };

  ChangeDetector() = default;

  // Borrowed (must outlive this).
  core::Device* device_ = nullptr;
  core::Allocator* allocator_ = nullptr;
  ChangeDetectorConfig config_;
  std::uint32_t max_workgroup_count_x_ = 0;
  VkDeviceSize max_storage_buffer_range_ = 0;
  // The detection kernel, its pool and one set a camera slot.
  core::ComputeKernel kernel_;
  core::DescriptorPool pool_;
  core::KernelSets sets_;
  // A word bound in the gate's place when a record has none.
  core::Buffer no_gate_;
  std::vector<Camera> cameras_;
};

}  // namespace volumetric_kit::recon::tsdf
