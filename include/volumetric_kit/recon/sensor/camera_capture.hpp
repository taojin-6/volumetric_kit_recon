// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file camera_capture.hpp
/// @brief The capture contract: a posed RGB-D frame, and the interface a device
///        implements to produce them.
///
/// This tier is a **contract, not a driver collection** (the 2026-08-02
/// decision). A driver ships here only when this repo can build *and test* it;
/// a platform-bound one — ARKit, which is iOS-only Objective-C and needs LiDAR
/// hardware to exercise — lives with the application that can, and implements
/// @ref ICameraCapture from outside. The same split is why
/// `volumetric_kit_gfx`'s windowing tier takes a consumer-supplied surface
/// rather than owning a window system, and why it ports untouched.
///
/// That is why the camera types come from `core/camera_params.hpp` rather than
/// from the fusion tiers that also consume them: this header — and
/// `recon_sensor`'s whole link line — reaches `core` alone, so a driver
/// implementing @ref ICameraCapture out of tree (an ARKit source in
/// `volumetric_kit_ios` is Objective-C++) compiles against the math vocabulary
/// and never preprocesses the Vulkan surface. *Consuming* a frame still means
/// including the fusion headers; *producing* one does not.

#include <cstdint>
#include <optional>

#include "volumetric_kit/recon/core/camera_params.hpp"
#include "volumetric_kit/recon/core/color_space.hpp"
#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/sensor/export.hpp"

namespace volumetric_kit::recon::sensor {

/// @brief One posed RGB-D frame, in the shape the fusion tiers already consume.
///
/// A **non-owning view**: the pixels belong to the capture device, which is
/// free to recycle them once the next @ref ICameraCapture::poll is called. Copy
/// anything that must outlive the frame. This mirrors how the fusion entry
/// points already take their inputs (`TsdfIntegrator::integrate` and
/// `VoxelHashMap::allocate_from_depth` both borrow a `const float*`), so a
/// captured frame feeds them with no repacking:
///
/// @code
/// // `failed` counts blocks the map had no room for. Neither call reports a
/// // shortfall any other way, and neither return is [[nodiscard]], so
/// // dropping them fuses a frame with silent holes -- grow and retry instead.
/// VR_ASSIGN(std::uint32_t failed,
///           grid.map().allocate_from_depth(frame.depth, frame.depth_camera));
/// if (failed != 0) {
///   VR_TRY(grid.resize(grid.grid().num_buckets * 2));  // then retry the frame
///   return Status::out_of_memory("map full; grew it, frame not fused");
/// }
/// // Carry the encoding across. `ColorFrame::encoding` defaults to canonical,
/// // so leaving it out does not mean "unspecified" -- it *declares* canonical,
/// // and a non-canonical frame would then be fused through the wrong curve
/// // instead of refused. Pass it and a driver that forgot to call
/// // sensor::to_canonical gets an error rather than a dim reconstruction.
/// tsdf::ColorFrame color{frame.color, frame.color_camera,
///                        frame.color_encoding};
/// VR_TRY(integrator.integrate(grid, frame.depth, frame.depth_camera, 5.0f,
///                             tsdf::IntegrationMode::Classic,
///                             frame.has_color() ? &color : nullptr));
/// @endcode
struct CapturedFrame {
  /// Row-major depth in **metres**, `depth_camera.width * height` samples. A
  /// sample of 0 (or outside the camera's depth range) is "no return" and is
  /// skipped by the fusion kernels — which is how a driver forwards a
  /// confidence mask, by zeroing the samples it does not trust.
  const float* depth = nullptr;

  /// Row-major colour, `color_camera.width * height` pixels, RGB packed in each
  /// `uint32`'s low three bytes — the layout the `tsdf` and `mesh` tiers use.
  /// Null when the device produced no colour for this frame.
  const std::uint32_t* color = nullptr;

  /// Depth intrinsics, size, range and camera-to-world pose. The pose must
  /// already be in this repo's convention; see @ref cv_from_gl_camera for a
  /// source that reports the OpenGL/ARKit one.
  DepthCameraParams depth_camera{};

  /// Colour intrinsics, size and pose. Where depth is registered to colour
  /// (ARKit), derive the depth camera from this one with
  /// @ref depth_from_registered_color so the two cannot drift apart.
  /// Meaningful only when @ref color is set.
  ColorCameraParams color_camera{};

  /// What @ref color is encoded as — the declaration a driver makes and never
  /// acts on. Defaults to the canonical form (sRGB transfer, BT.709 primaries),
  /// so a source already producing canonical bytes says nothing; ARKit declares
  /// `{Transfer::Bt709, Primaries::Bt709}`, which @ref is_canonical accepts, so
  /// its frames need no conversion either.
  ///
  /// A frame that is *not* canonical is converted once, here at the boundary,
  /// with @ref to_canonical — the fusion tiers decode with exactly one curve
  /// and reject anything else rather than guess. Meaningful only when
  /// @ref color is set.
  ColorEncoding color_encoding{};

  /// Device timestamp in nanoseconds; monotonic within one capture session.
  /// Zero when the device reports none.
  std::uint64_t timestamp_ns = 0;

  /// @return `true` if this frame carries colour.
  bool has_color() const noexcept { return color != nullptr; }
};

/// @brief A source of posed RGB-D frames.
///
/// Implementations are expected to be **polled**, not to call back: a
/// push-based device (ARKit delivers frames on a session queue) buffers its
/// newest frame and hands it over on the next @ref poll. Polling is the common
/// denominator — it wraps a push source, while the reverse forces every
/// consumer onto the device's thread.
///
/// @warning Not thread-safe unless an implementation documents otherwise. Poll
///          from one thread.
class VR_SENSOR_API ICameraCapture {
 public:
  virtual ~ICameraCapture() = default;

  ICameraCapture(const ICameraCapture&) = delete;
  ICameraCapture& operator=(const ICameraCapture&) = delete;

  /// @brief Begin producing frames. Idempotent: starting a running device is
  /// OK.
  /// @return OK once running, or why the device could not start.
  virtual Status start() = 0;

  /// @brief Stop producing frames and release the device.
  ///
  /// Idempotent, and safe to call on a device that never started — it is the
  /// destructor's fallback, so it cannot fail in a way a caller must handle.
  virtual void stop() noexcept = 0;

  /// @brief The "no new frame this tick" return, spelled out for implementers.
  ///
  /// @ref poll returns `Result<std::optional<CapturedFrame>>`, and @ref Result
  /// converts implicitly only from *exactly* its value type. Every natural
  /// spelling therefore fails, and none of the failures is obvious from the
  /// signature:
  /// - `return std::nullopt;` needs two user-defined conversions
  ///   (`nullopt_t` → `std::optional` → @ref Result) and does not compile;
  /// - `return {};` is ambiguous between @ref Result's value and @ref Status
  ///   constructors;
  /// - `return Status{};` compiles and then **aborts**, because an OK
  ///   @ref Status is not a failure and @ref Result checks that.
  ///
  /// Use this and @ref some_frame instead of rediscovering the wrapping.
  ///
  /// @return An OK @ref Result holding an empty optional.
  static Result<std::optional<CapturedFrame>> no_frame() {
    return std::optional<CapturedFrame>{};
  }

  /// @brief The "here is the frame" return, the counterpart to @ref no_frame.
  ///
  /// `return frame;` does not compile for the same reason `return
  /// std::nullopt;` does not: `CapturedFrame` → `std::optional` → @ref Result
  /// is two user-defined conversions. Both of @ref poll's success paths
  /// therefore go through a helper rather than through a wrap the caller has to
  /// get right.
  ///
  /// @param frame  The frame to hand over.
  /// @return An OK @ref Result holding @p frame.
  static Result<std::optional<CapturedFrame>> some_frame(CapturedFrame frame) {
    return std::optional<CapturedFrame>{frame};
  }

  /// @brief Take the newest frame not yet returned, if there is one.
  ///
  /// Returns an empty optional (@ref no_frame) when no new frame has arrived
  /// since the last call — the ordinary case for a consumer polling faster
  /// than the sensor runs, and **not** an error. Only a genuine device failure
  /// is a non-OK @ref Status. (Same shape as `gfx`'s
  /// `WindowedApp::begin_frame`, which likewise separates "nothing this tick"
  /// from "something is wrong".)
  ///
  /// Frames may be **dropped**, not queued: a consumer slower than the sensor
  /// gets the newest frame rather than a backlog, which is what a live
  /// reconstruction wants.
  ///
  /// @code
  /// Result<std::optional<CapturedFrame>> MyCapture::poll() {
  ///   if (faulted_) return Status::io_error("sensor stopped responding");
  ///   if (!newest_) return no_frame();   // ordinary; not an error
  ///   CapturedFrame frame = *newest_;
  ///   newest_.reset();                   // hand each frame over once
  ///   return some_frame(frame);
  /// }
  /// @endcode
  ///
  /// @return The frame; an empty optional if none is ready; or a device error.
  virtual Result<std::optional<CapturedFrame>> poll() = 0;

 protected:
  ICameraCapture() = default;
  // Move operations stay available to implementations but are not part of the
  // polymorphic interface -- callers hold these behind a pointer.
  ICameraCapture(ICameraCapture&&) = default;
  ICameraCapture& operator=(ICameraCapture&&) = default;
};

}  // namespace volumetric_kit::recon::sensor
