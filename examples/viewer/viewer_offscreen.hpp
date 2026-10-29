// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/viewer/viewer_offscreen.hpp
/// @brief `rig_viewer` where there is no display: the device both libraries
///        adopt with no surface, gfx's half of it with an offscreen frame
///        loop where a window's app would be, and the frames it saves as BMP
///        files, each the bytes it read back behind a header.

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "fuse_device.hpp"
#include "viewer_common.hpp"
#include "volumetric_kit/core/base/file_io.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/core/vulkan/shared_device.hpp"
#include "volumetric_kit/core/vulkan/vulkan.hpp"
#include "volumetric_kit/gfx/app/windowed_app.hpp"
#include "volumetric_kit/gfx/core/offscreen_target.hpp"
#include "volumetric_kit/gfx/windowing/frame_loop.hpp"
#include "volumetric_kit/gfx/windowing/swapchain.hpp"

namespace fuse_viewer {

/// The pace of an offscreen frame loop: 60 Hz, a common display's, where a
/// window's FIFO swapchain would wait for the display. Any rate of 50 Hz or
/// more draws every mesh of a 25 Hz rig; unpaced, the loop would draw as fast
/// as frames complete, loading the graphics queue beside the fusion's.
inline constexpr std::chrono::nanoseconds kOffscreenFrameInterval =
    std::chrono::nanoseconds(std::chrono::seconds(1)) / 60;

/// @brief The device gfx, recon and the sensor package all adopt where there
///        is no display: core's `SharedDevice` with no surface, recon's and
///        the sensor package's requirements as compute and @p app's as
///        graphics, which presents nothing. Prints the device's summary, as
///        @ref make_viewer_device does.
/// @param app         As @ref viewer_device_config.
/// @param validation  As @ref viewer_device_config.
/// @return The device, to be declared before, and so outlive, everything
///         adopted or made on it; or the requirements' merge's or
///         `SharedDevice::create`'s failure.
inline vkc::Result<std::unique_ptr<vkc::SharedDevice>>
make_offscreen_viewer_device(const vg::app::WindowedAppConfig& app,
                             bool validation) {
  const vg::app::WindowDeviceConfig settings =
      viewer_device_config(app, validation);
  vkc::SharedDeviceConfig config;
  config.instance = settings.instance;
  VKC_ASSIGN(config.compute, vr_example::fuse_device_requirements());
  config.graphics = settings.graphics;
  config.graphics.needs_present = false;
  VKC_ASSIGN(std::unique_ptr<vkc::SharedDevice> shared,
             vkc::SharedDevice::create(config));
  std::printf("shared device: %s\n", shared->summary().c_str());
  return shared;
}

/// @brief What an offscreen frame loop draws into in place of @p swapchain:
///        its extent, its preferred colour format and its depth format, read
///        back, paced at @ref kOffscreenFrameInterval.
inline vg::windowing::OffscreenLoopDesc offscreen_frames(
    const vg::windowing::SwapchainConfig& swapchain) {
  vg::windowing::OffscreenLoopDesc desc;
  desc.target.extent = swapchain.extent;
  desc.target.color_format = swapchain.preferred_format;
  desc.target.depth_format = swapchain.depth_format;
  desc.target.readback = true;
  desc.frame_interval = kOffscreenFrameInterval;
  return desc;
}

/// @brief gfx's half of the device where there is no display: the graphics
///        payload adopted, its allocator, and an offscreen frame loop -- what
///        a `WindowedApp` holds where there is a window.
class OffscreenRenderer {
 public:
  /// @brief Adopt @p shared's graphics payload with @p app's requirements,
  ///        make its allocator, and an offscreen loop of
  ///        `app.frames_in_flight` over @ref offscreen_frames
  ///        `(app.swapchain)`.
  /// @param shared  From @ref make_offscreen_viewer_device; it must outlive
  ///                the renderer.
  /// @param app     The config @p shared was built for.
  /// @return The renderer, or the failing step's status.
  static vkc::Result<std::unique_ptr<OffscreenRenderer>> adopt(
      const vkc::SharedDevice& shared, const vg::app::WindowedAppConfig& app) {
    std::unique_ptr<OffscreenRenderer> renderer(new OffscreenRenderer());
    VKC_ASSIGN(vkc::Device device,
               vkc::Device::adopt(shared.graphics_payload(), app.device));
    renderer->device_.emplace(std::move(device));
    VKC_ASSIGN(
        vkc::Allocator allocator,
        vkc::Allocator::create(shared.instance().handle(), *renderer->device_));
    renderer->allocator_.emplace(std::move(allocator));
    VKC_ASSIGN(renderer->loop_,
               vg::windowing::FrameLoop::create(
                   *renderer->device_, *renderer->allocator_,
                   offscreen_frames(app.swapchain), app.frames_in_flight));
    return renderer;
  }

  OffscreenRenderer(const OffscreenRenderer&) = delete;
  OffscreenRenderer& operator=(const OffscreenRenderer&) = delete;
  OffscreenRenderer(OffscreenRenderer&&) = delete;
  OffscreenRenderer& operator=(OffscreenRenderer&&) = delete;
  /// The loop drains its frames first, then the allocator and the device go.
  ~OffscreenRenderer() = default;

  vkc::Device& device() noexcept { return *device_; }
  vkc::Allocator& allocator() noexcept { return *allocator_; }
  vg::windowing::FrameLoop& frame_loop() noexcept { return *loop_; }

 private:
  OffscreenRenderer() = default;

  std::optional<vkc::Device> device_;
  std::optional<vkc::Allocator> allocator_;
  std::unique_ptr<vg::windowing::FrameLoop> loop_;
};

/// The BMP header's size: the file header (14 bytes) and BITMAPINFOHEADER
/// (40).
inline constexpr std::size_t kBmpHeaderBytes = 54;

/// @brief A 32-bit BI_RGB BMP of @p bgra, top-down (a negative height): the
///        54-byte header, then the bytes as they are. A `B8G8R8A8` readback
///        is exactly its pixel array, the fourth byte unused, so nothing is
///        done to a pixel.
/// @param bgra    `extent.width * extent.height` texels of 4 bytes, rows top
///                to bottom, packed.
/// @param extent  The image's size.
/// @return The file's bytes; `InvalidArgument` for null @p bgra, a zero
///         extent, or an image a BMP's 32-bit sizes cannot hold.
inline vkc::Result<std::string> bmp_bytes(const void* bgra, VkExtent2D extent) {
  if (bgra == nullptr || extent.width == 0 || extent.height == 0) {
    return vkc::Status::invalid_argument(
        "bmp_bytes: no pixels, or a zero extent");
  }
  const std::uint64_t pixel_bytes =
      std::uint64_t{extent.width} * extent.height * 4u;
  if (extent.width > 0x7FFFFFFFu || extent.height > 0x7FFFFFFFu ||
      pixel_bytes + kBmpHeaderBytes > 0xFFFFFFFFu) {
    return vkc::Status::invalid_argument(
        "bmp_bytes: the image is too large for a BMP");
  }
  std::string bytes(kBmpHeaderBytes + pixel_bytes, '\0');
  std::size_t at = 0;
  // Little-endian, whatever the host's order.
  const auto put = [&bytes, &at](std::uint32_t value, int size) {
    for (int i = 0; i < size; ++i) {
      bytes[at++] = static_cast<char>((value >> (8 * i)) & 0xFFu);
    }
  };
  const std::uint32_t file_bytes =
      static_cast<std::uint32_t>(kBmpHeaderBytes + pixel_bytes);
  // A negative height: the first row is the top one, as read back.
  const std::uint32_t height =
      static_cast<std::uint32_t>(-static_cast<std::int32_t>(extent.height));
  // The file header: "BM", the file's size, reserved, the pixels' offset.
  put('B' | ('M' << 8), 2);
  put(file_bytes, 4);
  put(0, 4);
  put(static_cast<std::uint32_t>(kBmpHeaderBytes), 4);
  // BITMAPINFOHEADER: its size, width, height, one plane, 32 bits a pixel,
  // BI_RGB (uncompressed), the pixels' size, 72 dpi (2835 pixels a metre)
  // both ways, and no palette.
  put(40, 4);
  put(extent.width, 4);
  put(height, 4);
  put(1, 2);
  put(32, 2);
  put(0, 4);
  put(static_cast<std::uint32_t>(pixel_bytes), 4);
  put(2835, 4);
  put(2835, 4);
  put(0, 4);
  put(0, 4);
  std::memcpy(&bytes[kBmpHeaderBytes], bgra,
              static_cast<std::size_t>(pixel_bytes));
  return bytes;
}

/// @brief Write @p bgra to @p path as @ref bmp_bytes says, through
///        `core::write_file`, replacing a file there.
/// @return OK; @ref bmp_bytes ' refusal; or `write_file`'s failure, which
///         leaves what was at @p path as it was.
inline vkc::Status write_bmp(const std::string& path, const void* bgra,
                             VkExtent2D extent) {
  VKC_ASSIGN(const std::string bytes, bmp_bytes(bgra, extent));
  return vkc::write_file(path, bytes, vkc::WriteMode::Replace);
}

/// @brief Saves some of an offscreen loop's frames as BMP files: the first
///        frame that draws a mesh, then one each @p every, and the last.
///
/// A saved frame costs its readback, recorded into the frame after its pass,
/// and a file write on the render thread once the frame has completed:
/// @ref collect writes it when its slot's next frame begins, which waited
/// for the slot's fence, and the frame's target holds it until that frame is
/// ended (`windowing::Frame::offscreen`); @ref flush writes what is left
/// once the device has idled, before the loop goes. Each frame of the loop
/// goes through it in that order:
///
/// @code
/// const windowing::Frame& frame = *begun;
/// saver.collect(frame);
/// // ... the frame's pass ...
/// saver.record(frame, draws_mesh, last, std::chrono::steady_clock::now());
/// VKC_TRY(loop.end_frame(frame));
/// saver.ended(frame);
/// // ... after the loop: device.wait_idle(), then
/// saver.flush();
/// @endcode
class FrameSaver {
 public:
  /// @param path              Where the frames go, `<stem>.bmp`: frame N is
  ///                          `<stem>_<N>.bmp` beside it (@ref name). Empty
  ///                          saves nothing.
  /// @param every             The least time between two saved frames that
  ///                          draw a mesh.
  /// @param frames_in_flight  The loop's.
  /// @param says              Where the first failed write is said.
  FrameSaver(std::string path, std::chrono::nanoseconds every,
             std::uint32_t frames_in_flight, std::FILE* says = stderr)
      : path_(std::move(path)),
        every_(every),
        says_(says),
        pending_(frames_in_flight) {}

  /// @return Whether it saves frames at all.
  bool enabled() const noexcept { return !path_.empty(); }

  /// @return The file frame @p number is saved to.
  std::string name(std::uint64_t number) const {
    std::filesystem::path file(path_);
    file.replace_filename(file.stem().string() + "_" + std::to_string(number) +
                          file.extension().string());
    return file.string();
  }

  /// @brief @p frame has begun: write the frame its slot carried before, if
  ///        that one was saved and submitted.
  void collect(const vg::windowing::Frame& frame) {
    if (frame.slot >= pending_.size()) return;
    Pending& slot = pending_[frame.slot];
    if (slot.submitted) write(slot);
    slot = Pending{};
  }

  /// @brief Whether to save @p frame and, if so, record its readback into
  ///        its command buffer. Call after the frame's pass, before its end.
  /// @param draws_mesh  Whether the frame draws a mesh.
  /// @param last        Whether it is the run's last frame, saved always.
  /// @param now         The time the frame is drawn at.
  /// @return Whether the frame is saved; never for a frame with no offscreen
  ///         target (a window's).
  bool record(const vg::windowing::Frame& frame, bool draws_mesh, bool last,
              std::chrono::steady_clock::time_point now) {
    if (!enabled() || frame.offscreen == nullptr ||
        frame.slot >= pending_.size()) {
      return false;
    }
    const bool due =
        last || (draws_mesh && (!mesh_saved_ || now - *mesh_saved_ >= every_));
    if (!due) return false;
    frame.offscreen->record_readback(frame.cmd);
    pending_[frame.slot] = Pending{frame.number, frame.offscreen, false};
    if (draws_mesh) mesh_saved_ = now;
    return true;
  }

  /// @brief @p frame was ended: its readback runs, so it is written later.
  ///        A frame whose end failed is not, as its commands never ran.
  void ended(const vg::windowing::Frame& frame) {
    if (frame.slot < pending_.size() &&
        pending_[frame.slot].number == frame.number) {
      pending_[frame.slot].submitted = true;
    }
  }

  /// @brief Write the saved frames not yet written, in order. Call once the
  ///        device has idled, while the loop's targets are alive.
  void flush() {
    std::vector<Pending*> left;
    for (Pending& slot : pending_) {
      if (slot.submitted) left.push_back(&slot);
    }
    std::sort(left.begin(), left.end(), [](const Pending* a, const Pending* b) {
      return a->number < b->number;
    });
    for (Pending* slot : left) write(*slot);
    std::fill(pending_.begin(), pending_.end(), Pending{});
  }

  /// @return The files written, in order.
  const std::vector<std::string>& saved() const noexcept { return saved_; }
  /// @return The saved frames whose write failed.
  std::size_t failures() const noexcept { return failures_; }

 private:
  struct Pending {
    std::uint64_t number = 0;  // 0: nothing saved in the slot
    const vg::OffscreenTarget* target = nullptr;
    bool submitted = false;
  };

  void write(const Pending& slot) {
    const std::string file = name(slot.number);
    const vkc::Status written =
        write_bmp(file, slot.target->pixels(), slot.target->extent());
    if (written.ok()) {
      saved_.push_back(file);
      return;
    }
    // Said once: a path that cannot be written fails every frame alike, and
    // drawing goes on without it.
    if (failures_++ == 0) {
      std::fprintf(says_,
                   "rig_viewer: frame %llu not saved (%s); later failures "
                   "are not said\n",
                   static_cast<unsigned long long>(slot.number),
                   written.message().c_str());
    }
  }

  std::string path_;
  std::chrono::nanoseconds every_;
  std::FILE* says_;
  std::vector<Pending> pending_;  // per slot
  std::optional<std::chrono::steady_clock::time_point> mesh_saved_;
  std::vector<std::string> saved_;
  std::size_t failures_ = 0;
};

}  // namespace fuse_viewer
