// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// sender::fuse_inputs, the hand-off from prepared frames to the fuser's
// inputs, field by field: an empty entry is skipped; each frame's slot is its
// index in the set, its noise its frame's depth_noise as its producer stated
// it (an unknown one stays unknown, for Dynamic to refuse), its camera the
// frame's depth camera, and its colour the frame's, through its own camera
// with the coverage in the high byte, or none; moved, the inputs' colours
// still point into their own list. Host only: no buffer is bound.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "volumetric_kit/core/testing/test_check.hpp"
#include "volumetric_kit/core/vulkan/buffer.hpp"
#include "volumetric_kit/recon/sender/fuse_inputs.hpp"
#include "volumetric_kit/sensor/frame/depth_noise.hpp"
#include "volumetric_kit/sensor/frame/device_frame.hpp"

namespace vkc = volumetric_kit::core;
namespace sensor = volumetric_kit::sensor;
namespace sender = volumetric_kit::recon::sender;
namespace tsdf = volumetric_kit::recon::tsdf;

namespace {

using Frames = std::vector<std::optional<sensor::DeviceFrame>>;

// A prepared frame whose cameras' sizes tell it apart, its buffers empty.
sensor::DeviceFrame frame(std::uint32_t width, bool color,
                          const sensor::DepthNoise& noise) {
  sensor::DeviceFrame f;
  f.depth = std::make_shared<const vkc::Buffer>();
  if (color) f.color = std::make_shared<const vkc::Buffer>();
  f.depth_camera.width = width;
  f.depth_camera.height = 4;
  f.depth_camera.fx = float(width);
  f.color_camera.width = 2 * width;
  f.color_camera.height = 8;
  f.depth_noise = noise;
  return f;
}

int run() {
  const sensor::DepthNoise femto{0.0f, 0.00095f, 0.0f};
  const sensor::DepthNoise stereo =
      sensor::DepthNoise::stereo(0.04, 500.0, 0.1);
  const sensor::DepthNoise constant{0.001f, 0.0f, 0.0f};
  CHECK(femto.known() && stereo.known() && constant.known());

  // Five cameras, the second with no frame this set, the fourth's producer
  // stating no noise.
  Frames set(5);
  set[0] = frame(10, true, femto);
  set[2] = frame(12, false, stereo);
  set[3] = frame(14, true, {});
  set[4] = frame(16, true, constant);
  sender::FuseInputs in = sender::fuse_inputs(set);
  CHECK(in.frames.size() == 4);
  CHECK(in.colors.size() == 3);
  const std::uint32_t slots[] = {0, 2, 3, 4};
  std::size_t colors = 0;
  for (std::size_t k = 0; k < in.frames.size(); ++k) {
    const tsdf::FrameInput& f = in.frames[k];
    const sensor::DeviceFrame& d = *set[slots[k]];
    CHECK(f.slot == slots[k]);
    CHECK(f.noise == d.depth_noise);
    CHECK(f.camera.width == d.depth_camera.width &&
          f.camera.fx == d.depth_camera.fx);
    if (!d.has_color()) {
      CHECK(f.color == nullptr);
      continue;
    }
    CHECK(f.color == &in.colors[colors++]);
    CHECK(f.color->buffer == d.color.get() && f.color->pixels == nullptr);
    CHECK(f.color->cam.width == d.color_camera.width);
    CHECK(f.color->coverage_in_alpha);
  }
  CHECK(in.frames[0].noise == femto);
  CHECK(in.frames[1].noise == stereo);
  CHECK(!in.frames[2].noise.known() &&
        in.frames[2].noise == sensor::DepthNoise{});
  CHECK(in.frames[3].noise == constant);

  // Moved, the colours are still the inputs' own.
  const sender::FuseInputs moved = std::move(in);
  CHECK(moved.frames[0].color == &moved.colors[0]);
  CHECK(moved.frames[3].color == &moved.colors[2]);

  // A set with no frame gives no input.
  CHECK(sender::fuse_inputs(Frames{}).frames.empty());
  const sender::FuseInputs none = sender::fuse_inputs(Frames(3));
  CHECK(none.frames.empty() && none.colors.empty());
  std::puts("recon sender fuse_inputs test passed");
  return 0;
}

}  // namespace

int main() { return run(); }
