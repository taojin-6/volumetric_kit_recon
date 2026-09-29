// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "device_picture_frame.hpp"

#include <cstring>

#include "hevc_color.hpp"

namespace volumetric_kit::recon::sensor::orbbec {
namespace {

// The metadata that marks the frame: no SDK frame, and no I420 frame of the
// decoder's (which carries a PlanesColor), has these bytes.
constexpr char kTag[8] = {'v', 'r', 'd', 'e', 'v', 'p', 'i', 'c'};
static_assert(sizeof(kTag) != sizeof(PlanesColor),
              "the tag must not pass for a PlanesColor");

}  // namespace

std::shared_ptr<ob::Frame> device_picture_frame(const DecodedPicture& picture) {
  std::unique_ptr<DecodedPicture> held(new DecodedPicture(picture));
  // The SDK reads none of the bytes: they are the picture, and the buffer is
  // freed as one.
  auto frame = ob::FrameFactory::createFrameFromBuffer(
      OB_FRAME_COLOR, OB_FORMAT_NV12,
      reinterpret_cast<std::uint8_t*>(held.get()),
      [](std::uint8_t* b) { delete reinterpret_cast<DecodedPicture*>(b); },
      static_cast<std::uint32_t>(sizeof(DecodedPicture)));
  held.release();  // the frame's now, freed by the callback
  frame->updateMetadata(reinterpret_cast<const std::uint8_t*>(kTag),
                        static_cast<std::uint32_t>(sizeof(kTag)));
  return frame;
}

const DecodedPicture* device_picture(const ob::Frame& frame) {
  if (frame.getMetadataSize() != sizeof(kTag) ||
      std::memcmp(frame.getMetadata(), kTag, sizeof(kTag)) != 0 ||
      frame.getDataSize() != sizeof(DecodedPicture)) {
    return nullptr;
  }
  return reinterpret_cast<const DecodedPicture*>(frame.getData());
}

}  // namespace volumetric_kit::recon::sensor::orbbec
