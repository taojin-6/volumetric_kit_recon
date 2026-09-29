// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// Internal (not installed), and built only with the video decoder: a
// decoder's picture left on the GPU, carried from the decode thread through
// the mailbox inside an SDK colour frame whose buffer owns it. The picture
// then lives exactly as long as its frame -- dropped with it, grouped into a
// rig's set with it, held with it -- and every step between the SDK and the
// raw frame goes on handling frame sets.

#include <memory>

#include <libobsensor/ObSensor.hpp>

#include "volumetric_kit/recon/sensor/video/decoded_picture.hpp"

namespace volumetric_kit::recon::sensor::orbbec {

// A colour frame owning a copy of @p picture, which holds its device buffer
// or images. Not a video frame: its bytes are the picture, not pixels, so
// its size is the picture's to say.
std::shared_ptr<ob::Frame> device_picture_frame(const DecodedPicture& picture);

// The picture a frame from device_picture_frame carries; null for any other
// frame.
const DecodedPicture* device_picture(const ob::Frame& frame);

}  // namespace volumetric_kit::recon::sensor::orbbec
