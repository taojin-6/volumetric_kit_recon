// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// VideoToolbox's pictures, handed over on the device (Apple): each plane of
// the picture's IOSurface becomes a Metal texture, imported into Vulkan as an
// image (VK_EXT_metal_objects), so the picture never crosses to the host. A
// whole NV12 surface does not import as one image: MoltenVK refuses its
// bytes per element.

#include <CoreVideo/CoreVideo.h>

#include <cstdint>
#include <memory>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/recon/core/fwd.hpp"
#include "volumetric_kit/recon/sensor/video/decoded_picture.hpp"

namespace volumetric_kit::recon::sensor::video {

class VtPictures {
 public:
  // Unsupported where @p device imports no Metal textures. @p who names the
  // decoder in errors.
  static core::Result<std::unique_ptr<VtPictures>> create(
      const core::Device& device, const char* who);
  ~VtPictures();
  VtPictures(const VtPictures&) = delete;
  VtPictures& operator=(const VtPictures&) = delete;

  // @p pixels' two planes as images holding a @p width x @p height NV12
  // picture from their corner, in GENERAL, each image holding the pixel
  // buffer: fills @p out's size, layout and images and returns true. A
  // surface's images are made the first time a picture arrives on it and
  // kept while pictures keep arriving. False for a picture
  // this does not take -- not 8-bit NV12, not on an IOSurface, or smaller
  // than the picture -- which comes to the host instead.
  core::Result<bool> import(CVPixelBufferRef pixels, std::uint32_t width,
                            std::uint32_t height, DecodedPicture& out);

 private:
  struct Impl;
  explicit VtPictures(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
};

}  // namespace volumetric_kit::recon::sensor::video
