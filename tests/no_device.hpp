// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// What a GPU test returns when it finds no usable Vulkan device. It skips by
// default, so the suite still runs on a machine without a driver. CI sets
// VR_REQUIRE_VULKAN_DEVICE on every leg, all of which have a device, and there
// it fails instead: a test whose own device creation failed would otherwise
// pass having tested nothing.

#include <cstdio>
#include <cstdlib>
#include <string>

namespace vr_test {

// Reports `what` (and `why`, when given) and returns the test's exit code: 0
// to skip, or 1 when VR_REQUIRE_VULKAN_DEVICE is set.
inline int no_device(const char* what, const std::string& why = {}) {
  const char* require = std::getenv("VR_REQUIRE_VULKAN_DEVICE");
  const bool required = require != nullptr && *require != '\0';
  if (why.empty()) {
    std::fprintf(stderr, "%s; %s\n", what, required ? "failing" : "skipping");
  } else {
    std::fprintf(stderr, "%s (%s); %s\n", what, why.c_str(),
                 required ? "failing" : "skipping");
  }
  return required ? 1 : 0;
}

// What a decoder test returns when the decoders have no device path here (no
// GPU this build's hardware path reaches): it skips, or fails where CI sets
// VR_TEST_HEVC_BACKEND, as on every leg that promises one.
inline int no_decoder(const std::string& why) {
  const char* promise = std::getenv("VR_TEST_HEVC_BACKEND");
  const bool required = promise != nullptr && *promise != '\0';
  std::fprintf(stderr, "no device path for the decoders (%s); %s\n",
               why.c_str(), required ? "failing" : "skipping");
  return required ? 1 : 0;
}

// As no_decoder for JpegDecoder, except that a GPU with no hardware JPEG
// engine always skips: NVIDIA's are few (the RTX 5090 has one, the RTX 4090
// on one CI host none), and nothing stands in for it.
inline int no_jpeg_decoder(const std::string& why) {
  if (why.find("no hardware JPEG engine") != std::string::npos) {
    std::fprintf(stderr, "%s; skipping\n", why.c_str());
    return 0;
  }
  return no_decoder(why);
}

}  // namespace vr_test
