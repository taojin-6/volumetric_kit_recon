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

}  // namespace vr_test
