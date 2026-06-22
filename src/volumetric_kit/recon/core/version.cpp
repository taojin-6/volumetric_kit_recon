// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/recon/version.hpp"

namespace volumetric_kit::recon {

const char* version_string() noexcept { return VR_VERSION_STRING; }

int version_major() noexcept { return VR_VERSION_MAJOR; }

int version_minor() noexcept { return VR_VERSION_MINOR; }

int version_patch() noexcept { return VR_VERSION_PATCH; }

}  // namespace volumetric_kit::recon
