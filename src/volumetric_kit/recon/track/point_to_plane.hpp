// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// FitKind::PointToPlane (point_to_plane.cpp). Internal: callers reach it
// through create_fitter.

#include <memory>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/fwd.hpp"
#include "volumetric_kit/recon/track/fitter.hpp"

namespace volumetric_kit::recon::track::detail {

// @pre config was checked (create_fitter.cpp's check_config).
core::Result<std::unique_ptr<IFitter>> create_point_to_plane(
    core::Device& device, core::Allocator& allocator,
    const FitterConfig& config);

}  // namespace volumetric_kit::recon::track::detail
