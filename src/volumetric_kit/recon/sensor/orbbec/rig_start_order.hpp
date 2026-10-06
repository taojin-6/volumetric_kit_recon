// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// Internal (not installed), SDK-free: the order OrbbecRig starts its cameras
// in. The grouping of their frames is the sensor tier's TriggerGrouper.

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/recon/sensor/orbbec/orbbec_capture.hpp"

namespace volumetric_kit::recon::sensor::orbbec {

/// The order a rig's cameras start in -- every secondary before the primary,
/// since the primary's first trigger is what the secondaries wait for -- or
/// why the rig cannot run: no primary, more than one, or a camera that would
/// stream on its own clock rather than on the primary's trigger.
core::Result<std::vector<std::size_t>> rig_start_order(
    const std::vector<OrbbecSyncMode>& modes,
    const std::vector<std::string>& serials);

}  // namespace volumetric_kit::recon::sensor::orbbec
