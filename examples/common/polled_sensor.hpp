// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/polled_sensor.hpp
/// @brief A sensor polled on a thread of its own: what a live link needs,
///        whose own poll waits for records.

#include <memory>

#include "volumetric_kit/sensor/rgbd_sensor.hpp"

namespace vr_example {

namespace sensor = volumetric_kit::sensor;

/// @return @p sensor polled on a thread of its own from its start, so a
///         @ref sensor::IRgbdSensor::poll or `drain` never waits on it: what
///         a live link needs, whose own poll waits for records. It keeps the
///         interface's queue: `set_queue_depth` frames (1 by default), the
///         oldest let go (counted dropped) when a newer one arrives on a
///         full one; `poll` takes the newest, `drain` every frame, oldest
///         first. Its other counters and `exhausted` are the inner sensor's
///         as of its last poll; an error the inner poll returns comes out of
///         the next poll or drain once, and the polling goes on.
std::unique_ptr<sensor::IRgbdSensor> polled_in_background(
    std::unique_ptr<sensor::IRgbdSensor> sensor);

}  // namespace vr_example
