// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/send_set.hpp
/// @brief A camera set sent through `sender::SetSender`, its front batch
///        waiting for the set's pictures. Header-only, as `fuse_frame.hpp`:
///        it includes the `sender` tier.

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/base/stage_metrics.hpp"
#include "volumetric_kit/core/vulkan/command_batch.hpp"
#include "volumetric_kit/core/vulkan/sync.hpp"
#include "volumetric_kit/recon/sender/set_sender.hpp"
#include "volumetric_kit/sensor/array/sensor_array.hpp"
#include "volumetric_kit/sensor/frame/device_frame.hpp"
#include "volumetric_kit/sensor/prep/gpu_frame_prep.hpp"

namespace vr_example {

/// @brief Send @p set through @p out (`sender::SetSender::send`), its prep
///        recorded into @p front as @p prepared
///        (`sensor::SensorArray::record_process`). The batch waits on the
///        device for the set's pictures' ready values
///        (`sensor::ready_points`), gathered here before the set moves into
///        what the batch holds until it has run.
///        @p fillers, a phone's placed frames (`live_mobile.hpp`), fuse after
///        the set's cameras.
/// @return What `send` returns.
inline volumetric_kit::core::Result<
    volumetric_kit::recon::sender::SetSender::Sent>
send_set(volumetric_kit::recon::sender::SetSender& out,
         volumetric_kit::core::CommandBatch& front,
         const volumetric_kit::sensor::DeviceFrameSet& prepared,
         const std::vector<volumetric_kit::recon::sender::Filler>& fillers,
         volumetric_kit::sensor::FrameSet&& set, std::size_t index,
         std::uint64_t pts_ns,
         volumetric_kit::core::StageMetrics* rows = nullptr,
         const std::function<void()>& sent = {}) {
  namespace sensor = volumetric_kit::sensor;
  const std::vector<volumetric_kit::core::TimelinePoint> ready =
      sensor::ready_points(set.frames);
  return out.send(front, prepared.frames, fillers, ready,
                  std::make_shared<sensor::FrameSet>(std::move(set)), index,
                  pts_ns, rows, sent);
}

/// @brief @ref send_set with no fillers.
inline volumetric_kit::core::Result<
    volumetric_kit::recon::sender::SetSender::Sent>
send_set(volumetric_kit::recon::sender::SetSender& out,
         volumetric_kit::core::CommandBatch& front,
         const volumetric_kit::sensor::DeviceFrameSet& prepared,
         volumetric_kit::sensor::FrameSet&& set, std::size_t index,
         std::uint64_t pts_ns,
         volumetric_kit::core::StageMetrics* rows = nullptr,
         const std::function<void()>& sent = {}) {
  return send_set(out, front, prepared, {}, std::move(set), index, pts_ns, rows,
                  sent);
}

}  // namespace vr_example
