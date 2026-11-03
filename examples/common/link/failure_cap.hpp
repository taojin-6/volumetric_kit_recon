// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/link/failure_cap.hpp
/// @brief When a live link's failures end its stream
///        (`RgbdStreamSensor::listen`): at @ref kLinkFailures within
///        @ref kLinkFailureWindow, so drops spread over a long take never add
///        up to an end.
///
/// @code
/// link::FailureCap cap;
/// if (cap.fail(std::chrono::steady_clock::now())) end_the_stream();
/// @endcode

#include <chrono>
#include <cstddef>
#include <deque>

namespace vr_example::link {

/// The failures within @ref kLinkFailureWindow that end a live stream.
inline constexpr std::size_t kLinkFailures = 3;
/// How long a live link's failure counts towards @ref kLinkFailures.
inline constexpr std::chrono::seconds kLinkFailureWindow{10};

/// @brief A live link's failures over a sliding window.
class FailureCap {
 public:
  using Clock = std::chrono::steady_clock;

  /// @brief A cap at @p max failures within @p window, which is positive.
  explicit FailureCap(std::size_t max = kLinkFailures,
                      Clock::duration window = kLinkFailureWindow)
      : max_(max), window_(window) {}

  /// @brief A failure at @p now, no earlier than the last one's.
  /// @return Whether it brings the failures within the window to the cap's
  ///         max: one a window or more before @p now no longer counts.
  bool fail(Clock::time_point now) {
    times_.push_back(now);
    while (now - times_.front() >= window_) times_.pop_front();
    return times_.size() >= max_;
  }

 private:
  std::size_t max_;
  Clock::duration window_;
  std::deque<Clock::time_point> times_;  // the failures within the window
};

}  // namespace vr_example::link
