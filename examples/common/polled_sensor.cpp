// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "polled_sensor.hpp"

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <utility>
#include <vector>

#include "volumetric_kit/core/base/check.hpp"
#include "volumetric_kit/core/base/result.hpp"

namespace vr_example {

namespace vkc = volumetric_kit::core;

namespace {

class PolledInBackground final : public sensor::IRgbdSensor {
 public:
  explicit PolledInBackground(std::unique_ptr<sensor::IRgbdSensor> inner)
      : inner_(std::move(inner)) {}
  ~PolledInBackground() override { stop(); }
  PolledInBackground(const PolledInBackground&) = delete;
  PolledInBackground& operator=(const PolledInBackground&) = delete;

  const sensor::SensorInfo& info() const noexcept override {
    return inner_->info();
  }
  vkc::Status set_queue_depth(std::size_t frames) override {
    if (frames == 0 || thread_.joinable()) {
      return vkc::Status::invalid_argument(
          "polled_in_background: set_queue_depth takes 1 or more, before the "
          "start");
    }
    depth_ = frames;
    return {};
  }
  // The inner sensor is started here and then used by the polling thread
  // alone until stop joins it: one thread at a time, as it requires.
  vkc::Status start() override {
    if (thread_.joinable()) return {};
    VKC_TRY(inner_->start());
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = false;
      ended_ = false;
      error_ = {};
      queue_.clear();
      dropped_ = 0;
      stats_ = inner_->stats();
    }
    thread_ = std::thread([this] { run(); });
    return {};
  }
  void stop() noexcept override {
    if (!thread_.joinable()) return;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    idle_.notify_all();
    thread_.join();
    inner_->stop();
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.clear();
  }
  vkc::Result<std::optional<sensor::RgbdFrame>> poll() override {
    std::lock_guard<std::mutex> lock(mutex_);
    VKC_TRY(take_error());
    if (queue_.empty()) return std::optional<sensor::RgbdFrame>{};
    dropped_ += queue_.size() - 1;  // the newest only
    sensor::RgbdFrame f = std::move(queue_.back());
    queue_.clear();
    ++delivered_;
    return std::optional<sensor::RgbdFrame>(std::move(f));
  }
  vkc::Status drain(std::vector<sensor::RgbdFrame>* out) override {
    VKC_CHECK(out != nullptr, "polled_in_background: drain into nothing");
    std::lock_guard<std::mutex> lock(mutex_);
    VKC_TRY(take_error());
    for (sensor::RgbdFrame& f : queue_) out->push_back(std::move(f));
    delivered_ += queue_.size();
    queue_.clear();
    return {};
  }
  bool exhausted() const noexcept override {
    std::lock_guard<std::mutex> lock(mutex_);
    return ended_ && queue_.empty() && error_.ok();
  }
  sensor::SensorStats stats() const noexcept override {
    std::lock_guard<std::mutex> lock(mutex_);
    sensor::SensorStats s = stats_;
    s.delivered = delivered_;
    s.dropped += dropped_;
    return s;
  }

 private:
  // An error the inner poll returned, once; then OK.
  vkc::Status take_error() {
    vkc::Status e = std::move(error_);
    error_ = {};
    return e;
  }

  void run() {
    for (;;) {
      vkc::Result<std::optional<sensor::RgbdFrame>> polled = inner_->poll();
      const bool ended = inner_->exhausted();
      std::unique_lock<std::mutex> lock(mutex_);
      stats_ = inner_->stats();
      if (!polled.ok()) {
        if (error_.ok()) error_ = polled.status();
      } else if (*polled) {
        // A full queue lets its oldest go, so the inner link is always read.
        while (queue_.size() >= depth_) {
          queue_.pop_front();
          ++dropped_;
        }
        queue_.push_back(std::move(**polled));
      } else if (ended) {
        ended_ = true;
        return;
      } else {
        // A poll that does not wait of its own: wait a little before the next.
        idle_.wait_for(lock, std::chrono::milliseconds(1),
                       [this] { return stopping_; });
      }
      if (stopping_) return;
    }
  }

  std::unique_ptr<sensor::IRgbdSensor> inner_;
  std::size_t depth_ = 1;
  mutable std::mutex mutex_;
  std::condition_variable idle_;
  std::deque<sensor::RgbdFrame> queue_;
  vkc::Status error_;
  sensor::SensorStats stats_{};
  std::uint64_t delivered_ = 0;
  std::uint64_t dropped_ = 0;
  bool stopping_ = false;
  bool ended_ = false;
  std::thread thread_;
};

}  // namespace

std::unique_ptr<sensor::IRgbdSensor> polled_in_background(
    std::unique_ptr<sensor::IRgbdSensor> sensor) {
  return std::make_unique<PolledInBackground>(std::move(sensor));
}

}  // namespace vr_example
