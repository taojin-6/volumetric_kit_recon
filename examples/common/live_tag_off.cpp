// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The origin tag's watch (live_tag.hpp) in a build without VR_WITH_CALIB:
// it refuses, and no watch is ever made.

#include "live_tag.hpp"

namespace vr_example {

struct TagWatch::State {
  Counts counts;
};

vkc::Status TagWatch::check(const sensor::camera::WorldFrame&) {
  return vkc::Status::unsupported(
      "TagWatch: this build registers no phone by the origin tag: build with "
      "-DVR_WITH_CALIB=ON, or give source.mobile_registration");
}

vkc::Result<std::unique_ptr<TagWatch>> TagWatch::create(const vkc::Device&,
                                                        vkc::Allocator&,
                                                        Options options) {
  return check(options.world);
}

TagWatch::TagWatch(std::unique_ptr<State> state) noexcept
    : state_(std::move(state)) {}
TagWatch::~TagWatch() = default;
vkc::Status TagWatch::see(const sensor::RgbdFrame&) { return check({}); }
void TagWatch::split() noexcept {}
vkc::Result<TagNews> TagWatch::take() { return check({}); }
const TagWatch::Counts& TagWatch::counts() const noexcept {
  return state_->counts;
}

}  // namespace vr_example
