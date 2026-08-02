// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/viewer/stage_metrics.hpp
/// @brief Wall-clock timing of the viewer's pipeline stages, published in the
///        renderer's @ref volumetric_kit::gfx::FrameMetrics shape.
///
/// The viewer already links both siblings, so it fills gfx's plain-data metrics
/// contract directly from recon-side timings rather than introducing a shared
/// metrics package or a recon->gfx dependency. recon's compute tiers carry no
/// profiler of their own; every recon dispatch goes through
/// `Device::submit_single_time`, which blocks on a fence, so the wall-clock
/// span around a call is that stage's true end-to-end cost -- host record
/// *plus* GPU execution. Those spans are therefore reported as CPU time with
/// `has_gpu` false: honest under the contract (`cpu_ms` is documented as
/// wall-clock), and not a claim of a separate device measurement.
///
/// TODO(examples): split host from device time once recon can write timestamp
/// queries into the dispatch command buffer (the device reports 64
/// `timestampValidBits` through MoltenVK on Apple silicon, so real GPU spans
/// are available -- it needs a query pool plumbed through
/// `submit_single_time`).

#include <chrono>
#include <cstring>
#include <vector>

#include "volumetric_kit/gfx/core/frame_metrics.hpp"

namespace fuse_viewer {

namespace vg = volumetric_kit::gfx;

/// @brief A set of named wall-clock spans, shaped as @ref
///        vg::FrameMetrics::Section rows.
///
/// Repeat calls under one @p name accumulate into a single row, so a stage that
/// runs more than once for a frame (an allocate retried across a map resize)
/// reports its total rather than several rows or only the last.
class StageTimes {
 public:
  /// @brief Drop every recorded span, keeping the storage for reuse.
  void clear() noexcept { sections_.clear(); }

  /// @brief Add @p milliseconds to @p name's row, creating it if new.
  ///
  /// Rows are matched by string *content*, not pointer: seeding a row (see
  /// @ref seed) and timing it name the same stage from two places in the
  /// source, and identical string literals are not guaranteed to be pooled into
  /// one address.
  ///
  /// @param name          Stage label; must still have string-literal lifetime,
  ///                      which @ref vg::FrameMetrics::Section requires -- it
  ///                      stores the pointer rather than a copy.
  /// @param milliseconds  The span to add.
  void add(const char* name, double milliseconds) {
    for (vg::FrameMetrics::Section& section : sections_) {
      if (std::strcmp(section.name, name) == 0) {
        section.cpu_ms += milliseconds;
        return;
      }
    }
    sections_.push_back({name, milliseconds, 0.0, false});
  }

  /// @brief Create @p name's row at zero if it does not exist yet.
  ///
  /// Seeding every stage a frame *might* run keeps the overlay's table a stable
  /// shape: a stage that did not run this frame (the mesh extract on a frame
  /// between remeshes) reports 0.000 rather than dropping its row and making
  /// the rows below it jump.
  void seed(const char* name) { add(name, 0.0); }

  /// @return The recorded rows, in first-seen order.
  const std::vector<vg::FrameMetrics::Section>& sections() const noexcept {
    return sections_;
  }

  /// @return The sum of every row's CPU milliseconds.
  double total_ms() const noexcept {
    double total = 0.0;
    for (const vg::FrameMetrics::Section& section : sections_) {
      total += section.cpu_ms;
    }
    return total;
  }

 private:
  std::vector<vg::FrameMetrics::Section> sections_;
};

/// @brief Times its own scope into a @ref StageTimes row.
///
/// @warning The @ref StageTimes and the @p name literal must outlive the scope.
class StageScope {
 public:
  /// @brief Start timing @p name into @p times.
  StageScope(StageTimes& times, const char* name)
      : times_(times), name_(name), start_(Clock::now()) {}

  /// @brief Stop timing and add the elapsed span to the row.
  ~StageScope() {
    times_.add(name_,
               std::chrono::duration<double, std::milli>(Clock::now() - start_)
                   .count());
  }

  StageScope(const StageScope&) = delete;
  StageScope& operator=(const StageScope&) = delete;
  StageScope(StageScope&&) = delete;
  StageScope& operator=(StageScope&&) = delete;

 private:
  using Clock = std::chrono::steady_clock;

  StageTimes& times_;
  const char* name_;
  Clock::time_point start_;
};

}  // namespace fuse_viewer
