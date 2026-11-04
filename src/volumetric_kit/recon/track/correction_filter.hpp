// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// One chain's judge (correction.hpp's gates): the chain's correction, the
// prior its next fit starts from, and its verdict on each fit. Internal: the
// Tracker is its only caller, and tests reach it through the Tracker. Host
// only, one thread.

#include <cstdint>
#include <optional>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/recon/core/math/vector_types.hpp"
#include "volumetric_kit/recon/track/correction.hpp"
#include "volumetric_kit/recon/track/fitter.hpp"

namespace volumetric_kit::recon::track::detail {

// OK, or InvalidArgument naming the first field out of range.
core::Status check_config(const CorrectionFilterConfig& config,
                          const char* who);

class CorrectionFilter {
 public:
  CorrectionFilter(const CorrectionFilterConfig& config,
                   const Mat4f& initial) noexcept
      : config_(config), correction_(initial) {}

  // The next fit's seed and prior: the chain's correction, its sigma grown
  // since the last Anchored fit; none before the first.
  PosePrior prior(std::uint64_t pts_ns) const noexcept;

  // Judge `fit`, which started from `prior` at `pts_ns`, and take it.
  Correction update(const TrackFit& fit, const PosePrior& prior,
                    std::uint64_t pts_ns);

  const Mat4f& correction() const noexcept { return correction_; }

 private:
  // Seconds since the last Anchored fit (0 before the first).
  double since_anchor(std::uint64_t pts_ns) const noexcept;

  CorrectionFilterConfig config_;
  Mat4f correction_;
  std::optional<std::uint64_t> anchored_ns_;  // the last Anchored fit's time
  // The last fit's proposal when its jump was refused; cleared by any other
  // verdict.
  std::optional<Mat4f> refused_jump_;
};

}  // namespace volumetric_kit::recon::track::detail
