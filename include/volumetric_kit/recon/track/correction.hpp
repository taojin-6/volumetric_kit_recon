// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file track/correction.hpp
/// @brief A chain's correction, its verdict on each fit, and the judge's
///        gates. The judge itself is the @ref Tracker's: one per chain.

#include <cstdint>

#include "volumetric_kit/recon/core/math/vector_types.hpp"

namespace volumetric_kit::recon::track {

/// @brief How a chain's judge took a fit.
enum class TrackState : std::uint8_t {
  /// Every gate passed and every direction constrained: the chain takes the
  /// fit's correction.
  Anchored = 0,
  /// Every gate passed, some direction weak: the chain moves along the
  /// directions at or above
  /// @ref CorrectionFilterConfig::partial_min_relative_eigenvalue only.
  Partial = 1,
  /// The fit solved nothing: the chain keeps its correction.
  Coasting = 2,
  /// A gate refused the fit: the chain keeps its correction.
  Rejected = 3,
};

/// @brief A chain's correction and its last verdict.
struct Correction {
  Mat4f correction{1.0f};
  TrackState state = TrackState::Coasting;
};

/// @brief One chain's judge, in 0b's gate order. A fit that solved nothing
///        is Coasting. Then conditioning: a fit with a held direction
///        (`TrackFit::held`) can be Partial at best, never Anchored, and
///        proposes its step along its strong directions only. Then the
///        inlier count, coverage and the inlier ratio over hits, and last
///        the jump of the proposed correction under the grown prior. A fit
///        any gate refuses is Rejected. Defaults are 0b's window gates [M];
///        `frame_method()` sets the single-frame ones.
///
/// Times are the fits' newest frames' timestamps. Before the chain's first
/// Anchored fit there is no prior and no jump gate.
struct CorrectionFilterConfig {
  std::uint32_t min_inliers = 5000;  ///< A fit with fewer is Rejected.
  /// hits / attempts, at least; in [0, 1].
  float min_coverage = 0.15f;
  /// inliers / hits, at least; in [0, 1].
  float min_inlier_ratio = 0.7f;
  /// A Partial fit moves only directions at or above this relative
  /// eigenvalue; in (0, 1).
  float partial_min_relative_eigenvalue = 1e-2f;
  /// The jump gate: a fit moving the chain's correction more than this at the
  /// frames' mean camera centre is Rejected, unless the fit before it
  /// proposed the same correction (within 2 cm / 0.5 deg) and was Rejected
  /// for its jump too. It grows at the rates below with the time since the
  /// last Anchored fit.
  float jump_m = 0.03f;
  float jump_deg = 1.0f;           ///< As @ref jump_m, the rotation.
  float jump_growth_m_s = 0.02f;   ///< Per second since the last Anchored fit.
  float jump_growth_deg_s = 0.3f;  ///< As @ref jump_growth_m_s.
  /// The prior's sigma at an Anchored fit; 0 holds nothing.
  float prior_sigma_m = 0.005f;
  float prior_sigma_deg = 0.3f;  ///< As @ref prior_sigma_m, the rotation.
  /// The prior's sigma grows at these per second since the last Anchored
  /// fit, as the device's pose drifts.
  float prior_growth_m_s = 0.008f;
  float prior_growth_deg_s = 0.3f;  ///< As @ref prior_growth_m_s.
  /// The see-through check (metres): above 0, hits more than this in front
  /// of or behind the model's surface (`TrackFit::in_front`,
  /// `TrackFit::behind`) leave the coverage and the inlier ratio, so a
  /// person in view or an object that left does not refuse the fit. 0 (the
  /// default) is off. The one home of the distance: `create_fitters` gives
  /// it to the chain's fitter, and the `Tracker` refuses a fit that counted
  /// at another (`TrackFit::see_through_m`).
  float see_through_m = 0.0f;
};

}  // namespace volumetric_kit::recon::track
