// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "correction_filter.hpp"

#include <cmath>
#include <string>

#include "se3.hpp"

namespace volumetric_kit::recon::track::detail {
namespace {

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
// A jump the fit before proposed too, within this, is taken.
constexpr double kRepeatM = 0.02;
constexpr double kRepeatDeg = 0.5;

bool non_negative(float x) { return std::isfinite(x) && x >= 0.0f; }
bool unit(float x) { return std::isfinite(x) && x >= 0.0f && x <= 1.0f; }

// The fit's step from prior to fit, kept along the directions at or above
// `min_relative` only.
Mat4f strong_directions(const Mat4f& prior, const TrackFit& fit,
                        float min_relative) {
  const Vec3d o(fit.origin);
  const double scale = fit.scale_m > 0.0f ? double(fit.scale_m) : 1.0;
  const Twist x = twist_between(prior, fit.correction, o);
  const double xs[6] = {x.v.x / scale, x.v.y / scale, x.v.z / scale,
                        x.w.x,         x.w.y,         x.w.z};
  double kept[6] = {0, 0, 0, 0, 0, 0};
  for (int i = 0; i < 6; ++i) {
    if (!(fit.relative_eigenvalues[std::size_t(i)] >= min_relative)) continue;
    const float* d = &fit.directions[std::size_t(6 * i)];
    double along = 0.0;
    for (int k = 0; k < 6; ++k) along += double(d[k]) * xs[k];
    for (int k = 0; k < 6; ++k) kept[k] += along * double(d[k]);
  }
  Twist y;
  y.v = Vec3d(kept[0], kept[1], kept[2]) * scale;
  y.w = Vec3d(kept[3], kept[4], kept[5]);
  return apply_twist(y, o, prior);
}

}  // namespace

core::Status check_config(const CorrectionFilterConfig& c, const char* who) {
  const auto refuse = [who](const char* field) {
    return core::Status::invalid_argument(std::string(who) + ": judge." +
                                          field + " is out of range");
  };
  if (!unit(c.min_coverage)) return refuse("min_coverage");
  if (!unit(c.min_inlier_ratio)) return refuse("min_inlier_ratio");
  if (!(c.partial_min_relative_eigenvalue > 0.0f &&
        c.partial_min_relative_eigenvalue < 1.0f)) {
    return refuse("partial_min_relative_eigenvalue");
  }
  if (!non_negative(c.jump_m)) return refuse("jump_m");
  if (!non_negative(c.jump_deg)) return refuse("jump_deg");
  if (!non_negative(c.jump_growth_m_s)) return refuse("jump_growth_m_s");
  if (!non_negative(c.jump_growth_deg_s)) return refuse("jump_growth_deg_s");
  if (!non_negative(c.prior_sigma_m)) return refuse("prior_sigma_m");
  if (!non_negative(c.prior_sigma_deg)) return refuse("prior_sigma_deg");
  if (!non_negative(c.prior_growth_m_s)) return refuse("prior_growth_m_s");
  if (!non_negative(c.prior_growth_deg_s)) return refuse("prior_growth_deg_s");
  if (!non_negative(c.see_through_m)) return refuse("see_through_m");
  return {};
}

double CorrectionFilter::since_anchor(std::uint64_t pts_ns) const noexcept {
  if (!anchored_ns_ || pts_ns <= *anchored_ns_) return 0.0;
  return double(pts_ns - *anchored_ns_) * 1e-9;
}

PosePrior CorrectionFilter::prior(std::uint64_t pts_ns) const noexcept {
  PosePrior p;
  p.correction = correction_;
  if (!anchored_ns_) return p;  // no prior before the first lock
  const double t = since_anchor(pts_ns);
  p.sigma_m = static_cast<float>(double(config_.prior_sigma_m) +
                                 double(config_.prior_growth_m_s) * t);
  p.sigma_rad = static_cast<float>((double(config_.prior_sigma_deg) +
                                    double(config_.prior_growth_deg_s) * t) *
                                   kDegToRad);
  return p;
}

Correction CorrectionFilter::update(const TrackFit& fit, const PosePrior& prior,
                                    std::uint64_t pts_ns) {
  if (!fit.solved()) {
    refused_jump_.reset();
    return {correction_, TrackState::Coasting};
  }
  const auto reject = [this] {
    refused_jump_.reset();
    return Correction{correction_, TrackState::Rejected};
  };
  // Conditioning first: a held direction caps the fit at Partial, which
  // proposes its step along the strong directions only.
  const bool conditioned = (fit.held & 0x3fu) == 0u;
  const Mat4f proposal =
      conditioned ? fit.correction
                  : strong_directions(prior.correction, fit,
                                      config_.partial_min_relative_eigenvalue);
  // The see-through check: hits far off the model's surface leave the
  // coverage and the ratio.
  std::uint64_t off = 0;
  if (config_.see_through_m > 0.0f) {
    off = std::uint64_t(fit.in_front) + fit.behind;
    if (off > fit.hits) off = fit.hits;
  }
  const double attempts = double(fit.attempts) - double(off);
  const double hits = double(fit.hits) - double(off);
  if (fit.inliers < config_.min_inliers) return reject();
  if (!(attempts > 0.0) || hits / attempts < double(config_.min_coverage)) {
    return reject();
  }
  if (!(hits > 0.0) ||
      double(fit.inliers) / hits < double(config_.min_inlier_ratio)) {
    return reject();
  }
  if (anchored_ns_) {
    const double t = since_anchor(pts_ns);
    const double gate_m =
        double(config_.jump_m) + double(config_.jump_growth_m_s) * t;
    const double gate_deg =
        double(config_.jump_deg) + double(config_.jump_growth_deg_s) * t;
    const Vec3d o(fit.origin);
    double m = 0.0;
    double deg = 0.0;
    motion_at(correction_, proposal, o, m, deg);
    if (m > gate_m || deg > gate_deg) {
      bool repeated = false;
      if (refused_jump_) {
        double rm = 0.0;
        double rdeg = 0.0;
        motion_at(*refused_jump_, proposal, o, rm, rdeg);
        repeated = rm < kRepeatM && rdeg < kRepeatDeg;
      }
      if (!repeated) {
        refused_jump_ = proposal;
        return {correction_, TrackState::Rejected};
      }
    }
  }
  refused_jump_.reset();
  correction_ = proposal;
  if (conditioned) {
    anchored_ns_ = pts_ns;
    return {correction_, TrackState::Anchored};
  }
  return {correction_, TrackState::Partial};
}

}  // namespace volumetric_kit::recon::track::detail
