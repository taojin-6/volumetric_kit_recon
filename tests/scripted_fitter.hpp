// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// A fitter whose fits are scripted, for the tests of what drives fitters
// (track::Tracker, and later the mobile lane): it records nothing into the
// batch and returns what its script makes of each record's frames and prior.
// It keeps the IFitter contract the tracker relies on -- one record
// outstanding, a finish only of the last record, an abandon that drops it, a
// refused finish that keeps it (scripted: refuse_next_finish) -- and logs
// each record's frames, prior and row names.

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/base/stage_metrics.hpp"
#include "volumetric_kit/recon/track/fitter.hpp"

namespace vr_test {

namespace track = volumetric_kit::recon::track;

// One record a scripted fitter took.
struct ScriptedRecord {
  std::vector<std::uint64_t> pts;  // the frames' timestamps
  track::PosePrior prior;
  std::string total_row;  // the rows' names it was given
  std::string icp_row;
  bool finished = false;
  bool abandoned = false;
};

class ScriptedFitter final : public track::IFitter {
 public:
  // What a record returns: the fit, or a refusal (nothing recorded).
  using Script = std::function<volumetric_kit::core::Result<track::TrackFit>(
      const std::vector<volumetric_kit::sensor::DeviceFrame>&,
      const track::PosePrior&)>;

  // @param cost_ms  Host time each record adds to its total row, so two
  //                 chains' rows can be told apart.
  explicit ScriptedFitter(Script script, double cost_ms = 0.0)
      : script_(std::move(script)),
        cost_ms_(cost_ms),
        log_(std::make_shared<std::vector<ScriptedRecord>>()) {}

  // Every record, oldest first; it outlives the fitter.
  std::shared_ptr<const std::vector<ScriptedRecord>> log() const {
    return log_;
  }

  volumetric_kit::core::Result<track::PendingFit> record(
      volumetric_kit::core::CommandBatch&,
      const volumetric_kit::grid::volume::VoxelBlockGrid&,
      const std::vector<volumetric_kit::sensor::DeviceFrame>& frames,
      const track::PosePrior& prior,
      volumetric_kit::core::StageMetrics* metrics,
      const track::FitRows& rows) override {
    if (pending_ != 0) {
      return volumetric_kit::core::Status::invalid_argument(
          "ScriptedFitter: a record is outstanding");
    }
    volumetric_kit::core::Result<track::TrackFit> fit = script_(frames, prior);
    if (!fit.ok()) return fit.status();
    ScriptedRecord rec;
    for (const auto& f : frames) rec.pts.push_back(f.timestamp_ns);
    rec.prior = prior;
    rec.total_row = rows.total;
    rec.icp_row = rows.icp;
    log_->push_back(std::move(rec));
    if (metrics != nullptr) {
      metrics->add_cpu(rows.total, cost_ms_);
      metrics->add_cpu(rows.icp, cost_ms_);
    }
    fit_ = *fit;
    pending_ = next_++;
    return track::PendingFit{pending_};
  }

  volumetric_kit::core::Result<track::TrackFit> finish(
      const track::PendingFit& pending) override {
    if (pending_ == 0 || pending.serial != pending_) {
      return volumetric_kit::core::Status::invalid_argument(
          "ScriptedFitter: not the last record");
    }
    if (refuse_finish_) {
      // As PointToPlane refuses a readback that never came: the record stays
      // outstanding until an abandon.
      refuse_finish_ = false;
      return volumetric_kit::core::Status::invalid_argument(
          "ScriptedFitter: a scripted finish refusal");
    }
    pending_ = 0;
    log_->back().finished = true;
    return fit_;
  }

  void abandon(const track::PendingFit& pending) noexcept override {
    if (pending_ == 0 || pending.serial != pending_) return;
    pending_ = 0;
    log_->back().abandoned = true;
  }

  // Whether a record is outstanding.
  bool outstanding() const noexcept { return pending_ != 0; }
  // The next finish of the last record refuses and keeps the record.
  void refuse_next_finish() noexcept { refuse_finish_ = true; }

 private:
  Script script_;
  double cost_ms_ = 0.0;
  bool refuse_finish_ = false;
  std::shared_ptr<std::vector<ScriptedRecord>> log_;
  track::TrackFit fit_;
  std::uint64_t pending_ = 0;
  std::uint64_t next_ = 1;
};

// A fit the judge anchors: every direction strong, ample inliers.
inline track::TrackFit anchored_fit(
    const volumetric_kit::recon::Mat4f& correction) {
  track::TrackFit fit;
  fit.correction = correction;
  fit.lambda_max = 1.0f;
  fit.relative_eigenvalues = {1, 1, 1, 1, 1, 1};
  for (int i = 0; i < 6; ++i) fit.directions[std::size_t(7 * i)] = 1.0f;
  fit.held = 0;
  fit.frames = 1;
  fit.attempts = 20000;
  fit.hits = 18000;
  fit.inliers = 17000;
  return fit;
}

// Anchored at `at` when `when(the newest frame's pts)`, else nothing solved.
inline ScriptedFitter::Script anchors_when(
    std::function<bool(std::uint64_t)> when, volumetric_kit::recon::Mat4f at) {
  return [when = std::move(when), at](
             const std::vector<volumetric_kit::sensor::DeviceFrame>& frames,
             const track::PosePrior& prior)
             -> volumetric_kit::core::Result<track::TrackFit> {
    if (when(frames.back().timestamp_ns)) return anchored_fit(at);
    track::TrackFit none;
    none.correction = prior.correction;
    return none;
  };
}

}  // namespace vr_test
