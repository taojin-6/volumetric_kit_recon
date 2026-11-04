// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include <cmath>
#include <memory>
#include <string>

#include "point_to_plane.hpp"
#include "volumetric_kit/recon/track/fitter.hpp"

namespace volumetric_kit::recon::track {
namespace {

bool positive(float x) { return std::isfinite(x) && x > 0.0f; }

core::Status check_config(const FitterConfig& c) {
  const auto refuse = [](const char* field) {
    return core::Status::invalid_argument(std::string("create_fitter: ") +
                                          field + " is out of range");
  };
  const IcpConfig& icp = c.icp;
  for (const int n : icp.iterations) {
    if (n < 0 || n > 100) return refuse("icp.iterations");
  }
  if (icp.passes < 1 || icp.passes > 8) return refuse("icp.passes");
  for (const float g : icp.coarse_gates_m) {
    if (!positive(g)) return refuse("icp.coarse_gates_m");
  }
  if (!(std::isfinite(icp.gate_sigmas) && icp.gate_sigmas >= 0.0f)) {
    return refuse("icp.gate_sigmas");
  }
  if (!positive(icp.finest_gate_floor_m)) {
    return refuse("icp.finest_gate_floor_m");
  }
  if (!(icp.normal_cos > 0.0f && icp.normal_cos <= 1.0f)) {
    return refuse("icp.normal_cos");
  }
  if (!positive(icp.huber_sigmas)) return refuse("icp.huber_sigmas");
  if (!(std::isfinite(icp.model_sigma_m) && icp.model_sigma_m >= 0.0f)) {
    return refuse("icp.model_sigma_m");
  }
  if (!(icp.grazing_deg > 0.0f && icp.grazing_deg <= 90.0f)) {
    return refuse("icp.grazing_deg");
  }
  if (icp.edge_px < 0 || icp.edge_px > 4) return refuse("icp.edge_px");
  if (!(icp.min_relative_eigenvalue > 0.0f &&
        icp.min_relative_eigenvalue < 1.0f)) {
    return refuse("icp.min_relative_eigenvalue");
  }
  if (!positive(icp.prior_n_eff)) return refuse("icp.prior_n_eff");
  if (!positive(c.model_filter.min_weight)) {
    return refuse("model_filter.min_weight");
  }
  if (!(std::isfinite(c.see_through_m) && c.see_through_m >= 0.0f)) {
    return refuse("see_through_m");
  }
  return {};
}

}  // namespace

const char* to_string(FitKind kind) noexcept {
  switch (kind) {
    case FitKind::PointToPlane:
      return "point_to_plane";
  }
  return "unknown";
}

core::Result<std::unique_ptr<IFitter>> create_fitter(
    core::Device& device, core::Allocator& allocator,
    const FitterConfig& config) {
  VKC_TRY(check_config(config));
  switch (config.kind) {
    case FitKind::PointToPlane:
      return detail::create_point_to_plane(device, allocator, config);
  }
  return core::Status::invalid_argument(
      "create_fitter: a fit kind this version does not know (" +
      std::to_string(int(config.kind)) + ")");
}

}  // namespace volumetric_kit::recon::track
