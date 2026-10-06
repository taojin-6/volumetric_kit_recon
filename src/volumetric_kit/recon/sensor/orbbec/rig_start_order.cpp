// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "rig_start_order.hpp"

#include <optional>

namespace volumetric_kit::recon::sensor::orbbec {

core::Result<std::vector<std::size_t>> rig_start_order(
    const std::vector<OrbbecSyncMode>& modes,
    const std::vector<std::string>& serials) {
  std::vector<std::size_t> order;
  std::optional<std::size_t> primary;
  std::string roles;
  for (std::size_t i = 0; i < modes.size(); ++i) {
    roles += (i == 0 ? "" : ", ") + serials[i] + " " + to_string(modes[i]);
  }
  for (std::size_t i = 0; i < modes.size(); ++i) {
    if (modes[i] == OrbbecSyncMode::Primary) {
      if (primary) {
        return core::Status::unsupported(
            "OrbbecRig: more than one sync primary (" + roles + ")");
      }
      primary = i;
    } else if (waits_for_primary(modes[i])) {
      order.push_back(i);
    } else {
      return core::Status::unsupported(
          "OrbbecRig: camera " + serials[i] + " is " + to_string(modes[i]) +
          ", so it streams on its own clock rather than on the primary's "
          "trigger (" +
          roles + ")");
    }
  }
  if (!primary) {
    return core::Status::unsupported(
        "OrbbecRig: no sync primary, so nothing triggers the others (" + roles +
        ")");
  }
  order.push_back(*primary);
  return order;
}

}  // namespace volumetric_kit::recon::sensor::orbbec
