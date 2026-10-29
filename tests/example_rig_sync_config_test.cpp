// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// config/femto_mega_sync.json, the lab rig's sync file that the Orbbec
// examples' configurations name: the sensor package's reader takes it, it has
// one primary, and it staggers every secondary's depth delay so no two ToF
// exposures overlap. No camera.

#include <cstdio>
#include <vector>

#include "volumetric_kit/core/testing/test_check.hpp"
#include "volumetric_kit/sensor/orbbec/orbbec_sync_config.hpp"

namespace sensor = volumetric_kit::sensor;

int main() {
  const auto r = sensor::read_orbbec_sync_config(VR_RIG_SYNC_CONFIG);
  if (!r.ok()) std::fprintf(stderr, "%s\n", r.status().message().c_str());
  CHECK(r.ok());
  int primaries = 0;
  std::vector<int> delays;
  for (const sensor::OrbbecSyncDevice& d : r.value().devices) {
    if (d.sync.mode == sensor::OrbbecSyncMode::Primary) {
      ++primaries;
    } else {
      CHECK(sensor::waits_for_primary(d.sync.mode));
      for (const int other : delays) CHECK(other != d.sync.depth_delay_us);
      delays.push_back(d.sync.depth_delay_us);
    }
  }
  CHECK(primaries == 1 && !delays.empty());
  std::printf("rig sync config: %d primary, %zu secondaries\n", primaries,
              delays.size());
  return 0;
}
