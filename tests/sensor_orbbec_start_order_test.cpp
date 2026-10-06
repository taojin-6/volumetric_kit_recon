// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The order OrbbecRig starts a rig in: every secondary before the primary, and
// the rigs it refuses. No camera.

#include <cstdio>
#include <string>
#include <vector>

#include "rig_start_order.hpp"

namespace vkc = volumetric_kit::core;
namespace sensor = volumetric_kit::recon::sensor;
namespace orbbec = volumetric_kit::recon::sensor::orbbec;

#define CHECK(cond)                                                        \
  do {                                                                     \
    if (!(cond)) {                                                         \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      return 1;                                                            \
    }                                                                      \
  } while (0)

namespace {

int test_start_order() {
  using M = sensor::OrbbecSyncMode;
  const std::vector<std::string> sns = {"G", "A4", "N", "6G"};
  // The rig as wired: the primary is started last.
  const auto order = orbbec::rig_start_order(
      {M::SecondarySynced, M::SecondarySynced, M::Primary, M::SecondarySynced},
      sns);
  CHECK(order.ok());
  CHECK((order.value() == std::vector<std::size_t>{0, 1, 3, 2}));
  const auto unsupported = [&](std::vector<M> modes) {
    const auto r = orbbec::rig_start_order(modes, sns);
    if (r.ok()) return false;
    std::printf("  refused as expected: %s\n", r.status().message().c_str());
    return r.status().domain() == vkc::Status::Code::Unsupported;
  };
  CHECK(unsupported({M::SecondarySynced, M::SecondarySynced, M::SecondarySynced,
                     M::SecondarySynced}));  // no primary
  CHECK(unsupported(
      {M::Primary, M::SecondarySynced, M::Primary, M::SecondarySynced}));
  CHECK(unsupported(
      {M::Standalone, M::SecondarySynced, M::Primary, M::SecondarySynced}));
  return 0;
}

}  // namespace

int main() {
  if (test_start_order() != 0) return 1;
  std::printf("orbbec start order tests passed\n");
  return 0;
}
