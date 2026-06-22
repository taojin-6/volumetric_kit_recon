// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// Minimal smoke test exercising the core tier without a test framework: it
// returns non-zero on any failed assertion so CTest reports it. This is the
// placeholder that the GoogleTest suite replaces once the backend tiers land.

#include <cstdio>
#include <string>

#include "volumetric_kit/recon/core/log.hpp"
#include "volumetric_kit/recon/core/math/vector_types.hpp"
#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/version.hpp"

namespace vr = volumetric_kit::recon;

namespace {

int g_failures = 0;

void check(bool cond, const char* what) {
  if (!cond) {
    std::fprintf(stderr, "FAIL: %s\n", what);
    ++g_failures;
  }
}

vr::Result<int> halve(int n) {
  if (n % 2 != 0) return vr::Status::invalid_argument("odd input");
  return n / 2;
}

vr::Status run_assign() {
  VR_ASSIGN(int h, halve(8));
  return h == 4 ? vr::Status{} : vr::Status::unsupported("unexpected");
}

}  // namespace

int main() {
  // Status basics.
  check(vr::Status{}.ok(), "default Status is ok");
  const vr::Status err = vr::Status::not_found("x");
  check(!err.ok(), "error Status is not ok");
  check(err.domain() == vr::Status::Code::NotFound, "domain preserved");
  check(vr::to_string(err.domain()) == "NotFound", "to_string(domain)");
  check(vr::Status::backend_error(42, "gpu").detail() == 42, "backend detail");

  // Result<T> success + failure.
  check(halve(10).ok() && halve(10).value() == 5, "Result success");
  check(!halve(7).ok(), "Result failure");

  // VR_ASSIGN propagation.
  check(run_assign().ok(), "VR_ASSIGN unwraps value");

  // POD math.
  const vr::Vec3f a{1.0f, 0.0f, 0.0f};
  const vr::Vec3f b{0.0f, 1.0f, 0.0f};
  check(vr::dot(a, b) == 0.0f, "dot of orthogonal vectors");
  check(vr::cross(a, b).z == 1.0f, "cross is right-handed");
  const vr::Mat4f id = vr::Mat4f::identity();
  const vr::Vec4f p = id * vr::Vec4f{2.0f, 3.0f, 4.0f, 1.0f};
  check(p.x == 2.0f && p.y == 3.0f && p.z == 4.0f, "identity transform");

  // Version is wired through CMake.
  check(vr::version_major() >= 0, "version_major");
  check(std::string(vr::version_string()) == "0.0.1", "version_string");

  // Logging seam does not crash.
  vr::log_message(vr::LogLevel::Info, "smoke test ran");

  if (g_failures == 0) {
    std::puts("recon_core smoke test passed");
    return 0;
  }
  std::fprintf(stderr, "%d checks failed\n", g_failures);
  return 1;
}
