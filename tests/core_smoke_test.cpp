// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// Minimal smoke test exercising the core tier without a test framework: it
// returns non-zero on any failed assertion so CTest reports it. This is the
// placeholder that the GoogleTest suite replaces once the backend tiers land.

#include <cstdio>
#include <string>
#include <string_view>

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

  // Result<T> success + failure (one Result, checked for both ok and value).
  const vr::Result<int> five = halve(10);
  check(five.ok() && five.value() == 5, "Result success");
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

  // Non-symmetric transform: a column-major translation by (10,20,30) carries
  // the offset in the last *column* (m[12..14]). Testing it distinguishes a
  // correct column-major matrix*vector from a transposed (row-major) one -- the
  // identity case above cannot, since identity is symmetric.
  vr::Mat4f t = vr::Mat4f::identity();
  t.m[12] = 10.0f;
  t.m[13] = 20.0f;
  t.m[14] = 30.0f;
  const vr::Vec4f tp = t * vr::Vec4f{1.0f, 2.0f, 3.0f, 1.0f};
  check(tp.x == 11.0f && tp.y == 22.0f && tp.z == 33.0f && tp.w == 1.0f,
        "column-major translation transform");

  // Matrix*matrix: composing two translations sums their offsets. Exercises the
  // index arithmetic in operator*(Mat4f, Mat4f), which no identity test covers.
  vr::Mat4f t2 = vr::Mat4f::identity();
  t2.m[12] = 1.0f;
  t2.m[13] = 1.0f;
  t2.m[14] = 1.0f;
  const vr::Vec4f cp = (t * t2) * vr::Vec4f{0.0f, 0.0f, 0.0f, 1.0f};
  check(cp.x == 11.0f && cp.y == 21.0f && cp.z == 31.0f,
        "composed translations");

  // Version is wired through CMake: the integer accessors must compose to the
  // string form. This exercises all three accessors -- a swapped
  // MAJOR/MINOR/PATCH is caught, since version_string() comes from the
  // independent PROJECT_VERSION macro -- without hard-coding a literal that a
  // version bump would invalidate.
  const std::string composed = std::to_string(vr::version_major()) + "." +
                               std::to_string(vr::version_minor()) + "." +
                               std::to_string(vr::version_patch());
  check(composed == vr::version_string(),
        "version components compose to version_string");

  // Logging seam: an installed handler receives every level (including Info,
  // which the default sink drops); restoring the empty handler falls back to the
  // default sink.
  vr::LogLevel seen_level = vr::LogLevel::Error;
  std::string seen_message;
  int seen_count = 0;
  vr::set_log_handler([&](vr::LogLevel level, std::string_view message) {
    seen_level = level;
    seen_message.assign(message.data(), message.size());
    ++seen_count;
  });
  vr::log_message(vr::LogLevel::Info, "smoke test ran");
  check(seen_count == 1 && seen_level == vr::LogLevel::Info &&
            seen_message == "smoke test ran",
        "installed log handler receives the message");
  vr::set_log_handler({});  // restore the default sink
  vr::log_message(vr::LogLevel::Info, "after reset");
  check(seen_count == 1, "empty handler restores default sink (Info dropped)");

  if (g_failures == 0) {
    std::puts("recon_core smoke test passed");
    return 0;
  }
  std::fprintf(stderr, "%d checks failed\n", g_failures);
  return 1;
}
