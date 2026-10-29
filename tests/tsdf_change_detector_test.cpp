// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// tsdf::ChangeDetector on synthetic depth sequences, a camera slot each,
// every slot tested in one batch a frame and its mask read back. White
// Gaussian noise in time, independent per pixel, at the sensor's noise
// floor; changes start at frame 150, after the histories have settled.
//
// - Static ToF-like noise (sigma = 0.95 mm per metre, a plane at 2 m) and
//   stereo-like noise (sigma growing as z^2 over a slanted plane from 1 to
//   3 m), each pixel's noise scattered around the floor as the Femto's is:
//   false alarms near the rate measured on the rig, the same at 1 m and 3 m.
// - Steps of 5, 10, 20 and 40 mm, either way: frames to the first alarm as
//   measured on the rig's interior pixels, and the alarm's direction.
// - Ramps of 1, 3 and 10 mm a frame: the first alarm as measured.
// - A foreground 0.5 m nearer sweeping in at a pixel a frame, its boundary
//   column a mixed sample: a clean pixel alarms on the frame the sweep
//   reaches it or the next.
// - Dropouts (zero, NaN, infinity, out of range) of pixels valid from the
//   camera's start change nothing: a camera given 20 invalid frames flags
//   none of them and then matches, mask for mask, a twin that skipped them.
// - A camera that leaves and returns compares with its own history; one whose
//   image changes size starts a new history.
// - Over no return (pixels first valid once their camera tests): a mover's
//   departure flags Vanished, with its depth, whatever alarms it raised
//   while there; a sporadic return of two samples ends silently, one of
//   three flags; and such a pixel tests from its second sample, as the
//   warm-up is the camera's, not the pixel's.
// - A closed gate word tests nothing: a 5 cm step behind it is flagged on
//   the next frame the gate opens, as the history never saw it; the word is
//   found at its offset, and a bad one is refused.
// - The refusals, and move construction.
// Skips where no device is present.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <random>
#include <string_view>
#include <utility>
#include <vector>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/testing/test_check.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/command_batch.hpp"
#include "volumetric_kit/core/vulkan/compute_util.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"
#include "volumetric_kit/recon/tsdf/change_detector.hpp"

#include "volumetric_kit/core/testing/buffer_readback.hpp"
#include "volumetric_kit/core/testing/gpu_test.hpp"

namespace vr = volumetric_kit::recon;
namespace sensor = volumetric_kit::sensor;
namespace vkc = volumetric_kit::core;
namespace tsdf = volumetric_kit::recon::tsdf;
namespace vol = volumetric_kit::grid::volume;

namespace {

constexpr std::uint32_t kSide = 128;
constexpr std::uint32_t kPixels = kSide * kSide;
constexpr int kFrames = 300;
constexpr int kOnset = 150;  // changes start here
constexpr int kWindow = 60;  // and are watched this long
constexpr int kWarmUp = 30;  // the default warm-up, in samples
constexpr float kFps = 30.0f;

// The slots, one scenario each.
enum Slot : std::uint32_t {
  kTof = 0,  // static, sigma = 0.95 mm/m, plane at 2 m
  kStereo,   // static, sigma = c z^2, slanted plane 1 to 3 m
  kSteps,    // steps of +-5/10/20/40 mm at the onset
  kRamps,    // ramps of +-1/3/10 mm a frame from the onset
  kSweep,    // a foreground sweeping in from the right
  kDropout,  // invalid frames 100-119
  kTwin,     // kDropout's twin, skipping those frames
  kLeaver,   // absent frames 100-149, then back, then a 5 cm step
  kResized,  // 64 x 64 from frame 100
  kSlots
};

constexpr float kStereoC = 0.002f;  // stereo-like: 2 mm at 1 m

// Deterministic on every standard library: mt19937 is specified, and the
// Gaussian is our own Box-Muller.
class Noise {
 public:
  explicit Noise(std::uint32_t seed) : rng_(seed) {}
  float uniform() {
    return static_cast<float>((double(rng_()) + 0.5) / 4294967296.0);
  }
  float gauss() {
    if (spare_) {
      spare_ = false;
      return next_;
    }
    const double u = (double(rng_()) + 0.5) / 4294967296.0;
    const double v = (double(rng_()) + 0.5) / 4294967296.0;
    const double r = std::sqrt(-2.0 * std::log(u));
    next_ = static_cast<float>(r * std::sin(6.283185307179586 * v));
    spare_ = true;
    return static_cast<float>(r * std::cos(6.283185307179586 * v));
  }

 private:
  std::mt19937 rng_;
  bool spare_ = false;
  float next_ = 0.0f;
};

sensor::DepthCameraParams camera(std::uint32_t side) {
  sensor::DepthCameraParams cam{};
  cam.fx = cam.fy = 100.0f;
  cam.cx = cam.cy = 0.5f * float(side - 1);
  cam.min_depth = 0.25f;
  cam.max_depth = 5.0f;
  cam.width = side;
  cam.height = side;
  return cam;
}

float tof_sigma(float z) { return 0.00095f * z; }

// Each pixel's noise around the floor, as the rig's interior pixels scatter
// around the fitted model (p10 0.79x, p90 1.53x): lognormal.
std::vector<float> scatter(Noise& noise) {
  std::vector<float> s(kPixels);
  for (float& x : s) x = std::exp(0.095f + 0.258f * noise.gauss());
  return s;
}

// The step and ramp groups: pixel i's size and sign.
constexpr std::array<float, 4> kStepsMm = {5.0f, 10.0f, 20.0f, 40.0f};
constexpr std::array<float, 3> kRampsMm = {1.0f, 3.0f, 10.0f};
int group_of(std::uint32_t i, std::size_t groups) {
  return static_cast<int>(i % (2 * groups));  // size i/2, sign i%2
}

// The sweep: from the onset, columns past c(t) = side - 1 - (t - onset) are
// covered by a foreground 0.5 m nearer; column c(t) is the boundary, a mixed
// sample between the two.
int sweep_column(int t) { return int(kSide) - 1 - (t - kOnset); }

// Every slot's frame t.
struct Scene {
  Noise noise{20261029u};
  std::vector<float> tof_scale;
  std::vector<float> stereo_scale;
  std::vector<float> step_scale;

  Scene() : tof_scale(scatter(noise)), stereo_scale(scatter(noise)) {}

  float stereo_plane(std::uint32_t x) const {
    return 1.0f + 2.0f * float(x) / float(kSide - 1);
  }

  std::vector<float> frame(Slot slot, int t, std::uint32_t side = kSide) {
    std::vector<float> d(std::size_t(side) * side);
    for (std::uint32_t i = 0; i < d.size(); ++i) {
      const std::uint32_t x = i % side;
      const float g = noise.gauss();
      switch (slot) {
        case kTof:
          d[i] = 2.0f + tof_sigma(2.0f) * tof_scale[i] * g;
          break;
        case kStereo: {
          const float z = stereo_plane(x);
          d[i] = z + kStereoC * z * z * stereo_scale[i] * g;
          break;
        }
        case kSteps: {
          const int grp = group_of(i, kStepsMm.size());
          float off = 0.0f;
          if (t >= kOnset) {
            off = 0.001f * kStepsMm[std::size_t(grp / 2)] *
                  (grp % 2 == 0 ? 1.0f : -1.0f);
          }
          d[i] = 2.0f + off + tof_sigma(2.0f) * g;
          break;
        }
        case kRamps: {
          const int grp = group_of(i, kRampsMm.size());
          float off = 0.0f;
          if (t >= kOnset) {
            off = 0.001f * kRampsMm[std::size_t(grp / 2)] *
                  float(t - kOnset + 1) * (grp % 2 == 0 ? 1.0f : -1.0f);
          }
          d[i] = 2.0f + off + tof_sigma(2.0f) * g;
          break;
        }
        case kSweep: {
          const float back = 2.0f + tof_sigma(2.0f) * g;
          const int c = sweep_column(t);
          if (t < kOnset || int(x) < c) {
            d[i] = back;
          } else if (int(x) == c) {
            d[i] = back - 0.5f * noise.uniform();
          } else {
            d[i] = back - 0.5f;
          }
          break;
        }
        case kDropout:
        case kTwin:
        case kLeaver:
        case kResized:
          d[i] = 2.0f + tof_sigma(2.0f) * g;
          break;
        case kSlots:
          break;
      }
    }
    return d;
  }
};

// What every slot's mask said, a byte a pixel a frame (frames a slot was not
// tested hold 0xFF).
using Masks = std::vector<std::vector<std::uint8_t>>;

std::uint8_t at(const Masks& m, Slot slot, int t, std::uint32_t i) {
  return m[std::size_t(slot)][std::size_t(t) * kPixels + i];
}

constexpr std::uint8_t kNone = 0;
constexpr std::uint8_t kNearer = 1;
constexpr std::uint8_t kFarther = 2;
constexpr std::uint8_t kUntested = 0xFF;

// Frames from the onset to the first alarm, or -1 for none in the window.
int first_alarm(const Masks& m, Slot slot, std::uint32_t i,
                std::uint8_t* kind = nullptr) {
  for (int t = kOnset; t < kOnset + kWindow; ++t) {
    const std::uint8_t a = at(m, slot, t, i);
    if (a != kNone && a != kUntested) {
      if (kind != nullptr) *kind = a;
      return t - kOnset;
    }
  }
  return -1;
}

struct Delays {
  std::vector<int> found;
  std::size_t missed = 0;
  std::size_t wrong_way = 0;
  double share(int at_most) const {
    const std::size_t n = found.size() + missed;
    return double(std::count_if(found.begin(), found.end(),
                                [&](int d) { return d <= at_most; })) /
           double(n);
  }
  int quantile(double q) const {
    std::vector<int> all = found;
    all.insert(all.end(), missed, std::numeric_limits<int>::max());
    std::sort(all.begin(), all.end());
    return all[std::min(all.size() - 1, std::size_t(q * double(all.size())))];
  }
};

Delays delays(const Masks& m, Slot slot, std::size_t groups, std::size_t size) {
  Delays out;
  for (std::uint32_t i = 0; i < kPixels; ++i) {
    const int grp = group_of(i, groups);
    if (std::size_t(grp / 2) != size) continue;
    std::uint8_t kind = 0;
    const int d = first_alarm(m, slot, i, &kind);
    if (d < 0) {
      ++out.missed;
      continue;
    }
    out.found.push_back(d);
    const std::uint8_t want = grp % 2 == 0 ? kFarther : kNearer;
    if (kind != want) ++out.wrong_way;
  }
  return out;
}

std::string_view quantile_text(int q, char* buf) {
  if (q == std::numeric_limits<int>::max()) return "-";
  std::snprintf(buf, 16, "%d", q);
  return buf;
}

// Alarms of a static slot over frames [from, to) and pixels [x0, x1).
double alarms_per_pixel_second(const Masks& m, Slot slot, int from, int to,
                               std::uint32_t x0 = 0, std::uint32_t x1 = kSide) {
  std::size_t alarms = 0;
  std::size_t samples = 0;
  for (int t = from; t < to; ++t) {
    for (std::uint32_t i = 0; i < kPixels; ++i) {
      const std::uint32_t x = i % kSide;
      if (x < x0 || x >= x1) continue;
      ++samples;
      alarms += at(m, slot, t, i) != kNone ? 1 : 0;
    }
  }
  return double(alarms) / double(samples) * kFps;
}

// The gate: slot 20, a 32 x 32 plane at 2 m, its word the third of four.
int gate_case(vkc::test::GpuContext& gpu, tsdf::ChangeDetector& detector) {
  constexpr std::uint32_t kSlot = 20;
  constexpr std::uint32_t kGateSide = 32;
  const sensor::DepthCameraParams cam = camera(kGateSide);
  auto words = vkc::device_storage_buffer(gpu.allocator, 16);
  CHECK(words.ok());
  const vol::DeviceCount gate{&words.value(), 8, 1};
  const auto set_gate = [&](std::uint32_t open) {
    return vkc::test::write_back(gpu.device, gpu.allocator, words.value(),
                                 std::vector<std::uint32_t>{1, 1, open, 1});
  };
  Noise noise(7u);
  std::vector<float> depth(kGateSide * kGateSide);
  const auto frame = [&](float z, std::uint32_t open,
                         std::vector<std::uint32_t>& mask) -> int {
    for (float& d : depth) d = z + tof_sigma(z) * noise.gauss();
    CHECK(set_gate(open).ok());
    vkc::CommandBatch batch(gpu.device, gpu.allocator);
    CHECK(detector
              .record_detect(batch, kSlot, vkc::StorageInput(depth.data()), cam,
                             {}, &gate)
              .ok());
    mask.assign(depth.size(), 0xFFu);
    CHECK(batch
              .readback(*detector.mask(kSlot), 0,
                        mask.size() * sizeof(std::uint32_t), mask.data())
              .ok());
    CHECK(batch.submit().ok());
    return 0;
  };
  std::vector<std::uint32_t> mask;
  for (int t = 0; t < 40; ++t) CHECK(frame(2.0f, 1, mask) == 0);
  const std::vector<std::uint32_t> before = mask;
  // Closed: the step is not seen, and the mask stays as it was.
  CHECK(frame(2.05f, 0, mask) == 0);
  CHECK(mask == before);
  // Open: the history never saw the step, so it is flagged now.
  CHECK(frame(2.05f, 1, mask) == 0);
  std::size_t farther = 0;
  for (const std::uint32_t m : mask) farther += m == 2 ? 1 : 0;
  CHECK(double(farther) > 0.98 * double(mask.size()));
  // A gate word misaligned, past its buffer, or with no buffer is refused.
  const auto refused = [&](const vol::DeviceCount& bad) {
    vkc::CommandBatch batch(gpu.device, gpu.allocator);
    return detector
               .record_detect(batch, kSlot, vkc::StorageInput(depth.data()),
                              cam, {}, &bad)
               .domain() == vkc::Status::Code::InvalidArgument;
  };
  CHECK(refused({&words.value(), 6, 1}));
  CHECK(refused({&words.value(), 16, 1}));
  CHECK(refused({nullptr, 0, 1}));
  return 0;
}

// Pixels first valid once their camera tests, over no return: slots 21 and
// 22, 32 x 32, no return for frames 0-59.
// - Slot 21: a mover at 1.5 m over frames 60-62 that steps to 1.3 m at 63
//   (flagged Nearer), gone at 65, and a background at 3 m returning at 80.
//   Frame 65 flags Vanished on every pixel, with the depth that left; the
//   background's first sample starts a new history, which flags nothing.
// - Slot 22: sporadic returns -- 2.0 m over frames 60-61, none until 70,
//   2.5 m over 70-72, none after: the 2-sample run ends silently, the
//   3-sample run flags Vanished at 73 with its depth.
int no_return_case(vkc::test::GpuContext& gpu, tsdf::ChangeDetector& detector) {
  constexpr std::uint32_t kMover = 21;
  constexpr std::uint32_t kSporadic = 22;
  constexpr std::uint32_t kLateSide = 32;
  constexpr std::size_t kLatePixels = kLateSide * kLateSide;
  const sensor::DepthCameraParams cam = camera(kLateSide);
  Noise noise(11u);
  const auto sample = [&](float z) { return z + tof_sigma(z) * noise.gauss(); };
  const auto mover_depth = [&](int t) {
    if (t >= 60 && t < 63) return sample(1.5f);
    if (t >= 63 && t < 65) return sample(1.3f);
    if (t >= 80) return sample(3.0f);
    return 0.0f;
  };
  const auto sporadic_depth = [&](int t) {
    if (t >= 60 && t < 62) return sample(2.0f);
    if (t >= 70 && t < 73) return sample(2.5f);
    return 0.0f;
  };
  std::vector<float> mover(kLatePixels);
  std::vector<float> sporadic(kLatePixels);
  std::vector<std::uint32_t> mover_mask(kLatePixels);
  std::vector<std::uint32_t> sporadic_mask(kLatePixels);
  // Per frame: pixels of each kind, and the Vanished depths' worst error.
  std::size_t nearer_63 = 0;
  std::size_t vanished_mover = 0;
  std::size_t vanished_sporadic = 0;
  std::size_t other_flags = 0;
  float depth_error = 0.0f;
  for (int t = 0; t <= 82; ++t) {
    for (std::size_t i = 0; i < kLatePixels; ++i) {
      mover[i] = mover_depth(t);
      sporadic[i] = sporadic_depth(t);
    }
    vkc::CommandBatch batch(gpu.device, gpu.allocator);
    CHECK(
        detector
            .record_detect(batch, kMover, vkc::StorageInput(mover.data()), cam)
            .ok());
    CHECK(detector
              .record_detect(batch, kSporadic,
                             vkc::StorageInput(sporadic.data()), cam)
              .ok());
    CHECK(batch
              .readback(*detector.mask(kMover), 0,
                        kLatePixels * sizeof(std::uint32_t), mover_mask.data())
              .ok());
    CHECK(batch
              .readback(*detector.mask(kSporadic), 0,
                        kLatePixels * sizeof(std::uint32_t),
                        sporadic_mask.data())
              .ok());
    CHECK(batch.submit().ok());
    for (std::size_t i = 0; i < kLatePixels; ++i) {
      const std::uint32_t m = mover_mask[i];
      const std::uint32_t p = sporadic_mask[i];
      if (t == 63) {
        nearer_63 += tsdf::change_of(m) == tsdf::PixelChange::Nearer ? 1 : 0;
      } else if (t == 65) {
        if (tsdf::change_of(m) == tsdf::PixelChange::Vanished) {
          ++vanished_mover;
          depth_error =
              std::max(depth_error, std::abs(tsdf::vanished_depth(m) - 1.3f));
        }
      } else if (m != 0u) {
        ++other_flags;
      }
      if (t == 73) {
        if (tsdf::change_of(p) == tsdf::PixelChange::Vanished) {
          ++vanished_sporadic;
          depth_error =
              std::max(depth_error, std::abs(tsdf::vanished_depth(p) - 2.5f));
        }
      } else if (p != 0u) {
        ++other_flags;
      }
    }
  }
  std::printf(
      "over no return: %zu of %zu flag the mover's step, %zu its departure "
      "and %zu the sporadic run's end (depth error %.2f mm); %zu other "
      "flags\n",
      nearer_63, kLatePixels, vanished_mover, vanished_sporadic,
      double(depth_error) * 1000.0, other_flags);
  CHECK(nearer_63 == kLatePixels);
  CHECK(vanished_mover == kLatePixels);
  CHECK(vanished_sporadic == kLatePixels);
  CHECK(other_flags == 0);
  // The history's depth is a mean of two (1.3 m) or three (2.5 m) samples,
  // sigma 0.9 and 1.4 mm: 1024 pixels' worst within 5 sigma.
  CHECK(depth_error < 0.007f);
  // A word of another kind holds no depth.
  CHECK(tsdf::vanished_depth(2u) == 0.0f);
  CHECK(tsdf::change_of(2u) == tsdf::PixelChange::Farther);
  return 0;
}

int refusals(vkc::test::GpuContext& gpu, tsdf::ChangeDetector& detector) {
  const sensor::DepthCameraParams cam = camera(kSide);
  const std::vector<float> depth(kPixels, 2.0f);
  const vkc::StorageInput host(depth.data());
  const auto refused = [](const vkc::Status& s) {
    return s.domain() == vkc::Status::Code::InvalidArgument;
  };
  CHECK(refused(detector.detect(tsdf::ChangeDetector::kMaxCameras, host, cam)));
  CHECK(refused(detector.detect(
      0, vkc::StorageInput(static_cast<const void*>(nullptr)), cam)));
  sensor::DepthCameraParams empty = cam;
  empty.width = 0;
  CHECK(refused(detector.detect(0, host, empty)));
  sensor::DepthCameraParams unbounded = cam;
  unbounded.max_depth = std::numeric_limits<float>::infinity();
  CHECK(refused(detector.detect(0, host, unbounded)));
  for (const tsdf::DepthNoise& bad :
       {tsdf::DepthNoise{0.0f, 0.0f, 0.0f},
        tsdf::DepthNoise{-1e-3f, 1e-3f, 0.0f},
        tsdf::DepthNoise{0.0f, std::nanf(""), 0.0f}}) {
    CHECK(refused(detector.detect(0, host, cam, bad)));
  }
  auto small = vkc::device_storage_buffer(gpu.allocator, 64);
  CHECK(small.ok());
  CHECK(refused(detector.detect(0, vkc::StorageInput(small.value()), cam)));
  // Nothing refused made a history.
  CHECK(detector.mask(0) == nullptr);

  for (const tsdf::ChangeDetectorConfig& bad :
       {tsdf::ChangeDetectorConfig{-0.5f},
        tsdf::ChangeDetectorConfig{0.5f, 0.0f},
        tsdf::ChangeDetectorConfig{0.5f, 8.0f, 0.0f},
        tsdf::ChangeDetectorConfig{0.5f, 8.0f, 6.0f, 0},
        tsdf::ChangeDetectorConfig{0.5f, 8.0f, 6.0f, 1024},
        tsdf::ChangeDetectorConfig{0.5f, 8.0f, 6.0f, 30, 30, 512}}) {
    CHECK(refused(
        tsdf::ChangeDetector::create(gpu.device, gpu.allocator, bad).status()));
  }
  return 0;
}

}  // namespace

int gpu_main(vkc::test::GpuContext& gpu) {
  vkc::Device& dev = gpu.device;
  vkc::Allocator& alloc = gpu.allocator;

  auto made = tsdf::ChangeDetector::create(dev, alloc);
  CHECK(made.ok());
  // Moved, the source is empty and refuses.
  tsdf::ChangeDetector detector(std::move(made.value()));
  CHECK(detector.valid());
  CHECK(!made.value().valid());
  {
    const std::vector<float> depth(kPixels, 2.0f);
    CHECK(made.value()
              .detect(0, vkc::StorageInput(depth.data()), camera(kSide))
              .domain() == vkc::Status::Code::InvalidArgument);
  }
  if (refusals(gpu, detector) != 0) return 1;
  if (gate_case(gpu, detector) != 0) return 1;
  if (no_return_case(gpu, detector) != 0) return 1;

  const tsdf::DepthNoise tof{};  // the Femto's
  const tsdf::DepthNoise stereo{0.0f, 0.0f, kStereoC};
  const sensor::DepthCameraParams cam = camera(kSide);

  Scene scene;
  Masks masks(kSlots, std::vector<std::uint8_t>(std::size_t(kFrames) * kPixels,
                                                kUntested));
  std::vector<std::vector<std::uint32_t>> words(
      kSlots, std::vector<std::uint32_t>(kPixels));
  std::vector<std::uint32_t> resized_words(64 * 64);
  std::size_t resized_alarms = 0;
  // Frames 100-119 of kDropout: every way a sample is invalid.
  const auto invalidate = [](std::vector<float>& d) {
    const float bad[] = {
        0.0f, std::nanf(""), std::numeric_limits<float>::infinity(),
        0.1f,  // below min_depth
        6.0f,  // past max_depth
        -2.0f};
    for (std::size_t i = 0; i < d.size(); ++i) d[i] = bad[i % 6];
  };
  for (int t = 0; t < kFrames; ++t) {
    vkc::CommandBatch batch(dev, alloc);
    std::vector<std::vector<float>> frames(kSlots);
    std::vector<bool> tested(kSlots, false);
    for (std::uint32_t s = 0; s < kSlots; ++s) {
      const Slot slot = static_cast<Slot>(s);
      if (slot == kTwin && t >= 100 && t < 120) continue;    // skips them
      if (slot == kLeaver && t >= 100 && t < 150) continue;  // away
      if (slot == kResized && t >= 100) {
        frames[s] = scene.frame(slot, t, 64);
        for (float& z : frames[s]) z += 0.3f;  // and elsewhere
        CHECK(detector
                  .record_detect(batch, s, vkc::StorageInput(frames[s].data()),
                                 camera(64), tof)
                  .ok());
        tested[s] = true;
        continue;
      }
      if (slot == kTwin) {
        frames[s] = frames[kDropout];  // the same samples as its twin
      } else {
        frames[s] = scene.frame(slot, t);
      }
      if (slot == kDropout && t >= 100 && t < 120) invalidate(frames[s]);
      if (slot == kLeaver && t >= 160) {
        for (float& z : frames[s]) z += 0.05f;
      }
      CHECK(detector
                .record_detect(batch, s, vkc::StorageInput(frames[s].data()),
                               cam, slot == kStereo ? stereo : tof)
                .ok());
      tested[s] = true;
    }
    for (std::uint32_t s = 0; s < kSlots; ++s) {
      if (!tested[s]) continue;
      if (s == kResized && t >= 100) {
        CHECK(batch
                  .readback(*detector.mask(s), 0,
                            resized_words.size() * sizeof(std::uint32_t),
                            resized_words.data())
                  .ok());
        continue;
      }
      CHECK(batch
                .readback(*detector.mask(s), 0, kPixels * sizeof(std::uint32_t),
                          words[s].data())
                .ok());
    }
    CHECK(batch.submit().ok());
    for (std::uint32_t s = 0; s < kSlots; ++s) {
      if (!tested[s]) continue;
      if (s == kResized && t >= 100) {
        if (t < 100 + kWarmUp) {
          for (const std::uint32_t w : resized_words) {
            resized_alarms += w != 0 ? 1 : 0;
          }
        }
        continue;
      }
      std::uint8_t* row = &masks[s][std::size_t(t) * kPixels];
      for (std::uint32_t i = 0; i < kPixels; ++i) {
        CHECK(words[s][i] <= 2);
        row[i] = static_cast<std::uint8_t>(words[s][i]);
      }
    }
  }

  // Nothing alarms while the histories warm up.
  for (std::uint32_t s = 0; s < kSlots; ++s) {
    for (int t = 0; t < kWarmUp; ++t) {
      for (std::uint32_t i = 0; i < kPixels; ++i) {
        const std::uint8_t a = at(masks, static_cast<Slot>(s), t, i);
        CHECK(a == kNone || a == kUntested);
      }
    }
  }

  // Static noise: false alarms near the rig's 4.3e-4 per interior pixel per
  // second (one per pixel per ~39 min), ToF-like and stereo-like alike, and
  // as many at 1 m as at 3 m, where stereo's noise is 9 times larger.
  const double tof_rate =
      alarms_per_pixel_second(masks, kTof, kWarmUp, kFrames);
  const double stereo_rate =
      alarms_per_pixel_second(masks, kStereo, kWarmUp, kFrames);
  const double near_rate =
      alarms_per_pixel_second(masks, kStereo, kWarmUp, kFrames, 0, kSide / 2);
  const double far_rate = alarms_per_pixel_second(masks, kStereo, kWarmUp,
                                                  kFrames, kSide / 2, kSide);
  std::printf(
      "false alarms per pixel per second: ToF %.2e, stereo %.2e (1-2 m "
      "%.2e, 2-3 m %.2e)\n",
      tof_rate, stereo_rate, near_rate, far_rate);
  CHECK(tof_rate > 4.3e-5 && tof_rate < 4.3e-3);
  CHECK(stereo_rate > 4.3e-5 && stereo_rate < 4.3e-3);
  CHECK(near_rate < 3.0 * far_rate + 1e-4 && far_rate < 3.0 * near_rate + 1e-4);

  // Steps: frames to the first alarm, the rig's interior pixels measured
  // median/p95 5/- (6.5% missed), 2/6, 0/2 (81% the same frame) and 0/0 (98%).
  char a[16];
  char b[16];
  for (std::size_t s = 0; s < kStepsMm.size(); ++s) {
    const Delays d = delays(masks, kSteps, kStepsMm.size(), s);
    const int med = d.quantile(0.5);
    const int p95 = d.quantile(0.95);
    const double missed = double(d.missed) / double(d.found.size() + d.missed);
    std::printf(
        "step %2.0f mm: median %s, p95 %s, same frame %.1f%%, missed %.1f%%, "
        "wrong way %zu\n",
        double(kStepsMm[s]), quantile_text(med, a).data(),
        quantile_text(p95, b).data(), 100.0 * d.share(0), 100.0 * missed,
        d.wrong_way);
    CHECK(d.wrong_way == 0);
    switch (s) {
      case 0:  // 5 mm
        CHECK(med <= 8 && missed < 0.15);
        break;
      case 1:  // 10 mm
        CHECK(med <= 3 && p95 <= 8 && missed < 0.02);
        break;
      case 2:  // 20 mm
        CHECK(d.share(0) > 0.65 && p95 <= 3);
        break;
      default:  // 40 mm
        CHECK(d.share(0) > 0.95 && p95 == 0);
        break;
    }
  }

  // Ramps: the rig's first alarm, median/p95, 7/11, 3/5 and 1/2 frames.
  for (std::size_t s = 0; s < kRampsMm.size(); ++s) {
    const Delays d = delays(masks, kRamps, kRampsMm.size(), s);
    const int med = d.quantile(0.5);
    const int p95 = d.quantile(0.95);
    std::printf("ramp %2.0f mm a frame: median %s, p95 %s, missed %zu\n",
                double(kRampsMm[s]), quantile_text(med, a).data(),
                quantile_text(p95, b).data(), d.missed);
    CHECK(d.missed == 0 && d.wrong_way == 0);
    const int want_med[] = {9, 4, 2};
    const int want_p95[] = {14, 7, 3};
    CHECK(med <= want_med[s] && p95 <= want_p95[s]);
  }

  // The sweep: every pixel the sweep reaches in the window, on the frame it
  // becomes the boundary or covered (the rig's 95.7%), the rest the next.
  {
    std::size_t touched = 0;
    std::size_t same = 0;
    std::size_t next = 0;
    for (std::uint32_t i = 0; i < kPixels; ++i) {
      const int x = int(i % kSide);
      const int touch = kOnset + (int(kSide) - 1 - x);
      if (touch + 1 >= kOnset + kWindow) continue;
      ++touched;
      same += at(masks, kSweep, touch, i) == kNearer ? 1 : 0;
      next += (at(masks, kSweep, touch, i) == kNearer ||
               at(masks, kSweep, touch + 1, i) == kNearer)
                  ? 1
                  : 0;
    }
    std::printf("sweep: %zu pixels, %.1f%% on the touch frame, %.1f%% by +1\n",
                touched, 100.0 * double(same) / double(touched),
                100.0 * double(next) / double(touched));
    CHECK(double(same) > 0.9 * double(touched));
    CHECK(next == touched);
  }

  // Dropouts: no flag on an invalid frame, and after them the twin's masks,
  // bit for bit.
  for (int t = 100; t < 120; ++t) {
    for (std::uint32_t i = 0; i < kPixels; ++i) {
      CHECK(at(masks, kDropout, t, i) == kNone);
    }
  }
  for (int t = 120; t < kFrames; ++t) {
    for (std::uint32_t i = 0; i < kPixels; ++i) {
      CHECK(at(masks, kDropout, t, i) == at(masks, kTwin, t, i));
    }
  }

  // The leaver: back at frame 150 to the plane it left, it raises no more
  // alarms than noise does; its 5 cm step at 160 flags in that frame.
  {
    std::size_t back = 0;
    std::size_t stepped = 0;
    for (std::uint32_t i = 0; i < kPixels; ++i) {
      back += at(masks, kLeaver, 150, i) != kNone ? 1 : 0;
      stepped += at(masks, kLeaver, 160, i) == kFarther ? 1 : 0;
    }
    std::printf("leaver: %zu alarms on return, %zu of %u on its step\n", back,
                stepped, kPixels);
    CHECK(back <= kPixels / 1000);
    CHECK(double(stepped) > 0.98 * double(kPixels));
  }

  // The resized camera started over: no alarm in its first 30 frames at the
  // new size, though its plane moved 30 cm.
  CHECK(resized_alarms == 0);
  CHECK(detector.mask(kResized)->size() >= 64 * 64 * sizeof(std::uint32_t));

  std::puts("tsdf_change_detector: OK");
  return 0;
}

int main() {
  return vkc::test::run_on_gpu(vr::device_requirements(), gpu_main);
}
