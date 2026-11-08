// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The live examples' run (examples/common/live_fusion.hpp, run_live_fusion),
// headless on the test's device and policy: its cameras one fake camera that
// sees a wall 2 m ahead on the host's clock and tells the array of each frame
// (after sender_set_sender_test's Camera), in place of a driver's discovery;
// its phones writers that dial 127.0.0.1 as an iPad's does (link_dialler.hpp),
// depth alone, placed by a registration file. Each run's stdout is read back.
// - A rig-only run of 3 frames takes 3 from the camera, fuses them and writes
//   its PLY, whose vertices are those the run prints; with output.stream, the
//   stream holds the 3 sets.
// - A live source.mobile with no phone: the start line says the phones are
//   listening, the cameras open within a second of it, and the rig runs its
//   3 sets.
// - A phone that dials once the sets run joins, and the report counts it: the
//   door's one admitted, its session's line, its leave at the run's end. The
//   run has no run.frames: the test raises SIGINT once the join is in
//   sessions.csv and the phone has sent its frames, and the run ends after
//   the set in hand, as a Ctrl+C ends it.
// - An address another socket holds: the rig runs its 3 sets, and the report
//   says the phones are not listening.
// - Classic fusion rules phones out: the start line says why, a phone that
//   dials while the cameras open is turned away and counted, and the rig runs
//   its 3 sets.

#include <fcntl.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "config.hpp"
#include "depth_inputs.hpp"
#include "fuse_device.hpp"
#include "interrupt.hpp"
#include "link_dialler.hpp"
#include "live_fusion.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/base/unique_fd.hpp"
#include "volumetric_kit/core/testing/gpu_test.hpp"
#include "volumetric_kit/core/testing/net_loopback.hpp"
#include "volumetric_kit/core/testing/scratch_dir.hpp"
#include "volumetric_kit/core/testing/test_check.hpp"
#include "volumetric_kit/grid/stream/stream_reader.hpp"
#include "volumetric_kit/sensor/array/sensor_array.hpp"
#include "volumetric_kit/sensor/camera/array_calibration.hpp"
#include "volumetric_kit/sensor/camera/tracked_registration.hpp"

namespace {

namespace vkc = volumetric_kit::core;
namespace cfg = volumetric_kit::core::config;
namespace sensor = volumetric_kit::sensor;
namespace camera = volumetric_kit::sensor::camera;
namespace stream = volumetric_kit::grid::stream;
namespace fs = std::filesystem;
using Gpu = vkc::test::GpuContext;

constexpr std::uint32_t kW = 64;
constexpr std::uint32_t kH = 48;
constexpr auto kLimit = std::chrono::seconds(30);  // a failure's, never met

std::uint64_t now_ns() {
  return static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count());
}

bool has(const std::string& text, const std::string& part) {
  if (text.find(part) != std::string::npos) return true;
  std::fprintf(stderr, "no \"%s\" in the run's output\n", part.c_str());
  return false;
}

bool lacks(const std::string& text, const std::string& part) {
  if (text.find(part) == std::string::npos) return true;
  std::fprintf(stderr, "\"%s\" in the run's output\n", part.c_str());
  return false;
}

// The file's text; empty where there is none yet.
std::string text_of(const fs::path& path) {
  std::ifstream in(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

std::vector<std::string> lines_of(const fs::path& path) {
  std::ifstream in(path);
  std::vector<std::string> out;
  for (std::string line; std::getline(in, line);) out.push_back(line);
  return out;
}

// The vertices a PLY's header states, if it reads as one.
std::optional<std::size_t> ply_vertices(const fs::path& path) {
  std::ifstream in(path, std::ios::binary);
  std::string line;
  if (!std::getline(in, line) || line != "ply") return std::nullopt;
  const std::string vertex = "element vertex ";
  while (std::getline(in, line) && line != "end_header") {
    if (line.rfind(vertex, 0) == 0) {
      return std::stoul(line.substr(vertex.size()));
    }
  }
  return std::nullopt;
}

// A camera that sees a wall 2 m ahead, on the host's clock: each frame made
// on a thread of its own once the array has taken the last, and told to the
// array (set_frame_notify), as a driver tells it. The frames it handed over
// are counted in `taken`, which outlives it.
class WallCamera final : public sensor::IRgbdSensor {
 public:
  WallCamera(std::string id, std::shared_ptr<std::atomic<std::size_t>> taken)
      : taken_(std::move(taken)) {
    info_.id = std::move(id);
    info_.model = "wall";
    info_.role = sensor::SyncRole::Primary;
    info_.clock = sensor::ClockDomain::Host;
    info_.fps = 25;
  }
  ~WallCamera() override { stop(); }

  const sensor::SensorInfo& info() const noexcept override { return info_; }
  vkc::Status set_queue_depth(std::size_t) override { return {}; }
  vkc::Status set_frame_notify(std::function<void()> notify) override {
    std::lock_guard<std::mutex> lock(mutex_);
    notify_ = std::move(notify);
    return {};
  }
  vkc::Status start() override {
    if (thread_.joinable()) return {};
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = false;
    }
    thread_ = std::thread([this] { run(); });
    return {};
  }
  void stop() noexcept override {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    taken_wake_.notify_all();
    if (thread_.joinable()) thread_.join();
    std::lock_guard<std::mutex> lock(mutex_);
    frames_.clear();
  }
  vkc::Result<std::optional<sensor::RgbdFrame>> poll() override {
    std::vector<sensor::RgbdFrame> held;
    VKC_TRY(drain(&held));
    if (held.empty()) return no_frame();
    return std::optional<sensor::RgbdFrame>(std::move(held.back()));
  }
  vkc::Status drain(std::vector<sensor::RgbdFrame>* out) override {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (sensor::RgbdFrame& f : frames_) out->push_back(std::move(f));
      *taken_ += frames_.size();
      frames_.clear();
    }
    taken_wake_.notify_all();
    return {};
  }
  sensor::SensorStats stats() const noexcept override {
    std::lock_guard<std::mutex> lock(mutex_);
    sensor::SensorStats s;
    s.received = made_;
    s.delivered = taken_->load();
    return s;
  }

 private:
  void run() {
    for (std::uint64_t k = 0;; ++k) {
      std::function<void()> notify;
      {
        std::unique_lock<std::mutex> lock(mutex_);
        taken_wake_.wait(lock, [this] { return stopping_ || frames_.empty(); });
        if (stopping_) return;
        frames_.push_back(frame(k));
        ++made_;
        notify = notify_;
      }
      if (notify) notify();
    }
  }

  static sensor::RgbdFrame frame(std::uint64_t k) {
    struct Pixels {
      std::vector<std::uint16_t> depth;
      std::vector<std::uint32_t> color;
    };
    auto pixels = std::make_shared<Pixels>();
    pixels->depth.assign(std::size_t{kW} * kH, 2000);  // 2 m
    pixels->color.assign(std::size_t{kW} * kH, 0x00806040u);
    sensor::RgbdFrame f;
    f.depth = pixels->depth.data();
    f.depth_camera = {{kW, kH}, {48.0, 48.0, 31.5, 23.5}, {}};
    f.min_depth = 0.1f;
    f.max_depth = 5.0f;
    f.depth_noise = vr_test::kFemtoNoise;
    f.color_packed = pixels->color.data();
    f.color_camera = f.depth_camera;
    f.timestamp_ns = now_ns();
    f.arrival_ns = f.timestamp_ns;
    f.sequence = k;
    f.pixels = std::move(pixels);
    return f;
  }

  sensor::SensorInfo info_;
  const std::shared_ptr<std::atomic<std::size_t>> taken_;
  mutable std::mutex mutex_;
  std::condition_variable taken_wake_;
  std::function<void()> notify_;           // under mutex_
  std::vector<sensor::RgbdFrame> frames_;  // under mutex_
  std::uint64_t made_ = 0;                 // under mutex_
  bool stopping_ = false;                  // under mutex_
  std::thread thread_;                     // start's and stop's
};

// The run's cameras, in place of a driver's discovery: one wall camera,
// "rig-0", read as an array that prepares each set on the run's device;
// `on_open`, if set, runs first, as the cameras open.
struct Rig {
  std::function<void()> on_open;
  std::shared_ptr<std::atomic<std::size_t>> taken =
      std::make_shared<std::atomic<std::size_t>>(0);
  std::size_t opened = 0;

  vr_example::OpenArray opener() {
    return
        [this](const cfg::Config&, vkc::Device& device,
               vkc::Allocator& allocator) -> vkc::Result<sensor::SensorArray> {
          if (on_open) on_open();
          std::vector<std::unique_ptr<sensor::IRgbdSensor>> sensors;
          sensors.push_back(std::make_unique<WallCamera>("rig-0", taken));
          sensor::SensorArray::Options options;
          options.device = &device;
          options.allocator = &allocator;
          VKC_ASSIGN(sensor::SensorArray array,
                     sensor::SensorArray::open(std::move(sensors), options));
          ++opened;
          std::printf("camera rig-0: a wall 2 m ahead, %ux%u\n", kW, kH);
          return array;
        };
  }
};

// Everything written to stdout, by any thread, while it lives, into `path`;
// read back, and echoed to the test's own stdout, by `text`.
class Captured {
 public:
  explicit Captured(fs::path path) : path_(std::move(path)) {
    std::fflush(stdout);
    saved_ = vkc::UniqueFd(::dup(STDOUT_FILENO));
    const vkc::UniqueFd file(
        ::open(path_.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644));
    if (saved_.valid() && file.valid()) {
      ok_ = ::dup2(file.get(), STDOUT_FILENO) >= 0;
    }
  }
  ~Captured() { (void)text(); }
  Captured(const Captured&) = delete;
  Captured& operator=(const Captured&) = delete;

  bool ok() const { return ok_; }

  // Ends the capture, once; what it caught.
  const std::string& text() {
    if (!saved_.valid()) return text_;
    std::fflush(stdout);
    (void)::dup2(saved_.get(), STDOUT_FILENO);
    saved_.reset();
    text_ = text_of(path_);
    std::fputs(text_.c_str(), stdout);
    return text_;
  }

 private:
  fs::path path_;
  vkc::UniqueFd saved_;
  bool ok_ = false;
  std::string text_;
};

struct Ran {
  vkc::Status status;
  std::string out;  // its stdout
};

// `opt` run on the test's device with `rig`'s cameras, its stdout in `dir`.
Ran run(Gpu& gpu, const cfg::Config& opt, Rig& rig, const fs::path& dir) {
  Ran r;
  Captured captured(dir / "stdout.txt");
  if (!captured.ok()) {
    r.status = vkc::Status::io_error("cannot capture stdout");
    return r;
  }
  r.status = vr_example::run_live_fusion(
      opt, "live_fusion_test", rig.opener(), /*host_clock=*/true,
      vr_example::sensor_counters, &gpu.device, &gpu.allocator);
  r.out = captured.text();
  if (!r.status.ok()) {
    std::fprintf(stderr, "run: %s\n", r.status.message().c_str());
  }
  return r;
}

// A fuse_orbbec scenario: `source` keys after the live kind, `rest` keys at
// the top level after the source.
vkc::Result<cfg::Config> scenario(const std::string& source,
                                  const std::string& rest) {
  return cfg::parse(R"({"version": 1, "operation": "fuse",
      "source": {"kind": "live", "sync_config": "sync.json")" +
                        source + "}" + rest + "}",
                    cfg::Application::FuseOrbbec);
}

std::string output(const fs::path& ply, const std::string& more = {}) {
  return R"(, "output": {"path": ")" + ply.string() + "\"" + more + "}";
}

// A depth-alone phone's session, its id ending `id`, and its frames.
struct Phone {
  stream::Session session;
  vr_test::PatternFrames frames;
};

Phone phone(std::uint8_t id) {
  Phone p{{}, vr_test::pattern_frames(vr_test::phone_info(), 10)};
  p.session = vr_test::session_of(p.frames.info, id);
  return p;
}

// The source keys of a live source.mobile on 127.0.0.1:`port`: the rig's
// calibration, its world the camera (no tag), and `p`'s registration in it,
// both written into `dir`.
vkc::Result<std::string> mobile(const fs::path& dir, const Phone& p,
                                std::uint16_t port) {
  camera::ArrayCalibration array;
  array.sensors.push_back({"rig-0", camera::Mat4d(1.0), {}, {}, {}, 0.0});
  array.world = {"rig-0", std::nullopt};
  const std::string calibration = (dir / "array_calibration.json").string();
  VKC_TRY(camera::write_array_calibration(calibration, array,
                                          vkc::WriteMode::Replace));
  camera::TrackedRegistration reg;
  reg.sensor = p.frames.info.id;
  reg.session = p.session.id;
  reg.world = array.world;
  reg.rms_px = 0.5;
  reg.views = 6;
  reg.edge_m = 0.15;
  const std::string registration = (dir / "phone.registration.json").string();
  VKC_TRY(camera::write_tracked_registration(registration, reg,
                                             vkc::WriteMode::Replace));
  return ", \"calibration\": \"" + calibration +
         "\", \"mobile\": \"tcp://127.0.0.1:" + std::to_string(port) +
         "\", \"mobile_registration\": \"" + registration + "\"";
}

std::string listening_on(std::uint16_t port) {
  return "phones: listening on tcp://127.0.0.1:" + std::to_string(port);
}

// --- The cases --------------------------------------------------------------

// Three frames, fused into a PLY; then coded into a stream too.
int test_rig_only(Gpu& gpu, const fs::path& dir) {
  fs::create_directories(dir);
  const fs::path ply = dir / "rig.ply";
  auto opt = scenario("", R"(, "run": {"frames": 3})" + output(ply));
  CHECK(opt.ok());
  Rig rig;
  const Ran r = run(gpu, *opt, rig, dir);
  CHECK(r.status.ok() && rig.opened == 1 && rig.taken->load() == 3);
  CHECK(has(r.out, "done: fused 3 frames") && lacks(r.out, "codec: ") &&
        lacks(r.out, "phones"));
  const std::optional<std::size_t> vertices = ply_vertices(ply);
  CHECK(vertices && *vertices > 0 &&
        has(r.out, "mesh " + std::to_string(*vertices) + " vertices"));

  const fs::path coded_ply = dir / "coded.ply";
  const fs::path vrst = dir / "rig.vrst";
  auto coded = scenario(
      "", R"(, "run": {"frames": 3})" +
              output(coded_ply, R"(, "stream": ")" + vrst.string() + "\""));
  CHECK(coded.ok());
  Rig again;
  const Ran c = run(gpu, *coded, again, dir);
  CHECK(c.status.ok() && again.taken->load() == 3);
  CHECK(has(c.out, "codec: ") && has(c.out, "done: fused 3 frames"));
  CHECK(ply_vertices(coded_ply).value_or(0) > 0);
  auto reader = stream::StreamReader::open(vrst.string());
  CHECK(reader.ok() && reader.value()->recovery().finished &&
        !reader.value()->stats().empty() &&
        reader.value()->stats()[0].records == 3);
  return 0;
}

// A live address no phone dials: listening before the cameras open, which
// open at once, and the rig runs every set.
int test_no_phone(Gpu& gpu, const fs::path& dir) {
  fs::create_directories(dir);
  const std::uint16_t port = vkc::test::closed_port();
  auto keys = mobile(dir, phone(0x51), port);
  CHECK(keys.ok());
  auto opt =
      scenario(*keys, R"(, "run": {"frames": 3})" + output(dir / "a.ply"));
  CHECK(opt.ok());
  const std::string listening =
      listening_on(port) + " for up to 4 phones at once";
  // Nothing waits for a phone: the cameras open within a second of the
  // listening line, which a watcher reads in the run's stdout as it is
  // written. Its reading late only shortens the gap; a wait such as listen()'s
  // 5 s handshake, or any of a second or more, fails.
  Rig rig;
  std::atomic<bool> opening{false};
  std::atomic<bool> over{false};         // the run returned
  vr_test::Clock::time_point opened_at;  // the run's thread's
  rig.on_open = [&] {
    opened_at = vr_test::Clock::now();
    opening = true;
  };
  vr_test::Clock::time_point heard_at;  // the watcher's
  std::thread watcher([&] {
    (void)vr_test::wait_for(
        [&] {
          return opening || over ||
                 text_of(dir / "stdout.txt").find(listening) !=
                     std::string::npos;
        },
        kLimit);
    heard_at = vr_test::Clock::now();
  });
  const Ran r = run(gpu, *opt, rig, dir);
  over = true;
  watcher.join();
  CHECK(r.status.ok() && rig.taken->load() == 3);
  CHECK(has(r.out, listening) &&
        has(r.out, "; the rig fuses alone until one joins\n") &&
        has(r.out, "opening the cameras"));
  CHECK(r.out.find(listening) < r.out.find("opening the cameras"));
  const long long gap_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(opened_at -
                                                            heard_at)
          .count();
  if (gap_ms >= 1000) {
    std::fprintf(stderr,
                 "the cameras opened %lld ms after the listening line\n",
                 gap_ms);
  }
  CHECK(gap_ms < 1000);
  CHECK(has(r.out, ": 0 accepted, 0 arrived, 0 admitted") &&
        has(r.out, "done: fused 3 frames"));
  return 0;
}

// A phone that dials once the sets run: it joins, the test's SIGINT once the
// join is in sessions.csv ends the run, and the report counts it.
int test_phone_joins(Gpu& gpu, const fs::path& dir) {
  fs::create_directories(dir);
  const Phone a = phone(0x52);
  const std::uint16_t port = vkc::test::closed_port();
  auto keys = mobile(dir, a, port);
  CHECK(keys.ok());
  const fs::path log = dir / "log";
  const fs::path ply = dir / "a.ply";
  // No run.frames: until a Ctrl+C.
  auto opt =
      scenario(*keys, output(ply) + R"(, "diagnostics": {"track_log": ")" +
                          log.string() + "\"}");
  CHECK(opt.ok());
  Rig rig;
  std::atomic<bool> over{false};  // the run returned
  std::atomic<bool> hang_up{false};
  std::atomic<bool> sent{false};
  // The phone: it dials once the rig's first set is taken, sends its frames
  // and holds on until the run is over; where it gives up, it says why.
  std::thread writer([&] {
    if (!vr_test::wait_for([&] { return rig.taken->load() > 0 || over; },
                           kLimit) ||
        over) {
      std::fputs("phone: the run took no set to dial after\n", stderr);
      return;
    }
    auto d = vr_test::Dialler::dial(a.session, port, false, kLimit);
    if (!d.ok()) {
      std::fprintf(stderr, "phone: %s\n", d.status().message().c_str());
      return;
    }
    for (std::size_t i = 0; i < a.frames.frames.size(); ++i) {
      vr_test::Record rec;
      vkc::Status st =
          vr_test::depth_record(a.frames.info, a.frames.frames[i], 0, i, &rec);
      if (st.ok()) st = (*d)->publish(rec);
      if (!st.ok()) {
        std::fprintf(stderr, "phone: frame %zu: %s\n", i, st.message().c_str());
        return;
      }
    }
    sent = true;
    (void)vr_test::wait_for([&] { return hang_up.load(); }, 2 * kLimit);
  });
  // Its join, read in sessions.csv, once it has sent its frames, is the
  // Ctrl+C.
  bool joined = false;
  std::thread watcher([&] {
    const auto in_csv = [&] {
      for (const std::string& row : lines_of(log / "sessions.csv")) {
        if (row.rfind("join,", 0) == 0) return true;
      }
      return false;
    };
    joined =
        vr_test::wait_for([&] { return over || (sent && in_csv()); }, kLimit) &&
        !over;
    if (!over) std::raise(SIGINT);
  });
  const Ran r = run(gpu, *opt, rig, dir);
  over = true;
  watcher.join();
  hang_up = true;
  writer.join();
  // The flag is the process's: the next run starts as a new process would.
  vgrid_example::g_interrupted.store(false);
  CHECK(r.status.ok());
  CHECK(sent);
  CHECK(joined);
  CHECK(has(r.out, listening_on(port)) && has(r.out, "  interrupted after "));
  CHECK(has(r.out, ": 1 accepted, 1 arrived, 1 admitted, 0 refused"));
  const std::string hex = vr_test::hex_of(a.session.id);
  CHECK(has(r.out, "phones: 01 " + hex.substr(0, 8) + " (ipad) from ") &&
        has(r.out, "mobile 01 (ipad): "));
  const std::vector<std::string> rows = lines_of(log / "sessions.csv");
  CHECK(rows.size() == 3 && rows[1].rfind("join,", 0) == 0 &&
        rows[2].rfind("leave,", 0) == 0 && has(rows[2], hex) &&
        has(rows[2], ",the run's end"));
  CHECK(fs::exists(log / ("01-ipad-" + hex.substr(0, 8)) / "phone.vrst"));
  CHECK(has(r.out, "done: fused ") && ply_vertices(ply).value_or(0) > 0);
  return 0;
}

// An address another socket holds: the rig runs, and the report says the
// phones are not listening.
int test_bind_failure(Gpu& gpu, const fs::path& dir) {
  fs::create_directories(dir);
  vkc::test::LoopbackListener held = vkc::test::bind_loopback();
  CHECK(held.fd.valid());
  auto keys = mobile(dir, phone(0x53), held.port);
  CHECK(keys.ok());
  auto opt =
      scenario(*keys, R"(, "run": {"frames": 3})" + output(dir / "a.ply"));
  CHECK(opt.ok());
  Rig rig;
  const Ran r = run(gpu, *opt, rig, dir);
  CHECK(r.status.ok() && rig.taken->load() == 3);
  CHECK(lacks(r.out, listening_on(held.port)) &&
        has(r.out, "phones: not listening: ") &&
        has(r.out, "done: fused 3 frames"));
  return 0;
}

// Classic fusion: every phone turned away, one that dials refused and
// counted, the rig running its sets.
int test_ruled_out(Gpu& gpu, const fs::path& dir) {
  fs::create_directories(dir);
  const Phone a = phone(0x54);
  const std::uint16_t port = vkc::test::closed_port();
  auto keys = mobile(dir, a, port);
  CHECK(keys.ok());
  auto opt = scenario(*keys, R"(, "fusion": {"dynamic": false},
      "run": {"frames": 3})" + output(dir / "a.ply"));
  CHECK(opt.ok());
  Rig rig;
  bool refused = false;
  // As the cameras open, the door bound: the phone dials and is let go
  // right after its session's chunk.
  rig.on_open = [&] {
    auto d = vr_test::Dialler::dial(a.session, port, false, kLimit);
    if (!d.ok()) {
      std::fprintf(stderr, "phone: %s\n", d.status().message().c_str());
      return;
    }
    refused = (*d)->let_go(kLimit);
    if (!refused) std::fputs("phone: not let go\n", stderr);
  };
  const Ran r = run(gpu, *opt, rig, dir);
  CHECK(r.status.ok() && refused && rig.taken->load() == 3);
  CHECK(has(r.out, listening_on(port) + ", every phone turned away: ") &&
        has(r.out, "fusion.dynamic"));
  CHECK(has(r.out, "1 refused (1 turned away)") &&
        has(r.out, "done: fused 3 frames"));
  return 0;
}

int gpu_main(Gpu& gpu) {
  const vkc::test::Scratch scratch("recon_example_live_fusion_");
  CHECK(test_rig_only(gpu, scratch.path / "rig") == 0);
  CHECK(test_no_phone(gpu, scratch.path / "no_phone") == 0);
  CHECK(test_phone_joins(gpu, scratch.path / "joins") == 0);
  CHECK(test_bind_failure(gpu, scratch.path / "bind") == 0);
  CHECK(test_ruled_out(gpu, scratch.path / "ruled_out") == 0);
  std::puts("recon_example_live_fusion: OK");
  return 0;
}

}  // namespace

int main() {
  std::setvbuf(stdout, nullptr, _IONBF, 0);
  const vkc::Result<vkc::DeviceRequirements> reqs =
      vr_example::fuse_device_requirements();
  if (!reqs.ok()) {
    std::fprintf(stderr, "FAIL: %s\n", reqs.status().message().c_str());
    return 1;
  }
  return vkc::test::run_on_gpu(reqs.value(), gpu_main);
}
