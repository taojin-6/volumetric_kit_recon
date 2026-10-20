// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// fuse_orbbec's coded sets (examples/common/set_codec.hpp), with no camera
// driver: two fake cameras 3 m apart, read as a sensor array and prepared on
// the device as a rig's are. Three sets fused and coded at an intra period of
// 30 into a .vrst: a Geometry record each, only the first a key, each record's
// set the set's index and its pts the set's time, and every frame decodes
// through codec::decode_growing. Each camera sees a wall 1 m ahead over the
// left of its picture in the first set and over the right in the others: with
// fusion.accumulate false the left walls are gone from the second and third
// frames' decodes, and with it true they stay. On the host, a path that is the
// stream's or the CSV's file through a link refused, and one beside them
// taken. The device part skips with no device.

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "gpu_test.hpp"
#include "scratch_dir.hpp"
#include "set_codec.hpp"
#include "test_check.hpp"
#include "volumetric_kit/recon/camera/array_calibration.hpp"
#include "volumetric_kit/recon/codec/decode_grid.hpp"
#include "volumetric_kit/recon/codec/decoder.hpp"
#include "volumetric_kit/recon/codec/encoder.hpp"
#include "volumetric_kit/recon/mesh/marching_cubes.hpp"
#include "volumetric_kit/recon/mesh/mesh.hpp"
#include "volumetric_kit/recon/sensor/array/sensor_array.hpp"
#include "volumetric_kit/recon/stream/stream_reader.hpp"
#include "volumetric_kit/recon/tsdf/fuser.hpp"

namespace vr = volumetric_kit::recon;
namespace vkc = volumetric_kit::core;
namespace camera = volumetric_kit::recon::camera;
namespace codec = volumetric_kit::recon::codec;
namespace cfg = volumetric_kit::core::config;
namespace sensor = volumetric_kit::recon::sensor;
namespace stream = volumetric_kit::recon::stream;
namespace fs = std::filesystem;

namespace {

constexpr std::uint32_t kW = 64, kH = 48;
constexpr std::uint32_t kGap = 6;  // columns left empty each side of centre
constexpr std::size_t kSets = 3;
// A host-clock time, and a set period no frame rate's pts would give.
constexpr std::uint64_t kStartNs = 1'760'000'000'123'456'789;
constexpr std::uint64_t kPeriodNs = 33'366'701;
constexpr double kOffsetM = 3.0;  // camera B's x; camera A's is 0

std::uint64_t set_time(std::size_t set) { return kStartNs + set * kPeriodNs; }

// A camera on the host's clock at 30 fps whose frames the test pushes: a
// wall 1 m ahead over the left or the right of its picture, no return
// elsewhere, and its colour as packed words on the host.
class Camera final : public sensor::IRgbdSensor {
 public:
  Camera(std::string id, sensor::SyncRole role) {
    info_.id = std::move(id);
    info_.role = role;
    info_.clock = sensor::ClockDomain::Host;
    info_.fps = 30;
  }

  void push(std::size_t set, bool left) {
    struct Pixels {
      std::vector<std::uint16_t> depth;
      std::vector<std::uint32_t> color;
    };
    auto pixels = std::make_shared<Pixels>();
    pixels->depth.assign(std::size_t{kW} * kH, 0);
    pixels->color.assign(std::size_t{kW} * kH, 0x00806040u);
    for (std::uint32_t y = 0; y < kH; ++y) {
      for (std::uint32_t x = 0; x < kW; ++x) {
        const bool wall = left ? x + kGap < kW / 2 : x >= kW / 2 + kGap;
        if (wall) pixels->depth[std::size_t{y} * kW + x] = 1000;  // 1 m
      }
    }
    sensor::RgbdFrame f;
    f.depth = pixels->depth.data();
    f.depth_camera = {{kW, kH}, {48.0, 48.0, 31.5, 23.5}, {}};
    f.min_depth = 0.1f;
    f.max_depth = 5.0f;
    f.color_packed = pixels->color.data();
    f.color_camera = f.depth_camera;
    f.timestamp_ns = set_time(set);
    f.sequence = set;
    f.pixels = std::move(pixels);
    frames_.push_back(std::move(f));
  }

  const sensor::SensorInfo& info() const noexcept override { return info_; }
  vkc::Status set_queue_depth(std::size_t) override { return {}; }
  vkc::Status start() override { return {}; }
  void stop() noexcept override { frames_.clear(); }
  vkc::Result<std::optional<sensor::RgbdFrame>> poll() override {
    return no_frame();
  }
  vkc::Status drain(std::vector<sensor::RgbdFrame>* out) override {
    for (sensor::RgbdFrame& f : frames_) out->push_back(std::move(f));
    frames_.clear();
    return {};
  }
  sensor::SensorStats stats() const noexcept override { return {}; }

 private:
  sensor::SensorInfo info_;
  std::vector<sensor::RgbdFrame> frames_;
};

// Vertices of a mesh on each camera's walls: A's left and right, then B's.
using Walls = std::array<std::size_t, 4>;

Walls walls(const vr::mesh::Mesh& mesh) {
  Walls count{};
  for (const vr::mesh::Vertex& v : mesh.vertices) {
    const double x = v.position.x;
    const bool b = x > kOffsetM / 2;
    const bool right = x > (b ? kOffsetM : 0.0);
    ++count[(b ? 2 : 0) + (right ? 1 : 0)];
  }
  return count;
}

// Push, poll and prepare kSets sets, the first's walls on the left and the
// others' on the right, fusing and coding each through fuse_and_code into a
// new stream at `path`.
int code_sets(vr_test::GpuContext& gpu, const cfg::Fusion& fusion,
              const std::string& path) {
  auto a = std::make_unique<Camera>("A", sensor::SyncRole::Primary);
  auto b = std::make_unique<Camera>("B", sensor::SyncRole::Secondary);
  Camera* cameras[] = {a.get(), b.get()};
  std::vector<std::unique_ptr<sensor::IRgbdSensor>> sensors;
  sensors.push_back(std::move(a));
  sensors.push_back(std::move(b));
  sensor::SensorArray::Options options;
  for (const auto& [id, x] : {std::pair{"A", 0.0}, std::pair{"B", kOffsetM}}) {
    camera::SensorCalibration posed;
    posed.id = id;
    posed.color_to_world = camera::Mat4d(1.0);
    (*posed.color_to_world)[3] = glm::dvec4(x, 0.0, 0.0, 1.0);
    options.calibration.sensors.push_back(posed);
  }
  options.device = &gpu.device;
  options.allocator = &gpu.allocator;
  auto opened = sensor::SensorArray::open(std::move(sensors), options);
  CHECK(opened.ok());
  sensor::SensorArray array = std::move(opened).value();
  CHECK(array.start().ok());

  auto grid = vr_example::create_fusion_grid(
      gpu.device, gpu.allocator, fusion.voxel_size_m, fusion.truncation_m);
  CHECK(grid.ok());
  auto fuser = vr::tsdf::Fuser::create(gpu.device, gpu.allocator);
  CHECK(fuser.ok());
  codec::EncoderConfig config;
  config.intra_period = 30;
  vr_example::CodecOutputs outputs;
  outputs.stream = path;
  auto coded = vr_example::CodecStream::create(gpu.device, gpu.allocator,
                                               config, outputs);
  CHECK(coded.ok());
  for (std::size_t s = 0; s < kSets; ++s) {
    for (Camera* c : cameras) c->push(s, s == 0);
    auto set = array.poll_set();
    for (int ms = 0; ms < 200 && set.ok() && !set.value(); ++ms) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      set = array.poll_set();
    }
    CHECK(set.ok() && set.value() && set.value()->complete());
    auto prepared = array.process(*set.value());
    CHECK(prepared.ok() && prepared->timestamp_ns == set_time(s));
    CHECK(vr_example::fuse_and_code(*fuser, *grid, *prepared, s, fusion,
                                    &*coded, nullptr)
              .ok());
  }
  CHECK(coded->rows().size() == kSets);
  CHECK(coded->finish().ok());
  return 0;
}

// The stream at `path`: a Geometry record per set, only the first a key,
// each at its set and the set's time, and each decoding. The second set
// on, the left walls are there only if `accumulate`.
int check_stream(vr_test::GpuContext& gpu, const std::string& path,
                 bool accumulate) {
  auto opened = stream::StreamReader::open(path);
  CHECK(opened.ok());
  const stream::StreamReader& reader = **opened;
  CHECK(reader.recovery().finished);
  CHECK(reader.session().tracks.size() == 1);
  CHECK(reader.session().tracks[0].kind == stream::TrackKind::Geometry);
  CHECK(reader.stats()[0].records == kSets && reader.stats()[0].keys == 1);

  auto decoder = codec::Decoder::create(gpu.device, gpu.allocator);
  CHECK(decoder.ok());
  auto extractor = vr::mesh::MarchingCubes::create(gpu.device, gpu.allocator);
  CHECK(extractor.ok());
  std::optional<vr::volume::VoxelBlockGrid> player;
  stream::Position pos = reader.begin();
  stream::Record record;
  std::vector<std::uint8_t> payload;
  for (std::size_t s = 0; s < kSets; ++s) {
    const vkc::Result<bool> more = reader.next(&pos, &record);
    CHECK(more.ok() && *more);
    CHECK(record.info.key == (s == 0));
    CHECK(record.info.seq == s && record.info.set == s);
    CHECK(record.info.pts_ns == set_time(s));
    CHECK(reader.read(record, &payload).ok());
    if (!player) {
      auto made = codec::make_decode_grid(gpu.device, gpu.allocator,
                                          payload.data(), payload.size());
      CHECK(made.ok());
      player.emplace(std::move(made).value());
    }
    CHECK(
        codec::decode_growing(*decoder, payload.data(), payload.size(), *player)
            .ok());
    auto decoded = extractor->extract_host(*player);
    CHECK(decoded.ok());
    const Walls seen = walls(*decoded);
    const bool left = s == 0 || accumulate;
    const bool right = s > 0;
    for (const std::size_t c : {std::size_t{0}, std::size_t{2}}) {
      CHECK((seen[c] > 0) == left);
      CHECK((seen[c + 1] > 0) == right);
    }
  }
  const vkc::Result<bool> end = reader.next(&pos, &record);
  CHECK(end.ok() && !*end);
  return 0;
}

int gpu_main(vr_test::GpuContext& gpu) {
  vr_test::Scratch scratch("recon_example_set_codec_");
  for (const bool accumulate : {false, true}) {
    cfg::Fusion fusion;  // 2 cm voxels, an 8 cm band, Classic
    fusion.accumulate = accumulate;
    const std::string path =
        (scratch.path / (accumulate ? "accumulated.vrst" : "per_set.vrst"))
            .string();
    CHECK(code_sets(gpu, fusion, path) == 0);
    CHECK(check_stream(gpu, path, accumulate) == 0);
  }
  std::puts("example_set_codec: OK");
  return 0;
}

// check_apart: a path that does not exist, or is another file, is taken; a
// link to the stream or the CSV is refused, naming both.
int apart() {
  vr_test::Scratch scratch("recon_example_set_codec_apart_");
  vr_example::CodecOutputs outputs;
  outputs.stream = (scratch.path / "frames.vrst").string();
  outputs.csv = (scratch.path / "rows.csv").string();
  const fs::path other = scratch.path / "other.ply";
  for (const fs::path& file :
       {fs::path(outputs.stream), fs::path(outputs.csv), other}) {
    std::ofstream(file) << "x";
  }
  CHECK(vr_example::check_apart((scratch.path / "rig.ply").string(), outputs)
            .ok());
  CHECK(vr_example::check_apart(other.string(), outputs).ok());
  CHECK(vr_example::check_apart(other.string(), {}).ok());
  for (const std::string* target : {&outputs.stream, &outputs.csv}) {
    const fs::path link =
        scratch.path / (fs::path(*target).filename().string() + ".ply");
    fs::create_symlink(*target, link);
    const vkc::Status refused = vr_example::check_apart(link.string(), outputs);
    CHECK(refused.domain() == vkc::Status::Code::InvalidArgument);
    CHECK(refused.message().find(link.string()) != std::string::npos &&
          refused.message().find(*target) != std::string::npos);
  }
  return 0;
}

}  // namespace

int main() {
  CHECK(apart() == 0);
  return vr_test::run_on_gpu(gpu_main);
}
