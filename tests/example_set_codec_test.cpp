// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// fuse_orbbec's coded sets (examples/common/set_codec.hpp), with no camera
// driver: two fake cameras 3 m apart, read as a sensor array and prepared on
// the device as a rig's are. Each camera sees a back wall 2 m ahead and, in
// the first three sets, a near wall 1 m ahead over the left of its picture,
// which then leaves, so the cameras see past it to the back wall. Nine sets
// fused through fuse_and_code at a fusion.free_after of 3 and a
// fusion.max_weight of 1.5, and coded at an intra period of 30 into a .vrst:
// a Geometry record each, only the first a key, each record's set the set's
// index and its pts the set's time, and every frame decodes through
// codec::decode_growing. The last set is stamped before the one before it,
// as a clock stepping back stamps it, and is coded at that one's time. After
// the second set the heaviest voxel is at the cap, which the near walls' two
// sets pass. Dynamic: the near walls are in the frames that see them and
// gone from the first that sees past them; the free pass after set 5 frees
// the near blocks the last frame to see the walls left out, and the pass
// after set 8 the rest. Classic: no voxel loses its weight, so every later
// frame keeps the near blocks the last frame to see the walls held, and the
// pass after set 5 frees exactly the near blocks that never held an observed
// voxel. The back walls stay in every frame and grid either way, and a
// free_after of 0 frees nothing. The grid holds the same near blocks with a
// stream and without. On the host, a path that is the stream's or the CSV's
// file through a link refused, one beside them taken, and one that cannot be
// checked refused; and the stream decoding only with a CSV or a sweep. The
// device part skips with no device.

#include <algorithm>
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
#include "grid_readback.hpp"
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
constexpr std::size_t kSets = 9;
constexpr std::size_t kNearSets = 3;  // the sets that see the near walls
constexpr int kFreeAfter = 3;
constexpr float kMaxWeight = 1.5F;  // two sets of the near walls pass it
// The sets whose free passes free near blocks, from the grid's clock. It
// starts at 1, and each set's fuse advances it by one after the set's
// allocation: set s asks for blocks at tick s + 1, and a pass after it runs
// at tick s + 2. Set 2 last asks for the near blocks, at tick 3, and the pass
// after it, at tick 4, stamps those holding an observed voxel. The pass after
// set 5, at tick 7, finds the rest 4 ticks old, more than kFreeAfter, and
// frees them; the stamped ones are 3 ticks old and stay. Under Dynamic set 3
// cleared those, so no later pass stamps them, and the pass after set 8, at
// tick 10, frees them.
constexpr std::size_t kUnobservedFreedSet = 5;
constexpr std::size_t kClearedFreedSet = 8;
// A host-clock time, and a set period no frame rate's pts would give.
constexpr std::uint64_t kStartNs = 1'760'000'000'123'456'789;
constexpr std::uint64_t kPeriodNs = 33'366'701;
constexpr double kOffsetM = 3.0;  // camera B's x; camera A's is 0
constexpr double kMidM = 1.5;     // between the near walls, 1 m, and the back

// A set's time: the last set's 1 us before the one before it.
std::uint64_t set_time(std::size_t set) {
  return set + 1 < kSets ? kStartNs + set * kPeriodNs
                         : set_time(set - 1) - 1000;
}

// The pts a set is coded at: its time, or the last frame's when earlier.
std::uint64_t coded_pts(std::size_t set) {
  return set + 1 < kSets ? set_time(set) : set_time(set - 1);
}

// A camera on the host's clock at 30 fps whose frames the test pushes: the
// back wall 2 m ahead, the near wall 1 m ahead over the left of its picture
// in the first kNearSets sets, and its colour as packed words on the host.
class Camera final : public sensor::IRgbdSensor {
 public:
  Camera(std::string id, sensor::SyncRole role) {
    info_.id = std::move(id);
    info_.role = role;
    info_.clock = sensor::ClockDomain::Host;
    info_.fps = 30;
  }

  void push(std::size_t set) {
    struct Pixels {
      std::vector<std::uint16_t> depth;
      std::vector<std::uint32_t> color;
    };
    auto pixels = std::make_shared<Pixels>();
    pixels->depth.assign(std::size_t{kW} * kH, 2000);  // 2 m
    pixels->color.assign(std::size_t{kW} * kH, 0x00806040u);
    if (set < kNearSets) {
      for (std::uint32_t y = 0; y < kH; ++y) {
        for (std::uint32_t x = 0; x < kW / 2; ++x) {
          pixels->depth[std::size_t{y} * kW + x] = 1000;  // 1 m
        }
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

// What a grid holds of the walls: its surface's vertices on each, A's near
// and back walls then B's, and the coordinates of its blocks nearer than
// kMidM, the near walls' and their bands', sorted.
struct Walls {
  std::array<std::size_t, 4> vertices{};
  std::vector<vr::Vec3i> near_blocks;

  bool near() const { return vertices[0] > 0 && vertices[2] > 0; }
  bool no_near() const { return vertices[0] == 0 && vertices[2] == 0; }
  bool back() const { return vertices[1] > 0 && vertices[3] > 0; }
};

int read_walls(vr::mesh::MarchingCubes& extractor,
               vr::volume::VoxelBlockGrid& grid, Walls* out) {
  *out = {};
  auto mesh = extractor.extract_host(grid);
  CHECK(mesh.ok());
  for (const vr::mesh::Vertex& v : mesh->vertices) {
    const bool b = v.position.x > kOffsetM / 2;
    ++out->vertices[(b ? 2 : 0) + (v.position.z > kMidM ? 1 : 0)];
  }
  auto blocks = grid.map().compact_active_blocks();
  CHECK(blocks.ok());
  const vr::volume::VoxelGridParams& params = grid.map().grid();
  const double extent = double(params.voxel_size) * params.block_size;
  for (const vr::volume::BlockIndex& block : *blocks) {
    if ((block.coord.z + 0.5) * extent < kMidM) {
      out->near_blocks.push_back(block.coord);
    }
  }
  std::sort(out->near_blocks.begin(), out->near_blocks.end(),
            vr::volume::coord_less);
  return 0;
}

// Whether every block of `part` is in `whole`, both sorted.
bool holds(const std::vector<vr::Vec3i>& whole,
           const std::vector<vr::Vec3i>& part) {
  return std::includes(whole.begin(), whole.end(), part.begin(), part.end(),
                       vr::volume::coord_less);
}

// Push, poll and prepare kSets sets, fusing each through fuse_and_code and
// coding it into a new stream at `path`, unless empty, and check the grid
// after each, kept in `grids`: after the second set its heaviest voxel at
// fusion.max_weight, which the near walls' two sets pass; the back walls
// always; the near walls' surface while the cameras see them, and under
// Dynamic not after; and the near walls' blocks all kept until the pass
// after kUnobservedFreedSet, then some (check_stream says which) until,
// under Dynamic, the pass after kClearedFreedSet frees the rest; with no
// pass, all kept.
int code_sets(vr_test::GpuContext& gpu, const cfg::Fusion& fusion,
              const std::string& path, std::vector<Walls>* grids) {
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

  // 1024 buckets hold the walls' blocks (at most 2250) without a grow. The
  // examples' default table has 256 MiB attributes, past the 2^27
  // maxStorageBufferRange Vulkan guarantees, and lavapipe refuses that grid.
  auto grid = vr_example::create_fusion_grid(gpu.device, gpu.allocator,
                                             fusion.voxel_size_m,
                                             fusion.truncation_m, 1024);
  CHECK(grid.ok());
  auto fuser = vr::tsdf::Fuser::create(gpu.device, gpu.allocator);
  CHECK(fuser.ok());
  auto extractor = vr::mesh::MarchingCubes::create(gpu.device, gpu.allocator);
  CHECK(extractor.ok());
  std::optional<vr_example::CodecStream> coded;
  if (!path.empty()) {
    codec::EncoderConfig config;
    config.intra_period = 30;
    vr_example::CodecOutputs outputs;
    outputs.stream = path;
    auto made = vr_example::CodecStream::create(gpu.device, gpu.allocator,
                                                config, outputs);
    CHECK(made.ok());
    coded.emplace(std::move(made).value());
  }
  grids->clear();
  for (std::size_t s = 0; s < kSets; ++s) {
    for (Camera* c : cameras) c->push(s);
    auto set = array.poll_set();
    for (int ms = 0; ms < 200 && set.ok() && !set.value(); ++ms) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      set = array.poll_set();
    }
    CHECK(set.ok() && set.value() && set.value()->complete());
    auto prepared = array.process(*set.value());
    CHECK(prepared.ok() && prepared->timestamp_ns == set_time(s));
    CHECK(vr_example::fuse_and_code(*fuser, *grid, *prepared, s, fusion,
                                    coded ? &*coded : nullptr, nullptr)
              .ok());
    if (s == 1) {  // read once: the whole attribute is slow in Debug
      auto weights = vr_test::read_attribute<float>(gpu.device, gpu.allocator,
                                                    *grid, "weight");
      CHECK(weights.ok() && !weights->empty());
      CHECK(*std::max_element(weights->begin(), weights->end()) ==
            fusion.max_weight);
    }

    grids->emplace_back();
    Walls& walls = grids->back();
    CHECK(read_walls(*extractor, *grid, &walls) == 0);
    const Walls& first = grids->front();
    CHECK(walls.back() && !first.near_blocks.empty());
    if (s < kNearSets) CHECK(walls.near());
    if (s >= kNearSets && fusion.dynamic) CHECK(walls.no_near());
    if (s < kUnobservedFreedSet || fusion.free_after == 0) {
      CHECK(walls.near_blocks == first.near_blocks);
    } else if (fusion.dynamic && s >= kClearedFreedSet) {
      CHECK(walls.near_blocks.empty());
    } else {
      CHECK(!walls.near_blocks.empty() &&
            walls.near_blocks.size() < first.near_blocks.size() &&
            holds(first.near_blocks, walls.near_blocks));
    }
  }
  if (coded) {
    CHECK(coded->rows().size() == kSets);
    CHECK(coded->finish().ok());
  }
  return 0;
}

// The stream at `path`, coded from `grids`: a Geometry record per set, only
// the first a key, each at its set and coded pts, and each decoding. Every
// frame holds the back walls, and the near walls while the cameras see them.
// After that, under Dynamic, the frames hold no near block, since the codec
// leaves out a block without an observed voxel, and from the pass after
// kUnobservedFreedSet until the one after kClearedFreedSet the grid holds
// exactly the near blocks the last frame to see the near walls held. Under
// Classic each frame holds every near block that frame held, since no weight
// falls, and from the pass after kUnobservedFreedSet the grid holds exactly
// the frame's near blocks: the pass frees those that never held an observed
// voxel and keeps the rest.
int check_stream(vr_test::GpuContext& gpu, const std::string& path,
                 bool dynamic, const std::vector<Walls>& grids) {
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
  constexpr std::uint32_t kMaxBlocks = 1u << 20;  // far past the scene's
  stream::Position pos = reader.begin();
  stream::Record record;
  std::vector<std::uint8_t> payload;
  Walls last_seen;  // the frame of the last set to see the near walls
  for (std::size_t s = 0; s < kSets; ++s) {
    const vkc::Result<bool> more = reader.next(&pos, &record);
    CHECK(more.ok() && *more);
    CHECK(record.info.key == (s == 0));
    CHECK(record.info.seq == s && record.info.set == s);
    CHECK(record.info.pts_ns == coded_pts(s));
    CHECK(reader.read(record, &payload).ok());
    if (!player) {
      auto made =
          codec::make_decode_grid(gpu.device, gpu.allocator, payload.data(),
                                  payload.size(), kMaxBlocks);
      CHECK(made.ok());
      player.emplace(std::move(made).value());
    }
    CHECK(codec::decode_growing(*decoder, payload.data(), payload.size(),
                                *player, kMaxBlocks)
              .ok());

    Walls walls;
    CHECK(read_walls(*extractor, *player, &walls) == 0);
    CHECK(walls.back());
    if (s < kNearSets) {
      CHECK(walls.near() && !walls.near_blocks.empty());
      last_seen = walls;
    } else if (dynamic) {
      CHECK(walls.no_near() && walls.near_blocks.empty());
      if (s >= kUnobservedFreedSet && s < kClearedFreedSet) {
        CHECK(grids[s].near_blocks == last_seen.near_blocks);
      }
    } else {
      CHECK(holds(walls.near_blocks, last_seen.near_blocks));
      if (s >= kUnobservedFreedSet) {
        CHECK(walls.near_blocks == grids[s].near_blocks);
      }
    }
  }
  const vkc::Result<bool> end = reader.next(&pos, &record);
  CHECK(end.ok() && !*end);
  return 0;
}

int gpu_main(vr_test::GpuContext& gpu) {
  vr_test::Scratch scratch("recon_example_set_codec_");
  for (const bool dynamic : {true, false}) {
    cfg::Fusion fusion;  // 2 cm voxels, an 8 cm band
    fusion.dynamic = dynamic;
    fusion.free_after = kFreeAfter;
    fusion.max_weight = kMaxWeight;
    const std::string path =
        (scratch.path / (dynamic ? "dynamic.vrst" : "classic.vrst")).string();
    std::vector<Walls> coded;
    CHECK(code_sets(gpu, fusion, path, &coded) == 0);
    CHECK(check_stream(gpu, path, dynamic, coded) == 0);
    // Fused alone, the grid holds the same near blocks after every set.
    std::vector<Walls> alone;
    CHECK(code_sets(gpu, fusion, "", &alone) == 0);
    for (std::size_t s = 0; s < kSets; ++s) {
      CHECK(alone[s].near_blocks == coded[s].near_blocks);
    }
  }
  cfg::Fusion never;  // Dynamic, freeing nothing
  never.dynamic = true;
  never.free_after = 0;
  never.max_weight = kMaxWeight;
  std::vector<Walls> kept;
  CHECK(code_sets(gpu, never, "", &kept) == 0);
  std::puts("example_set_codec: OK");
  return 0;
}

// check_apart: a path that does not exist, or is another file, or beside an
// output that does not exist, is taken; a link to the stream or the CSV is
// refused, naming both; and a path that cannot be looked up is refused.
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
  vr_example::CodecOutputs missing;
  missing.stream = (scratch.path / "missing.vrst").string();
  CHECK(vr_example::check_apart(other.string(), missing).ok());
  // A name past every file system's limit cannot be looked up.
  const fs::path unknowable = scratch.path / std::string(300, 'x');
  CHECK(vr_example::check_apart(unknowable.string(), outputs).domain() ==
        vkc::Status::Code::IoError);
  return 0;
}

// set_codec_outputs: codec_outputs' paths and port, decoding only with
// output.csv or codec.sweep, not for a recording or a server alone.
int decodes() {
  cfg::Config opt;
  opt.output.stream = "rig.vrst";
  opt.output.serve = 47801;
  const vr_example::CodecOutputs live = vr_example::set_codec_outputs(opt);
  CHECK(!live.decode && live.stream == "rig.vrst" && live.csv.empty());
  CHECK(live.serve && live.serve->port == 47801);
  cfg::Config listed = opt;
  listed.output.csv = "rig.csv";
  const vr_example::CodecOutputs rows = vr_example::set_codec_outputs(listed);
  CHECK(rows.decode && rows.csv == "rig.csv");
  cfg::Config swept = opt;
  swept.codec.sweep = true;
  CHECK(vr_example::set_codec_outputs(swept).decode);
  return 0;
}

}  // namespace

int main() {
  CHECK(apart() == 0);
  CHECK(decodes() == 0);
  return vr_test::run_on_gpu(gpu_main);
}
