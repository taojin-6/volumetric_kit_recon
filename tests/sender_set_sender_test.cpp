// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// A sender's coded sets (sender/set_sender.hpp), with no camera driver: two
// fake cameras 3 m apart, read as a sensor array and prepared on the device
// as a rig's are. Each camera sees a back wall 2 m ahead and, in the first
// three sets, a near wall 1 m ahead over the left of its picture, which then
// leaves, so the cameras see past it to the back wall. Nine sets sent through
// sender::SetSender, a free pass every 3 sets and a max_weight of 1.5, and
// coded at an intra period of 30 into a .vrst:
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
// stream and without. A coded set is 2 submits and at most 2 host waits, a
// set fused alone 1 and 1, a free pass's adding none, a set fused again 3
// more or 2 and one for each further round its allocation takes, warm sets
// no staging buffer but a payload buffer's growth, at most two, and the front
// batch's dispatches stay below 80, at two cameras and at a rig's four; the
// blocking calls code the same frames, and so do they on a grid of 32 buckets,
// whose first set's fusion is done again and its front half, its wait timed.
// Each set is published once, its frame's type and rows those the frame went
// out with, the front batch's device rows the frame's only for its front
// half. Served without replay_gop, keys on demand: with no viewer
// joining, the frames follow the intra period; viewers joining before a set
// make its frame a key, one however many join, the period following from it,
// through the sender and through code, a set whose fusion is done again
// included, and each viewer's first frame is that key. A sender over a
// moved-from stream is refused. On a grid holding a sphere, with no frames to
// fuse, a sender's frames are the file CodecStream::code writes, byte for
// byte, frames the decoder refuses and one out of order included. The device
// part skips with no device.

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <initializer_list>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "volumetric_kit/core/testing/gpu_test.hpp"
#include "volumetric_kit/core/testing/scratch_dir.hpp"
#include "volumetric_kit/core/testing/test_check.hpp"
#include "volumetric_kit/core/vulkan/command_batch.hpp"
#include "volumetric_kit/grid/codec/decode_grid.hpp"
#include "volumetric_kit/grid/codec/decoder.hpp"
#include "volumetric_kit/grid/codec/encoder.hpp"
#include "volumetric_kit/grid/codec_stream/codec_stream.hpp"
#include "volumetric_kit/grid/mesh/marching_cubes.hpp"
#include "volumetric_kit/grid/mesh/mesh.hpp"
#include "volumetric_kit/grid/stream/stream_reader.hpp"
#include "volumetric_kit/grid/testing/codec_fixture.hpp"
#include "volumetric_kit/grid/testing/grid_readback.hpp"
#include "volumetric_kit/grid/testing/networking_loopback.hpp"
#include "volumetric_kit/grid/testing/stage_rows.hpp"
#include "volumetric_kit/recon/camera/array_calibration.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"
#include "volumetric_kit/recon/sender/fuse_inputs.hpp"
#include "volumetric_kit/recon/sender/set_sender.hpp"
#include "volumetric_kit/recon/sensor/array/sensor_array.hpp"
#include "volumetric_kit/recon/tsdf/fuser.hpp"
#include "volumetric_kit/recon/tsdf/fusion_grid.hpp"

namespace vr = volumetric_kit::recon;

namespace vgrid = volumetric_kit::grid;
namespace vkc = volumetric_kit::core;
namespace camera = volumetric_kit::recon::camera;
namespace codec = volumetric_kit::grid::codec;
namespace codec_stream = volumetric_kit::grid::codec_stream;
namespace sender = volumetric_kit::recon::sender;
namespace sensor = volumetric_kit::recon::sensor;
namespace stream = volumetric_kit::grid::stream;
namespace networking = volumetric_kit::grid::networking;
namespace net = vgrid_test::net;

namespace {

// How the sets fuse and free: 2 cm voxels and an 8 cm band, Classic or
// Dynamic, a free pass every `free_after` sets (none at 0), and a running
// average capped at `max_weight`.
struct Fusion {
  float voxel_size_m = 0.02F;
  float truncation_m = 0.08F;
  bool dynamic = false;
  int free_after = 0;
  float max_weight = 5.0F;
};

vr::tsdf::IntegrationMode mode_of(const Fusion& fusion) {
  return fusion.dynamic ? vr::tsdf::IntegrationMode::Dynamic
                        : vr::tsdf::IntegrationMode::Classic;
}

// A stream's session: one Geometry track at 30 a second.
stream::Session geometry_session() {
  stream::Session session;
  session.writer = "recon_sender_set_sender";
  stream::Track track;
  track.kind = stream::TrackKind::Geometry;
  track.rate = {30, 1};
  session.tracks.push_back(track);
  return session;
}

// A set as the test sees it: its rows, its device counters from its begin to
// its publish, and how many times it was published.
struct LoggedSet {
  explicit LoggedSet(vkc::Device& device)
      : device_(&device), begin(device.counters()) {}
  void published() {
    ++publishes;
    end = device_->counters();
  }
  vkc::DeviceCounters counted() const {
    vkc::DeviceCounters d;
    d.submits = end.submits - begin.submits;
    d.host_waits = end.host_waits - begin.host_waits;
    d.staging_buffers = end.staging_buffers - begin.staging_buffers;
    d.dispatches = end.dispatches - begin.dispatches;
    d.barriers = end.barriers - begin.barriers;
    return d;
  }

  vkc::Device* device_;
  vkc::DeviceCounters begin;
  vkc::DeviceCounters end;
  vkc::StageMetrics rows;
  int publishes = 0;
};

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
// Before set `patch_until`, it sees only a 4x4-pixel patch of the back wall.
class Camera final : public sensor::IRgbdSensor {
 public:
  Camera(std::string id, sensor::SyncRole role, std::size_t patch_until = 0)
      : patch_until_(patch_until) {
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
    if (set < patch_until_) {
      for (std::uint32_t y = 0; y < kH; ++y) {
        for (std::uint32_t x = 0; x < kW; ++x) {
          const bool patch =
              x >= kW / 2 && x < kW / 2 + 4 && y >= kH / 2 && y < kH / 2 + 4;
          if (!patch) pixels->depth[std::size_t{y} * kW + x] = 0;  // none
        }
      }
    } else if (set < kNearSets) {
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
  std::size_t patch_until_;
  sensor::SensorInfo info_;
  std::vector<sensor::RgbdFrame> frames_;
};

// `count` cameras, A and B 3 m apart, a rig's third and fourth at A's and
// B's poses, each seeing only a patch before set `patch_until`, read as a
// started sensor array on `gpu`'s device; `cameras` takes them, to push to.
vkc::Result<sensor::SensorArray> open_cameras(vkc::test::GpuContext& gpu,
                                              std::size_t count,
                                              std::size_t patch_until,
                                              std::vector<Camera*>* cameras) {
  std::vector<std::unique_ptr<sensor::IRgbdSensor>> sensors;
  sensor::SensorArray::Options options;
  for (std::size_t c = 0; c < count; ++c) {
    const std::string id(1, static_cast<char>('A' + c));
    auto made = std::make_unique<Camera>(
        id, c == 0 ? sensor::SyncRole::Primary : sensor::SyncRole::Secondary,
        patch_until);
    cameras->push_back(made.get());
    sensors.push_back(std::move(made));
    camera::SensorCalibration posed;
    posed.id = id;
    posed.color_to_world = camera::Mat4d(1.0);
    (*posed.color_to_world)[3] =
        glm::dvec4(c % 2 == 0 ? 0.0 : kOffsetM, 0.0, 0.0, 1.0);
    options.calibration.sensors.push_back(posed);
  }
  options.device = &gpu.device;
  options.allocator = &gpu.allocator;
  VKC_ASSIGN(sensor::SensorArray array,
             sensor::SensorArray::open(std::move(sensors), options));
  VKC_TRY(array.start());
  return array;
}

// Set `s`, pushed to every camera and polled from `array` once whole, within
// 200 ms; none when it is not.
std::optional<sensor::FrameSet> next_set(sensor::SensorArray& array,
                                         const std::vector<Camera*>& cameras,
                                         std::size_t s) {
  for (Camera* c : cameras) c->push(s);
  auto set = array.poll_set();
  for (int ms = 0; ms < 200 && set.ok() && !set.value(); ++ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    set = array.poll_set();
  }
  if (!set.ok() || !set.value() || !set.value()->complete()) {
    return std::nullopt;
  }
  return std::move(set).value();
}

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

int read_walls(vgrid::mesh::MarchingCubes& extractor,
               vgrid::volume::VoxelBlockGrid& grid, Walls* out) {
  *out = {};
  auto mesh = extractor.extract_host(grid);
  CHECK(mesh.ok());
  for (const vgrid::mesh::Vertex& v : mesh->vertices) {
    const bool b = v.position.x > kOffsetM / 2;
    ++out->vertices[(b ? 2 : 0) + (v.position.z > kMidM ? 1 : 0)];
  }
  auto blocks = grid.map().compact_active_blocks();
  CHECK(blocks.ok());
  const vgrid::volume::VoxelGridParams& params = grid.map().grid();
  const double extent = double(params.voxel_size) * params.block_size;
  for (const vgrid::volume::BlockIndex& block : *blocks) {
    if ((block.coord.z + 0.5) * extent < kMidM) {
      out->near_blocks.push_back(block.coord);
    }
  }
  std::sort(out->near_blocks.begin(), out->near_blocks.end(),
            vgrid::volume::coord_less);
  return 0;
}

// Whether every block of `part` is in `whole`, both sorted.
bool holds(const std::vector<vr::Vec3i>& whole,
           const std::vector<vr::Vec3i>& part) {
  return std::includes(whole.begin(), whole.end(), part.begin(), part.end(),
                       vgrid::volume::coord_less);
}

// What one set cost on the device, as Device::counters count it, and whether
// its fusion was done again.
struct SetCounts {
  std::uint64_t submits = 0;
  std::uint64_t host_waits = 0;
  std::uint64_t staging = 0;
  std::uint64_t dispatches = 0;
  std::uint64_t barriers = 0;
  bool redone = false;
  bool tail = false;         // its payload outgrew its guess (Encoder::finish)
  bool redo_waited = false;  // a "wait: redo" row: its front half again
};

// Whether `sent`, a set sent through the sender, went out as the set's own
// rows and publish say: published once; coded, with a stream, its frame of
// the type the file records, its encode rows both halves' and in the set's
// rows with the write's and the waits', and its decode rows; not coded,
// without one. Of the front batch's device rows, the front half's are the
// frame's, the set's holding them once, and the prep's and the fusion's the
// set's alone.
bool sent_as_logged(const sender::SetSender::Sent& sent,
                    const LoggedSet& logged, bool coded) {
  if (logged.publishes != 1 || sent.coded.has_value() != coded) return false;
  const vkc::StageMetrics& rows = logged.rows;
  for (const char* row : {"frame prep", "wait: front"}) {
    if (vgrid_test::find_row(rows, row) == nullptr) return false;
  }
  if (!coded) return vgrid_test::find_row(rows, "write") == nullptr;
  for (const char* row : {"codec encode", "  ..forward", "  ..tables", "crc",
                          "write", "wait: frame"}) {
    if (vgrid_test::find_row(rows, row) == nullptr) return false;
  }
  const vkc::StageRow* front =
      vgrid_test::find_row(sent.coded->encode, "  ..forward");
  const vkc::StageRow* in_set = vgrid_test::find_row(rows, "  ..forward");
  return front != nullptr && front->device != vkc::DeviceTime::None &&
         in_set->device == front->device && in_set->gpu_ms == front->gpu_ms &&
         vgrid_test::find_row(sent.coded->encode, "frame prep") == nullptr &&
         vgrid_test::find_row(sent.coded->encode, "integrate") == nullptr &&
         vgrid_test::find_row(sent.coded->encode, "  ..tables") != nullptr &&
         vgrid_test::find_row(sent.coded->decode, "codec decode") != nullptr &&
         sent.coded->bytes > 0 && sent.coded->blocks > 0;
}

// How code_sets sends its sets: through the sender, or the blocking calls a
// set took before it; into a grid of `buckets`; each set's counts kept; from
// `cameras` cameras, A and B 3 m apart, a rig's third and fourth at A's and
// B's poses.
struct Run {
  bool sender = true;
  std::int32_t buckets = 1024;
  std::vector<SetCounts>* counts = nullptr;
  std::size_t cameras = 2;
};

// Push, poll and prepare kSets sets, sending each through the sender
// (sender::SetSender), or the blocking calls with `run`, and coding it
// into a new stream at `path`, unless empty, and check the grid
// after each, kept in `grids`: after the second set its heaviest voxel at
// fusion.max_weight, which the near walls' two sets pass; the back walls
// always; the near walls' surface while the cameras see them, and under
// Dynamic not after; and the near walls' blocks all kept until the pass
// after kUnobservedFreedSet, then some (check_stream says which) until,
// under Dynamic, the pass after kClearedFreedSet frees the rest; with no
// pass, all kept. Each sent set as sent_as_logged says.
int code_sets(vkc::test::GpuContext& gpu, const Fusion& fusion,
              const std::string& path, std::vector<Walls>* grids,
              const Run& run = {}) {
  std::vector<Camera*> cameras;
  auto opened = open_cameras(gpu, run.cameras, 0, &cameras);
  CHECK(opened.ok());
  sensor::SensorArray array = std::move(opened).value();

  // 1024 buckets hold the walls' blocks (at most 2250) without a grow, on
  // every device alike: the default start is capped by the device's range.
  auto grid = vr::tsdf::create_fusion_grid(gpu.device, gpu.allocator,
                                           fusion.voxel_size_m,
                                           fusion.truncation_m, run.buckets);
  CHECK(grid.ok());
  auto fuser = vr::tsdf::Fuser::create(gpu.device, gpu.allocator);
  CHECK(fuser.ok());
  auto extractor =
      vgrid::mesh::MarchingCubes::create(gpu.device, gpu.allocator);
  CHECK(extractor.ok());
  std::optional<codec_stream::CodecStream> coded;
  if (!path.empty()) {
    codec::EncoderConfig config;
    config.intra_period = 30;
    codec_stream::CodecOutputs outputs;
    outputs.stream = path;
    auto made = codec_stream::CodecStream::create(
        gpu.device, gpu.allocator, config, geometry_session(), outputs);
    CHECK(made.ok());
    coded.emplace(std::move(made).value());
  }
  sender::SetSender::Config sending;
  sending.max_weight = fusion.max_weight;
  sending.mode = mode_of(fusion);
  sending.free_after = fusion.free_after;
  auto out = sender::SetSender::create(gpu.device, *fuser, *grid,
                                       coded ? &*coded : nullptr, sending);
  CHECK(out.ok());
  grids->clear();
  for (std::size_t s = 0; s < kSets; ++s) {
    std::optional<sensor::FrameSet> set = next_set(array, cameras, s);
    CHECK(set.has_value());
    LoggedSet logged(gpu.device);
    std::optional<sender::SetSender::Sent> sent;
    if (run.sender) {
      // The sender's set: the prep recorded into its front batch.
      vkc::CommandBatch front(gpu.device, gpu.allocator);
      front.time(out->timer());
      auto prepared = array.record_process(front, *set, &logged.rows);
      CHECK(prepared.ok() && prepared->timestamp_ns == set_time(s));
      const std::uint64_t pts_ns =
          std::max(prepared->timestamp_ns, coded ? coded->last_pts_ns() : 0);
      auto done =
          out->send(front, prepared->frames,
                    std::make_shared<sensor::FrameSet>(std::move(*set)), s,
                    pts_ns, &logged.rows, [&logged] { logged.published(); });
      if (!done.ok()) {
        std::fprintf(stderr, "set %zu: %s\n", s,
                     done.status().message().c_str());
      }
      CHECK(done.ok());
      CHECK(sent_as_logged(*done, logged, coded.has_value()));
      sent = std::move(done).value();
    } else {
      // The blocking calls, each a batch of its own, as a set was sent
      // before the sender.
      auto prepared = array.process(*set);
      CHECK(prepared.ok() && prepared->timestamp_ns == set_time(s));
      const sender::FuseInputs in = sender::fuse_inputs(prepared->frames);
      CHECK(fuser
                ->fuse(*grid, in.frames, fusion.max_weight, mode_of(fusion),
                       nullptr)
                .ok());
      if (fusion.free_after > 0 &&
          (s + 1) % static_cast<std::size_t>(fusion.free_after) == 0) {
        CHECK(grid->free_stale_blocks(
                      static_cast<std::uint32_t>(fusion.free_after), nullptr)
                  .ok());
      }
      if (coded) {
        CHECK(coded
                  ->code(*grid, s,
                         std::max(prepared->timestamp_ns, coded->last_pts_ns()))
                  .ok());
      }
    }
    // The set's own, from its begin to its publish: a decode after it is
    // the stream's diagnostic, not the set's.
    if (run.counts != nullptr) {
      CHECK(sent.has_value());
      const vkc::DeviceCounters c = logged.counted();
      run.counts->push_back(
          {c.submits, c.host_waits, c.staging_buffers, c.dispatches, c.barriers,
           sent->fused.redone, sent->coded && sent->coded->tail,
           vgrid_test::find_row(logged.rows, "wait: redo") != nullptr});
    }
    if (s == 1) {  // read once: the whole attribute is slow in Debug
      auto weights = vgrid_test::read_attribute<float>(
          gpu.device, gpu.allocator, *grid, vgrid::volume::kWeightAttribute);
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
    CHECK(coded->coded() == kSets);
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
int check_stream(vkc::test::GpuContext& gpu, const std::string& path,
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
  auto extractor =
      vgrid::mesh::MarchingCubes::create(gpu.device, gpu.allocator);
  CHECK(extractor.ok());
  std::optional<vgrid::volume::VoxelBlockGrid> player;
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

// Whether the streams at `a` and `b` hold the same records, info and bytes.
int same_streams(const std::string& a, const std::string& b) {
  auto ra = stream::StreamReader::open(a);
  auto rb = stream::StreamReader::open(b);
  CHECK(ra.ok() && rb.ok());
  stream::Position pa = (*ra)->begin();
  stream::Position pb = (*rb)->begin();
  for (;;) {
    stream::Record x;
    stream::Record y;
    const vkc::Result<bool> more_a = (*ra)->next(&pa, &x);
    const vkc::Result<bool> more_b = (*rb)->next(&pb, &y);
    CHECK(more_a.ok() && more_b.ok() && *more_a == *more_b);
    if (!*more_a) return 0;
    CHECK(x.info.key == y.info.key && x.info.seq == y.info.seq &&
          x.info.set == y.info.set && x.info.pts_ns == y.info.pts_ns);
    std::vector<std::uint8_t> px;
    std::vector<std::uint8_t> py;
    CHECK((*ra)->read(x, &px).ok() && (*rb)->read(y, &py).ok() && px == py);
  }
}

// A set's device counts, printed: a coded set is 2 submits and at most 2
// host waits -- the front batch's and the frame's; a wait that finds its
// batch done blocks nothing and counts none -- and a set fused alone 1 and
// 1, a free pass's set adding none. A payload past its guess adds its tail's
// submit, wait and gather. A warm set places no staging buffer but the one a
// payload buffer's growth takes, for a frame larger than any before: at most
// two over the run's warm sets, where a readback of every set's would place
// six. The front batch's dispatches stay below the Apple step near 80 a
// batch (the 2026-10-24 entry); the back batch's are 4. A set fused again
// adds its blocking allocation, a submit a round while lost races remain,
// and its fuse, and a coded one its front half again, whose wait it times.
// The first set's round, into an empty grid, can lose a race, and the set is
// then fused again.
int counts_case(const std::vector<SetCounts>& counts, bool coded,
                const char* what) {
  std::uint64_t warm_staging = 0;
  for (std::size_t s = 0; s < counts.size(); ++s) {
    const SetCounts& c = counts[s];
    std::printf(
        "  %s set %zu: %llu submits, %llu waits, %llu staging, %llu "
        "dispatches, %llu barriers%s%s\n",
        what, s, static_cast<unsigned long long>(c.submits),
        static_cast<unsigned long long>(c.host_waits),
        static_cast<unsigned long long>(c.staging),
        static_cast<unsigned long long>(c.dispatches),
        static_cast<unsigned long long>(c.barriers),
        c.redone ? ", fused again" : "", c.tail ? ", a payload tail" : "");
    CHECK(c.redo_waited == (coded && c.redone));
    const std::uint64_t tail = c.tail ? 1 : 0;
    const std::uint64_t batches = (coded ? 2 : 1) + tail;
    if (s >= 3) {
      CHECK(c.staging <= 1);
      warm_staging += c.staging;
    }
    if (c.redone) {
      // Its fuse and a coded set's front half again, and its allocation's
      // rounds, a submit each while lost races remain: one to ten
      // (VoxelHashMap::allocate_from_depth).
      const std::uint64_t again = coded ? 2 : 1;
      CHECK(c.submits > batches + again && c.submits <= batches + again + 10 &&
            c.host_waits <= c.submits);
      continue;
    }
    CHECK(c.submits == batches && c.host_waits <= batches);
    const std::uint64_t front = coded ? c.dispatches - 4 - tail : c.dispatches;
    CHECK(front < 80);
  }
  CHECK(warm_staging <= 2);
  return 0;
}

// How keyed_sets sends its sets: through the sender, or the blocking calls;
// into a grid of `buckets`; the cameras seeing only a patch before set
// `patch_until`; coded at intra period `period`; and the viewers that join
// before set s, joins[s] of them.
struct Keyed {
  bool sender = true;
  std::int32_t buckets = 1024;
  std::size_t patch_until = 0;
  std::uint32_t period = 4;
  std::array<int, kSets> joins{};
};

// Send kSets sets from two cameras as `run` says, Dynamic, freeing nothing,
// each coded into a new stream at `path` and served on 127.0.0.1 without
// replay_gop, as a live sender serves, each joiner counted before the next
// set is sent. Each viewer gets the frame of the set it joined before, a key,
// then every frame after it. `keys` takes whether each set's record is a key,
// from the file, and `redone` whether each set's fusion was done again.
int keyed_sets(vkc::test::GpuContext& gpu, const Keyed& run,
               const std::string& path, std::vector<bool>* keys,
               std::vector<bool>* redone) {
  std::vector<Camera*> cameras;
  auto opened = open_cameras(gpu, 2, run.patch_until, &cameras);
  CHECK(opened.ok());
  sensor::SensorArray array = std::move(opened).value();
  Fusion fusion;  // 2 cm voxels, an 8 cm band
  fusion.dynamic = true;
  fusion.max_weight = kMaxWeight;
  auto grid = vr::tsdf::create_fusion_grid(gpu.device, gpu.allocator,
                                           fusion.voxel_size_m,
                                           fusion.truncation_m, run.buckets);
  CHECK(grid.ok());
  auto fuser = vr::tsdf::Fuser::create(gpu.device, gpu.allocator);
  CHECK(fuser.ok());
  codec::EncoderConfig config;
  config.intra_period = run.period;
  codec_stream::CodecOutputs outputs;
  outputs.stream = path;
  outputs.serve.emplace();
  outputs.serve->host = "127.0.0.1";
  outputs.serve->port = 0;
  outputs.serve->replay_gop = false;
  auto coded = codec_stream::CodecStream::create(
      gpu.device, gpu.allocator, config, geometry_session(), outputs);
  CHECK(coded.ok() && coded->server() != nullptr);
  const networking::StreamServer& server = *coded->server();
  sender::SetSender::Config sending;
  sending.max_weight = fusion.max_weight;
  sending.mode = mode_of(fusion);
  auto out = sender::SetSender::create(gpu.device, *fuser, *grid,
                                       &coded.value(), sending);
  CHECK(out.ok());
  struct Viewer {
    std::size_t set = 0;  // the set it joined before
    std::unique_ptr<networking::StreamClient> client;
  };
  std::vector<Viewer> viewers;
  redone->clear();
  for (std::size_t s = 0; s < kSets; ++s) {
    for (int j = 0; j < run.joins[s]; ++j) {
      // Counted before its session is sent.
      viewers.push_back({s, net::connect(server.port())});
      CHECK(viewers.back().client != nullptr &&
            net::wait_session(*viewers.back().client));
      CHECK(server.key_wanted() == viewers.size());
    }
    std::optional<sensor::FrameSet> set = next_set(array, cameras, s);
    CHECK(set.has_value());
    bool fused_again = false;
    if (run.sender) {
      vkc::CommandBatch front(gpu.device, gpu.allocator);
      front.time(out->timer());
      auto prepared = array.record_process(front, *set, nullptr);
      CHECK(prepared.ok());
      const std::uint64_t pts_ns =
          std::max(prepared->timestamp_ns, coded->last_pts_ns());
      auto sent = out->send(front, prepared->frames,
                            std::make_shared<sensor::FrameSet>(std::move(*set)),
                            s, pts_ns);
      CHECK(sent.ok() && sent->coded.has_value());
      fused_again = sent->fused.redone;
    } else {
      auto prepared = array.process(*set);
      CHECK(prepared.ok());
      const sender::FuseInputs in = sender::fuse_inputs(prepared->frames);
      CHECK(fuser
                ->fuse(*grid, in.frames, fusion.max_weight, mode_of(fusion),
                       nullptr)
                .ok());
      CHECK(coded
                ->code(*grid, s,
                       std::max(prepared->timestamp_ns, coded->last_pts_ns()))
                .ok());
    }
    redone->push_back(fused_again);
    // Read as it is published, so no viewer is behind when a key comes.
    for (const Viewer& viewer : viewers) {
      net::Got got;
      CHECK(net::next_within(*viewer.client, &got));
      CHECK(got.record.info.set == s && got.record.info.seq == s);
      CHECK(s != viewer.set || got.record.info.key);
    }
  }
  CHECK(coded->finish().ok());
  auto reader = stream::StreamReader::open(path);
  CHECK(reader.ok());
  stream::Position pos = (*reader)->begin();
  stream::Record record;
  keys->clear();
  for (;;) {
    const vkc::Result<bool> more = (*reader)->next(&pos, &record);
    CHECK(more.ok());
    if (!*more) break;
    CHECK(record.info.set == keys->size());
    keys->push_back(record.info.key);
  }
  CHECK(keys->size() == kSets);
  return 0;
}

// Keys on demand, through the sender and through code, served without
// replay_gop: with no viewer joining or falling back, the frames
// follow the intra period of 4, keys at sets 0, 4 and 8; three viewers
// joining before set 2 make its frame a key, one key, and the frames after it
// follow the period from it, a key at 6; one joining before set 7 keys it
// too, though 6 was a key. A key per join would key 3 and 4 as well, and a
// stream keyed while any viewer is counted every set from 2. On a grid of 32
// buckets, the cameras seeing only a patch until set 3, set 3's fusion is
// done again, and its frame's front half after it: a viewer joining before
// it makes that frame a key still, where with none it is a P-frame.
int keys_case(vkc::test::GpuContext& gpu, const vkc::test::Scratch& scratch) {
  const auto keys_at = [](std::initializer_list<std::size_t> sets) {
    std::vector<bool> keys(kSets, false);
    for (const std::size_t s : sets) keys[s] = true;
    return keys;
  };
  std::vector<bool> keys;
  std::vector<bool> redone;
  for (const bool sender : {true, false}) {
    const std::string name = sender ? "keys_sender" : "keys_code";
    Keyed none;
    none.sender = sender;
    CHECK(keyed_sets(gpu, none, (scratch.path / (name + "_none.vrst")).string(),
                     &keys, &redone) == 0);
    CHECK(keys == keys_at({0, 4, 8}));
    Keyed joined = none;
    joined.joins[2] = 3;
    joined.joins[7] = 1;
    CHECK(keyed_sets(gpu, joined,
                     (scratch.path / (name + "_joined.vrst")).string(), &keys,
                     &redone) == 0);
    CHECK(keys == keys_at({0, 2, 6, 7}));
  }
  Keyed late;
  late.buckets = 32;
  late.patch_until = 3;
  late.period = 30;
  CHECK(keyed_sets(gpu, late, (scratch.path / "keys_redo_none.vrst").string(),
                   &keys, &redone) == 0);
  CHECK(redone[3] && keys == keys_at({0}));
  late.joins[3] = 1;
  CHECK(keyed_sets(gpu, late, (scratch.path / "keys_redo.vrst").string(), &keys,
                   &redone) == 0);
  CHECK(redone[3] && keys == keys_at({0, 3}));
  return 0;
}

// A sender over a stream since moved from refuses a set it would code, before
// recording anything of it; one over the stream it moved into codes it.
int moved_from(vkc::test::GpuContext& gpu) {
  std::vector<Camera*> cameras;
  auto opened = open_cameras(gpu, 2, 0, &cameras);
  CHECK(opened.ok());
  sensor::SensorArray array = std::move(opened).value();
  const Fusion fusion;
  auto grid = vr::tsdf::create_fusion_grid(gpu.device, gpu.allocator,
                                           fusion.voxel_size_m,
                                           fusion.truncation_m, 1024);
  auto fuser = vr::tsdf::Fuser::create(gpu.device, gpu.allocator);
  auto made = codec_stream::CodecStream::create(gpu.device, gpu.allocator, {},
                                                geometry_session());
  CHECK(grid.ok() && fuser.ok() && made.ok());
  codec_stream::CodecStream stream = std::move(made).value();
  auto left = sender::SetSender::create(gpu.device, *fuser, *grid, &*made, {});
  auto moved =
      sender::SetSender::create(gpu.device, *fuser, *grid, &stream, {});
  CHECK(left.ok() && moved.ok());
  for (std::size_t s = 0; s < 2; ++s) {
    std::optional<sensor::FrameSet> set = next_set(array, cameras, s);
    CHECK(set.has_value());
    vkc::CommandBatch front(gpu.device, gpu.allocator);
    auto prepared = array.record_process(front, *set, nullptr);
    CHECK(prepared.ok());
    sender::SetSender& out = s == 0 ? *left : *moved;
    auto sent = out.send(front, prepared->frames,
                         std::make_shared<sensor::FrameSet>(std::move(*set)), s,
                         prepared->timestamp_ns);
    if (s == 0) {
      CHECK(sent.status().domain() == vkc::Status::Code::InvalidArgument);
      CHECK(made->coded() == 0 && made->player() == nullptr);
    } else {
      CHECK(sent.ok() && sent->coded.has_value() && stream.coded() == 1);
    }
  }
  return 0;
}

// A sender's frames (SetSender::send with no frames to fuse, so each set is
// its frame alone), recorded into a front batch, then finished: the file
// CodecStream::code writes, byte for byte, I- and P-frames, and two frames on
// `other` that the decoder refuses, the second an I-frame as code() makes it,
// where the encoder would code a P-frame on the first; each set published
// once, before its decode, its rows its frame's encode with "crc", "write",
// "publish" and the waits, and each frame as code() returns it. A frame out
// of order is refused, its front dropped, and the stream goes on.
int halves(vkc::test::GpuContext& gpu, vgrid::volume::VoxelBlockGrid& source,
           vgrid::volume::VoxelBlockGrid& other,
           const vkc::test::Scratch& scratch) {
  codec::EncoderConfig config;
  config.intra_period = 3;
  codec_stream::CodecOutputs by_code;
  by_code.stream = (scratch.path / "halves_code.vrst").string();
  codec_stream::CodecOutputs by_halves;
  by_halves.stream = (scratch.path / "halves.vrst").string();
  by_halves.serve.emplace();
  by_halves.serve->host = "127.0.0.1";
  by_halves.serve->port = 0;
  vkc::Result<codec_stream::CodecStream> a = codec_stream::CodecStream::create(
      gpu.device, gpu.allocator, config, geometry_session(), by_code);
  vkc::Result<codec_stream::CodecStream> b = codec_stream::CodecStream::create(
      gpu.device, gpu.allocator, config, geometry_session(), by_halves);
  vkc::Result<vr::tsdf::Fuser> fuser =
      vr::tsdf::Fuser::create(gpu.device, gpu.allocator);
  CHECK(a.ok() && b.ok() && fuser.ok());
  const auto send = [&](vgrid::volume::VoxelBlockGrid& grid, std::size_t frame,
                        vkc::StageMetrics* rows, int* published) {
    vkc::Result<sender::SetSender> out =
        sender::SetSender::create(gpu.device, *fuser, grid, &*b, {});
    if (!out.ok()) return vkc::Result<sender::SetSender::Sent>(out.status());
    vkc::CommandBatch front(gpu.device, gpu.allocator);
    front.time(out->timer());
    return out->send(front, {}, nullptr, frame, set_time(frame), rows,
                     [published] { ++*published; });
  };
  for (std::size_t frame = 0; frame < 5; ++frame) {
    const bool refused = frame == 2 || frame == 3;
    vgrid::volume::VoxelBlockGrid& grid = refused ? other : source;
    const vkc::Result<codec_stream::SentFrame> x =
        a->code(grid, frame, set_time(frame));
    CHECK(x.ok() != refused);
    vkc::StageMetrics rows;
    int published = 0;
    const vkc::Result<sender::SetSender::Sent> y =
        send(grid, frame, &rows, &published);
    CHECK(y.ok() != refused && published == 1);
    for (const char* row : {"codec encode", "  ..forward", "crc", "write",
                            "publish", "wait: front", "wait: frame"}) {
      CHECK(vgrid_test::find_row(rows, row) != nullptr);
    }
    if (refused) continue;
    CHECK(y->coded.has_value() && !y->fused.redone);
    const codec_stream::SentFrame& sent = *y->coded;
    CHECK(sent.type == x->type && sent.bytes == x->bytes &&
          sent.blocks == x->blocks);
    CHECK(vgrid_test::find_row(sent.encode, "codec encode") != nullptr &&
          vgrid_test::find_row(sent.decode, "codec decode") != nullptr);
  }
  // Out of order: refused, its front dropped, so the next records afresh.
  vkc::StageMetrics rows;
  int published = 0;
  CHECK(send(source, 4, &rows, &published).status().domain() ==
            vkc::Status::Code::InvalidArgument &&
        published == 0);
  CHECK(a->code(source, 5, set_time(5)).ok());
  CHECK(send(source, 5, &rows, &published).ok() && published == 1);
  CHECK(a->finish().ok() && b->finish().ok());
  CHECK(a->coded() == 6 && b->coded() == 6);
  CHECK(vkc::test::read_bytes(by_code.stream) ==
        vkc::test::read_bytes(by_halves.stream));
  return 0;
}

int gpu_main(vkc::test::GpuContext& gpu) {
  vkc::test::Scratch scratch("recon_sender_set_sender_");
  for (const bool dynamic : {true, false}) {
    Fusion fusion;  // 2 cm voxels, an 8 cm band
    fusion.dynamic = dynamic;
    fusion.free_after = kFreeAfter;
    fusion.max_weight = kMaxWeight;
    const std::string name = dynamic ? "dynamic" : "classic";
    const std::string path = (scratch.path / (name + ".vrst")).string();
    std::vector<Walls> coded;
    std::vector<SetCounts> counts;
    CHECK(code_sets(gpu, fusion, path, &coded, Run{true, 1024, &counts}) == 0);
    CHECK(check_stream(gpu, path, dynamic, coded) == 0);
    CHECK(counts_case(counts, true, name.c_str()) == 0);
    // The blocking calls code the same frames.
    const std::string blocking =
        (scratch.path / (name + "_blocking.vrst")).string();
    std::vector<Walls> before;
    CHECK(code_sets(gpu, fusion, blocking, &before, Run{false}) == 0);
    CHECK(same_streams(path, blocking) == 0);
    // Fused alone, the grid holds the same near blocks after every set.
    std::vector<Walls> alone;
    std::vector<SetCounts> fused;
    CHECK(code_sets(gpu, fusion, "", &alone, Run{true, 1024, &fused}) == 0);
    CHECK(counts_case(fused, false, (name + " fused").c_str()) == 0);
    for (std::size_t s = 0; s < kSets; ++s) {
      CHECK(alone[s].near_blocks == coded[s].near_blocks);
    }
  }
  // A rig's four cameras: the Dynamic coded sets' counts, free passes
  // included, against the same bounds.
  {
    Fusion fusion;
    fusion.dynamic = true;
    fusion.free_after = kFreeAfter;
    fusion.max_weight = kMaxWeight;
    const std::string path = (scratch.path / "rig.vrst").string();
    std::vector<Walls> grids;
    std::vector<SetCounts> counts;
    CHECK(code_sets(gpu, fusion, path, &grids, Run{true, 1024, &counts, 4}) ==
          0);
    CHECK(counts_case(counts, true, "dynamic rig") == 0);
  }
  // A grid of 32 buckets: the first set's round meets the capacity limit, so
  // its fusion is done again and grown, and its frame's front half after it,
  // whose wait is the set's "wait: redo". The frames are the blocking calls'.
  {
    Fusion fusion;
    fusion.dynamic = true;
    fusion.free_after = kFreeAfter;
    fusion.max_weight = kMaxWeight;
    const std::string path = (scratch.path / "redo.vrst").string();
    const std::string blocking = (scratch.path / "redo_blocking.vrst").string();
    std::vector<Walls> grids;
    std::vector<SetCounts> counts;
    CHECK(code_sets(gpu, fusion, path, &grids, Run{true, 32, &counts}) == 0);
    CHECK(!counts.empty() && counts[0].redone && counts[0].redo_waited);
    std::printf("  redo set 0: %llu submits, %llu waits, %llu dispatches\n",
                static_cast<unsigned long long>(counts[0].submits),
                static_cast<unsigned long long>(counts[0].host_waits),
                static_cast<unsigned long long>(counts[0].dispatches));
    CHECK(code_sets(gpu, fusion, blocking, &grids, Run{false, 32}) == 0);
    CHECK(same_streams(path, blocking) == 0);
  }
  Fusion never;  // Dynamic, freeing nothing
  never.dynamic = true;
  never.free_after = 0;
  never.max_weight = kMaxWeight;
  std::vector<Walls> kept;
  CHECK(code_sets(gpu, never, "", &kept) == 0);
  CHECK(keys_case(gpu, scratch) == 0);
  CHECK(moved_from(gpu) == 0);
  // A sphere's grid, and one at twice its voxel size, which the player of a
  // stream begun on the first refuses.
  const codec_fixture::Sphere sphere{vr::Vec3f(0.01f, -0.02f, 0.03f), 0.09f};
  codec_fixture::GridShape coarser;
  coarser.voxel_size = 2.0f * codec_fixture::kVoxel;
  auto source = codec_fixture::sphere_grid(gpu, sphere);
  auto other = codec_fixture::sphere_grid(gpu, sphere, coarser);
  CHECK(source.ok() && other.ok());
  CHECK(halves(gpu, *source, *other, scratch) == 0);
  std::puts("recon_sender_set_sender: OK");
  return 0;
}

}  // namespace

int main() {
  return vkc::test::run_on_gpu(vr::device_requirements(), gpu_main);
}
