// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// The link tests' writer side: a phone-like sensor's pattern frames, their
// records in sensor's wire form, and a writer that dials a reader's listener
// as an iPad's does (grid's StreamServer with listen false, core::net::dial,
// add_viewer), each frame's colour record, if any, before its depth record.

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>

#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/base/unique_fd.hpp"
#include "volumetric_kit/core/net/dial.hpp"
#include "volumetric_kit/grid/networking/stream_server.hpp"
#include "volumetric_kit/grid/stream/chunks.hpp"
#include "volumetric_kit/grid/stream/session.hpp"
#include "volumetric_kit/grid/stream/stream_writer.hpp"
#include "volumetric_kit/sensor/camera/camera_model.hpp"
#include "volumetric_kit/sensor/rgbd_frame.hpp"
#include "volumetric_kit/sensor/rgbd_sensor.hpp"
#include "volumetric_kit/sensor/testing/rgbd_pattern.hpp"
#include "volumetric_kit/sensor/wire/rgbd_wire.hpp"

namespace vr_test {

namespace vkc = volumetric_kit::core;
namespace sensor = volumetric_kit::sensor;
namespace stream = volumetric_kit::grid::stream;
namespace gnet = volumetric_kit::grid::networking;

using Clock = std::chrono::steady_clock;

inline constexpr std::uint64_t kFirstNs = 1'762'000'000'000'000'000;
inline constexpr std::uint64_t kPeriodNs = 100'000'000;  // 10 Hz

inline sensor::camera::CameraModel pinhole(std::uint32_t width,
                                           std::uint32_t height) {
  sensor::camera::CameraModel m;
  m.size = {width, height};
  m.intrinsics = {0.8 * width, 0.8 * width + 0.125, 0.5 * width - 0.25,
                  0.5 * height + 0.375};
  return m;
}

// A phone's: ARKit's depth, a colour camera of `color_width` x
// `color_height` (none at 0), posed by its own tracking, on the host's clock,
// written at 10 Hz.
inline sensor::SensorInfo phone_info(std::uint32_t color_width = 0,
                                     std::uint32_t color_height = 0) {
  sensor::SensorInfo info;
  info.id = "ipad";
  info.model = "iPad Pro";
  info.depth = pinhole(256, 192);
  if (color_width != 0) info.color = pinhole(color_width, color_height);
  info.role = sensor::SyncRole::FreeRun;
  info.clock = sensor::ClockDomain::Host;
  info.pose = sensor::PoseSource::Tracked;
  info.fps = 10;
  return info;
}

// `info`'s session: its picture track 0 and depth track 1, or its depth
// track 0 alone, under session id ending `id`.
inline stream::Session session_of(const sensor::SensorInfo& info,
                                  std::uint8_t id = 0x6b) {
  stream::Session session;
  session.id = {0x8f, 0x3c, 0x0a, 0x7e, 0x5b, 0x2d, 0x4e, 0x6f,
                0x9a, 0x1b, 0x0c, 0x2d, 0x3e, 0x4f, 0x5a, id};
  session.created_ns = kFirstNs;
  session.writer = "recon_link_test";
  stream::Track depth;
  depth.kind = stream::TrackKind::RgbdDepth;
  depth.clock = stream::kHostClock;
  depth.rate = {info.fps, 1};
  depth.width = info.depth->size.width;
  depth.height = info.depth->size.height;
  depth.sensor = info.id;
  const vkc::Result<std::string> params = sensor::wire::encode_sensor(info);
  if (params.ok()) depth.params = params.value();
  if (info.color) {
    stream::Track color = depth;
    color.kind = stream::TrackKind::RgbdColor;
    color.width = info.color->size.width;
    color.height = info.color->size.height;
    color.params =
        sensor::wire::encode_color_params(sensor::wire::ColorCodec::HevcMain8);
    session.tracks = {color, depth};
  } else {
    session.tracks = {depth};
  }
  return session;
}

// Pattern frames: frame i's depth carries index `index(i)`, its plane at
// floor 2, its pose and time its own.
struct PatternFrames {
  sensor::SensorInfo info;
  std::vector<sensor::testing::PatternDepth> depth;
  std::vector<sensor::RgbdFrame> frames;
};

inline PatternFrames pattern_frames(
    const sensor::SensorInfo& info, std::size_t count,
    const std::function<std::uint32_t(std::size_t)>& index = [](std::size_t i) {
      return static_cast<std::uint32_t>(i);
    }) {
  PatternFrames p;
  p.info = info;
  p.depth.reserve(count);
  for (std::size_t i = 0; i < count; ++i) {
    p.depth.push_back(sensor::testing::pattern_depth(256, 192, index(i)));
  }
  for (std::size_t i = 0; i < count; ++i) {
    sensor::RgbdFrame f;
    f.depth = p.depth[i].depth.data();
    f.depth_confidence = p.depth[i].confidence.data();
    f.min_confidence = 2;
    f.depth_camera = *info.depth;
    f.metres_per_unit = 0.001f;
    f.min_depth = 0.25f;
    f.max_depth = 5.0f;
    f.depth_noise = {0.001f, 0.01f, 0.0f};
    f.color_to_world = sensor::camera::Mat4d(1.0);
    f.color_to_world[3][0] = 0.05 * static_cast<double>(i);
    f.timestamp_ns = kFirstNs + i * kPeriodNs;
    f.sequence = i;
    p.frames.push_back(f);
  }
  return p;
}

// One record to write or serve: its header and its payload's runs, which
// point into what made them and into its own prefix, so it stays put.
struct Record {
  Record() = default;
  Record(const Record&) = delete;
  Record& operator=(const Record&) = delete;
  stream::RecordInfo info;
  stream::RecordPayload payload = stream::RecordPayload::of(nullptr, 0);
  std::array<std::uint8_t, sensor::wire::kDepthPrefixBytes> depth_prefix{};
};

// Frame `f`'s depth record on `track`, in set `set`. Its prefix is copied
// into the record, which must not move after (payload points at it).
inline vkc::Status depth_record(const sensor::SensorInfo& info,
                                const sensor::RgbdFrame& f, std::uint16_t track,
                                std::uint64_t set, Record* out) {
  VKC_ASSIGN(const sensor::wire::DepthRecord d,
             sensor::wire::encode_depth(info, f));
  out->depth_prefix = d.prefix;
  out->info.track = track;
  out->info.key = true;
  out->info.pts_ns = f.timestamp_ns;
  out->info.seq = f.sequence;
  out->info.set = set;
  out->payload = stream::RecordPayload::of(
      out->depth_prefix.data(), out->depth_prefix.size(), d.depth,
      d.depth_bytes, d.confidence, d.confidence_bytes);
  return {};
}

// Waits up to `limit` for `done`, checking every millisecond.
inline bool wait_for(
    const std::function<bool()>& done,
    std::chrono::milliseconds limit = std::chrono::seconds(5)) {
  const Clock::time_point end = Clock::now() + limit;
  while (!done()) {
    if (Clock::now() >= end) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return true;
}

// A writer that dials 127.0.0.1:`port`, as an iPad's does: its server binds
// nothing, keys on a join (replay_gop false), and the socket it dials is its
// one viewer, whose notes it reads with `read_notes`. It dials until the
// listener is there.
class Dialler {
 public:
  static vkc::Result<std::unique_ptr<Dialler>> dial(
      const stream::Session& session, std::uint16_t port,
      bool read_notes = false,
      std::chrono::milliseconds limit = std::chrono::seconds(5)) {
    VKC_ASSIGN(std::unique_ptr<gnet::StreamServer> server,
               gnet::StreamServer::start(session, serving(read_notes)));
    const Clock::time_point end = Clock::now() + limit;
    for (;;) {
      vkc::Result<vkc::UniqueFd> fd = volumetric_kit::core::net::dial(
          "127.0.0.1", port, std::chrono::milliseconds(500));
      if (fd.ok()) {
        VKC_TRY(server->add_viewer(std::move(fd).value()));
        return std::unique_ptr<Dialler>(new Dialler(std::move(server)));
      }
      if (Clock::now() >= end) return fd.status();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  // As dial, once, its socket bound to `from` before it connects: a numeric
  // IPv4 address of this host other than 127.0.0.1, a second peer for a
  // reader on 127.0.0.1 (127.0.0.2 on Linux; macOS gives loopback no other
  // address, and a LAN one serves).
  static vkc::Result<std::unique_ptr<Dialler>> dial_from(
      const stream::Session& session, std::uint16_t port,
      const std::string& from, bool read_notes = false) {
    VKC_ASSIGN(std::unique_ptr<gnet::StreamServer> server,
               gnet::StreamServer::start(session, serving(read_notes)));
    VKC_ASSIGN(vkc::UniqueFd fd, connect_from(from, port));
    VKC_TRY(server->add_viewer(std::move(fd)));
    return std::unique_ptr<Dialler>(new Dialler(std::move(server)));
  }

  // A blocking connection to 127.0.0.1:`port` from `from` (above).
  static vkc::Result<vkc::UniqueFd> connect_from(const std::string& from,
                                                 std::uint16_t port) {
    vkc::UniqueFd fd(::socket(AF_INET, SOCK_STREAM, 0));
    sockaddr_in local{};
    local.sin_family = AF_INET;
    if (!fd.valid() ||
        ::inet_pton(AF_INET, from.c_str(), &local.sin_addr) != 1 ||
        ::bind(fd.get(), reinterpret_cast<const sockaddr*>(&local),
               sizeof local) != 0) {
      return vkc::Status::io_error("Dialler: cannot bind " + from);
    }
    sockaddr_in to{};
    to.sin_family = AF_INET;
    to.sin_port = htons(port);
    to.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (::connect(fd.get(), reinterpret_cast<const sockaddr*>(&to),
                  sizeof to) != 0) {
      return vkc::Status::io_error("Dialler: cannot connect from " + from);
    }
    return fd;
  }

  // A key on track 0 waits until the reader has sent the last key's records
  // (StreamServer::draining), as an iPad's writer holds its key, so newest
  // key wins never skips a record the test means to arrive.
  vkc::Status publish(const stream::RecordInfo& info,
                      const stream::RecordPayload& payload) {
    if (info.track == 0 && info.key &&
        !wait_for([&] { return server_->draining() == 0; })) {
      return vkc::Status::io_error("Dialler: the reader takes no records");
    }
    return server_->publish(info, payload);
  }
  vkc::Status publish(const Record& r) { return publish(r.info, r.payload); }

  // Whether the reader let the connection go within `limit`.
  bool let_go(std::chrono::milliseconds limit = std::chrono::seconds(5)) {
    return wait_for([&] { return server_->viewers() == 0; }, limit);
  }

  gnet::StreamServer& server() { return *server_; }

 private:
  explicit Dialler(std::unique_ptr<gnet::StreamServer> server)
      : server_(std::move(server)) {}

  // An iPad's writer's server: no listener, a key on a join, one viewer.
  static gnet::ServerOptions serving(bool read_notes) {
    gnet::ServerOptions options;
    options.listen = false;
    options.replay_gop = false;
    options.max_viewers = 1;
    options.read_notes = read_notes;
    return options;
  }
  std::unique_ptr<gnet::StreamServer> server_;
};

}  // namespace vr_test
