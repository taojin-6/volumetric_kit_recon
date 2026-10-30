// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// A sender's set into the run's figures (examples/common/fuse_sender.hpp's
// report_sent), the fusing half of example_codec_report_test.cpp, and a live
// run's session (live_session), with a mic's Audio track and without. Host
// only.

#include <cstdint>
#include <cstdio>

#include "codec_config.hpp"
#include "codec_report.hpp"
#include "fuse_sender.hpp"
#include "set_log.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/config/config.hpp"
#include "volumetric_kit/core/testing/test_check.hpp"
#include "volumetric_kit/grid/codec/codec_params.hpp"
#include "volumetric_kit/grid/stream/audio.hpp"
#include "volumetric_kit/grid/stream/session.hpp"
#include "volumetric_kit/grid/testing/stream_files.hpp"
#include "volumetric_kit/recon/sender/set_sender.hpp"
#include "volumetric_kit/sensor/audio_sensor.hpp"

namespace codec = volumetric_kit::grid::codec;
namespace sender = volumetric_kit::recon::sender;
namespace stream = volumetric_kit::grid::stream;
namespace vkc = volumetric_kit::core;

namespace {

// A sender's set into the run's figures: a set fused again whose frame, a
// P-frame, took a tail, stamped so, its frame's row kept under the set's
// index; a set not coded stamped with no type, and no row.
int sent_sets() {
  vkc::Result<vgrid_example::CodecReport> made =
      vgrid_example::CodecReport::create("", "", false, 30);
  CHECK(made.ok());
  vgrid_example::CodecReport& report = made.value();
  vr_example::SetLog log(nullptr);
  sender::SetSender::Sent coded;
  coded.fused.redone = true;
  coded.coded.emplace();
  coded.coded->type = codec::FrameType::kInter;
  coded.coded->bytes = 1234;
  coded.coded->blocks = 56;
  coded.coded->tail = true;
  vr_example::SetLog::Set& first = log.begin();
  CHECK(vr_example::report_sent(coded, 4, "example_fuse_sender", first, &report)
            .ok());
  CHECK(first.redone && first.tail && first.type == codec::FrameType::kInter);
  CHECK(report.rows().size() == 1 && report.rows()[0].frame == 4);
  CHECK(report.rows()[0].bytes == 1234 && report.rows()[0].blocks == 56);
  CHECK(!report.rows()[0].decode_ms);
  const sender::SetSender::Sent fused;
  vr_example::SetLog::Set& second = log.begin();
  CHECK(
      vr_example::report_sent(fused, 5, "example_fuse_sender", second, &report)
          .ok());
  CHECK(!second.redone && !second.tail && !second.type);
  CHECK(report.rows().size() == 1);
  return 0;
}

// A live run's session is the codec session at the source's rate, its
// geometry on the host's system clock when the cameras stamp on it, else on
// clock domain 1, theirs; with no mic, nothing else: the codec session's own
// fields and one track.
int live_sessions() {
  volumetric_kit::core::config::Config opt;
  opt.source.fps = 15;
  for (const bool host : {true, false}) {
    const stream::Session session =
        vr_example::live_session("fuse_orbbec", opt, host, nullptr);
    stream::Session codec =
        vgrid_example::codec_session("fuse_orbbec", {15, 1});
    if (host) codec.tracks[0].clock = stream::kHostClock;
    CHECK(vgrid_test::vrst::same(session, codec));
    CHECK(session.writer == "fuse_orbbec" && session.tracks.size() == 1);
    const stream::Track& geometry = session.tracks[0];
    CHECK(geometry.kind == stream::TrackKind::Geometry);
    CHECK(geometry.rate.num == 15 && geometry.rate.den == 1);
    CHECK(geometry.clock == (host ? stream::kHostClock : std::uint16_t{1}));
  }
  return 0;
}

// With a mic, the session gains its Audio track after the geometry's, as
// track kAudioTrack: on the host's clock, a record a chunk at its nominal
// rate, its id as the sensor, no picture size, and params that read back as
// signed 16-bit samples at its channels and rate. The geometry is as without.
int audio_sessions() {
  volumetric_kit::core::config::Config opt;
  opt.source.fps = 25;
  volumetric_kit::sensor::AudioInfo mic;
  mic.id = "hw:2,0";
  mic.model = "a USB mic";
  mic.sample_rate = 44100;
  mic.channels = 2;
  mic.chunk_frames = 441;
  const stream::Session session =
      vr_example::live_session("fuse_orbbec", opt, true, &mic);
  CHECK(session.tracks.size() == 2 && vr_example::kAudioTrack == 1);
  CHECK(vgrid_test::vrst::same(
      session.tracks[0],
      vr_example::live_session("fuse_orbbec", opt, true, nullptr).tracks[0]));
  const stream::Track& audio = session.tracks[vr_example::kAudioTrack];
  CHECK(audio.kind == stream::TrackKind::Audio);
  CHECK(audio.clock == stream::kHostClock);
  CHECK(audio.rate.num == 44100 && audio.rate.den == 441);
  CHECK(audio.width == 0 && audio.height == 0);
  CHECK(audio.sensor == "hw:2,0");
  const vkc::Result<stream::AudioParams> params =
      stream::decode_audio_params(audio.params);
  CHECK(params.ok());
  CHECK(params->codec == stream::AudioCodec::PcmS16);
  CHECK(params->channels == 2 && params->sample_rate == 44100);
  CHECK(audio.params.size() == stream::kAudioParamsBytes);
  return 0;
}

}  // namespace

int main() {
  CHECK(sent_sets() == 0);
  CHECK(live_sessions() == 0);
  CHECK(audio_sessions() == 0);
  std::puts("example_fuse_sender: OK");
  return 0;
}
