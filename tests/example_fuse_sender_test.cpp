// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// A sender's set into the run's figures (examples/common/fuse_sender.hpp's
// report_sent), the fusing half of example_codec_report_test.cpp, and a live
// run's session (live_session). Host only.

#include <cstdint>
#include <cstdio>

#include "codec_report.hpp"
#include "fuse_sender.hpp"
#include "set_log.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/config/config.hpp"
#include "volumetric_kit/core/testing/test_check.hpp"
#include "volumetric_kit/grid/codec/codec_params.hpp"
#include "volumetric_kit/grid/stream/session.hpp"
#include "volumetric_kit/recon/sender/set_sender.hpp"

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
// clock domain 1, theirs.
int live_sessions() {
  volumetric_kit::core::config::Config opt;
  opt.source.fps = 15;
  for (const bool host : {true, false}) {
    const stream::Session session =
        vr_example::live_session("fuse_orbbec", opt, host);
    CHECK(session.writer == "fuse_orbbec" && session.tracks.size() == 1);
    const stream::Track& geometry = session.tracks[0];
    CHECK(geometry.kind == stream::TrackKind::Geometry);
    CHECK(geometry.rate.num == 15 && geometry.rate.den == 1);
    CHECK(geometry.clock == (host ? stream::kHostClock : std::uint16_t{1}));
  }
  return 0;
}

}  // namespace

int main() {
  CHECK(sent_sets() == 0);
  CHECK(live_sessions() == 0);
  std::puts("example_fuse_sender: OK");
  return 0;
}
