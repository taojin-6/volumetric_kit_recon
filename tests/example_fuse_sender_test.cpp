// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// A sender's set into the run's figures (examples/common/fuse_sender.hpp's
// report_sent), the fusing half of example_codec_report_test.cpp; the world's
// up a calibration names (world_up.hpp's calibration_up), and a live run's
// calibration file (live_up), by the same rule; and a live run's session
// (live_session), stating that up, with a mic's Audio track and without. Host
// only.

#include <array>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <string>

#include "codec_config.hpp"
#include "codec_report.hpp"
#include "fuse_sender.hpp"
#include "set_log.hpp"
#include "volumetric_kit/core/base/file_io.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/config/config.hpp"
#include "volumetric_kit/core/testing/scratch_dir.hpp"
#include "volumetric_kit/core/testing/test_check.hpp"
#include "volumetric_kit/grid/codec/codec_params.hpp"
#include "volumetric_kit/grid/stream/audio.hpp"
#include "volumetric_kit/grid/stream/session.hpp"
#include "volumetric_kit/grid/testing/stream_files.hpp"
#include "volumetric_kit/recon/sender/set_sender.hpp"
#include "volumetric_kit/sensor/audio_sensor.hpp"
#include "volumetric_kit/sensor/camera/array_calibration.hpp"
#include "world_up.hpp"

namespace camera = volumetric_kit::sensor::camera;
namespace codec = volumetric_kit::grid::codec;
namespace sender = volumetric_kit::recon::sender;
namespace stream = volumetric_kit::grid::stream;
namespace vkc = volumetric_kit::core;

namespace {

using Code = vkc::Status::Code;
using Up = std::array<float, 3>;

constexpr Up kUnknown{0.0f, 0.0f, 0.0f};
constexpr Up kMinusY{0.0f, -1.0f, 0.0f};
constexpr Up kPlusZ{0.0f, 0.0f, 1.0f};

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

// The up of a world, from what a calibration says it is: a tag's +z, the tag
// winning over a sensor named too; a sensor's own -y, and -y for one posing no
// sensor, an empty one too (no file); nothing where any camera is posed in a
// world it does not name, one among lenses alone too.
int calibration_ups() {
  camera::ArrayCalibration rig;
  CHECK(vr_example::calibration_up(rig) == kMinusY);
  rig.sensors.push_back({"rig-0", std::nullopt, {}, {}, {}, 0.0});
  CHECK(vr_example::calibration_up(rig) == kMinusY);
  rig.sensors.push_back({"rig-1", camera::Mat4d(1.0), {}, {}, {}, 0.0});
  CHECK(vr_example::calibration_up(rig) == kUnknown);
  rig.world = {"rig-1", std::nullopt};
  CHECK(vr_example::calibration_up(rig) == kMinusY);
  rig.world.apriltag = camera::AprilTagWorld{"tag36h11", 0, 0.15};
  CHECK(vr_example::calibration_up(rig) == kPlusZ);
  rig.world.sensor.clear();
  CHECK(vr_example::calibration_up(rig) == kPlusZ);
  return 0;
}

// The up of a run's world, from what its calibration file says the world is,
// by calibration_up of the file read: a tag's +z; a sensor's own -y; nothing
// where any camera is posed in a world the file does not name, as one picked
// out of a list with lenses alone for the rest; and -y where each frame stays
// in its own camera's frame, with no file. The file's read error comes back,
// through live_session too, and live_session states the up.
int live_ups() {
  const vkc::test::Scratch scratch("recon_example_fuse_sender_");
  const auto file = [&scratch](const char* name) {
    return (scratch.path / name).string();
  };
  const auto write = [&file](const char* name,
                             const camera::ArrayCalibration& array) {
    return camera::write_array_calibration(file(name), array,
                                           vkc::WriteMode::CreateNew);
  };
  const auto session_for = [](const std::string& calibration) {
    volumetric_kit::core::config::Config opt;
    opt.source.fps = 15;
    opt.source.calibration = calibration;
    return vr_example::live_session("fuse_orbbec", opt, true, nullptr);
  };
  const auto says = [&session_for](const std::string& calibration,
                                   const Up& up) {
    volumetric_kit::core::config::Config opt;
    opt.source.calibration = calibration;
    const vkc::Result<Up> got = vr_example::live_up(opt);
    const vkc::Result<stream::Session> session = session_for(calibration);
    const vkc::Result<camera::ArrayCalibration> rig =
        calibration.empty()
            ? vkc::Result<camera::ArrayCalibration>(camera::ArrayCalibration{})
            : camera::read_array_calibration(calibration);
    return got.ok() && *got == up && session.ok() && session->up == up &&
           rig.ok() && vr_example::calibration_up(*rig) == up;
  };

  camera::ArrayCalibration array;
  array.sensors.push_back({"rig-0", camera::Mat4d(1.0), {}, {}, {}, 0.0});
  CHECK(write("unnamed.json", array).ok());
  array.world.apriltag = camera::AprilTagWorld{"tag36h11", 0, 0.15};
  CHECK(write("tag.json", array).ok());
  array.world = {"rig-0", std::nullopt};
  CHECK(write("reference.json", array).ok());
  camera::ArrayCalibration lenses;
  lenses.sensors.push_back({"rig-0", std::nullopt, {}, {}, {}, 0.0});
  CHECK(write("unposed.json", lenses).ok());
  lenses.sensors.push_back({"rig-1", camera::Mat4d(1.0), {}, {}, {}, 0.0});
  CHECK(write("one_posed.json", lenses).ok());
  CHECK(vkc::write_file(file("version1.json"),
                        R"({"device_calibration": {"rig-0": {"pose":
                            {"rvec": [0, 0, 0], "tvec": [0, 0, 0]}}}})",
                        vkc::WriteMode::CreateNew)
            .ok());

  CHECK(says(file("tag.json"), kPlusZ));
  CHECK(says(file("reference.json"), kMinusY));
  CHECK(says(file("unposed.json"), kMinusY));
  CHECK(says("", kMinusY));
  CHECK(says(file("version1.json"), kUnknown));
  CHECK(says(file("unnamed.json"), kUnknown));
  CHECK(says(file("one_posed.json"), kUnknown));

  volumetric_kit::core::config::Config opt;
  opt.source.calibration = file("missing.json");
  CHECK(vr_example::live_up(opt).status().domain() == Code::NotFound);
  CHECK(session_for(file("missing.json")).status().domain() == Code::NotFound);
  return 0;
}

// A live run's session is the codec session at the source's rate, stating the
// world's up (-y here, with no calibration file), its geometry on the host's
// system clock when the cameras stamp on it, else on clock domain 1, theirs;
// with no mic, nothing else: the codec session's own fields and one track.
int live_sessions() {
  volumetric_kit::core::config::Config opt;
  opt.source.fps = 15;
  for (const bool host : {true, false}) {
    const vkc::Result<stream::Session> made =
        vr_example::live_session("fuse_orbbec", opt, host, nullptr);
    CHECK(made.ok());
    const stream::Session& session = *made;
    stream::Session codec =
        vgrid_example::codec_session("fuse_orbbec", {15, 1});
    codec.up = kMinusY;
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
  const vkc::Result<stream::Session> made =
      vr_example::live_session("fuse_orbbec", opt, true, &mic);
  const vkc::Result<stream::Session> silent =
      vr_example::live_session("fuse_orbbec", opt, true, nullptr);
  CHECK(made.ok() && silent.ok());
  const stream::Session& session = *made;
  CHECK(session.tracks.size() == 2 && vr_example::kAudioTrack == 1);
  CHECK(vgrid_test::vrst::same(session.tracks[0], silent->tracks[0]));
  CHECK(session.up == silent->up);
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
  CHECK(calibration_ups() == 0);
  CHECK(live_ups() == 0);
  CHECK(live_sessions() == 0);
  CHECK(audio_sessions() == 0);
  std::puts("example_fuse_sender: OK");
  return 0;
}
