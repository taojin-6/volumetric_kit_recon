// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/fuse_sender.hpp
/// @brief The fusing examples' `sender::SetSender`: its config from the
///        scenario, the session a live run's stream writes, and what each
///        set it sent did, into the run's figures. The codec helpers
///        (`codec_config.hpp`, `codec_report.hpp`) include no fusion; this
///        header joins the two.

#include <cstdint>
#include <cstdio>
#include <utility>

#include "codec_config.hpp"
#include "codec_report.hpp"
#include "config.hpp"
#include "fuse_frame.hpp"
#include "set_log.hpp"
#include "volumetric_kit/core/base/check.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/grid/stream/audio.hpp"
#include "volumetric_kit/grid/stream/session.hpp"
#include "volumetric_kit/recon/sender/set_sender.hpp"
#include "volumetric_kit/sensor/audio_sensor.hpp"

namespace vr_example {

namespace config = volumetric_kit::core::config;

/// @brief The track @ref live_session gives a mic: the one after the
///        Geometry track.
inline constexpr std::uint16_t kAudioTrack = 1;

/// @return The session a live run's stream writes and serves: @p program's
///         `codec_session` at `source.fps`, its Geometry track on the host's
///         system clock (`stream::kHostClock`) when @p host_clock -- every
///         camera stamping on it, as each set's pts then is -- or else on
///         clock domain 1, the cameras' own; and, with @p audio, the mic's
///         Audio track as track @ref kAudioTrack: on the host's clock, a
///         record a chunk at its nominal rate, its id as the sensor, and
///         signed 16-bit samples at its channels and rate as the params.
/// @param audio  The mic's info (`IAudioSensor::info`), its chunks stamped on
///               the host's clock (`VKC_CHECK`), or null for none: then the
///               session is exactly the one before audio.
inline volumetric_kit::grid::stream::Session live_session(
    const char* program, const config::Config& opt, bool host_clock,
    const volumetric_kit::sensor::AudioInfo* audio) {
  namespace stream = volumetric_kit::grid::stream;
  stream::Session session =
      vgrid_example::codec_session(program, {opt.source.fps, 1});
  if (host_clock) session.tracks[0].clock = stream::kHostClock;
  if (audio != nullptr) {
    VKC_CHECK(audio->clock == volumetric_kit::sensor::ClockDomain::Host,
              "a mic's track is on the host's clock: its chunks must be too");
    stream::AudioParams params;
    params.codec = stream::AudioCodec::PcmS16;
    params.channels = audio->channels;
    params.sample_rate = audio->sample_rate;
    stream::Track track;
    track.kind = stream::TrackKind::Audio;
    track.clock = stream::kHostClock;
    track.rate = {audio->sample_rate, audio->chunk_frames};
    track.sensor = audio->id;
    track.params = stream::encode_audio_params(params);
    session.tracks.push_back(std::move(track));
  }
  return session;
}

/// @return @p opt's sender config (`sender::SetSender::Config`): fusion as
///         `fusion` says, and every @p code_every-th set coded.
inline volumetric_kit::recon::sender::SetSender::Config sender_config(
    const config::Config& opt, int code_every) {
  volumetric_kit::recon::sender::SetSender::Config c;
  c.max_weight = opt.fusion.max_weight;
  c.mode = integration_mode(opt.fusion.dynamic);
  c.free_after = opt.fusion.free_after;
  c.code_every = code_every;
  return c;
}

/// @brief What a sender's set did, into the run's figures: a free pass's
///        refusal on stderr as "<program>: free blocks: <why>", what the
///        fusion did to the grid (@ref print_fuse_report), the set's row
///        stamped -- fused again, its frame's type and tail -- and its
///        frame's row added to @p report, when given.
/// @return OK, or `vgrid_example::CodecReport::add`'s error.
inline vkc::Status report_sent(const vr::sender::SetSender::Sent& sent,
                               std::uint64_t index, const char* program,
                               SetLog::Set& set,
                               vgrid_example::CodecReport* report) {
  if (!sent.freed.ok()) {
    std::fprintf(stderr, "%s: free blocks: %s\n", program,
                 sent.freed.message().c_str());
  }
  print_fuse_report(sent.fused);
  set.redone = sent.fused.redone;
  if (!sent.coded) return {};
  set.type = sent.coded->type;
  set.tail = sent.coded->tail;
  return report != nullptr ? report->add(index, *sent.coded) : vkc::Status{};
}

}  // namespace vr_example
