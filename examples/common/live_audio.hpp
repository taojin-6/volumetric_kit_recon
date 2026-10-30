// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file examples/common/live_audio.hpp
/// @brief The live examples' mic: the input `source.audio` names, opened on
///        the backend the build has (sensor's `Microphone`, where sensor builds
///        sensor_microphone: VR_EXAMPLE_MICROPHONE), and its chunks into the
///        run's codec stream as its Audio track's records. The one place that
///        knows audio backends; the track itself is @ref live_session's.
///        Header-only.

#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "live_sets.hpp"
#include "volumetric_kit/core/base/check.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/config/config.hpp"
#include "volumetric_kit/grid/codec_stream/codec_stream.hpp"
#include "volumetric_kit/grid/stream/session.hpp"
#include "volumetric_kit/sensor/audio_sensor.hpp"
#ifdef VR_EXAMPLE_MICROPHONE
#include <utility>

#include "volumetric_kit/sensor/audio/microphone.hpp"
#endif

namespace vr_example {

namespace config = volumetric_kit::core::config;
namespace sensor = volumetric_kit::sensor;
namespace vkc = volumetric_kit::core;

/// @return The input `source.audio` names, opened and not started: null when
///         it is empty; the system's default input for `default`, else the
///         input of that name (`sensor::Microphone::open`'s refusals, such as
///         `Status::Code::NotFound` naming the inputs found);
///         `Status::Code::Unsupported` in a build without sensor's
///         sensor_microphone (VS_WITH_PORTAUDIO).
inline vkc::Result<std::unique_ptr<sensor::IAudioSensor>> open_audio(
    const config::Config& opt) {
  if (opt.source.audio.empty()) return std::unique_ptr<sensor::IAudioSensor>();
#ifdef VR_EXAMPLE_MICROPHONE
  sensor::Microphone::Options options;
  if (opt.source.audio != "default") options.device = opt.source.audio;
  VKC_ASSIGN(sensor::Microphone mic, sensor::Microphone::open(options));
  return std::unique_ptr<sensor::IAudioSensor>(
      std::make_unique<sensor::Microphone>(std::move(mic)));
#else
  return vkc::Status::unsupported(
      "source.audio \"" + opt.source.audio +
      "\": this build has no microphone driver; configure sensor with "
      "-DVS_WITH_PORTAUDIO=ON (PortAudio installed)");
#endif
}

/// @brief A mic's chunks into a codec stream, as its Audio track's records,
///        on the run's thread: started once the cameras stream, drained after
///        each set is sent, and once more before the stream finishes.
///
/// Each chunk is one record: a key, its pts the chunk's capture time, its seq
/// the chunk's sequence, its payload the chunk's samples as they lie (signed
/// 16-bit, little-endian on every host the family builds on). The stream holds
/// a record until a frame at or past its pts goes out
/// (`CodecStream::append`). A drain that fails means the mic has gone: the
/// feed warns once on stderr and records no more, and the run goes on. With no
/// mic every call does nothing, so the stream is the one a run without audio
/// writes.
///
/// @warning The mic and the stream must outlive the feed. Not thread-safe:
///          one call at a time, on the stream's thread.
class AudioFeed {
 public:
  /// @param mic      The mic, opened and not started (@ref open_audio), or
  ///                 null for none.
  /// @param stream   The stream whose session holds the mic's track as
  ///                 @p track (@ref live_session); not null with a mic.
  /// @param track    The mic's track in the stream's session.
  /// @param program  The program's name, for the warning.
  AudioFeed(sensor::IAudioSensor* mic,
            volumetric_kit::grid::codec_stream::CodecStream* stream,
            std::uint16_t track, const char* program)
      : mic_(mic), stream_(stream), track_(track), program_(program) {
    VKC_CHECK(mic_ == nullptr || stream_ != nullptr,
              "a mic's chunks need a stream to go to");
  }

  AudioFeed(const AudioFeed&) = delete;
  AudioFeed& operator=(const AudioFeed&) = delete;

  /// @brief Start the mic.
  /// @return OK, with no mic too; or the mic's `start` error.
  vkc::Status start() {
    if (mic_ == nullptr) return {};
    VKC_TRY(mic_->start());
    running_ = true;
    return {};
  }

  /// @brief Append every chunk the mic holds to the stream, oldest first.
  ///        A drain that fails ends the audio: a warning on stderr, and no
  ///        later call drains again.
  /// @return OK, with no mic, before @ref start, after the audio ended and
  ///         when the drain fails; or the stream's `append` error.
  vkc::Status send() {
    namespace stream = volumetric_kit::grid::stream;
    if (!running_) return {};
    chunks_.clear();
    const vkc::Status drained = mic_->drain(&chunks_);
    if (!drained.ok()) {
      running_ = false;
      lost_ = true;
      std::fprintf(stderr, "%s: audio from %s ends here: %s\n", program_,
                   mic_->info().id.c_str(), drained.message().c_str());
      return {};
    }
    for (const sensor::AudioChunk& chunk : chunks_) {
      stream::RecordInfo info;
      info.track = track_;
      info.key = true;
      info.pts_ns = chunk.timestamp_ns;
      info.seq = chunk.sequence;
      VKC_TRY(stream_->append(
          info, reinterpret_cast<const std::uint8_t*>(chunk.samples.data()),
          chunk.samples.size() * sizeof(std::int16_t)));
    }
    return {};
  }

  /// @brief The last @ref send, then stop the mic; later calls do nothing.
  ///        Before the stream finishes, which writes what it holds.
  /// @return As @ref send.
  vkc::Status finish() {
    if (mic_ == nullptr) return {};
    const vkc::Status last = send();
    running_ = false;
    mic_->stop();
    return last;
  }

  /// @return Whether a drain failed, ending the audio before @ref finish.
  bool lost() const noexcept { return lost_; }

  /// @return The mic's line beside the cameras' (@ref array_report):
  ///         "  audio <id>: " and its counters (@ref counters_text), and
  ///         "; ended early" once a drain failed, with a newline; empty with
  ///         no mic.
  std::string report() const {
    if (mic_ == nullptr) return {};
    return "  audio " + mic_->info().id + ": " + counters_text(mic_->stats()) +
           (lost_ ? "; ended early\n" : "\n");
  }

 private:
  sensor::IAudioSensor* mic_;
  volumetric_kit::grid::codec_stream::CodecStream* stream_;
  std::uint16_t track_;
  const char* program_;
  bool running_ = false;  // started, not finished, no drain failed
  bool lost_ = false;
  std::vector<sensor::AudioChunk> chunks_;  // the last drain's, reused
};

}  // namespace vr_example
