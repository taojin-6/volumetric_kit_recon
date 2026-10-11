// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The Status an FFmpeg error code becomes (ffmpeg_error), with no codec and no
// GPU: out of memory, a hardware call that failed inside FFmpeg, which names
// FFmpeg as the backend and keeps its code, and anything else as I/O.

#include <cerrno>
#include <cstdio>
#include <string>

#include "ffmpeg.hpp"
#include "test_check.hpp"

namespace vkc = volumetric_kit::core;
namespace video = volumetric_kit::recon::sensor::video;
using Code = vkc::Status::Code;

namespace {

int test_backend() {
  const vkc::Status s =
      video::ffmpeg_error("HevcDecoder", "decoding", AVERROR_EXTERNAL);
  CHECK(s.domain() == Code::Backend);
  CHECK(s.backend() == vkc::Status::Backend::Ffmpeg);
  CHECK(s.detail() == AVERROR_EXTERNAL);
  CHECK(s.message() ==
        "HevcDecoder: decoding: " + video::ffmpeg_message(AVERROR_EXTERNAL));
  return 0;
}

int test_other_domains() {
  const vkc::Status oom =
      video::ffmpeg_error("HevcDecoder", "decoding", AVERROR(ENOMEM));
  CHECK(oom.domain() == Code::OutOfMemory);
  CHECK(!oom.backend());
  const vkc::Status io =
      video::ffmpeg_error("HevcDecoder", "decoding", AVERROR_INVALIDDATA);
  CHECK(io.domain() == Code::IoError);
  CHECK(!io.backend());
  return 0;
}

}  // namespace

int main() {
  if (test_backend() != 0) return 1;
  if (test_other_domains() != 0) return 1;
  std::printf("ffmpeg error tests passed\n");
  return 0;
}
