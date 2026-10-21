// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The codec examples' stream (examples/common/codec_stream.hpp). On the host:
// a row's type and host and device milliseconds from its calls' stage rows,
// its CSV line, a frame type's sizes, a GOP's bytes and bitrate, a frame's
// pts, and the comparison options the codec examples share. On a grid
// holding a sphere: outputs it cannot make refused on
// create, before any coding, and none written without outputs; a row per
// decoded frame, under the source's index, whose type, bytes and blocks are
// the frame's, and each type's stage rows its own frames'; every encoded
// frame a record of the .vrst, a frame the decoder refuses included and the
// stream going on after it, byte for byte what an encoder returns, a key
// exactly when an I-frame, and the same file on a rerun; after a frame
// encoded but not decoded, or whose CSV row fails, an I-frame where the
// encoder would code a P-frame; a frame out of order, or any call once
// finished, refused before anything is encoded; a stream never finished
// still read whole, and one dropped before its first frame removed; the CSV
// replaced on create and extended as each frame is coded; and, served on
// 127.0.0.1 port 0 (never every interface), every frame the file records
// reaching a client as it is coded, with the file's seq, key, pts, set, bytes
// and offset, and the end once finished, a frame the file refuses never
// served, serving alone writing no file, a port in use refused before any
// file is made, and neither a client that leaves nor a record the server
// refuses failing a frame. The device part skips with no device.

#include <signal.h>
#include <sys/resource.h>

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "codec_fixture.hpp"
#include "codec_stream.hpp"
#include "gpu_test.hpp"
#include "networking_loopback.hpp"
#include "scratch_dir.hpp"
#include "volumetric_kit/core/base/file_io.hpp"
#include "volumetric_kit/recon/networking/stream_client.hpp"
#include "volumetric_kit/recon/networking/stream_server.hpp"
#include "volumetric_kit/recon/stream/stream_reader.hpp"

namespace codec = volumetric_kit::recon::codec;
namespace eval = volumetric_kit::recon::eval;
namespace networking = volumetric_kit::recon::networking;
namespace stream = volumetric_kit::recon::stream;
namespace net = vr_test::net;
namespace fs = std::filesystem;
using namespace codec_fixture;

namespace {

constexpr double kFps = 30.0;

std::uint64_t pts(std::size_t frame) {
  return vr_example::frame_pts_ns(frame, kFps);
}

int row_times() {
  // Host: the stage, its breakdowns inside it. Device: every span timed,
  // breakdowns included, as they are separate dispatches.
  vkc::StageMetrics encode;
  encode.add_cpu("codec encode", 3.0);
  encode.add_gpu("codec encode", 1.0);
  encode.add_cpu("  ..active set", 0.5);
  encode.add_gpu("  ..active set", 0.25);
  vkc::StageMetrics decode;  // no timer ran
  decode.add_cpu("codec decode", 2.0);
  decode.add_cpu("  ..rans decode", 1.5);
  const vr_example::CodecRow row = vr_example::make_codec_row(
      7, codec::FrameType::kInter, 4096, 12, encode, decode);
  CHECK(row.frame == 7 && row.type == codec::FrameType::kInter);
  CHECK(row.bytes == 4096 && row.blocks == 12);
  CHECK(row.encode_ms == 3.0 && row.encode_device_ms == 1.25);
  CHECK(row.decode_ms == 2.0 && !row.decode_device_ms);
  // The sweep's device columns count the same way.
  CHECK(vr_example::device_ms(encode) == 1.25 &&
        !vr_example::device_ms(decode));
  return 0;
}

int csv_lines() {
  // The header, then a line per row, its type I or P and an unset device
  // column empty.
  constexpr codec::FrameType I = codec::FrameType::kIntra;
  constexpr codec::FrameType P = codec::FrameType::kInter;
  const vr_example::CodecRow rows[] = {
      {3, I, 1000, 10, 1.5, 0.5, 2.0, std::nullopt},
      {7, P, 3000, 30, 2.5, std::nullopt, 1.0, std::nullopt},
      {11, P, 2000, 20, 0.25, 0.75, 3.0, std::nullopt}};
  std::string text = vr_example::kCodecRowsCsvHeader;
  for (const vr_example::CodecRow& row : rows) {
    text += vr_example::codec_row_csv(row);
  }
  CHECK(text ==
        "frame,type,bytes,blocks,encode_host_ms,encode_device_ms,"
        "decode_host_ms,decode_device_ms\n"
        "3,I,1000,10,1.500,0.500,2.000,\n"
        "7,P,3000,30,2.500,,1.000,\n"
        "11,P,2000,20,0.250,0.750,3.000,\n");
  return 0;
}

int type_sizes() {
  // Each type's frames apart: two I-frames, and between them P-frames of 200
  // down to 10 bytes, whose nearest-rank p50 and p95 are the 10th and 19th
  // smallest of 20.
  std::vector<vr_example::CodecRow> rows(2);
  rows[0].bytes = 1000;
  rows[1].bytes = 3000;
  for (std::size_t i = 1; i <= 20; ++i) {
    vr_example::CodecRow row;
    row.type = codec::FrameType::kInter;
    row.bytes = 10 * i;
    rows.insert(rows.begin() + 1, row);
  }
  const auto intra = vr_example::frame_sizes(rows, codec::FrameType::kIntra);
  CHECK(intra.count == 2 && intra.mean == 2000.0 && intra.p50 == 1000);
  CHECK(intra.p95 == 3000 && intra.max == 3000);
  const auto inter = vr_example::frame_sizes(rows, codec::FrameType::kInter);
  CHECK(inter.count == 20 && inter.mean == 105.0 && inter.p50 == 100);
  CHECK(inter.p95 == 190 && inter.max == 200);
  rows.resize(1);
  const auto none = vr_example::frame_sizes(rows, codec::FrameType::kInter);
  CHECK(none.count == 0 && none.mean == 0.0 && none.max == 0);
  return 0;
}

int gop_rates() {
  // An I-frame of 1000 bytes and P-frames of 100: a GOP of 4 holds one
  // I-frame and three P-frames, 325 bytes a frame; a GOP of 1 the I-frame.
  vr_example::FrameSizes intra;
  intra.mean = 1000.0;
  vr_example::FrameSizes inter;
  inter.mean = 100.0;
  CHECK(vr_example::gop_bytes(intra, inter, 4) == 325.0);
  CHECK(vr_example::gop_bytes(intra, inter, 1) == 1000.0);
  CHECK(vr_example::mbit_per_s(325.0, 30.0) == 0.078);
  return 0;
}

int frame_times() {
  // From frame 0 at 0, to the nearest ns.
  CHECK(vr_example::frame_pts_ns(0, 30.0) == 0);
  CHECK(vr_example::frame_pts_ns(1, 30.0) == 33'333'333);
  CHECK(vr_example::frame_pts_ns(2, 30.0) == 66'666'667);
  CHECK(vr_example::frame_pts_ns(3, 30.0) == 100'000'000);
  CHECK(vr_example::frame_pts_ns(400, 30.0) == 13'333'333'333);
  return 0;
}

int compare_options() {
  // The band's width of reach, 2 cm at least; 1 vertex in 4; F at half a
  // voxel.
  const eval::CompareOptions wide =
      vr_example::codec_compare_options(0.01f, 0.04f);
  CHECK(wide.reach == 0.04f && wide.stride == 4 &&
        wide.fscore_threshold == 0.005f);
  const eval::CompareOptions narrow =
      vr_example::codec_compare_options(0.002f, 0.008f);
  CHECK(narrow.reach == 0.02f && narrow.fscore_threshold == 0.001f);
  return 0;
}

std::vector<std::string> lines_of(const std::string& path) {
  std::ifstream file(path);
  std::vector<std::string> lines;
  for (std::string line; std::getline(file, line);) lines.push_back(line);
  return lines;
}

// Outputs that cannot be made are refused by create with the writer's or
// the stream's code, naming the path.
int refused(vr_test::GpuContext& gpu, const vr_test::Scratch& scratch) {
  const auto refuses = [&gpu](const vr_example::CodecOutputs& outputs,
                              vkc::Status::Code code,
                              const std::string& named) {
    vkc::Result<vr_example::CodecStream> made =
        vr_example::CodecStream::create(gpu.device, gpu.allocator, {}, outputs);
    return !made.ok() && made.status().domain() == code &&
           made.status().message().find(named) != std::string::npos;
  };
  // A stream must be new: what is at its path is left as it was, and the
  // CSV after it unwritten.
  vr_example::CodecOutputs existing;
  existing.stream = (scratch.path / "earlier.vrst").string();
  existing.csv = (scratch.path / "unwritten.csv").string();
  CHECK(vkc::write_file(existing.stream, "earlier", vkc::WriteMode::CreateNew)
            .ok());
  CHECK(refuses(existing, vkc::Status::Code::InvalidArgument, existing.stream));
  const vkc::Result<std::string> kept = vkc::read_file(existing.stream);
  CHECK(kept.ok() && *kept == "earlier");
  CHECK(!fs::exists(existing.csv));
  vr_example::CodecOutputs orphan;
  orphan.stream = (scratch.path / "missing" / "frames.vrst").string();
  CHECK(refuses(orphan, vkc::Status::Code::NotFound, orphan.stream));
  // A CSV refused after a new stream takes the stream's file back.
  vr_example::CodecOutputs late;
  late.stream = (scratch.path / "late.vrst").string();
  late.csv = (scratch.path / "missing" / "rows.csv").string();
  CHECK(refuses(late, vkc::Status::Code::IoError, late.csv));
  CHECK(!fs::exists(late.stream));
  return 0;
}

// The CSV must not truncate the stream through another name. Rejection
// removes the new stream and preserves any pre-existing symlinks.
int aliased_outputs(vr_test::GpuContext& gpu, const vr_test::Scratch& scratch) {
  const fs::path dir = scratch.path / "aliases";
  CHECK(fs::create_directory(dir));
  const fs::path stream_path = dir / "frames.vrst";
  const fs::path csv_link = dir / "rows.csv";
  fs::create_symlink(stream_path.filename(), csv_link);
  const fs::path dir_link = scratch.path / "linked_outputs";
  fs::create_directory_symlink(dir, dir_link);
  std::vector<fs::path> csv_paths = {csv_link, dir_link / "frames.vrst",
                                     stream_path, dir / "." / "frames.vrst"};
  // The temporary directory's filesystem determines whether case aliases
  // exist, independently of the operating system.
  if (fs::exists(scratch.path / "ALIASES")) {
    csv_paths.push_back(dir / "FRAMES.vrst");
  } else {
    std::puts("aliased_outputs: case-sensitive filesystem; case alias skipped");
  }
  for (const fs::path& csv_path : csv_paths) {
    CHECK(!fs::exists(stream_path));
    const vr_example::CodecOutputs outputs{csv_path.string(),
                                           stream_path.string()};
    vkc::Result<vr_example::CodecStream> made =
        vr_example::CodecStream::create(gpu.device, gpu.allocator, {}, outputs);
    CHECK(!made.ok());
    CHECK(made.status().domain() == vkc::Status::Code::InvalidArgument);
    CHECK(made.status().message().find(outputs.csv) != std::string::npos);
    CHECK(made.status().message().find(outputs.stream) != std::string::npos);
    CHECK(!fs::exists(stream_path));
    CHECK(fs::is_symlink(csv_link));
    CHECK(fs::read_symlink(csv_link) == stream_path.filename());
    CHECK(fs::is_symlink(dir_link));
  }
  // A corrected configuration can retry at the same stream path, with a
  // CSV that does not exist yet. Both files must be readable after finish.
  const vr_example::CodecOutputs distinct{(dir / "separate.csv").string(),
                                          stream_path.string()};
  vkc::Result<vr_example::CodecStream> made =
      vr_example::CodecStream::create(gpu.device, gpu.allocator, {}, distinct);
  CHECK(made.ok());
  CHECK(made->finish().ok());
  const vkc::Result<std::string> csv = vkc::read_file(distinct.csv);
  CHECK(csv.ok() && *csv == vr_example::kCodecRowsCsvHeader);
  CHECK(stream::StreamReader::open(distinct.stream).ok());
  return 0;
}

// With no outputs, the stream codes and keeps rows but writes no file, so
// nothing lands in the working directory; a frame out of order, and any call
// once finished, is refused all the same, with no writer to refuse it.
int writes_nothing(vr_test::GpuContext& gpu, vol::VoxelBlockGrid& source,
                   const vr_test::Scratch& scratch) {
  const fs::path quiet = scratch.path / "quiet";
  CHECK(fs::create_directory(quiet));
  vkc::Result<vr_example::CodecStream> made =
      vr_example::CodecStream::create(gpu.device, gpu.allocator, {});
  CHECK(made.ok());
  const fs::path before = fs::current_path();
  fs::current_path(quiet);
  const vkc::Status coded[] = {
      made->code(source, 0, pts(0)), made->code(source, 2, pts(2)),
      // Out of order: a frame not above the last, a pts below the last.
      made->code(source, 2, pts(3)), made->code(source, 3, pts(1)),
      made->finish(),
      // Finished: a frame in order, and a second finish.
      made->code(source, 4, pts(4)), made->finish()};
  fs::current_path(before);
  CHECK(coded[0].ok() && coded[1].ok() && coded[4].ok());
  for (const int i : {2, 3, 5, 6}) {
    CHECK(coded[i].domain() == vkc::Status::Code::InvalidArgument);
  }
  CHECK(made->rows().size() == 2);
  CHECK(fs::is_empty(quiet));
  return 0;
}

// Frames 3 and 7 of `source`, frames out of order, `other`'s frame 9, which
// the decoder refuses, `source`'s frame 10, then finish, into `outputs`, whose
// CSV an earlier run left. `rows_out` receives the rows.
int code_frames(vr_test::GpuContext& gpu, vol::VoxelBlockGrid& source,
                vol::VoxelBlockGrid& other,
                const vr_example::CodecOutputs& outputs,
                std::vector<vr_example::CodecRow>* rows_out) {
  CHECK(
      vkc::write_file(outputs.csv, "stale\nstale\n", vkc::WriteMode::CreateNew)
          .ok());
  vkc::Result<vr_example::CodecStream> made = vr_example::CodecStream::create(
      gpu.device, gpu.allocator, {}, outputs, "example_codec_stream");
  CHECK(made.ok());
  vr_example::CodecStream coder = std::move(made).value();
  // The CSV holds its header, and the stream's file is there, before any
  // frame is coded.
  const std::vector<std::string> started = lines_of(outputs.csv);
  CHECK(started.size() == 1 &&
        started[0] + "\n" == vr_example::kCodecRowsCsvHeader);
  CHECK(fs::is_regular_file(outputs.stream));
  CHECK(coder.coded() == 0);

  CHECK(coder.code(source, 3, pts(3)).ok());
  CHECK(coder.code(source, 7, pts(7)).ok());
  const std::vector<vr_example::CodecRow>& rows = coder.rows();
  CHECK(rows.size() == 2 && rows[0].frame == 3 && rows[1].frame == 7);
  CHECK(coder.coded() == 2);
  for (const vr_example::CodecRow& row : rows) {
    CHECK(row.bytes > 0 && row.blocks > 0);
    CHECK(row.encode_ms > 0.0 && row.decode_ms > 0.0);
    CHECK(!row.encode_device_ms || *row.encode_device_ms >= 0.0);
    CHECK(!row.decode_device_ms || *row.decode_device_ms >= 0.0);
  }

  // A frame index or a pts out of order is refused before it is encoded,
  // so it has neither a record nor a row.
  const vkc::Status again = coder.code(source, 7, pts(8));
  CHECK(again.domain() == vkc::Status::Code::InvalidArgument);
  CHECK(again.message().find("frame 7") != std::string::npos);
  CHECK(coder.code(source, 8, pts(6)).domain() ==
        vkc::Status::Code::InvalidArgument);
  CHECK(rows.size() == 2 && coder.coded() == 2);

  // A frame the decoder refuses -- another grid's geometry than the
  // player's -- has a record and no row, and the stream goes on after it.
  CHECK(coder.code(other, 9, pts(9)).domain() ==
        vkc::Status::Code::InvalidArgument);
  CHECK(rows.size() == 2 && coder.coded() == 3);
  CHECK(coder.code(source, 10, pts(10)).ok());
  CHECK(rows.size() == 3 && rows[2].frame == 10 && coder.coded() == 4);

  // Each row is in the CSV as soon as its frame decodes.
  const std::vector<std::string> lines = lines_of(outputs.csv);
  CHECK(lines.size() == 4);
  for (std::size_t i = 0; i < rows.size(); ++i) {
    CHECK(lines[i + 1] + "\n" == vr_example::codec_row_csv(rows[i]));
  }

  // Finished, nothing more is coded.
  CHECK(coder.finish().ok());
  CHECK(coder.code(source, 11, pts(11)).domain() ==
        vkc::Status::Code::InvalidArgument);
  CHECK(coder.finish().domain() == vkc::Status::Code::InvalidArgument);
  CHECK(rows.size() == 3);
  coder.report(kFps, 4);
  *rows_out = rows;
  return 0;
}

// code_frames' .vrst: one Geometry track, and a key record for each frame
// the encoder returned, in order, its payload what a separate encoder returns
// for the same grid.
int check_records(vr_test::GpuContext& gpu, vol::VoxelBlockGrid& source,
                  const std::string& path,
                  const std::vector<vr_example::CodecRow>& rows,
                  float other_voxel) {
  vkc::Result<std::unique_ptr<stream::StreamReader>> opened =
      stream::StreamReader::open(path);
  CHECK(opened.ok());
  const stream::StreamReader& reader = **opened;
  const stream::Session& session = reader.session();
  CHECK(session.writer == "example_codec_stream");
  CHECK(session.id == (std::array<std::uint8_t, 16>{}));
  CHECK(session.created_ns == 0 && session.calibration.empty());
  CHECK(session.tracks.size() == 1);
  const stream::Track& track = session.tracks[0];
  CHECK(track.kind == stream::TrackKind::Geometry && track.clock == 1);
  CHECK(track.rate.num == 0 && track.rate.den == 1);
  CHECK(track.width == 0 && track.height == 0);
  CHECK(track.sensor.empty() && track.params.empty());
  CHECK(reader.recovery().finished && reader.recovery().torn_bytes == 0);
  // Every frame an I-frame, at the default period of 1: every record a key.
  CHECK(reader.stats()[0].records == 4 && reader.stats()[0].keys == 4);

  vkc::Result<codec::Encoder> enc =
      codec::Encoder::create(gpu.device, gpu.allocator, {});
  CHECK(enc.ok());
  const vkc::Result<vr_example::Bytes> expected = enc->encode(source);
  CHECK(expected.ok());
  const std::size_t sets[] = {3, 7, 9, 10};
  stream::Position pos = reader.begin();
  stream::Record record;
  std::vector<std::uint8_t> payload;
  std::size_t row = 0;  // the row of the next frame that decoded
  for (std::size_t i = 0; i < std::size(sets); ++i) {
    const vkc::Result<bool> more = reader.next(&pos, &record);
    CHECK(more.ok() && *more);
    CHECK(record.info.track == 0 && record.info.key);
    CHECK(record.info.seq == i && record.info.set == sets[i]);
    CHECK(record.info.pts_ns == pts(sets[i]));
    CHECK(reader.read(record, &payload).ok());
    const vkc::Result<codec::FrameInfo> info =
        codec::read_frame_info(payload.data(), payload.size());
    CHECK(info.ok());
    if (row < rows.size() && rows[row].frame == sets[i]) {
      CHECK(payload == *expected);
      CHECK(payload.size() == rows[row].bytes &&
            info->block_count == rows[row].blocks);
      ++row;
    } else {  // the frame the decoder refused
      CHECK(info->voxel_size == other_voxel);
    }
  }
  CHECK(row == rows.size());
  const vkc::Result<bool> end = reader.next(&pos, &record);
  CHECK(end.ok() && !*end);
  return 0;
}

// A stream destroyed without finish leaves a file that reads as a killed
// writer's: not finished, every record whole. Destroyed before its first
// frame, it leaves none, so a rerun onto its path is not refused.
int unfinished(vr_test::GpuContext& gpu, vol::VoxelBlockGrid& source,
               const vr_test::Scratch& scratch) {
  vr_example::CodecOutputs outputs;
  outputs.stream = (scratch.path / "unfinished.vrst").string();
  {  // Dropped before its first frame: the file is taken back.
    vkc::Result<vr_example::CodecStream> made =
        vr_example::CodecStream::create(gpu.device, gpu.allocator, {}, outputs);
    CHECK(made.ok() && fs::exists(outputs.stream));
  }
  CHECK(!fs::exists(outputs.stream));
  {  // Created again at the same path, and dropped with two frames.
    vkc::Result<vr_example::CodecStream> made =
        vr_example::CodecStream::create(gpu.device, gpu.allocator, {}, outputs);
    CHECK(made.ok());
    CHECK(made->code(source, 0, pts(0)).ok());
    CHECK(made->code(source, 1, pts(1)).ok());
  }
  vkc::Result<std::unique_ptr<stream::StreamReader>> opened =
      stream::StreamReader::open(outputs.stream);
  CHECK(opened.ok());
  const stream::StreamReader& reader = **opened;
  CHECK(!reader.recovery().finished && reader.recovery().torn_bytes == 0);
  CHECK(reader.stats()[0].records == 2);
  stream::Position pos = reader.begin();
  stream::Record record;
  std::vector<std::uint8_t> payload;
  for (std::uint64_t seq = 0; seq < 2; ++seq) {
    const vkc::Result<bool> more = reader.next(&pos, &record);
    CHECK(more.ok() && *more && record.info.seq == seq);
    CHECK(reader.read(record, &payload).ok() && !payload.empty());
  }
  return 0;
}

// At a period of 4, `source` coded at frames 0-2 and 5-9 and `other`, which
// the player refuses, at 3 and 4: frame 3 an I-frame on its geometry, and 4
// one too, where the encoder would code a P-frame on 3, which the player
// lacks; 5 an I-frame, the player's geometry again, and the period from it.
// Each row has its frame's type, each type's stage rows hold its own frames'
// calls alone, and each record is a key exactly when it holds an I-frame.
int periodic(vr_test::GpuContext& gpu, vol::VoxelBlockGrid& source,
             vol::VoxelBlockGrid& other, const vr_test::Scratch& scratch) {
  constexpr codec::FrameType I = codec::FrameType::kIntra;
  constexpr codec::FrameType P = codec::FrameType::kInter;
  vr_example::CodecOutputs outputs;
  outputs.stream = (scratch.path / "periodic.vrst").string();
  codec::EncoderConfig config;
  config.intra_period = 4;
  vkc::Result<vr_example::CodecStream> made = vr_example::CodecStream::create(
      gpu.device, gpu.allocator, config, outputs);
  CHECK(made.ok());
  vr_example::CodecStream& coder = made.value();
  const codec::FrameType types[] = {I, P, P, I, I, I, P, P, P, I};
  for (std::size_t frame = 0; frame < std::size(types); ++frame) {
    const bool refused = frame == 3 || frame == 4;
    const vkc::Status s =
        coder.code(refused ? other : source, frame, pts(frame));
    CHECK(s.ok() != refused);
    CHECK(refused == (s.domain() == vkc::Status::Code::InvalidArgument));
  }
  CHECK(coder.finish().ok());
  const std::size_t decoded[] = {0, 1, 2, 5, 6, 7, 8, 9};
  const std::vector<vr_example::CodecRow>& rows = coder.rows();
  CHECK(rows.size() == std::size(decoded));
  for (std::size_t i = 0; i < rows.size(); ++i) {
    CHECK(rows[i].frame == decoded[i] && rows[i].type == types[decoded[i]]);
  }
  // A type's host spans, summed by stage, are its rows' milliseconds.
  const auto near = [](double a, double b) {
    return std::fabs(a - b) <= 1e-9 * std::fabs(b);
  };
  for (const codec::FrameType type : {I, P}) {
    double encode = 0.0;
    double decode = 0.0;
    for (const vr_example::CodecRow& row : rows) {
      if (row.type != type) continue;
      encode += row.encode_ms;
      decode += row.decode_ms;
    }
    CHECK(encode > 0.0 && decode > 0.0);
    CHECK(near(coder.encode_stages(type).total_cpu_ms(), encode));
    CHECK(near(coder.decode_stages(type).total_cpu_ms(), decode));
  }
  coder.report(kFps, 1);

  vkc::Result<std::unique_ptr<stream::StreamReader>> opened =
      stream::StreamReader::open(outputs.stream);
  CHECK(opened.ok());
  const stream::StreamReader& reader = **opened;
  CHECK(reader.stats()[0].records == 10 && reader.stats()[0].keys == 5);
  stream::Position pos = reader.begin();
  stream::Record record;
  std::vector<std::uint8_t> payload;
  for (std::size_t frame = 0; frame < std::size(types); ++frame) {
    const vkc::Result<bool> more = reader.next(&pos, &record);
    CHECK(more.ok() && *more && record.info.set == frame);
    CHECK(reader.read(record, &payload).ok());
    const vkc::Result<codec::FrameInfo> info =
        codec::read_frame_info(payload.data(), payload.size());
    CHECK(info.ok() && info->type == types[frame]);
    CHECK(record.info.key == (types[frame] == I));
  }
  return 0;
}

// At a period of 4, a CSV that stops taking rows after frame 0's -- the file
// size limit reached, SIGXFSZ ignored -- fails frame 1, a P-frame, once it
// decodes: its row is kept, IoError naming the CSV returned, and frame 2 is
// an I-frame where the encoder would code a P-frame. The failure sticks, so
// frame 3 fails too, an I-frame again. A failed append takes the same branch
// in code(), but a writer's I/O failure sticks, so no later frame decodes to
// show it.
int csv_fails(vr_test::GpuContext& gpu, vol::VoxelBlockGrid& source,
              const vr_test::Scratch& scratch) {
  constexpr codec::FrameType I = codec::FrameType::kIntra;
  constexpr codec::FrameType P = codec::FrameType::kInter;
  vr_example::CodecOutputs outputs;
  outputs.csv = (scratch.path / "limited.csv").string();
  codec::EncoderConfig config;
  config.intra_period = 4;
  vkc::Result<vr_example::CodecStream> made = vr_example::CodecStream::create(
      gpu.device, gpu.allocator, config, outputs);
  CHECK(made.ok());
  vr_example::CodecStream& coder = made.value();
  CHECK(coder.code(source, 0, pts(0)).ok());
  const std::uintmax_t size = fs::file_size(outputs.csv);
  // Only this process's limit and disposition change, for one frame, and
  // nothing else is written meanwhile.
  std::fflush(nullptr);
  const auto handler = ::signal(SIGXFSZ, SIG_IGN);
  CHECK(handler != SIG_ERR);
  rlimit previous{};
  CHECK(::getrlimit(RLIMIT_FSIZE, &previous) == 0);
  const rlimit limited{static_cast<rlim_t>(size), previous.rlim_max};
  CHECK(::setrlimit(RLIMIT_FSIZE, &limited) == 0);
  const vkc::Status first = coder.code(source, 1, pts(1));
  CHECK(::setrlimit(RLIMIT_FSIZE, &previous) == 0);
  CHECK(::signal(SIGXFSZ, handler) != SIG_ERR);
  CHECK(first.domain() == vkc::Status::Code::IoError &&
        first.message().find(outputs.csv) != std::string::npos);
  for (std::size_t frame = 2; frame < 4; ++frame) {
    CHECK(coder.code(source, frame, pts(frame)).domain() ==
          vkc::Status::Code::IoError);
  }
  const codec::FrameType types[] = {I, P, I, I};
  const std::vector<vr_example::CodecRow>& rows = coder.rows();
  CHECK(rows.size() == std::size(types));
  for (std::size_t i = 0; i < rows.size(); ++i) {
    CHECK(rows[i].frame == i && rows[i].type == types[i]);
  }
  CHECK(fs::file_size(outputs.csv) == size);
  return 0;
}

// Outputs serving on 127.0.0.1, a free port, besides the stream at
// `stream_path`, if any.
vr_example::CodecOutputs served_on_loopback(const fs::path& stream_path = {}) {
  vr_example::CodecOutputs outputs;
  outputs.stream = stream_path.string();
  outputs.serve.emplace();
  outputs.serve->host = "127.0.0.1";
  outputs.serve->port = 0;
  return outputs;
}

// A client of `coder`'s server that has its session, so it misses no frame
// coded after; null, the reason on stderr, if it cannot.
std::unique_ptr<networking::StreamClient> joined(
    const vr_example::CodecStream& coder) {
  if (coder.server() == nullptr) return nullptr;
  std::unique_ptr<networking::StreamClient> client =
      net::connect(coder.server()->port());
  if (!client || !net::wait_session(*client)) return nullptr;
  return client;
}

// At a period of 3, `source` coded at frames 0-6 but 3, `other`'s frame,
// which the decoder refuses, with a client joined from the start and reading
// each frame once it is coded: it gets every record of the file, the refused
// frame's included, with the file's seq, key, pts, set, payload and offset,
// under the file's session, then the end once the stream is finished, which
// leaves the stream without a server.
int serves(vr_test::GpuContext& gpu, vol::VoxelBlockGrid& source,
           vol::VoxelBlockGrid& other, const vr_test::Scratch& scratch) {
  const vr_example::CodecOutputs outputs =
      served_on_loopback(scratch.path / "served.vrst");
  codec::EncoderConfig config;
  config.intra_period = 3;
  vkc::Result<vr_example::CodecStream> made = vr_example::CodecStream::create(
      gpu.device, gpu.allocator, config, outputs, "example_codec_stream");
  CHECK(made.ok());
  vr_example::CodecStream& coder = made.value();
  CHECK(coder.server() != nullptr && coder.server()->port() != 0);
  const std::unique_ptr<networking::StreamClient> client = joined(coder);
  CHECK(client != nullptr);
  std::vector<net::Got> got;
  for (std::size_t frame = 0; frame < 7; ++frame) {
    const bool refused = frame == 3;
    CHECK(coder.code(refused ? other : source, frame, pts(frame)).ok() !=
          refused);
    net::Got record;
    CHECK(net::next_within(*client, &record));
    got.push_back(std::move(record));
  }
  CHECK(coder.finish().ok());
  CHECK(coder.server() == nullptr);
  CHECK(net::wait_end(*client));

  vkc::Result<std::unique_ptr<stream::StreamReader>> opened =
      stream::StreamReader::open(outputs.stream);
  CHECK(opened.ok());
  const stream::StreamReader& reader = **opened;
  CHECK(vr_test::vrst::same(*client->session(), reader.session()));
  CHECK(reader.stats()[0].records == got.size());
  stream::Position pos = reader.begin();
  stream::Record record;
  std::vector<std::uint8_t> payload;
  for (std::size_t i = 0; i < got.size(); ++i) {
    const vkc::Result<bool> more = reader.next(&pos, &record);
    CHECK(more.ok() && *more);
    CHECK(reader.read(record, &payload).ok());
    const stream::Record& served = got[i].record;
    CHECK(served.info.seq == i && served.info.set == i);
    CHECK(served.info.pts_ns == pts(i));
    CHECK(vr_test::vrst::same(served.info, record.info));
    CHECK(served.offset == record.offset && got[i].payload == payload);
  }
  return 0;
}

// Serving alone, the stream codes its frames and serves each, a key at the
// default period of 1, writing no file.
int serves_alone(vr_test::GpuContext& gpu, vol::VoxelBlockGrid& source,
                 const vr_test::Scratch& scratch) {
  const fs::path quiet = scratch.path / "serving";
  CHECK(fs::create_directory(quiet));
  vkc::Result<vr_example::CodecStream> made = vr_example::CodecStream::create(
      gpu.device, gpu.allocator, {}, served_on_loopback());
  CHECK(made.ok());
  vr_example::CodecStream& coder = made.value();
  const std::unique_ptr<networking::StreamClient> client = joined(coder);
  CHECK(client != nullptr);
  const fs::path before = fs::current_path();
  fs::current_path(quiet);
  vkc::Status coded[3];
  net::Got got[3];
  bool arrived[3] = {};
  for (std::size_t frame = 0; frame < 3; ++frame) {
    coded[frame] = coder.code(source, frame, pts(frame));
    arrived[frame] = net::next_within(*client, &got[frame]);
  }
  const vkc::Status finished = coder.finish();
  fs::current_path(before);
  CHECK(finished.ok() && net::wait_end(*client));
  CHECK(coder.rows().size() == 3);
  for (std::size_t frame = 0; frame < 3; ++frame) {
    CHECK(coded[frame].ok() && arrived[frame]);
    const stream::RecordInfo& info = got[frame].record.info;
    CHECK(info.seq == frame && info.set == frame && info.key);
    CHECK(info.pts_ns == pts(frame));
    CHECK(got[frame].payload.size() == coder.rows()[frame].bytes);
  }
  CHECK(fs::is_empty(quiet));
  return 0;
}

// A port in use fails create with the server's IoError, naming it, before
// the stream or the CSV is made.
int port_in_use(vr_test::GpuContext& gpu, const vr_test::Scratch& scratch) {
  const net::Listener taken = net::bind_loopback();
  CHECK(taken.port != 0);
  vr_example::CodecOutputs outputs =
      served_on_loopback(scratch.path / "unserved.vrst");
  outputs.csv = (scratch.path / "unserved.csv").string();
  outputs.serve->port = taken.port;
  vkc::Result<vr_example::CodecStream> made =
      vr_example::CodecStream::create(gpu.device, gpu.allocator, {}, outputs);
  CHECK(!made.ok() && made.status().domain() == vkc::Status::Code::IoError);
  CHECK(made.status().message().find(":" + std::to_string(taken.port)) !=
        std::string::npos);
  CHECK(!fs::exists(outputs.stream) && !fs::exists(outputs.csv));
  return 0;
}

// A client that leaves mid-run, after two frames, never fails a frame: the
// rest are coded and recorded as before.
int viewer_leaves(vr_test::GpuContext& gpu, vol::VoxelBlockGrid& source,
                  const vr_test::Scratch& scratch) {
  const vr_example::CodecOutputs outputs =
      served_on_loopback(scratch.path / "left.vrst");
  codec::EncoderConfig config;
  config.intra_period = 3;
  vkc::Result<vr_example::CodecStream> made = vr_example::CodecStream::create(
      gpu.device, gpu.allocator, config, outputs);
  CHECK(made.ok());
  vr_example::CodecStream& coder = made.value();
  std::unique_ptr<networking::StreamClient> client = joined(coder);
  CHECK(client != nullptr);
  for (std::size_t frame = 0; frame < 2; ++frame) {
    CHECK(coder.code(source, frame, pts(frame)).ok());
    net::Got got;
    CHECK(net::next_within(*client, &got) && got.record.info.seq == frame);
  }
  client.reset();
  for (std::size_t frame = 2; frame < 8; ++frame) {
    CHECK(coder.code(source, frame, pts(frame)).ok());
  }
  CHECK(coder.finish().ok() && coder.rows().size() == 8);
  vkc::Result<std::unique_ptr<stream::StreamReader>> opened =
      stream::StreamReader::open(outputs.stream);
  CHECK(opened.ok() && (*opened)->stats()[0].records == 8);
  return 0;
}

// A frame the file refuses is never served: with the stream's file at its
// size limit (SIGXFSZ ignored), frame 1's record cannot be appended, and the
// client, which got frame 0, sees the end once the stream is finished, with
// nothing between.
int unrecorded_unserved(vr_test::GpuContext& gpu, vol::VoxelBlockGrid& source,
                        const vr_test::Scratch& scratch) {
  const vr_example::CodecOutputs outputs =
      served_on_loopback(scratch.path / "full.vrst");
  vkc::Result<vr_example::CodecStream> made =
      vr_example::CodecStream::create(gpu.device, gpu.allocator, {}, outputs);
  CHECK(made.ok());
  vr_example::CodecStream& coder = made.value();
  const std::unique_ptr<networking::StreamClient> client = joined(coder);
  CHECK(client != nullptr);
  CHECK(coder.code(source, 0, pts(0)).ok());
  net::Got first;
  CHECK(net::next_within(*client, &first) && first.record.info.seq == 0);
  // Only this process's limit and disposition change, for one frame, and
  // nothing else is written meanwhile.
  const std::uintmax_t size = fs::file_size(outputs.stream);
  std::fflush(nullptr);
  const auto handler = ::signal(SIGXFSZ, SIG_IGN);
  CHECK(handler != SIG_ERR);
  rlimit previous{};
  CHECK(::getrlimit(RLIMIT_FSIZE, &previous) == 0);
  const rlimit limited{static_cast<rlim_t>(size), previous.rlim_max};
  CHECK(::setrlimit(RLIMIT_FSIZE, &limited) == 0);
  const vkc::Status full = coder.code(source, 1, pts(1));
  CHECK(::setrlimit(RLIMIT_FSIZE, &previous) == 0);
  CHECK(::signal(SIGXFSZ, handler) != SIG_ERR);
  CHECK(full.domain() == vkc::Status::Code::IoError);
  // The writer's failure sticks, so its footer fails too; the viewers are
  // closed all the same.
  CHECK(!coder.finish().ok() && coder.server() == nullptr);
  CHECK(net::wait_end(*client));
  return 0;
}

// A record the server refuses fails no frame: with a key at seq 1000 already
// published to the stream's server, each record after is out of order there,
// yet every frame is coded and recorded, each refusal a Warning naming it.
int refused_publish(vr_test::GpuContext& gpu, vol::VoxelBlockGrid& source,
                    const vr_test::Scratch& scratch) {
  const vr_example::CodecOutputs outputs =
      served_on_loopback(scratch.path / "refused.vrst");
  vkc::Result<vr_example::CodecStream> made =
      vr_example::CodecStream::create(gpu.device, gpu.allocator, {}, outputs);
  CHECK(made.ok());
  vr_example::CodecStream& coder = made.value();
  // server() hands the stream's own server out as const.
  auto* const server = const_cast<networking::StreamServer*>(coder.server());
  CHECK(server != nullptr);
  const net::Bytes ahead = net::record_payload(1000, 4);
  CHECK(
      server->publish(net::record_info(1000, true), ahead.data(), ahead.size())
          .ok());
  const std::size_t before = gpu.log.warnings().size();
  for (std::size_t frame = 0; frame < 3; ++frame) {
    CHECK(coder.code(source, frame, pts(frame)).ok());
  }
  const std::vector<std::string> logged = gpu.log.warnings();
  std::vector<std::string> warnings;  // the stream's since `before`
  for (std::size_t i = before; i < logged.size(); ++i) {
    if (logged[i].find("not served") != std::string::npos) {
      warnings.push_back(logged[i]);
    }
  }
  CHECK(warnings.size() == 3);
  for (std::size_t frame = 0; frame < 3; ++frame) {
    CHECK(warnings[frame].find("frame " + std::to_string(frame) +
                               " not served") != std::string::npos);
  }
  CHECK(coder.finish().ok());
  vkc::Result<std::unique_ptr<stream::StreamReader>> opened =
      stream::StreamReader::open(outputs.stream);
  CHECK(opened.ok() && (*opened)->stats()[0].records == 3);
  return 0;
}

int gpu_main(vr_test::GpuContext& gpu) {
  const Sphere sphere{vr::Vec3f(0.01f, -0.02f, 0.03f), 0.09f};
  vkc::Result<vol::VoxelBlockGrid> source = sphere_grid(gpu, sphere);
  CHECK(source.ok());
  GridShape coarser;
  coarser.voxel_size = 2.0f * kVoxel;
  vkc::Result<vol::VoxelBlockGrid> other = sphere_grid(gpu, sphere, coarser);
  CHECK(other.ok());

  vr_test::Scratch scratch("recon_example_codec_stream_");
  CHECK(refused(gpu, scratch) == 0);
  CHECK(aliased_outputs(gpu, scratch) == 0);
  CHECK(writes_nothing(gpu, *source, scratch) == 0);

  const auto outputs_in = [&scratch](const char* name) {
    const fs::path dir = scratch.path / name;
    fs::create_directory(dir);
    vr_example::CodecOutputs outputs;
    outputs.csv = (dir / "rows.csv").string();
    outputs.stream = (dir / "frames.vrst").string();
    return outputs;
  };
  const vr_example::CodecOutputs first = outputs_in("first");
  std::vector<vr_example::CodecRow> rows;
  CHECK(code_frames(gpu, *source, *other, first, &rows) == 0);
  CHECK(check_records(gpu, *source, first.stream, rows, coarser.voxel_size) ==
        0);
  // A rerun writes the same file, byte for byte.
  const vr_example::CodecOutputs second = outputs_in("second");
  std::vector<vr_example::CodecRow> rerun;
  CHECK(code_frames(gpu, *source, *other, second, &rerun) == 0);
  const std::vector<std::uint8_t> bytes = vr_test::read_bytes(first.stream);
  CHECK(!bytes.empty() && bytes == vr_test::read_bytes(second.stream));

  CHECK(periodic(gpu, *source, *other, scratch) == 0);
  CHECK(csv_fails(gpu, *source, scratch) == 0);
  CHECK(unfinished(gpu, *source, scratch) == 0);
  CHECK(serves(gpu, *source, *other, scratch) == 0);
  CHECK(serves_alone(gpu, *source, scratch) == 0);
  CHECK(port_in_use(gpu, scratch) == 0);
  CHECK(viewer_leaves(gpu, *source, scratch) == 0);
  CHECK(unrecorded_unserved(gpu, *source, scratch) == 0);
  CHECK(refused_publish(gpu, *source, scratch) == 0);
  std::puts("example_codec_stream: OK");
  return 0;
}

}  // namespace

int main() {
  CHECK(row_times() == 0);
  CHECK(csv_lines() == 0);
  CHECK(type_sizes() == 0);
  CHECK(gop_rates() == 0);
  CHECK(frame_times() == 0);
  CHECK(compare_options() == 0);
  return vr_test::run_on_gpu(gpu_main);
}
