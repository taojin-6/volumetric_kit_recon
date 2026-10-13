// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The codec examples' stream (examples/common/codec_stream.hpp). On the host:
// a row's host and device milliseconds from its calls' stage rows, its CSV
// line, and a frame's pts. On a grid holding a sphere: outputs it cannot make
// refused on create, before any coding, and none written without outputs; a
// row per decoded frame, under the source's index, whose bytes and blocks are
// the frame's; every encoded frame a record of the .vrst, a frame the decoder
// refuses included, byte for byte what an encoder returns, and the same file
// on a rerun; a frame out of order refused before it is encoded; a stream
// never finished still read whole; and the CSV replaced on create and extended
// as each frame is coded. The device part skips with no device.

#include <array>
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
#include "scratch_dir.hpp"
#include "volumetric_kit/core/base/file_io.hpp"
#include "volumetric_kit/recon/stream/stream_reader.hpp"

namespace codec = volumetric_kit::recon::codec;
namespace stream = volumetric_kit::recon::stream;
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
  const vr_example::CodecRow row =
      vr_example::make_codec_row(7, 4096, 12, encode, decode);
  CHECK(row.frame == 7 && row.bytes == 4096 && row.blocks == 12);
  CHECK(row.encode_ms == 3.0 && row.encode_device_ms == 1.25);
  CHECK(row.decode_ms == 2.0 && !row.decode_device_ms);
  // The sweep's device columns count the same way.
  CHECK(vr_example::device_ms(encode) == 1.25 &&
        !vr_example::device_ms(decode));
  return 0;
}

int csv_lines() {
  // The header, then a line per row, an unset device column empty.
  const vr_example::CodecRow rows[] = {
      {3, 1000, 10, 1.5, 0.5, 2.0, std::nullopt},
      {7, 3000, 30, 2.5, std::nullopt, 1.0, std::nullopt},
      {11, 2000, 20, 0.25, 0.75, 3.0, std::nullopt}};
  std::string text = vr_example::kCodecRowsCsvHeader;
  for (const vr_example::CodecRow& row : rows) {
    text += vr_example::codec_row_csv(row);
  }
  CHECK(text ==
        "frame,bytes,blocks,encode_host_ms,encode_device_ms,decode_host_ms,"
        "decode_device_ms\n"
        "3,1000,10,1.500,0.500,2.000,\n"
        "7,3000,30,2.500,,1.000,\n"
        "11,2000,20,0.250,0.750,3.000,\n");
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

// With no outputs, the stream codes and keeps rows but writes no file, so
// nothing lands in the working directory; a frame out of order is refused
// all the same.
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
      made->finish()};
  fs::current_path(before);
  CHECK(coded[0].ok() && coded[1].ok() && coded[4].ok());
  CHECK(coded[2].domain() == vkc::Status::Code::InvalidArgument);
  CHECK(coded[3].domain() == vkc::Status::Code::InvalidArgument);
  CHECK(made->rows().size() == 2);
  CHECK(fs::is_empty(quiet));
  return 0;
}

// Frames 3 and 7 of `source`, frames out of order, `other`'s frame 9, which
// the decoder refuses, then finish, into `outputs`, whose CSV an earlier run
// left. `rows` receives the rows.
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

  CHECK(coder.code(source, 3, pts(3)).ok());
  CHECK(coder.code(source, 7, pts(7)).ok());
  const std::vector<vr_example::CodecRow>& rows = coder.rows();
  CHECK(rows.size() == 2 && rows[0].frame == 3 && rows[1].frame == 7);
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
  CHECK(rows.size() == 2);

  // A frame the decoder refuses -- another grid's geometry than the
  // player's -- has a record and no row.
  CHECK(coder.code(other, 9, pts(9)).domain() ==
        vkc::Status::Code::InvalidArgument);
  CHECK(rows.size() == 2);

  // Each row is in the CSV as soon as its frame decodes.
  const std::vector<std::string> lines = lines_of(outputs.csv);
  CHECK(lines.size() == 3);
  CHECK(lines[1] + "\n" == vr_example::codec_row_csv(rows[0]));
  CHECK(lines[2] + "\n" == vr_example::codec_row_csv(rows[1]));

  // Finished, nothing more is coded.
  CHECK(coder.finish().ok());
  CHECK(coder.code(source, 11, pts(11)).domain() ==
        vkc::Status::Code::InvalidArgument);
  CHECK(coder.finish().domain() == vkc::Status::Code::InvalidArgument);
  CHECK(rows.size() == 2);
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
  CHECK(reader.stats()[0].records == 3 && reader.stats()[0].keys == 3);

  vkc::Result<codec::Encoder> enc =
      codec::Encoder::create(gpu.device, gpu.allocator, {});
  CHECK(enc.ok());
  const vkc::Result<vr_example::Bytes> expected = enc->encode(source);
  CHECK(expected.ok());
  const std::size_t sets[] = {3, 7, 9};
  stream::Position pos = reader.begin();
  stream::Record record;
  std::vector<std::uint8_t> payload;
  for (std::size_t i = 0; i < 3; ++i) {
    const vkc::Result<bool> more = reader.next(&pos, &record);
    CHECK(more.ok() && *more);
    CHECK(record.info.track == 0 && record.info.key);
    CHECK(record.info.seq == i && record.info.set == sets[i]);
    CHECK(record.info.pts_ns == pts(sets[i]));
    CHECK(reader.read(record, &payload).ok());
    const vkc::Result<codec::FrameInfo> info =
        codec::read_frame_info(payload.data(), payload.size());
    CHECK(info.ok());
    if (i < rows.size()) {
      CHECK(payload == *expected);
      CHECK(payload.size() == rows[i].bytes &&
            info->block_count == rows[i].blocks);
    } else {
      CHECK(info->voxel_size == other_voxel);
    }
  }
  const vkc::Result<bool> end = reader.next(&pos, &record);
  CHECK(end.ok() && !*end);
  return 0;
}

// A stream destroyed without finish leaves a file that reads as a killed
// writer's: not finished, every record whole.
int unfinished(vr_test::GpuContext& gpu, vol::VoxelBlockGrid& source,
               const vr_test::Scratch& scratch) {
  vr_example::CodecOutputs outputs;
  outputs.stream = (scratch.path / "unfinished.vrst").string();
  {
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

  CHECK(unfinished(gpu, *source, scratch) == 0);
  std::puts("example_codec_stream: OK");
  return 0;
}

}  // namespace

int main() {
  CHECK(row_times() == 0);
  CHECK(csv_lines() == 0);
  CHECK(frame_times() == 0);
  return vr_test::run_on_gpu(gpu_main);
}
