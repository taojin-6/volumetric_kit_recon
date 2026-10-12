// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The codec examples' stream (examples/common/codec_stream.hpp) on a grid
// holding a sphere: outputs it cannot make refused on create, before any
// coding, and none written without outputs; a row per coded frame, under the
// source's index, whose bytes and blocks are the frame's; every frame written
// to a new frame directory once, a refused one included, and to a further
// path it replaces; and the CSV replaced on create and extended as each frame
// is coded. Skips with no device.

#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "codec_fixture.hpp"
#include "codec_stream.hpp"
#include "gpu_test.hpp"
#include "scratch_dir.hpp"
#include "volumetric_kit/core/base/file_io.hpp"

namespace codec = volumetric_kit::recon::codec;
namespace fs = std::filesystem;
using namespace codec_fixture;

namespace {

std::vector<std::string> lines_of(const std::string& path) {
  std::ifstream file(path);
  std::vector<std::string> lines;
  for (std::string line; std::getline(file, line);) lines.push_back(line);
  return lines;
}

// Outputs that cannot be made are refused by create with the core's code,
// naming the path.
int refused(vr_test::GpuContext& gpu, const vr_test::Scratch& scratch) {
  const auto refuses = [&gpu](const vr_example::CodecOutputs& outputs,
                              vkc::Status::Code code,
                              const std::string& named) {
    vkc::Result<vr_example::CodecStream> made =
        vr_example::CodecStream::create(gpu.device, gpu.allocator, {}, outputs);
    return !made.ok() && made.status().domain() == code &&
           made.status().message().find(named) != std::string::npos;
  };
  vr_example::CodecOutputs csv;
  csv.csv = (scratch.path / "missing" / "rows.csv").string();
  CHECK(refuses(csv, vkc::Status::Code::IoError, csv.csv));
  // A frame directory must be new; the CSV after it is left unwritten.
  vr_example::CodecOutputs existing;
  existing.frame_directory = scratch.path.string();
  existing.csv = (scratch.path / "unwritten.csv").string();
  CHECK(refuses(existing, vkc::Status::Code::InvalidArgument,
                existing.frame_directory));
  CHECK(!fs::exists(existing.csv));
  vr_example::CodecOutputs orphan;
  orphan.frame_directory = (scratch.path / "missing" / "frames").string();
  CHECK(refuses(orphan, vkc::Status::Code::NotFound, orphan.frame_directory));
  // A CSV refused after a new frame directory takes the directory back.
  vr_example::CodecOutputs late;
  late.frame_directory = (scratch.path / "late").string();
  late.csv = csv.csv;
  CHECK(refuses(late, vkc::Status::Code::IoError, late.csv));
  CHECK(!fs::exists(late.frame_directory));
  return 0;
}

// With no outputs, the stream codes and keeps rows but writes no file, so
// no frame lands in the working directory.
int writes_nothing(vr_test::GpuContext& gpu, vol::VoxelBlockGrid& source,
                   const vr_test::Scratch& scratch) {
  const fs::path quiet = scratch.path / "quiet";
  CHECK(fs::create_directory(quiet));
  vkc::Result<vr_example::CodecStream> made =
      vr_example::CodecStream::create(gpu.device, gpu.allocator, {});
  CHECK(made.ok());
  const fs::path before = fs::current_path();
  fs::current_path(quiet);
  const vkc::Status coded = made->code(source, 0);
  fs::current_path(before);
  CHECK(coded.ok() && made->rows().size() == 1);
  CHECK(fs::is_empty(quiet));
  return 0;
}

int gpu_main(vr_test::GpuContext& gpu) {
  const Sphere sphere{vr::Vec3f(0.01f, -0.02f, 0.03f), 0.09f};
  vkc::Result<vol::VoxelBlockGrid> source = sphere_grid(gpu, sphere);
  CHECK(source.ok());

  vr_test::Scratch scratch("recon_example_codec_stream_");
  CHECK(refused(gpu, scratch) == 0);
  CHECK(writes_nothing(gpu, *source, scratch) == 0);
  vr_example::CodecOutputs outputs;
  outputs.csv = (scratch.path / "rows.csv").string();
  outputs.frame_directory = (scratch.path / "frames").string();
  // An earlier run's CSV is replaced.
  const std::string stale = "stale\nstale\n";
  CHECK(vkc::write_file(outputs.csv, stale, vkc::WriteMode::CreateNew).ok());
  vkc::Result<vr_example::CodecStream> made =
      vr_example::CodecStream::create(gpu.device, gpu.allocator, {}, outputs);
  CHECK(made.ok());
  vr_example::CodecStream stream = std::move(made).value();
  CHECK(fs::is_directory(outputs.frame_directory));
  // The CSV holds its header before any frame is coded.
  const std::vector<std::string> started = lines_of(outputs.csv);
  CHECK(started.size() == 1 &&
        started[0] + "\n" == vr_example::kCodecRowsCsvHeader);

  // A further path replaces what an earlier run left there.
  const std::string copy = (scratch.path / "copy.vrtc").string();
  CHECK(vkc::write_file(copy, "stale", vkc::WriteMode::CreateNew).ok());
  CHECK(stream.code(*source, 3).ok());
  CHECK(stream.code(*source, 7, copy).ok());
  const std::vector<vr_example::CodecRow>& rows = stream.rows();
  CHECK(rows.size() == 2 && rows[0].frame == 3 && rows[1].frame == 7);
  for (const vr_example::CodecRow& row : rows) {
    const std::vector<std::uint8_t> frame = vr_test::read_bytes(
        vr_example::frame_file(outputs.frame_directory, row.frame));
    CHECK(!frame.empty() && frame.size() == row.bytes);
    vkc::Result<codec::FrameInfo> info =
        codec::read_frame_info(frame.data(), frame.size());
    CHECK(info.ok() && info->block_count == row.blocks && row.blocks > 0);
    CHECK(row.encode_ms > 0.0 && row.decode_ms > 0.0);
    CHECK(!row.encode_device_ms || *row.encode_device_ms >= 0.0);
    CHECK(!row.decode_device_ms || *row.decode_device_ms >= 0.0);
  }
  // The same grid codes to the same frame, and the further path has it too.
  const std::string seventh =
      vr_example::frame_file(outputs.frame_directory, 7);
  const std::vector<std::uint8_t> first = vr_test::read_bytes(seventh);
  CHECK(rows[0].bytes == rows[1].bytes && rows[0].blocks == rows[1].blocks);
  CHECK(vr_test::read_bytes(copy) == first);

  // A frame index coded again is refused before it decodes: its file is
  // kept as it was, and it has no row.
  const vkc::Status again = stream.code(*source, 7);
  CHECK(again.domain() == vkc::Status::Code::InvalidArgument);
  CHECK(again.message().find(seventh) != std::string::npos);
  CHECK(vr_test::read_bytes(seventh) == first);
  CHECK(rows.size() == 2);

  // A frame the decoder refuses -- another grid's geometry than the
  // player's -- is in the frame directory, and has no row.
  GridShape coarser;
  coarser.voxel_size = 2.0f * kVoxel;
  vkc::Result<vol::VoxelBlockGrid> other = sphere_grid(gpu, sphere, coarser);
  CHECK(other.ok());
  const vkc::Status refused_frame = stream.code(*other, 9);
  CHECK(refused_frame.domain() == vkc::Status::Code::InvalidArgument);
  const std::vector<std::uint8_t> ninth =
      vr_test::read_bytes(vr_example::frame_file(outputs.frame_directory, 9));
  vkc::Result<codec::FrameInfo> ninth_info =
      codec::read_frame_info(ninth.data(), ninth.size());
  CHECK(ninth_info.ok() && ninth_info->voxel_size == coarser.voxel_size);
  CHECK(rows.size() == 2);

  // Each row is in the CSV as soon as its frame is coded.
  const std::vector<std::string> lines = lines_of(outputs.csv);
  CHECK(lines.size() == 3);
  CHECK(lines[1] + "\n" == vr_example::codec_row_csv(rows[0]));
  CHECK(lines[2] + "\n" == vr_example::codec_row_csv(rows[1]));
  stream.report(30.0, 4);
  std::puts("example_codec_stream: OK");
  return 0;
}

}  // namespace

int main() { return vr_test::run_on_gpu(gpu_main); }
