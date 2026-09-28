// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// codec_replica: the TSDF codec on real data. Fuses a posed Replica RGB-D
// sequence exactly as fuse_replica does, and as it goes streams the growing
// volume through the codec: every --encode-every frames the fusion grid is
// encoded as one intra frame and decoded into a player grid, built from the
// first frame's header the way a player would build it. It reports what a
// stream costs -- bytes per frame and per block, the bitrate at 30 fps, and
// where the encode and decode time goes, host and device -- and, at the end,
// how far the decoded surface is from the source: both grids meshed, and the
// decoded mesh measured against the source mesh (accuracy) and the source
// against the decoded (coverage).
//
// --sweep then encodes the final grid under a list of configurations and
// prints the rate-distortion table the codec's defaults are chosen from.
//
//   codec_replica <scene_dir> [--voxel 0.01] [--encode-every 1] [--sweep]
//                 [--k 32] [--dc-step 0.25] [--ac-step 0.05] [--segment 64]
//                 [--max-frames N] [--stride N] [--preload] [-o prefix]
//
// Configure with -DCMAKE_BUILD_TYPE=Release before quoting any timing.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "fuse_frame.hpp"
#include "mesh_distance.hpp"
#include "ply_writer.hpp"
#include "replica_capture.hpp"
#include "volumetric_kit/recon/codec/codec_params.hpp"
#include "volumetric_kit/recon/codec/decoder.hpp"
#include "volumetric_kit/recon/codec/encoder.hpp"
#include "volumetric_kit/recon/core/allocator.hpp"
#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/instance.hpp"
#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/core/stage_metrics.hpp"
#include "volumetric_kit/recon/mesh/marching_cubes.hpp"
#include "volumetric_kit/recon/mesh/mesh.hpp"
#include "volumetric_kit/recon/sensor/camera_capture.hpp"
#include "volumetric_kit/recon/tsdf/tsdf_integrator.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"
#include "volumetric_kit/recon/volume/voxel_grid.hpp"

namespace vr = volumetric_kit::recon;
namespace vol = volumetric_kit::recon::volume;
namespace tsdf = volumetric_kit::recon::tsdf;
namespace mesh = volumetric_kit::recon::mesh;
namespace codec = volumetric_kit::recon::codec;
namespace sensor = volumetric_kit::recon::sensor;

namespace {

using Bytes = std::vector<std::uint8_t>;

// Raw tsdf + weight, the two attributes a frame codes: what the codec is
// compressing, per block.
constexpr double kRawBytesPerBlock = 512.0 * 2.0 * sizeof(float);
constexpr double kFps = 30.0;

struct Options {
  std::string scene_dir;
  std::string cam_params;  // default: <scene_dir>/../cam_params.json
  std::string out_prefix;  // empty: write no meshes
  float voxel = 0.01f;
  float trunc = 0.0f;  // 0 => 4 * voxel
  float min_depth = 0.1f;
  float max_depth = 8.0f;
  float max_weight = 20.0f;
  int max_frames = 1 << 30;
  int stride = 1;
  int num_buckets = 16384;
  bool preload = false;
  // Encode + decode every N-th fused frame; 0 codes only the final grid.
  int encode_every = 1;
  codec::EncoderConfig codec;
  bool sweep = false;
  // Measure every N-th vertex in each direction of the mesh comparison.
  int metric_stride = 4;
};

const char* arg_value(int argc, char** argv, int& i) {
  return i + 1 < argc ? argv[++i] : nullptr;
}

vr::Result<Options> parse_args(int argc, char** argv) {
  Options opt;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto need = [&](float& dst) {
      const char* v = arg_value(argc, argv, i);
      if (v != nullptr) dst = std::strtof(v, nullptr);
      return v != nullptr;
    };
    auto need_int = [&](int& dst) {
      const char* v = arg_value(argc, argv, i);
      if (v != nullptr) dst = std::atoi(v);
      return v != nullptr;
    };
    bool ok = true;
    int k = 0;
    int segment = 0;
    if (a == "-o" || a == "--out") {
      const char* v = arg_value(argc, argv, i);
      ok = v != nullptr;
      if (ok) opt.out_prefix = v;
    } else if (a == "--cam-params") {
      const char* v = arg_value(argc, argv, i);
      ok = v != nullptr;
      if (ok) opt.cam_params = v;
    } else if (a == "--voxel") {
      ok = need(opt.voxel);
    } else if (a == "--trunc") {
      ok = need(opt.trunc);
    } else if (a == "--min-depth") {
      ok = need(opt.min_depth);
    } else if (a == "--max-depth") {
      ok = need(opt.max_depth);
    } else if (a == "--max-weight") {
      ok = need(opt.max_weight);
    } else if (a == "--max-frames") {
      ok = need_int(opt.max_frames);
    } else if (a == "--stride") {
      ok = need_int(opt.stride);
    } else if (a == "--buckets") {
      ok = need_int(opt.num_buckets);
    } else if (a == "--preload") {
      opt.preload = true;
    } else if (a == "--encode-every") {
      ok = need_int(opt.encode_every);
    } else if (a == "--k") {
      ok = need_int(k);
      opt.codec.params.coefficient_count = static_cast<std::uint32_t>(k);
    } else if (a == "--dc-step") {
      ok = need(opt.codec.params.dc_step);
    } else if (a == "--ac-step") {
      ok = need(opt.codec.params.ac_step);
    } else if (a == "--segment") {
      ok = need_int(segment);
      opt.codec.segment_size = static_cast<std::uint32_t>(segment);
    } else if (a == "--sweep") {
      opt.sweep = true;
    } else if (a == "--metric-stride") {
      ok = need_int(opt.metric_stride);
    } else if (a[0] == '-') {
      return vr::Status::invalid_argument("unknown flag: " + a);
    } else if (opt.scene_dir.empty()) {
      opt.scene_dir = a;
    } else {
      return vr::Status::invalid_argument("unexpected argument: " + a);
    }
    if (!ok) {
      return vr::Status::invalid_argument(a + " needs a value");
    }
  }
  if (opt.scene_dir.empty()) {
    return vr::Status::invalid_argument(
        "usage: codec_replica <scene_dir> [--voxel m] [--encode-every n] "
        "[--sweep] [--k n] [--dc-step f] [--ac-step f] [--segment n] "
        "[--max-frames n] [--stride n] [--preload] [-o prefix]");
  }
  if (opt.cam_params.empty()) {
    opt.cam_params = opt.scene_dir + "/../cam_params.json";
  }
  for (const float knob :
       {opt.voxel, opt.trunc, opt.min_depth, opt.max_depth, opt.max_weight}) {
    if (!std::isfinite(knob)) {
      return vr::Status::invalid_argument("numeric options must be finite");
    }
  }
  if (!(opt.voxel > 0.0f)) {
    return vr::Status::invalid_argument("--voxel must be > 0");
  }
  if (opt.trunc <= 0.0f) {
    opt.trunc = 4.0f * opt.voxel;
  }
  if (opt.min_depth < 0.0f || !(opt.min_depth < opt.max_depth)) {
    return vr::Status::invalid_argument("--min-depth must be in [0, max)");
  }
  if (opt.max_frames < 1 || opt.stride < 1 || opt.encode_every < 0 ||
      opt.metric_stride < 1) {
    return vr::Status::invalid_argument(
        "--max-frames, --stride and --metric-stride must be >= 1, "
        "--encode-every >= 0");
  }
  if (opt.num_buckets < 1 || std::int64_t(opt.num_buckets) * 8 >
                                 std::numeric_limits<std::int32_t>::max()) {
    return vr::Status::invalid_argument("--buckets out of range");
  }
  // Refused here rather than at the first encode, many frames in.
  if (vr::Status s = opt.codec.params.validate(); !s.ok()) {
    return s;
  }
  if (opt.codec.segment_size == 0) {
    return vr::Status::invalid_argument("--segment must be >= 1");
  }
  return opt;
}

// A grid for a stream, from its first frame's header: its geometry, the
// fusion grid's hash-table shape, and tsdf + weight only -- all a frame
// carries, and all the Decoder accepts.
vr::Result<vol::VoxelBlockGrid> player_grid(vr::Device& device,
                                            vr::Allocator& allocator,
                                            const Bytes& frame,
                                            std::int32_t num_buckets) {
  VR_ASSIGN(const codec::FrameInfo info,
            codec::read_frame_info(frame.data(), frame.size()));
  vol::VoxelGridParams gp{};
  gp.voxel_size = info.voxel_size;
  gp.block_size = codec::kBlockSize;
  gp.voxels_per_block = std::int32_t(codec::kVoxelsPerBlock);
  gp.trunc_dist = info.trunc_dist;
  gp.bucket_size = 8;
  gp.num_buckets = num_buckets;
  gp.num_blocks = gp.bucket_size * gp.num_buckets;
  gp.max_chain = 128;
  const vol::AttributeSpec attrs[] = {{"tsdf", sizeof(float)},
                                      {"weight", sizeof(float)}};
  return vol::VoxelBlockGrid::create(device, allocator, gp, attrs, 2);
}

// Decode, growing the player grid when the frame outgrows it -- the recovery
// the Decoder's OutOfMemory names, and the policy that belongs to a player,
// not to the library -- and retrying lock contention, which IoError names.
vr::Status decode_growing(codec::Decoder& dec, const Bytes& frame,
                          vol::VoxelBlockGrid& grid, vr::StageMetrics* rows,
                          int* grows) {
  for (int attempt = 0; attempt < 16; ++attempt) {
    const vr::Status s = dec.decode(frame.data(), frame.size(), grid, rows);
    if (s.ok()) {
      return s;
    }
    if (s.domain() == vr::Status::Code::OutOfMemory) {
      VR_TRY(grid.resize(grid.grid().num_buckets * 2));
      ++*grows;
    } else if (s.domain() != vr::Status::Code::IoError) {
      return s;
    }
  }
  return vr::Status::out_of_memory(
      "codec_replica: the player grid kept "
      "failing to hold the frame");
}

double ms_since(std::chrono::steady_clock::time_point t0) {
  return std::chrono::duration<double, std::milli>(
             std::chrono::steady_clock::now() - t0)
      .count();
}

void print_rows(const char* title, const vr::StageMetrics& rows,
                std::size_t per) {
  std::printf("  %s, per coded frame (host ms / device ms):\n", title);
  for (const vr::StageRow& r : rows.rows()) {
    if (r.has_gpu) {
      std::printf("    %-18s %8.3f  %8.3f\n", r.name, r.cpu_ms / double(per),
                  r.gpu_ms / double(per));
    } else {
      std::printf("    %-18s %8.3f         -\n", r.name,
                  r.cpu_ms / double(per));
    }
  }
}

const vr::StageRow* find_row(const vr::StageMetrics& m, const char* name) {
  for (const vr::StageRow& r : m.rows()) {
    if (std::strcmp(r.name, name) == 0) return &r;
  }
  return nullptr;
}

double row_ms(const vr::StageMetrics& m, const char* name) {
  const vr::StageRow* r = find_row(m, name);
  return r != nullptr ? r->cpu_ms : 0.0;
}

// The comparison's reach: far enough that a decoded surface is never "beyond"
// it for being merely inaccurate (a band's width), near enough that a missing
// piece of surface reads as missing.
float comparison_reach(float trunc) { return std::max(trunc, 0.02f); }

void print_comparison(const vr_example::MeshComparison& c, float voxel) {
  auto line = [voxel](const char* name, const vr_example::DistanceStats& s) {
    std::printf(
        "    %-9s mean %6.3f  rms %6.3f  p95 %6.3f  max %6.3f mm "
        "(%.3f voxels rms), %zu of %zu beyond reach\n",
        name, s.mean * 1e3, s.rms * 1e3, s.p95 * 1e3, s.max * 1e3,
        s.rms / double(voxel), s.beyond_reach, s.count);
  };
  line("accuracy", c.accuracy);
  line("coverage", c.coverage);
}

// --- The sweep.
// ---------------------------------------------------------------

struct SweepConfig {
  const char* family;
  std::uint32_t k;
  float dc;
  float ac;
};

// Four families over the final grid:
//  - "split": the current defaults' DC 5x AC, across K, and across step scale
//    at K = 32;
//  - "uniform": DC = AC -- what an orthonormal transform's squared error says
//    is optimal -- across step at K = 32 up to all 512;
//  - "ref": K = 512 at a fine step, near-lossless. Whatever error it still
//    shows is not the quantizer's, which is what separates the codec's tail
//    from the rest of the pipeline's.
std::vector<SweepConfig> sweep_configs() {
  std::vector<SweepConfig> c;
  for (std::uint32_t k : {8u, 16u, 32u, 64u, 128u}) {
    c.push_back({"split", k, 0.25f, 0.05f});
  }
  for (float s : {0.5f, 2.0f, 4.0f}) {
    c.push_back({"split", 32, 0.25f * s, 0.05f * s});
  }
  for (float step : {0.025f, 0.05f, 0.1f, 0.2f, 0.4f}) {
    c.push_back({"uniform", 32, step, step});
  }
  for (float step : {0.1f, 0.2f, 0.4f}) {
    c.push_back({"uniform", 64, step, step});
  }
  for (float step : {0.2f, 0.4f, 0.8f}) {
    c.push_back({"uniform", 128, step, step});
  }
  for (float step : {0.4f, 0.8f}) {
    c.push_back({"uniform", 256, step, step});
  }
  for (float step : {0.8f, 1.6f}) {
    c.push_back({"uniform", codec::kVoxelsPerBlock, step, step});
  }
  c.push_back({"ref", codec::kVoxelsPerBlock, 0.002f, 0.002f});
  return c;
}

vr::Status run_sweep(vr::Device& device, vr::Allocator& allocator,
                     vol::VoxelBlockGrid& source, const mesh::Mesh& reference,
                     mesh::MarchingCubes& extractor, const Options& opt) {
  VR_ASSIGN(codec::Decoder dec, codec::Decoder::create(device, allocator));
  const float reach = comparison_reach(opt.trunc);
  std::optional<vol::VoxelBlockGrid> player;
  std::printf(
      "\nsweep over the final grid (steps are fractions of trunc_dist; "
      "distances in mm):\n"
      "  %-8s %4s %6s %6s | %8s %7s %6s | %7s %7s %7s | %7s %7s | %7s %7s\n",
      "family", "K", "dc", "ac", "bytes", "B/block", "ratio", "acc rms",
      "acc p95", "acc max", "cov rms", "cov>r", "enc ms", "dec ms");
  for (const SweepConfig& cfg : sweep_configs()) {
    codec::EncoderConfig ec;
    ec.params.coefficient_count = cfg.k;
    ec.params.dc_step = cfg.dc;
    ec.params.ac_step = cfg.ac;
    ec.segment_size = opt.codec.segment_size;
    VR_ASSIGN(codec::Encoder enc,
              codec::Encoder::create(device, allocator, ec));
    // One untimed round first: a fresh encoder's first dispatches pay one-off
    // pipeline and allocation costs that measured anywhere from 5 to 35 ms on
    // the same grid, and would be read as the config's cost. The timed decode
    // is then of a frame the player already holds, a static scene's steady
    // state; the streaming run above is the moving-scene figure.
    VR_ASSIGN(const Bytes warm, enc.encode(source));
    if (!player) {
      VR_ASSIGN(vol::VoxelBlockGrid g,
                player_grid(device, allocator, warm, opt.num_buckets));
      player.emplace(std::move(g));
    }
    int grows = 0;
    VR_TRY(decode_growing(dec, warm, *player, nullptr, &grows));
    vr::StageMetrics enc_rows;
    VR_ASSIGN(const Bytes frame, enc.encode(source, &enc_rows));
    VR_ASSIGN(const codec::FrameInfo info,
              codec::read_frame_info(frame.data(), frame.size()));
    vr::StageMetrics dec_rows;
    VR_TRY(decode_growing(dec, frame, *player, &dec_rows, &grows));
    VR_ASSIGN(const mesh::Mesh decoded, extractor.extract_host(*player));
    const vr_example::MeshComparison c = vr_example::compare_meshes(
        reference, decoded, reach, std::size_t(opt.metric_stride));
    const double per_block =
        info.block_count > 0 ? double(frame.size()) / info.block_count : 0.0;
    std::printf(
        "  %-8s %4u %6.3f %6.3f | %8zu %7.1f %5.0fx | %7.3f %7.3f %7.3f | "
        "%7.3f %7zu | %7.2f %7.2f\n",
        cfg.family, cfg.k, double(cfg.dc), double(cfg.ac), frame.size(),
        per_block, per_block > 0 ? kRawBytesPerBlock / per_block : 0.0,
        c.accuracy.rms * 1e3, c.accuracy.p95 * 1e3, c.accuracy.max * 1e3,
        c.coverage.rms * 1e3, c.coverage.beyond_reach,
        row_ms(enc_rows, "codec encode"), row_ms(dec_rows, "codec decode"));
  }
  return {};
}

// --- The run.
// -----------------------------------------------------------------

vr::Status run(const Options& opt) {
  VR_ASSIGN(vr::Instance instance, vr::Instance::create({}));
  VR_ASSIGN(VkPhysicalDevice gpu, instance.select_physical_device());
  VR_ASSIGN(vr::Device device, vr::Device::create(instance, gpu, {}));
  VR_ASSIGN(vr::Allocator allocator,
            vr::Allocator::create(instance.handle(), device));

  vr_example::ReplicaCapture::Options capture_options;
  capture_options.frame_limit = static_cast<std::size_t>(opt.max_frames);
  capture_options.frame_stride = static_cast<std::size_t>(opt.stride);
  capture_options.min_depth = opt.min_depth;
  capture_options.max_depth = opt.max_depth;
  VR_ASSIGN(vr_example::ReplicaCapture replica,
            vr_example::ReplicaCapture::open(opt.scene_dir, opt.cam_params,
                                             capture_options));
  std::printf("capture: %zu frames; voxel %.3f m, trunc %.3f m\n",
              replica.frame_count(), double(opt.voxel), double(opt.trunc));
  std::printf(
      "codec: K %u, dc_step %.3f, ac_step %.3f, segment %u; coding "
      "every %d frame(s)\n",
      opt.codec.params.coefficient_count, double(opt.codec.params.dc_step),
      double(opt.codec.params.ac_step), opt.codec.segment_size,
      opt.encode_every);

  VR_ASSIGN(vol::VoxelBlockGrid volume,
            vr_example::create_fusion_grid(device, allocator, opt.voxel,
                                           opt.trunc, opt.num_buckets));
  VR_ASSIGN(tsdf::TsdfIntegrator integrator,
            tsdf::TsdfIntegrator::create(device, allocator));
  VR_ASSIGN(mesh::MarchingCubes extractor,
            mesh::MarchingCubes::create(device, allocator));
  VR_ASSIGN(codec::Encoder encoder,
            codec::Encoder::create(device, allocator, opt.codec));
  VR_ASSIGN(codec::Decoder decoder, codec::Decoder::create(device, allocator));
  std::optional<vol::VoxelBlockGrid> player;

  if (opt.preload) {
    VR_ASSIGN(const std::size_t cached, replica.preload());
    std::printf("preloaded %zu frames\n", cached);
  }
  sensor::ICameraCapture& capture = replica;
  VR_TRY(capture.start());

  // Per coded frame: its size and block count, and every stage row summed.
  std::size_t coded = 0;
  std::size_t fused = 0;
  std::size_t last_coded_at = 0;
  double sum_bytes = 0.0;
  double sum_blocks = 0.0;
  std::size_t max_bytes = 0;
  std::uint32_t max_blocks = 0;
  int grows = 0;
  vr::StageMetrics fuse_rows;
  vr::StageMetrics enc_rows;
  vr::StageMetrics dec_rows;
  double wall_codec_ms = 0.0;
  const auto code_frame = [&]() -> vr::Status {
    const auto t0 = std::chrono::steady_clock::now();
    VR_ASSIGN(const Bytes frame, encoder.encode(volume, &enc_rows));
    if (!player) {
      VR_ASSIGN(vol::VoxelBlockGrid g,
                player_grid(device, allocator, frame, opt.num_buckets));
      player.emplace(std::move(g));
    }
    VR_TRY(decode_growing(decoder, frame, *player, &dec_rows, &grows));
    wall_codec_ms += ms_since(t0);
    VR_ASSIGN(const codec::FrameInfo info,
              codec::read_frame_info(frame.data(), frame.size()));
    ++coded;
    last_coded_at = fused;
    sum_bytes += double(frame.size());
    sum_blocks += double(info.block_count);
    max_bytes = std::max(max_bytes, frame.size());
    max_blocks = std::max(max_blocks, info.block_count);
    return {};
  };

  const auto t_start = std::chrono::steady_clock::now();
  for (;;) {
    VR_ASSIGN(const std::optional<sensor::CapturedFrame> polled,
              capture.poll());
    if (!polled) {
      if (capture.exhausted()) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    VR_TRY(vr_example::fuse_frame(volume, integrator, *polled, opt.max_weight,
                                  &fuse_rows));
    ++fused;
    if (opt.encode_every > 0 &&
        fused % static_cast<std::size_t>(opt.encode_every) == 0) {
      VR_TRY(code_frame());
    }
  }
  // The final grid is always coded, so the comparison below is of the last
  // frame a player would hold.
  if (fused > 0 && last_coded_at != fused) {
    VR_TRY(code_frame());
  }
  const double total_s = ms_since(t_start) / 1e3;
  if (coded == 0) {
    return vr::Status::invalid_argument("codec_replica: no frames were fused");
  }

  const double mean_bytes = sum_bytes / double(coded);
  const double mean_blocks = sum_blocks / double(coded);
  const double per_block = sum_blocks > 0 ? sum_bytes / sum_blocks : 0.0;
  std::printf(
      "\nstream: %zu frames fused, %zu coded in %.1f s (codec %.1f "
      "ms per coded frame, wall)%s\n",
      fused, coded, total_s, wall_codec_ms / double(coded),
      grows > 0 ? " -- the player grid grew" : "");
  std::printf(
      "  per coded frame: %.0f bytes mean, %zu max; %.0f blocks mean, "
      "%u max\n",
      mean_bytes, max_bytes, mean_blocks, max_blocks);
  std::printf(
      "  %.1f bytes per block, %.0fx under raw tsdf + weight; %.2f "
      "Mbit/s at %.0f fps\n",
      per_block, per_block > 0 ? kRawBytesPerBlock / per_block : 0.0,
      mean_bytes * 8.0 * kFps / 1e6, kFps);
  print_rows("encode", enc_rows, coded);
  print_rows("decode", dec_rows, coded);
  // The question the next PR hangs on: does the host rANS coder fit a frame
  // interval?
  const double budget = 1e3 / kFps;
  const double rans_enc = row_ms(enc_rows, "  ..rans encode") / double(coded);
  const double rans_dec = row_ms(dec_rows, "  ..rans decode") / double(coded);
  std::printf(
      "  host rANS: %.2f ms encode, %.2f ms decode per frame, "
      "against a %.1f ms frame interval (%.0f%% / %.0f%%)\n",
      rans_enc, rans_dec, budget, 100.0 * rans_enc / budget,
      100.0 * rans_dec / budget);

  // --- The last frame's surface against the source's. ---
  VR_ASSIGN(const mesh::Mesh source_mesh, extractor.extract_host(volume));
  VR_ASSIGN(const mesh::Mesh decoded_mesh, extractor.extract_host(*player));
  const float reach = comparison_reach(opt.trunc);
  const auto t_cmp = std::chrono::steady_clock::now();
  const vr_example::MeshComparison cmp = vr_example::compare_meshes(
      source_mesh, decoded_mesh, reach, std::size_t(opt.metric_stride));
  std::printf(
      "\nlast frame's surface: source %zu triangles, decoded %zu "
      "(x%.4f); reach %.0f mm, every %d-th vertex (%.1f s):\n",
      source_mesh.triangle_count(), decoded_mesh.triangle_count(),
      double(decoded_mesh.triangle_count()) /
          double(std::max<std::size_t>(source_mesh.triangle_count(), 1)),
      double(reach) * 1e3, opt.metric_stride, ms_since(t_cmp) / 1e3);
  print_comparison(cmp, opt.voxel);
  if (!opt.out_prefix.empty()) {
    VR_TRY(vr_example::write_ply(opt.out_prefix + "_source.ply", source_mesh));
    VR_TRY(
        vr_example::write_ply(opt.out_prefix + "_decoded.ply", decoded_mesh));
    std::printf("wrote %s_source.ply and %s_decoded.ply\n",
                opt.out_prefix.c_str(), opt.out_prefix.c_str());
  }

  if (opt.sweep) {
    VR_TRY(run_sweep(device, allocator, volume, source_mesh, extractor, opt));
  }
  return {};
}

}  // namespace

int main(int argc, char** argv) {
  const vr::Result<Options> opt = parse_args(argc, argv);
  if (!opt.ok()) {
    std::fprintf(stderr, "%s\n", opt.status().message().c_str());
    return 2;
  }
  const vr::Status status = run(opt.value());
  if (!status.ok()) {
    std::fprintf(stderr, "codec_replica failed: %s\n",
                 status.message().c_str());
    return 1;
  }
  return 0;
}
