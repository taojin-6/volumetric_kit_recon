// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// codec_replica: the TSDF codec on real data. Fuses a Replica sequence as
// fuse_replica does and, every --encode-every frames, encodes the grid and
// decodes it into a player grid. Reports what the stream costs, and how far
// the last decoded surface is from the source's; --sweep then prints the
// rate-distortion table the codec's defaults come from.
//
//   codec_replica <scene_dir> [--voxel 0.01] [--encode-every 1] [--k 64]
//                 [--step 0.2] [--quant-table uniform|band|radial]
//                 [--max-frames N] [--preload] [--sweep]
//                 [-o prefix]
//
// Configure with -DCMAKE_BUILD_TYPE=Release before quoting any timing.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <thread>

#include "codec_stream.hpp"
#include "codec_sweep.hpp"
#include "fuse_frame.hpp"
#include "parse_number.hpp"
#include "replica_sensor.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/core/vulkan/instance.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"
#include "volumetric_kit/recon/eval/mesh_distance.hpp"
#include "volumetric_kit/recon/io/ply_writer.hpp"
#include "volumetric_kit/recon/mesh/marching_cubes.hpp"
#include "volumetric_kit/recon/sensor/utils/gpu_frame_prep.hpp"
#include "volumetric_kit/recon/tsdf/tsdf_integrator.hpp"

namespace vr = volumetric_kit::recon;
namespace vkc = volumetric_kit::core;
namespace codec = volumetric_kit::recon::codec;
namespace eval = volumetric_kit::recon::eval;
namespace mesh = volumetric_kit::recon::mesh;

namespace {

constexpr double kFps = 30.0;
constexpr float kMaxWeight = 20.0f;
constexpr std::int32_t kBuckets = 16384;  // the fusion grid's; it grows
constexpr std::size_t kMetricStride = 4;  // about 1 vertex in 4, both ways

struct Options {
  std::string scene_dir;
  std::string out_prefix;  // empty: write no meshes
  float voxel = 0.01f;
  int max_frames = 1 << 30;
  int encode_every = 1;  // 0: code only the final grid
  bool preload = false;
  bool sweep = false;
  std::string quant_table = "uniform";
  codec::EncoderConfig codec;  // --k, --step
};

vkc::Result<Options> parse_args(int argc, char** argv) {
  Options o;
  int k = int(o.codec.params.coefficient_count);
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    const bool takes_value =
        a == "-o" || a == "--voxel" || a == "--encode-every" || a == "--k" ||
        a == "--step" || a == "--max-frames" || a == "--quant-table" ||
        a == "--entropy" || a == "--segment-size";
    if (takes_value && i + 1 >= argc) {
      return vkc::Status::invalid_argument(a + " needs a value");
    }
    const char* v = takes_value ? argv[++i] : nullptr;
    if (a == "-o") {
      o.out_prefix = v;
    } else if (a == "--voxel") {
      VKC_TRY(vr_example::parse_number(a, v, o.voxel));
    } else if (a == "--encode-every") {
      VKC_TRY(vr_example::parse_number(a, v, o.encode_every));
    } else if (a == "--k") {
      VKC_TRY(vr_example::parse_number(a, v, k));
    } else if (a == "--step") {
      VKC_TRY(
          vr_example::parse_number(a, v, o.codec.params.quantization_scale));
    } else if (a == "--quant-table") {
      o.quant_table = v;
    } else if (a == "--max-frames") {
      VKC_TRY(vr_example::parse_number(a, v, o.max_frames));
    } else if (a == "--entropy") {
      VKC_TRY(vr_example::parse_entropy(a, v, o.codec.entropy));
    } else if (a == "--segment-size") {
      int r = 0;
      VKC_TRY(vr_example::parse_number(a, v, r));
      if (r < 1) return vkc::Status::invalid_argument(a + " must be >= 1");
      o.codec.segment_size = std::uint32_t(r);
    } else if (a == "--preload") {
      o.preload = true;
    } else if (a == "--sweep") {
      o.sweep = true;
    } else if (a[0] == '-' || !o.scene_dir.empty()) {
      return vkc::Status::invalid_argument("unexpected argument: " + a);
    } else {
      o.scene_dir = a;
    }
  }
  if (o.scene_dir.empty()) {
    return vkc::Status::invalid_argument(
        "usage: codec_replica <scene_dir> [--voxel m] [--encode-every n] "
        "[--k n] [--step f] [--quant-table uniform|band|radial] "
        "[--max-frames n] [--entropy auto|host|device] [--segment-size n] "
        "[--preload] [--sweep] "
        "[-o prefix]");
  }
  if (!(o.voxel > 0.0f) || o.max_frames < 1 || o.encode_every < 0 || k < 1) {
    return vkc::Status::invalid_argument(
        "--voxel must be > 0, --max-frames >= 1, --encode-every >= 0, "
        "--k >= 1");
  }
  o.codec.params.coefficient_count = std::uint32_t(k);
  VKC_TRY(vr_example::apply_quantization_table(o.codec.params, o.quant_table));
  VKC_TRY(o.codec.params.validate());
  return o;
}

vkc::Status run(const Options& opt) {
  VKC_ASSIGN(vkc::Instance instance, vkc::Instance::create({}));
  VKC_ASSIGN(vkc::PhysicalDeviceInfo gpu,
             instance.select_physical_device(vr::device_requirements()));
  VKC_ASSIGN(vkc::Device device,
             vkc::Device::create(instance, gpu, vr::device_requirements()));
  VKC_ASSIGN(vkc::Allocator allocator,
             vkc::Allocator::create(instance.handle(), device));

  vr_example::ReplicaSensor::Options capture_options;
  capture_options.frame_limit = std::size_t(opt.max_frames);
  VKC_ASSIGN(vr_example::ReplicaSensor capture,
             vr_example::ReplicaSensor::open(
                 opt.scene_dir, opt.scene_dir + "/../cam_params.json",
                 capture_options));
  if (opt.preload) {
    VKC_TRY(capture.preload().status());
  }

  const float trunc = 4.0f * opt.voxel;
  VKC_ASSIGN(vr::volume::VoxelBlockGrid volume,
             vr_example::create_fusion_grid(device, allocator, opt.voxel, trunc,
                                            kBuckets));
  VKC_ASSIGN(vr::tsdf::TsdfIntegrator integrator,
             vr::tsdf::TsdfIntegrator::create(device, allocator));
  VKC_ASSIGN(vr::sensor::GpuFramePrep prep,
             vr::sensor::GpuFramePrep::create(device, allocator));
  VKC_ASSIGN(mesh::MarchingCubes extractor,
             mesh::MarchingCubes::create(device, allocator));
  VKC_ASSIGN(vr_example::CodecStream stream,
             vr_example::CodecStream::create(device, allocator, opt.codec));
  std::printf("%zu frames at %.3f m voxels; K %u, table %s, scale %.3f\n",
              capture.frame_count(), double(opt.voxel),
              opt.codec.params.coefficient_count, opt.quant_table.c_str(),
              double(opt.codec.params.quantization_scale));

  // Fuse, and code the grid every --encode-every frames and at the end.
  VKC_TRY(capture.start());
  std::size_t fused = 0;
  std::size_t coded_at = 0;
  for (;;) {
    VKC_ASSIGN(const auto frame, capture.poll());
    if (!frame) {
      if (capture.exhausted()) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    VKC_ASSIGN(const vr::sensor::DeviceFrame prepared, prep.prepare(*frame));
    VKC_TRY(vr_example::fuse_set(volume, integrator, {prepared}, kMaxWeight,
                                 nullptr));
    ++fused;
    if (opt.encode_every > 0 && fused % std::size_t(opt.encode_every) == 0) {
      VKC_TRY(stream.code(volume));
      coded_at = fused;
    }
  }
  if (fused == 0) {
    return vkc::Status::invalid_argument("no frames were fused");
  }
  if (coded_at != fused) {
    VKC_TRY(stream.code(volume));
  }
  stream.report(kFps, opt.encode_every);

  // The last frame's decoded surface against the source's.
  VKC_ASSIGN(const mesh::Mesh source, extractor.extract_host(volume));
  VKC_ASSIGN(const mesh::Mesh decoded, extractor.extract_host(stream.player()));
  // Written first, so a comparison that refuses a mesh still leaves the
  // meshes to look at.
  if (!opt.out_prefix.empty()) {
    VKC_TRY(vr::io::write_ply(opt.out_prefix + "_source.ply", source));
    VKC_TRY(vr::io::write_ply(opt.out_prefix + "_decoded.ply", decoded));
  }
  // A band's width of reach, and an F-score at half a voxel, inside the
  // reconstruction's own resolution so it moves with the codec. The source is
  // indexed once, for this comparison and every one the sweep makes.
  eval::CompareOptions compare;
  compare.reach = std::max(trunc, 0.02f);
  compare.stride = kMetricStride;
  compare.fscore_threshold = 0.5f * opt.voxel;
  VKC_ASSIGN(const eval::ReferenceMesh reference,
             eval::ReferenceMesh::create(source, compare));
  VKC_ASSIGN(const eval::MeshComparison cmp, reference.compare(decoded));
  std::printf("surface: %zu triangles decoded against %zu:\n",
              decoded.triangle_count(), source.triangle_count());
  vr_example::print_comparison(cmp, opt.voxel);

  if (opt.sweep) {
    VKC_TRY(vr_example::run_codec_sweep(device, allocator, volume, reference,
                                        extractor, opt.codec, stream.player()));
  }
  return {};
}

}  // namespace

int main(int argc, char** argv) {
  const vkc::Result<Options> opt = parse_args(argc, argv);
  if (!opt.ok()) {
    std::fprintf(stderr, "%s\n", opt.status().message().c_str());
    return 2;
  }
  const vkc::Status status = run(opt.value());
  if (!status.ok()) {
    std::fprintf(stderr, "codec_replica failed: %s\n",
                 status.message().c_str());
    return 1;
  }
  return 0;
}
