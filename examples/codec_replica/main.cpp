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
//                 [-o prefix] ...
//
// It takes every Replica example's sequence and fusion flags. Configure with
// -DCMAKE_BUILD_TYPE=Release before quoting any timing.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <thread>

#include "cli.hpp"
#include "codec_flags.hpp"
#include "codec_stream.hpp"
#include "codec_sweep.hpp"
#include "fuse_frame.hpp"
#include "fusion_flags.hpp"
#include "replica_flags.hpp"
#include "replica_sensor.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/core/vulkan/instance.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"
#include "volumetric_kit/recon/eval/mesh_distance.hpp"
#include "volumetric_kit/recon/io/ply_writer.hpp"
#include "volumetric_kit/recon/mesh/marching_cubes.hpp"
#include "volumetric_kit/recon/sensor/utils/gpu_frame_prep.hpp"
#include "volumetric_kit/recon/tsdf/fuser.hpp"

namespace vr = volumetric_kit::recon;
namespace vkc = volumetric_kit::core;
namespace codec = volumetric_kit::recon::codec;
namespace eval = volumetric_kit::recon::eval;
namespace mesh = volumetric_kit::recon::mesh;

namespace {

constexpr double kFps = 30.0;
constexpr std::int32_t kBuckets = 16384;  // the fusion grid's; it grows
constexpr std::size_t kMetricStride = 4;  // about 1 vertex in 4, both ways

struct Options {
  vr_example::ReplicaFlags replica{1 << 30};
  vr_example::FusionFlags fusion{0.01f};
  vr_example::CodecFlags codec;
  std::string out_prefix;  // empty: write no meshes
  int encode_every = 1;    // 0: code only the final grid
  bool sweep = false;
};

vkc::Result<Options> parse_args(int argc, char** argv) {
  Options o;
  vr_example::Cli cli("codec_replica");
  o.replica.add_to(cli);
  o.fusion.add_to(cli);
  cli.option("--encode-every", "N", o.encode_every, 0);
  o.codec.add_to(cli);
  cli.flag("--sweep", o.sweep).option("-o", "prefix", o.out_prefix);
  VKC_TRY(cli.parse(argc, argv));
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

  VKC_ASSIGN(vr_example::ReplicaSensor capture, opt.replica.open(opt.fusion));
  if (opt.replica.preload) {
    VKC_TRY(vr_example::preload_frames(capture).status());
  }

  const float voxel = opt.fusion.voxel;
  const float trunc = opt.fusion.trunc;
  VKC_ASSIGN(vr::volume::VoxelBlockGrid volume,
             vr_example::create_fusion_grid(device, allocator, voxel, trunc,
                                            kBuckets));
  VKC_ASSIGN(vr::tsdf::Fuser fuser, vr::tsdf::Fuser::create(device, allocator));
  VKC_ASSIGN(vr::sensor::GpuFramePrep prep,
             vr::sensor::GpuFramePrep::create(device, allocator));
  VKC_ASSIGN(mesh::MarchingCubes extractor,
             mesh::MarchingCubes::create(device, allocator));
  const codec::EncoderConfig& config = opt.codec.config;
  VKC_ASSIGN(vr_example::CodecStream stream,
             vr_example::CodecStream::create(device, allocator, config));
  std::printf("%zu frames at %.3f m voxels; K %u, table %s, scale %.3f\n",
              capture.frame_count(), double(voxel),
              config.params.coefficient_count, opt.codec.quant_table.c_str(),
              double(config.params.quantization_scale));

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
    VKC_TRY(vr_example::fuse_set(fuser, volume, {prepared},
                                 opt.fusion.max_weight, nullptr));
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
  compare.fscore_threshold = 0.5f * voxel;
  VKC_ASSIGN(const eval::ReferenceMesh reference,
             eval::ReferenceMesh::create(source, compare));
  VKC_ASSIGN(const eval::MeshComparison cmp, reference.compare(decoded));
  std::printf("surface: %zu triangles decoded against %zu:\n",
              decoded.triangle_count(), source.triangle_count());
  vr_example::print_comparison(cmp, voxel);

  if (opt.sweep) {
    VKC_TRY(vr_example::run_codec_sweep(device, allocator, volume, reference,
                                        extractor, config, stream.player()));
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
