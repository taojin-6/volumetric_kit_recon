// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// codec_replica: the TSDF codec on real data. Fuses a Replica sequence as
// fuse_replica does and, every codec.encode_every frames, encodes the grid and
// decodes it into a player grid. Reports what the stream costs, and how far
// the last decoded surface is from the source's; codec.sweep then prints the
// rate-distortion table the codec's defaults come from.
//
//   codec_replica --config config/examples/codec_replica.json
//   codec_replica --help
//
// It takes every Replica example's sequence and fusion settings. Configure with
// -DCMAKE_BUILD_TYPE=Release before quoting any timing.

#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "codec_config.hpp"
#include "codec_stream.hpp"
#include "codec_sweep.hpp"
#include "config.hpp"
#include "fuse_frame.hpp"
#include "replica_config.hpp"
#include "replica_sensor.hpp"
#include "set_codec.hpp"
#include "set_log.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/command_batch.hpp"
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

constexpr std::int32_t kBuckets = 16384;  // the fusion grid's; it grows

using Options = vr_example::config::Config;

vkc::Status run(const Options& opt) {
  // First, so a step the codec refuses costs no GPU work.
  VKC_ASSIGN(const codec::EncoderConfig config,
             vr_example::encoder_config(opt.codec));
  VKC_ASSIGN(vkc::Instance instance, vkc::Instance::create({}));
  VKC_ASSIGN(vkc::PhysicalDeviceInfo gpu,
             instance.select_physical_device(vr::device_requirements()));
  VKC_ASSIGN(vkc::Device device,
             vkc::Device::create(instance, gpu, vr::device_requirements()));
  VKC_ASSIGN(vkc::Allocator allocator,
             vkc::Allocator::create(instance.handle(), device));

  VKC_ASSIGN(vr_example::ReplicaSensor capture, vr_example::open_replica(opt));
  if (opt.source.preload) {
    VKC_TRY(vr_example::preload_frames(capture).status());
  }

  const float voxel = opt.fusion.voxel_size_m;
  const float trunc = opt.fusion.truncation_m;
  VKC_ASSIGN(vr::volume::VoxelBlockGrid volume,
             vr_example::create_fusion_grid(device, allocator, voxel, trunc,
                                            kBuckets));
  VKC_ASSIGN(vr::tsdf::Fuser fuser, vr::tsdf::Fuser::create(device, allocator));
  VKC_ASSIGN(vr::sensor::GpuFramePrep prep,
             vr::sensor::GpuFramePrep::create(device, allocator));
  VKC_ASSIGN(mesh::MarchingCubes extractor,
             mesh::MarchingCubes::create(device, allocator));
  VKC_ASSIGN(
      vr_example::CodecStream stream,
      vr_example::CodecStream::create(
          device, allocator, config, vr_example::codec_outputs(opt.output),
          "codec_replica", {vr_example::kNominalFps, 1}));
  std::printf(
      "%zu frames at %.3f m voxels, %s fusion; %s\n", capture.frame_count(),
      double(voxel), opt.fusion.dynamic ? "Dynamic" : "Classic",
      vr_example::codec_settings(config, opt.codec.quantization_table).c_str());

  // Fuse, and code the grid every codec.encode_every frames and at the end,
  // each frame a sender's set (vr_example::SetSender, vr_example::SetLog).
  vr_example::SetSender::Config sending =
      vr_example::sender_config(opt, "codec_replica", opt.codec.encode_every);
  sending.free_after = 0;  // fuse_replica's fusion frees nothing
  VKC_ASSIGN(
      vr_example::SetSender sender,
      vr_example::SetSender::create(device, fuser, volume, &stream, sending));
  vr_example::SetLog log([&device] { return device.counters(); });
  VKC_TRY(capture.start());
  std::size_t fused = 0;
  std::size_t coded_at = 0;
  for (;;) {
    const auto t_poll = std::chrono::steady_clock::now();
    VKC_ASSIGN(auto frame, capture.poll());
    if (!frame) {
      if (capture.exhausted()) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    vr_example::SetLog::Set& s = log.begin();
    s.rows.add_cpu("poll", std::chrono::duration<double, std::milli>(
                               std::chrono::steady_clock::now() - t_poll)
                               .count());
    // A dataset's frame arrives when it is polled, on no capture clock.
    s.arrived(vr_example::system_now_ns(), 0);
    vkc::CommandBatch front(device, allocator);
    front.time(sender.timer());
    VKC_ASSIGN(const vr::sensor::DeviceFrame prepared,
               prep.record(front, *frame, &s.rows));
    VKC_TRY(sender.send(
        front, {prepared},
        std::make_shared<vr::sensor::RgbdFrame>(std::move(*frame)), fused,
        vr_example::frame_pts_ns(fused, vr_example::kNominalFps), s));
    ++fused;
    if (sender.code_due(fused - 1)) coded_at = fused;
  }
  if (fused == 0) {
    return vkc::Status::invalid_argument("no frames were fused");
  }
  std::fputs(log.table("per set").c_str(), stdout);
  if (coded_at != fused) {
    VKC_TRY(stream.code(
        volume, fused - 1,
        vr_example::frame_pts_ns(fused - 1, vr_example::kNominalFps)));
  }
  VKC_TRY(stream.finish());
  stream.report(vr_example::kNominalFps, opt.codec.encode_every);

  // The last frame's decoded surface against the source's.
  VKC_ASSIGN(const mesh::Mesh source, extractor.extract_host(volume));
  VKC_ASSIGN(const mesh::Mesh decoded,
             extractor.extract_host(*stream.player()));
  // Written first, so a comparison that refuses a mesh still leaves the
  // meshes to look at.
  if (!opt.output.prefix.empty()) {
    VKC_TRY(vr::io::write_ply(opt.output.prefix + "_source.ply", source));
    VKC_TRY(vr::io::write_ply(opt.output.prefix + "_decoded.ply", decoded));
  }
  // The source is indexed once, for this comparison and every one the sweep
  // makes.
  VKC_ASSIGN(const eval::ReferenceMesh reference,
             eval::ReferenceMesh::create(
                 source, vr_example::codec_compare_options(voxel, trunc)));
  VKC_ASSIGN(const eval::MeshComparison cmp, reference.compare(decoded));
  std::printf("surface: %zu triangles decoded against %zu:\n",
              decoded.triangle_count(), source.triangle_count());
  vr_example::print_comparison(cmp, voxel);

  if (opt.codec.sweep) {
    VKC_TRY(vr_example::run_codec_sweep(device, allocator, volume, reference,
                                        extractor, config, *stream.player()));
  }
  return {};
}

}  // namespace

int run_main(const Options& opt) {
  const vkc::Status status = run(opt);
  if (!status.ok()) {
    std::fprintf(stderr, "codec_replica failed: %s\n",
                 status.message().c_str());
    return 1;
  }
  return 0;
}

int main(int argc, char** argv) {
  return vr_example::configured_main(
      argc, argv, vr_example::config::Application::CodecReplica,
      "codec_replica", run_main);
}
