// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// Online, bounded dyadic reconstruction of the same Replica input as
// fuse_replica. Input decoding and final PLY export are outside online timings.
#include <algorithm>
#include <array>
#include <cerrno>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "grid_layout.hpp"
#include "ply_writer.hpp"
#include "replica_capture.hpp"
#include "volumetric_kit/recon/core/allocator.hpp"
#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/instance.hpp"
#include "volumetric_kit/recon/core/stage_metrics.hpp"
#include "volumetric_kit/recon/mesh/hierarchical_marching_cubes.hpp"
#include "volumetric_kit/recon/tsdf/hierarchical_tsdf_integrator.hpp"
#include "volumetric_kit/recon/volume/hierarchical_grid.hpp"

namespace vr = volumetric_kit::recon;
namespace vol = vr::volume;
namespace tsdf = vr::tsdf;
namespace mesh = vr::mesh;
namespace sensor = vr::sensor;

namespace {
using Clock = std::chrono::steady_clock;

double elapsed_ms(Clock::time_point start) {
  return std::chrono::duration<double, std::milli>(Clock::now() - start)
      .count();
}

struct Options {
  std::string scene_dir;
  std::string cam_params;
  std::string out = "fuse_room0_hierarchical.ply";
  std::string timings_csv;
  float voxel = 0.005f;
  float trunc = 0.04f;
  float min_depth = 0.1f;
  float max_depth = 8.0f;
  float max_weight = 20.0f;
  int levels = 3;
  int buckets = 2048;
  int max_nodes = 131072;
  int max_splits = 64;
  int max_merges = 64;
  int merge_stability = 8;
  int refine_every = 1;
  int pixel_stride = 4;
  int max_frames = 1 << 30;
  int stride = 1;
  int mesh_every = 1;
  bool preload = false;
  bool device_extract = false;
  tsdf::HierarchicalRefinementParams refinement{};
};

// Keep the uniform example's column names and order, then append adaptive
// phases. Missing GPU timestamps stay empty; a host span is never a GPU time.
enum TimingColumn : std::size_t {
  kPipelineHost,
  kFuseHost,
  kAllocateHost,
  kAllocateDevice,
  kActiveSetHost,
  kActiveSetDevice,
  kIntegrateHost,
  kIntegrateDevice,
  kResizeHost,
  kResizeDevice,
  kMeshHost,
  kMeshDevice,
  kMeshCompactHost,
  kMeshInputHost,
  kMeshArenaHost,
  kMeshDescriptorHost,
  kMeshDispatchHost,
  kMeshReadbackHost,
  kClassifyHost,
  kClassifyDevice,
  kSplitHost,
  kSplitDevice,
  kMergeHost,
  kMergeDevice,
  kLeavesHost,
  kLeavesDevice,
  kTopologyHost,
  kTopologyDevice,
  kTimingColumnCount,
};
constexpr std::array<const char*, kTimingColumnCount> kTimingNames = {
    "pipeline_host_ms",        "fuse_host_ms",          "allocate_host_ms",
    "allocate_device_ms",      "active_set_host_ms",    "active_set_device_ms",
    "integrate_host_ms",       "integrate_device_ms",   "resize_host_ms",
    "resize_device_ms",        "mesh_host_ms",          "mesh_device_ms",
    "mesh_compact_host_ms",    "mesh_input_host_ms",    "mesh_arena_host_ms",
    "mesh_descriptor_host_ms", "mesh_dispatch_host_ms", "mesh_readback_host_ms",
    "classify_host_ms",        "classify_device_ms",    "split_host_ms",
    "split_device_ms",         "merge_host_ms",         "merge_device_ms",
    "leaves_host_ms",          "leaves_device_ms",      "topology_host_ms",
    "topology_device_ms"};

struct FrameTiming {
  std::array<std::optional<double>, kTimingColumnCount> values{};
  bool mesh_present = false;
  mesh::ExtractTimings mesh{};
  std::array<std::uint32_t, 4> leaves{};
  vol::HierarchicalSplitStats split{};
  vol::HierarchicalMergeStats merge{};
  std::uint32_t allocate_retries = 0;
};

void record_stages(FrameTiming& frame, const vr::StageMetrics& metrics) {
  struct Mapping {
    const char* name;
    TimingColumn column;
  };
  constexpr Mapping mappings[] = {{"allocate", kAllocateHost},
                                  {"active set", kActiveSetHost},
                                  {"hierarchical integrate", kIntegrateHost},
                                  {"hierarchical classify", kClassifyHost},
                                  {"hierarchy split", kSplitHost},
                                  {"hierarchy merge", kMergeHost},
                                  {"hierarchy leaves", kLeavesHost},
                                  {"hierarchy update", kTopologyHost}};
  for (const auto& row : metrics.rows()) {
    for (const auto& mapping : mappings) {
      if (std::strcmp(row.name, mapping.name) != 0) continue;
      frame.values[mapping.column] = row.cpu_ms;
      if (row.has_gpu) frame.values[mapping.column + 1] = row.gpu_ms;
    }
  }
}

vr::Status write_timings(std::ofstream& out,
                         const std::vector<FrameTiming>& frames) {
  out << "frame";
  for (const char* name : kTimingNames) out << ',' << name;
  out << ",mesh_present,mesh_dispatches,mesh_retry,mesh_incremental,"
         "mesh_active_blocks,mesh_remeshed_blocks,mesh_triangles,mesh_arena_"
         "bytes,"
         "leaf_level_0,leaf_level_1,leaf_level_2,leaf_level_3,"
         "splits,split_deferred,split_exhausted,split_rejected,"
         "merges,merge_deferred,merge_pending,allocate_retries\n";
  out << std::fixed << std::setprecision(6);
  for (std::size_t i = 0; i < frames.size(); ++i) {
    const auto& f = frames[i];
    out << i + 1;
    for (const auto& value : f.values) {
      out << ',';
      if (value) out << *value;
    }
    const auto& m = f.mesh;
    out << ',' << f.mesh_present << ',' << m.dispatches << ','
        << (m.dispatches > 1) << ',' << m.incremental << ',' << m.active_blocks
        << ',' << m.active_blocks << ',' << m.emitted_triangles << ','
        << m.arena_bytes;
    for (auto count : f.leaves) out << ',' << count;
    out << ',' << f.split.split << ',' << f.split.deferred << ','
        << f.split.exhausted << ',' << f.split.rejected << ',' << f.merge.merged
        << ',' << f.merge.deferred << ',' << f.merge.pending << ','
        << f.allocate_retries << '\n';
  }
  out.close();
  if (!out) return vr::Status::io_error("failed to write --timings-csv file");
  std::printf(
      "online percentiles: all frames including startup/retries, nearest rank "
      "(ms)\n");
  for (std::size_t column = 0; column < kTimingNames.size(); ++column) {
    std::vector<double> samples;
    samples.reserve(frames.size());
    for (const auto& frame : frames)
      if (frame.values[column]) samples.push_back(*frame.values[column]);
    if (samples.empty()) continue;
    std::sort(samples.begin(), samples.end());
    const auto percentile = [&](double q) {
      return samples[static_cast<std::size_t>(std::ceil(q * samples.size())) -
                     1];
    };
    std::printf("  %-24s n=%zu p50 %.6f  p95 %.6f  p99 %.6f\n",
                kTimingNames[column], samples.size(), percentile(0.50),
                percentile(0.95), percentile(0.99));
  }
  std::printf(
      "  mesh_device_ms unavailable: extractor reports host phases only\n");
  return {};
}

vr::Result<Options> parse_args(int argc, char** argv) {
  Options opt;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto text = [&](std::string& dest) {
      if (i + 1 == argc || argv[i + 1][0] == '\0' || argv[i + 1][0] == '-')
        return false;
      dest = argv[++i];
      return true;
    };
    auto integer = [&](int& dest) {
      if (i + 1 == argc) return false;
      const char* value = argv[++i];
      const char* end = value + std::strlen(value);
      const auto result = std::from_chars(value, end, dest);
      return result.ec == std::errc{} && result.ptr == end;
    };
    auto real = [&](float& dest) {
      if (i + 1 == argc) return false;
      const char* value = argv[++i];
      char* end = nullptr;
      errno = 0;
      dest = std::strtof(value, &end);
      return value != end && *end == '\0' && errno != ERANGE &&
             std::isfinite(dest);
    };
    bool good = true;
    if (arg == "--out" || arg == "-o")
      good = text(opt.out);
    else if (arg == "--cam-params")
      good = text(opt.cam_params);
    else if (arg == "--timings-csv")
      good = text(opt.timings_csv);
    else if (arg == "--voxel")
      good = real(opt.voxel);
    else if (arg == "--trunc")
      good = real(opt.trunc);
    else if (arg == "--min-depth")
      good = real(opt.min_depth);
    else if (arg == "--max-depth")
      good = real(opt.max_depth);
    else if (arg == "--max-weight")
      good = real(opt.max_weight);
    else if (arg == "--surface-error")
      good = real(opt.refinement.surface_error);
    else if (arg == "--noise-floor")
      good = real(opt.refinement.noise_floor);
    else if (arg == "--depth-jump")
      good = real(opt.refinement.depth_discontinuity);
    else if (arg == "--levels")
      good = integer(opt.levels);
    else if (arg == "--buckets")
      good = integer(opt.buckets);
    else if (arg == "--max-nodes")
      good = integer(opt.max_nodes);
    else if (arg == "--max-splits")
      good = integer(opt.max_splits);
    else if (arg == "--max-merges")
      good = integer(opt.max_merges);
    else if (arg == "--merge-stability")
      good = integer(opt.merge_stability);
    else if (arg == "--refine-every")
      good = integer(opt.refine_every);
    else if (arg == "--pixel-stride")
      good = integer(opt.pixel_stride);
    else if (arg == "--max-frames")
      good = integer(opt.max_frames);
    else if (arg == "--stride")
      good = integer(opt.stride);
    else if (arg == "--mesh-every")
      good = integer(opt.mesh_every);
    else if (arg == "--preload")
      opt.preload = true;
    else if (arg == "--device-extract")
      opt.device_extract = true;
    else if (arg == "--support-coarsening")
      opt.refinement.support_coarsening = true;
    else if (arg.empty() || arg[0] == '-')
      return vr::Status::invalid_argument("unknown flag: " + arg);
    else if (opt.scene_dir.empty())
      opt.scene_dir = arg;
    else
      return vr::Status::invalid_argument("unexpected argument: " + arg);
    if (!good)
      return vr::Status::invalid_argument("invalid or missing value for " +
                                          arg);
  }
  if (opt.scene_dir.empty())
    return vr::Status::invalid_argument(
        "usage: fuse_replica_hierarchical <scene_dir> [--out path] "
        "[--voxel m] [--levels 1..4] [--trunc m] [--buckets n] [--max-nodes n] "
        "[--max-splits n] [--max-merges n] [--merge-stability n] "
        "[--refine-every n] [--surface-error m] [--noise-floor m] "
        "[--depth-jump m] "
        "[--pixel-stride n] [--support-coarsening] "
        "[--max-frames n] [--stride n] "
        "[--mesh-every n] [--preload] [--device-extract] [--timings-csv path]");
  if (!(opt.voxel > 0 && opt.trunc > 0 && opt.max_weight > 0) ||
      opt.min_depth < 0 || !(opt.max_depth > opt.min_depth) ||
      !(opt.refinement.surface_error > 0) || opt.refinement.noise_floor < 0 ||
      !(opt.refinement.depth_discontinuity > 0))
    return vr::Status::invalid_argument(
        "invalid resolution, truncation, weight, depth range or refinement "
        "threshold");
  if (opt.levels < 1 || opt.levels > 4 || opt.max_frames < 1 ||
      opt.stride < 1 || opt.mesh_every < 0 || opt.max_splits < 0 ||
      opt.max_merges < 0 || opt.merge_stability < 1 || opt.refine_every < 1 ||
      opt.pixel_stride < 1 || opt.pixel_stride > 64)
    return vr::Status::invalid_argument(
        "invalid level count, cadence or refinement budget");
  if (opt.buckets < 1 || opt.max_nodes < 1 || opt.max_nodes % 8 != 0 ||
      static_cast<std::int64_t>(opt.buckets) * 8 > opt.max_nodes ||
      opt.max_nodes > std::numeric_limits<std::int32_t>::max() / 512)
    return vr::Status::invalid_argument(
        "--max-nodes must be a multiple of 8, at least 8 * --buckets, and fit "
        "sample addressing");
  if (!opt.timings_csv.empty()) {
    std::error_code csv_error, out_error, same_error;
    const auto csv =
        std::filesystem::weakly_canonical(opt.timings_csv, csv_error);
    const auto ply = std::filesystem::weakly_canonical(opt.out, out_error);
    if ((!csv_error && !out_error && csv == ply) ||
        std::filesystem::equivalent(opt.timings_csv, opt.out, same_error))
      return vr::Status::invalid_argument(
          "--timings-csv and --out must differ");
  }
  if (opt.cam_params.empty())
    opt.cam_params = opt.scene_dir + "/../cam_params.json";
  opt.refinement.pixel_stride = static_cast<std::uint32_t>(opt.pixel_stride);
  return opt;
}

vr::Status run(const Options& opt) {
  std::ofstream csv;
  const bool record = !opt.timings_csv.empty();
  if (record) {
    csv.imbue(std::locale::classic());
    csv.open(opt.timings_csv);
    if (!csv)
      return vr::Status::io_error("cannot open --timings-csv: " +
                                  opt.timings_csv);
  }
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
  VR_ASSIGN(vr_example::ReplicaCapture capture,
            vr_example::ReplicaCapture::open(opt.scene_dir, opt.cam_params,
                                             capture_options));
  vol::HierarchicalGridConfig config;
  config.finest =
      vr_example::example_grid_params(opt.voxel, opt.trunc, opt.buckets);
  config.level_count = static_cast<std::uint32_t>(opt.levels);
  config.child_block_capacity =
      static_cast<std::uint32_t>(opt.max_nodes - opt.buckets * 8);
  config.color = true;
  VR_ASSIGN(vol::HierarchicalGrid grid,
            vol::HierarchicalGrid::create(device, allocator, config));
  VR_ASSIGN(tsdf::HierarchicalTsdfIntegrator integrator,
            tsdf::HierarchicalTsdfIntegrator::create(
                device, allocator, opt.refinement.depth_discontinuity));
  VR_ASSIGN(mesh::HierarchicalMarchingCubes extractor,
            mesh::HierarchicalMarchingCubes::create(device, allocator));
  std::printf("capture: %zu frames, %ux%u, depth scale %.1f\n",
              capture.frame_count(), capture.color_camera().width,
              capture.color_camera().height, capture.depth_scale());
  std::printf(
      "hierarchy: %d levels, finest %.6fm, coarsest %.6fm, truncation %.6fm\n"
      "  budgets: %d root slots, %d total nodes, %d splits / %d merges per "
      "update, merge stability %d\n"
      "  policy: refine every %d frames, error %.6fm, noise %.6fm, pixel "
      "stride %d, depth jump %.6fm, conservative coarsening support %s\n",
      opt.levels, opt.voxel, std::ldexp(opt.voxel, opt.levels - 1), opt.trunc,
      opt.buckets * 8, opt.max_nodes, opt.max_splits, opt.max_merges,
      opt.merge_stability, opt.refine_every, opt.refinement.surface_error,
      opt.refinement.noise_floor, opt.pixel_stride,
      opt.refinement.depth_discontinuity,
      opt.refinement.support_coarsening ? "on" : "off");
  if (opt.preload) {
    std::printf(
        "preloading %.0f MiB...\n",
        static_cast<double>(capture.preload_bytes_projected()) / (1024 * 1024));
    VR_ASSIGN(const std::size_t cached, capture.preload());
    std::printf("preloaded %zu frames\n", cached);
  }
  VR_TRY(capture.start());
  vr::StageMetrics totals, stages;
  std::vector<FrameTiming> timings;
  if (record) timings.reserve(capture.frame_count());
  std::size_t fused = 0, extracts = 0;
  double online_ms = 0, fuse_ms = 0, mesh_ms = 0;
  std::uint64_t splits = 0, merges = 0, deferred = 0, exhausted = 0;
  for (;;) {
    VR_ASSIGN(const std::optional<sensor::CapturedFrame> polled,
              capture.poll());
    if (!polled) {
      if (capture.exhausted()) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    const auto& frame = *polled;
    FrameTiming timing;
    stages.clear();
    const auto fuse_start = Clock::now();
    const tsdf::ColorFrame color{frame.color, frame.color_camera,
                                 frame.color_encoding};
    const tsdf::FrameInput input{
        {vr::StorageInput(frame.depth), frame.depth_camera},
        frame.has_color() ? &color : nullptr};
    const std::vector<tsdf::FrameInput> inputs{input};
    const std::vector<vol::DepthInput> depths{{input.depth, input.camera}};
    // Fixed storage: contention is retried; capacity exhaustion is an explicit
    // failure, never an unreported hole in the reconstructed room.
    for (int attempt = 0; attempt < 5; ++attempt) {
      vol::AllocFailures failures;
      VR_ASSIGN(const std::uint32_t failed,
                grid.allocate_from_depth(depths, &failures, &stages));
      if (failed == 0) break;
      if (failures.capacity_limited())
        return vr::Status::out_of_memory(
            "root allocation exhausted: increase --buckets and --max-nodes");
      if (attempt == 4)
        return vr::Status::out_of_memory(
            "root allocation still contended after five attempts");
      ++timing.allocate_retries;
    }
    VR_ASSIGN(vol::HierarchicalFieldView field, grid.prepare_leaves(&stages));
    if (opt.levels > 1 &&
        fused % static_cast<std::size_t>(opt.refine_every) == 0 &&
        (opt.max_splits > 0 || opt.max_merges > 0)) {
      VR_ASSIGN(const vr::Buffer* desired,
                integrator.classify(field, inputs, opt.refinement, &stages));
      VR_ASSIGN(
          const vol::HierarchicalTopologyStats topology,
          grid.update_topology(
              *desired, static_cast<std::uint32_t>(opt.max_splits),
              static_cast<std::uint32_t>(opt.max_merges),
              static_cast<std::uint32_t>(opt.merge_stability), 1.0f, &stages));
      timing.split = topology.split;
      timing.merge = topology.merge;
      VR_ASSIGN(field, grid.prepare_leaves(&stages));
    }
    VR_TRY(integrator.integrate(field, inputs, opt.max_weight,
                                tsdf::IntegrationMode::Classic, &stages));
    timing.values[kFuseHost] = elapsed_ms(fuse_start);
    timing.values[kPipelineHost] = timing.values[kFuseHost];
    fuse_ms += *timing.values[kFuseHost];
    ++fused;
    // This is metadata from the existing count readback, not a voxel survey.
    timing.leaves = grid.leaf_counts();
    if (opt.mesh_every > 0 &&
        fused % static_cast<std::size_t>(opt.mesh_every) == 0) {
      const auto mesh_start = Clock::now();
      if (opt.device_extract) {
        VR_ASSIGN(mesh::DeviceMesh preview,
                  extractor.extract_device(field, 0.0f, &timing.mesh));
        (void)preview;  // default single-slot output is retired before next
                        // extract
      } else {
        VR_ASSIGN(mesh::Mesh preview,
                  extractor.extract_host(field, 0.0f, &timing.mesh));
        (void)preview;
      }
      timing.values[kMeshHost] = elapsed_ms(mesh_start);
      *timing.values[kPipelineHost] += *timing.values[kMeshHost];
      mesh_ms += *timing.values[kMeshHost];
      timing.mesh_present = true;
      const auto& m = timing.mesh;
      timing.values[kMeshCompactHost] = m.compact_ms;
      timing.values[kMeshInputHost] = m.input_upload_ms;
      timing.values[kMeshArenaHost] = m.arena_alloc_ms;
      timing.values[kMeshDescriptorHost] = m.descriptor_ms;
      timing.values[kMeshDispatchHost] = m.dispatch_ms;
      timing.values[kMeshReadbackHost] = m.readback_ms;
      ++extracts;
    }
    online_ms += *timing.values[kPipelineHost];
    record_stages(timing, stages);
    totals.merge(stages);
    splits += timing.split.split;
    merges += timing.merge.merged;
    deferred += timing.split.deferred;
    exhausted += timing.split.exhausted;
    if (record) timings.push_back(timing);
    if (fused % 100 == 0)
      std::printf(
          "  fused %zu frames, %u leaves, %u triangles, splits %llu / merges "
          "%llu\n",
          fused, field.leaf_count, timing.mesh.emitted_triangles,
          static_cast<unsigned long long>(splits),
          static_cast<unsigned long long>(merges));
  }
  if (fused == 0)
    return vr::Status::invalid_argument("capture contains no frames");
  const double n = static_cast<double>(fused);
  std::printf(
      "online: %zu frames, %zu extracts, mean %.6f ms (fusion %.6f + scheduled "
      "mesh %.6f)\n"
      "  excludes input decode/preload, logging, CSV writes and final "
      "mesh/export\n",
      fused, extracts, online_ms / n, fuse_ms / n, mesh_ms / n);
  for (const auto& row : totals.rows()) {
    std::printf("  %-24s host %.6f ms", row.name, row.cpu_ms / n);
    if (row.has_gpu) std::printf(" device %.6f ms", row.gpu_ms / n);
    std::printf("\n");
  }
  std::printf(
      "refinement: %llu splits, %llu merges, %llu split deferrals, %llu "
      "exhausted requests (counts sum frames)\n",
      static_cast<unsigned long long>(splits),
      static_cast<unsigned long long>(merges),
      static_cast<unsigned long long>(deferred),
      static_cast<unsigned long long>(exhausted));
  VR_ASSIGN(const vol::HierarchicalFieldView field, grid.prepare_leaves());
  const auto leaves = grid.leaf_counts();
  for (int level = 0; level < opt.levels; ++level)
    std::printf("  level %d: %.6fm, %u leaves\n", level,
                std::ldexp(opt.voxel, level),
                leaves[static_cast<std::size_t>(level)]);
  // Report the explicit field allocation separately from the mesh arena and
  // image/preload storage. Includes inactive fixed-capacity payload slots.
  const auto field_bytes = field.root_hash->size() + field.nodes->size() +
                           field.leaf_indices->size() + field.tsdf->size() +
                           field.weight->size() +
                           (field.color ? field.color->size() : 0);
  std::printf(
      "field buffers: %llu bytes for %u node slots (excludes scratch/root-map "
      "metadata and mesh arena)\n",
      static_cast<unsigned long long>(field_bytes), grid.node_capacity());
  VR_ASSIGN(mesh::Mesh final_mesh, extractor.extract_host(field));
  VR_TRY(vr_example::write_ply(opt.out, final_mesh));
  std::printf("final mesh: %zu vertices / %zu triangles -> %s\n",
              final_mesh.vertices.size(), final_mesh.triangle_count(),
              opt.out.c_str());
  if (record) VR_TRY(write_timings(csv, timings));
  return {};
}
}  // namespace

int main(int argc, char** argv) {
  const auto options = parse_args(argc, argv);
  if (!options) {
    std::fprintf(stderr, "%s\n", options.status().message().c_str());
    return 2;
  }
  const auto status = run(options.value());
  if (!status.ok()) {
    std::fprintf(stderr, "fuse_replica_hierarchical failed: %s\n",
                 status.message().c_str());
    return 1;
  }
  return 0;
}
