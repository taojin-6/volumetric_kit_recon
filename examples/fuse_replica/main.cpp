// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// fuse_replica: the end-to-end reconstruction example. Polls a posed RGB-D
// sequence in the Replica-SLAM layout (nvblox's fuse_replica dataset) through
// the sensor tier's IRgbdSensor interface, prepares each frame on the GPU
// (sensor::GpuFramePrep) as a camera's, fuses it into a sparse TSDF volume
// (allocate the truncation band, then integrate depth + colour),
// periodically extracts a marching-cubes mesh, and writes the final coloured
// mesh to a binary PLY for inspection. This is the headless spine; the
// live-viewer variant renders the growing mesh each frame through the
// volumetric_kit_gfx sibling.
//
//   fuse_replica <scene_dir> [-o out.ply] [--voxel 0.02] [--max-frames N] ...
//
// <scene_dir> is a Replica scene folder (contains results/ and traj.txt); the
// intrinsics default to <scene_dir>/../cam_params.json.

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
#include <unordered_set>
#include <utility>
#include <vector>

#include "fuse_frame.hpp"
#include "replica_sensor.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/base/stage_metrics.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/core/vulkan/instance.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"
#include "volumetric_kit/recon/io/ply_writer.hpp"
#include "volumetric_kit/recon/mesh/marching_cubes.hpp"
#include "volumetric_kit/recon/mesh/mesh.hpp"
#include "volumetric_kit/recon/sensor/rgbd_frame.hpp"
#include "volumetric_kit/recon/sensor/rgbd_sensor.hpp"
#include "volumetric_kit/recon/sensor/utils/gpu_frame_prep.hpp"
#include "volumetric_kit/recon/tsdf/tsdf_integrator.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"
#include "volumetric_kit/recon/volume/voxel_grid.hpp"
#include "volumetric_kit/recon/volume/voxel_hash_map.hpp"

namespace vr = volumetric_kit::recon;
namespace vkc = volumetric_kit::core;
namespace vol = volumetric_kit::recon::volume;
namespace tsdf = volumetric_kit::recon::tsdf;
namespace mesh = volumetric_kit::recon::mesh;
namespace sensor = volumetric_kit::recon::sensor;

namespace {

// The active blocks stamped changed after `since`, and how many blocks those
// put back to marching cubes: a cell reads corners at `base + {0,1}^3`, so a
// block re-meshes when any of its own `+{0,1}^3` neighbourhood changed -- the
// set a changed-only re-mesh would redo.
struct ChangedBlocks {
  std::uint32_t changed = 0;
  std::uint32_t remesh = 0;
};

vkc::Result<ChangedBlocks> changed_since(
    const vol::VoxelBlockGrid& grid, const std::vector<vol::BlockIndex>& active,
    std::uint32_t since) {
  const auto vpb = static_cast<std::uint32_t>(grid.grid().voxels_per_block);
  if (vpb == 0) {
    return vkc::Status::invalid_argument(
        "changed_since: voxels_per_block is 0");
  }
  VKC_ASSIGN(const std::vector<vol::BlockStamp> stamps,
             grid.map().read_block_stamps());
  // 21 bits an axis, through unsigned casts since coordinates go negative.
  const auto key = [](const vr::Vec3i& c) {
    constexpr std::uint64_t kMask = (std::uint64_t{1} << 21) - 1;
    return ((std::uint64_t{static_cast<std::uint32_t>(c.x)} & kMask) << 42) |
           ((std::uint64_t{static_cast<std::uint32_t>(c.y)} & kMask) << 21) |
           (std::uint64_t{static_cast<std::uint32_t>(c.z)} & kMask);
  };
  std::unordered_set<std::uint64_t> changed;
  for (const vol::BlockIndex& b : active) {
    // A slot past the stamps counts as changed.
    const std::size_t slot = static_cast<std::uint32_t>(b.ptr) / vpb;
    if (slot >= stamps.size() || vol::tick_after(stamps[slot].changed, since)) {
      changed.insert(key(b.coord));
    }
  }
  ChangedBlocks out;
  out.changed = static_cast<std::uint32_t>(changed.size());
  for (const vol::BlockIndex& b : active) {
    bool hit = false;
    for (int d = 0; d < 8 && !hit; ++d) {
      hit = changed.count(
                key(b.coord + vr::Vec3i(d & 1, (d >> 1) & 1, d >> 2))) != 0;
    }
    out.remesh += hit ? 1u : 0u;
  }
  return out;
}

// Command-line options with reconstruction-friendly defaults for Replica.
struct Options {
  std::string scene_dir;
  std::string cam_params;  // default: <scene_dir>/../cam_params.json
  std::string out = "fuse_room0.ply";
  float voxel = 0.02f;     // metres
  float trunc = 0.0f;      // truncation distance (metres); 0 => 4 * voxel
  float min_depth = 0.1f;  // reject depth nearer than this (metres)
  float max_depth = 8.0f;  // reject depth beyond this (metres)
  float max_weight = 20.0f;
  int max_frames = 1 << 30;  // process every available frame by default
  int stride = 1;            // integrate every N-th frame
  int mesh_every = 50;       // re-extract + log this often (0 = only at end)
  // Share a vertex between the cells meeting on an edge, instead of giving
  // every triangle three private ones. This example isolates the extract: it
  // does no projective texturing and no rendering, so what the flag moves here
  // is the meshing cost and the vertex count alone. The viewer examples take
  // the same flag and are where its effect on a textured render is visible --
  // the texture tier stopped refusing a shared mesh when it moved to a
  // per-vertex dispatch.
  bool share_vertices = false;
  // Remesh through extract_device (no host copy) rather than extract. This is
  // the path fuse_viewer and the iOS scanner actually run, and it is the only
  // way to see the extract's real steady-state cost: extract() adds a full
  // readback of every vertex, which measured as 44% of the call at 1 cm and is
  // paid by no seam-B consumer.
  bool device_extract = false;
  // Report the TRUE changed-block fraction every N fused frames; 0 = off.
  // Unlike a frustum survey this counts only blocks the integrator actually
  // changed: those whose `changed` stamp is newer than the last report's tick.
  int dirty_every = 0;
  int num_buckets = 16384;  // initial map size; grows on overflow via resize
  bool preload = false;     // decode every frame up front (RAM for decode time)
};

const char* arg_value(int argc, char** argv, int& i) {
  if (i + 1 >= argc) {
    return nullptr;
  }
  return argv[++i];
}

vkc::Result<Options> parse_args(int argc, char** argv) {
  Options opt;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto need = [&](float& dst) -> bool {
      const char* v = arg_value(argc, argv, i);
      if (v == nullptr) return false;
      dst = std::strtof(v, nullptr);
      return true;
    };
    auto need_int = [&](int& dst) -> bool {
      const char* v = arg_value(argc, argv, i);
      if (v == nullptr) return false;
      dst = std::atoi(v);
      return true;
    };
    if (a == "-o" || a == "--out") {
      const char* v = arg_value(argc, argv, i);
      if (v == nullptr) return vkc::Status::invalid_argument("-o needs a path");
      opt.out = v;
    } else if (a == "--cam-params") {
      const char* v = arg_value(argc, argv, i);
      if (v == nullptr)
        return vkc::Status::invalid_argument("--cam-params path");
      opt.cam_params = v;
    } else if (a == "--device-extract") {
      opt.device_extract = true;
    } else if (a == "--share-vertices") {
      opt.share_vertices = true;
    } else if (a == "--dirty-every") {
      if (!need_int(opt.dirty_every))
        return vkc::Status::invalid_argument("--dirty-every");
    } else if (a == "--voxel") {
      if (!need(opt.voxel)) return vkc::Status::invalid_argument("--voxel");
    } else if (a == "--trunc") {
      if (!need(opt.trunc)) return vkc::Status::invalid_argument("--trunc");
    } else if (a == "--min-depth") {
      if (!need(opt.min_depth))
        return vkc::Status::invalid_argument("--min-depth");
    } else if (a == "--max-depth") {
      if (!need(opt.max_depth))
        return vkc::Status::invalid_argument("--max-depth");
    } else if (a == "--max-weight") {
      if (!need(opt.max_weight))
        return vkc::Status::invalid_argument("--max-weight");
    } else if (a == "--max-frames") {
      if (!need_int(opt.max_frames))
        return vkc::Status::invalid_argument("--max-frames");
    } else if (a == "--stride") {
      if (!need_int(opt.stride))
        return vkc::Status::invalid_argument("--stride");
    } else if (a == "--mesh-every") {
      if (!need_int(opt.mesh_every))
        return vkc::Status::invalid_argument("--mesh-every");
    } else if (a == "--buckets") {
      if (!need_int(opt.num_buckets))
        return vkc::Status::invalid_argument("--buckets");
    } else if (a == "--preload") {
      opt.preload = true;
    } else if (a[0] == '-') {
      return vkc::Status::invalid_argument("unknown flag: " + a);
    } else if (opt.scene_dir.empty()) {
      opt.scene_dir = a;
    } else {
      return vkc::Status::invalid_argument("unexpected argument: " + a);
    }
  }
  if (opt.scene_dir.empty()) {
    return vkc::Status::invalid_argument(
        "usage: fuse_replica <scene_dir> [-o out.ply] [--share-vertices] "
        "[--device-extract] [--dirty-every n] "
        "[--voxel m] "
        "[--max-frames n] [--stride n] [--max-depth m] [--preload]");
  }
  if (opt.cam_params.empty()) {
    opt.cam_params = opt.scene_dir + "/../cam_params.json";
  }
  if (opt.stride < 1) opt.stride = 1;

  // Validate the numeric knobs so a bad value (or garbage that strtof/atoi
  // turns into 0 or a negative) fails loudly here instead of silently producing
  // an empty or degenerate reconstruction downstream.
  //
  // strtof parses "nan"/"inf" without error, and a non-finite knob slips the
  // guards below (NaN compares false to every bound; +inf passes `> 0`) to
  // reach the grid params and the GPU -- a NaN trunc_dist gives an undefined
  // truncation-band width, a NaN min_depth makes the depth gate reject every
  // sample (a silent, empty reconstruction). Reject non-finite up front.
  for (const float knob :
       {opt.voxel, opt.trunc, opt.min_depth, opt.max_depth, opt.max_weight}) {
    if (!std::isfinite(knob)) {
      return vkc::Status::invalid_argument(
          "numeric options (--voxel/--trunc/--min-depth/--max-depth/"
          "--max-weight) must be finite");
    }
  }
  if (!(opt.voxel > 0.0f)) {
    return vkc::Status::invalid_argument("--voxel must be > 0");
  }
  if (opt.trunc <= 0.0f) {
    opt.trunc = 4.0f * opt.voxel;  // default the band to 4 voxels
  }
  if (!(opt.max_depth > 0.0f)) {
    return vkc::Status::invalid_argument("--max-depth must be > 0");
  }
  // Above 0, which the GPU pass reads as "no return".
  if (!(opt.min_depth > 0.0f) || opt.min_depth >= opt.max_depth) {
    return vkc::Status::invalid_argument(
        "--min-depth must be in (0, --max-depth)");
  }
  if (!(opt.max_weight > 0.0f)) {
    return vkc::Status::invalid_argument("--max-weight must be > 0");
  }
  if (opt.max_frames < 1) {
    return vkc::Status::invalid_argument("--max-frames must be >= 1");
  }
  // num_blocks = bucket_size (8) * num_buckets is an int32; keep the product in
  // range so it cannot overflow to a negative that still passes validate().
  constexpr std::int64_t kBucketSize = 8;
  if (opt.num_buckets < 1 ||
      static_cast<std::int64_t>(opt.num_buckets) * kBucketSize >
          std::numeric_limits<std::int32_t>::max()) {
    return vkc::Status::invalid_argument(
        "--buckets must be >= 1 and small enough that 8 * buckets fits int32");
  }
  return opt;
}

vkc::Status run(const Options& opt) {
  // --- Device bring-up (headless: no surface needed) ---
  VKC_ASSIGN(vkc::Instance instance, vkc::Instance::create({}));
  VKC_ASSIGN(vkc::PhysicalDeviceInfo gpu,
             instance.select_physical_device(vr::device_requirements()));
  VKC_ASSIGN(vkc::Device device,
             vkc::Device::create(instance, gpu, vr::device_requirements()));
  VKC_ASSIGN(vkc::Allocator allocator,
             vkc::Allocator::create(instance.handle(), device));

  // --- Capture ---
  // The sequence arrives through the sensor interface: frame selection and
  // the depth gate are the sensor's options, the cameras and pose ride on
  // each frame, and the loop below never learns it is reading a disk. A live
  // source replaces this one construction.
  vr_example::ReplicaSensor::Options capture_options;
  capture_options.frame_limit = static_cast<std::size_t>(opt.max_frames);
  capture_options.frame_stride = static_cast<std::size_t>(opt.stride);
  capture_options.min_depth = opt.min_depth;
  capture_options.max_depth = opt.max_depth;
  VKC_ASSIGN(vr_example::ReplicaSensor replica,
             vr_example::ReplicaSensor::open(opt.scene_dir, opt.cam_params,
                                             capture_options));
  const vr::camera::CameraModel& cam = *replica.info().color;
  std::printf(
      "capture: %zu frames to play, %ux%u @ fx=%.1f fy=%.1f cx=%.1f cy=%.1f\n",
      replica.frame_count(), cam.size.width, cam.size.height, cam.intrinsics.fx,
      cam.intrinsics.fy, cam.intrinsics.cx, cam.intrinsics.cy);

  // --- Volume + pipeline ---
  VKC_ASSIGN(vol::VoxelBlockGrid volume,
             vr_example::create_fusion_grid(device, allocator, opt.voxel,
                                            opt.trunc, opt.num_buckets));
  VKC_ASSIGN(tsdf::TsdfIntegrator integrator,
             tsdf::TsdfIntegrator::create(device, allocator));
  VKC_ASSIGN(sensor::GpuFramePrep prep,
             sensor::GpuFramePrep::create(device, allocator));
  VKC_ASSIGN(mesh::MarchingCubes extractor,
             mesh::MarchingCubes::create(device, allocator, [&] {
               mesh::MarchingCubesConfig c;
               c.share_vertices = opt.share_vertices;
               return c;
             }()));

  // Optionally decode the whole sequence up front. Deliberately *outside* the
  // timed region below: streaming spends ~75% of the loop in JPEG/PNG decode,
  // so preloading is what makes the reported fps a measure of fusion rather
  // than of the reader.
  if (opt.preload) {
    // Announce the cost before spending it: --preload has no frame cap of its
    // own, so a long sequence can quietly ask for many gigabytes.
    std::printf(
        "preloading %.0f MB...\n",
        static_cast<double>(replica.preload_bytes_projected()) / (1024 * 1024));
    const auto preload_start = std::chrono::steady_clock::now();
    VKC_ASSIGN(const std::size_t cached_frames, replica.preload());
    const double preload_seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                      preload_start)
            .count();
    std::printf("preloaded %zu frames (%.0f MB) in %.1fs\n", cached_frames,
                static_cast<double>(replica.preloaded_bytes()) / (1024 * 1024),
                preload_seconds);
  }

  // From here on the source is the interface, not the dataset.
  sensor::IRgbdSensor& capture = replica;
  VKC_TRY(capture.start());

  const auto t_start = std::chrono::steady_clock::now();
  std::size_t fused = 0;
  // Remesh accumulators; see the report below.
  std::size_t remeshes = 0;
  std::uint64_t sum_dispatches = 0;
  double sum_total = 0.0, sum_compact = 0.0, sum_arena = 0.0;
  double sum_dispatch = 0.0, sum_read = 0.0;
  mesh::ExtractTimings last_rt{};
  // Host and device spans per stage, summed across every fused frame -- rows
  // accumulate by name, so the loop below adds straight into this rather than
  // folding a per-frame set into it. The two halves are the point: the host row
  // is wall clock around a fence-blocked submit, the device row is the kernel
  // inside it, and the gap is submit overhead plus the host round trips the
  // stage makes.
  vkc::StageMetrics stage_totals;
  std::size_t dirty_samples = 0;
  std::uint64_t sum_dirty = 0, sum_remesh = 0, sum_active = 0;
  std::uint32_t last_dirty = 0, last_active_blocks = 0, last_remesh = 0;
  // The tick of the last --dirty-every report: a window is the blocks stamped
  // changed after it.
  std::uint32_t dirty_since = volume.map().tick();
  for (;;) {
    // An empty poll is "nothing this tick", which a replay and an idle live
    // device report alike; only the source knows whether that is the end. A
    // decode failure on a frame that is present is a real error and stops the
    // run.
    VKC_ASSIGN(const std::optional<sensor::RgbdFrame> polled, capture.poll());
    if (!polled) {
      if (capture.exhausted()) {
        break;
      }
      // A live sensor polled faster than it runs: yield and ask again.
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    VKC_ASSIGN(const sensor::DeviceFrame frame,
               prep.prepare(*polled, &stage_totals));
    VKC_TRY(vr_example::fuse_set(volume, integrator, {frame}, opt.max_weight,
                                 &stage_totals));
    ++fused;

    if (opt.dirty_every > 0 &&
        (fused % static_cast<std::size_t>(opt.dirty_every)) == 0) {
      // The sample is the UNION of this window's `--dirty-every` frames, which
      // is exactly what a changed-only re-mesh at that cadence would have to
      // redo.
      VKC_ASSIGN(std::vector<vol::BlockIndex> all,
                 volume.map().compact_active_blocks());
      VKC_ASSIGN(const ChangedBlocks sample,
                 changed_since(volume, all, dirty_since));
      dirty_since = volume.map().tick();
      const std::uint32_t dirty = sample.changed;
      const std::uint32_t remesh = sample.remesh;
      if (!all.empty()) {
        // Sums, not a running mean of per-window ratios. The windows are not
        // comparable: the first one builds the map from nothing, so every block
        // in it was allocated AND written and its ratio is ~1 by construction.
        // Averaging ratios gives that window the same vote as a steady-state
        // one; summing weights each by its own active count, which is the
        // quantity being asked about.
        sum_dirty += dirty;
        sum_remesh += remesh;
        sum_active += all.size();
        last_remesh = remesh;
        last_dirty = dirty;
        last_active_blocks = static_cast<std::uint32_t>(all.size());
        ++dirty_samples;
      }
    }

    if (opt.mesh_every > 0 &&
        (fused % static_cast<std::size_t>(opt.mesh_every)) == 0) {
      // Summed across every remesh, not sampled from one: the first extract of
      // a run faults in its arena and is not the steady state anyone lives in,
      // and reporting it as though it were is how a one-off allocation gets
      // mistaken for a per-frame cost.
      mesh::ExtractTimings rt{};
      std::size_t tris = 0;
      if (opt.device_extract) {
        // The DeviceMesh borrows the extractor's buffers and is dropped here --
        // at the default slot_count of 1 the next extract invalidates it, which
        // is exactly what a benchmark wants and what a real consumer must not
        // do.
        VKC_ASSIGN(mesh::DeviceMesh dm,
                   extractor.extract_device(volume, 0.0f, &rt));
        tris = dm.triangle_count;
      } else {
        VKC_ASSIGN(mesh::Mesh preview,
                   extractor.extract_host(volume, 0.0f, &rt));
        tris = preview.triangle_count();
      }
      ++remeshes;
      sum_total += rt.total_ms();
      sum_compact += rt.compact_ms;
      sum_arena += rt.arena_alloc_ms;
      sum_dispatch += rt.dispatch_ms;
      sum_read += rt.readback_ms;
      sum_dispatches += rt.dispatches;
      last_rt = rt;
      if (fused % 100 == 0) {
        std::printf("  fused %zu frames, %zu triangles so far\n", fused, tris);
      }
    }
  }

  if (remeshes > 0) {
    const double n = static_cast<double>(remeshes);
    const double cells = static_cast<double>(last_rt.active_blocks) *
                         static_cast<double>(volume.grid().voxels_per_block);
    std::printf(
        "remesh    %zu extracts, mean %.2f ms  (%s)\n"
        "  phases  compact %.2f  arena %.2f  dispatch %.2f  read %.2f\n"
        "  final   %u blocks -> %.2fM cells, %u tris (%.2f%% of cells), "
        "%.2f dispatches/extract\n",
        remeshes, sum_total / n,
        opt.device_extract ? "extract_device" : "extract + download",
        sum_compact / n, sum_arena / n, sum_dispatch / n, sum_read / n,
        last_rt.active_blocks, cells / 1e6, last_rt.emitted_triangles,
        cells > 0.0 ? 100.0 * last_rt.emitted_triangles / cells : 0.0,
        static_cast<double>(sum_dispatches) / n);
  }

  // Per-stage host vs device, averaged over the fused frames.
  //
  // The gap between the two columns is what a wall-clock stage row could never
  // show -- but read it for what it is, not as one thing. It holds the
  // command-buffer allocate, the submit and the fence wait around EACH
  // dispatch, every host round trip the stage makes (the active-set readback
  // and re-upload most of all), and any *other* dispatch inside the same stage.
  // The last is why `integrate` decomposes: its second kernel reports itself as
  // the indented `..active set` row, so what remains in the gap is genuinely
  // overhead rather than another kernel wearing overhead's clothes. Two things
  // follow. A stage whose device share is small is not a slow kernel and will
  // not be fixed by a faster one -- and a stage still carrying an untimed
  // dispatch has not yet earned that conclusion.
  //
  // The instrument is not free at this scale: one timed submit costs ~0.13 ms
  // more than an untimed one on MoltenVK (the first vkCmdWriteTimestamp in a
  // command buffer, measured -- see DECISIONS.md), so on sub-millisecond stages
  // it moves the host column it is quoted against. It is noise on a real
  // workload and is not on a toy one.
  if (fused > 0 && !stage_totals.empty()) {
    const double n = static_cast<double>(fused);
    std::printf("stages    per fused frame, mean over %zu frames\n", fused);
    for (const vkc::StageRow& row : stage_totals.rows()) {
      if (row.has_gpu) {
        std::printf("  %-9s host %7.3f ms   device %7.3f ms   (%5.1f%%)\n",
                    row.name, row.cpu_ms / n, row.gpu_ms / n,
                    row.cpu_ms > 0.0 ? 100.0 * row.gpu_ms / row.cpu_ms : 0.0);
      } else {
        std::printf("  %-9s host %7.3f ms   device       -\n", row.name,
                    row.cpu_ms / n);
      }
    }
  }

  if (dirty_samples > 0) {
    // Ratios of the summed counts, so each window is weighted by its own active
    // set rather than voting equally (see the accumulation site). "changed" is
    // what the fuse actually moved; "remesh" is that dilated into the -x/-y/-z
    // octant, which is the set a changed-only re-mesh would have to redo.
    //
    // Deliberately NOT reported as a speedup. Only the marching-cubes dispatch
    // scales with the block count; of the phases printed above it, compact
    // walks every table slot regardless, arena alloc is sized by the whole
    // surface, and readback copies all of it. The share below is the ceiling a
    // changed-only re-mesh could aim at, not a factor anything runs faster by.
    const auto pct = [](std::uint64_t num, std::uint64_t den) {
      return den > 0
                 ? 100.0 * static_cast<double>(num) / static_cast<double>(den)
                 : 0.0;
    };
    std::printf(
        "dirty     %zu windows of %d frame(s)\n"
        "  changed %.2f%% of active blocks\n"
        "  remesh  %.2f%% once dilated into -x/-y/-z (the real set)\n"
        "  final   %u changed -> %u to re-mesh of %u active (dilation %.2fx)\n",
        dirty_samples, opt.dirty_every, pct(sum_dirty, sum_active),
        pct(sum_remesh, sum_active), last_dirty, last_remesh,
        last_active_blocks,
        last_dirty > 0 ? static_cast<double>(last_remesh) / last_dirty : 1.0);
  }

  // --- Final mesh -> PLY ---
  //
  // Measured, because the extract's phases are invisible from outside and this
  // example is the scriptable place to see them: whole-volume meshing, the
  // on-device neighbour probe and a possible arena refit all hide inside one
  // call, and only the split says which one a slow extract is. The overlay in
  // fuse_viewer shows the same struct interactively; this prints it so a sweep
  // over --voxel can be diffed.
  mesh::ExtractTimings t{};
  VKC_ASSIGN(mesh::Mesh final_mesh, extractor.extract_host(volume, 0.0f, &t));
  // cells is what the dispatch actually walks: one workgroup per active block,
  // striding over that block's voxels. Printed beside the triangles because the
  // RATIO is the interesting number -- a low emit rate means the kernel is
  // dominated by gathering cells that produce nothing, which points somewhere
  // completely different from a kernel dominated by its output.
  const double cells = static_cast<double>(t.active_blocks) *
                       static_cast<double>(volume.grid().voxels_per_block);
  std::printf(
      "extract   %.1f ms in %u dispatch(es)\n"
      "  phases  compact %.2f  arena %.2f  desc %.2f  "
      "dispatch %.2f  read %.2f\n"
      "  blocks  %u active -> %.2fM cells, %u tris emitted (%.2f%% of cells)\n"
      "  arena   %.1f MB resident, %u tris planned (%.1f%% full)\n",
      t.total_ms(), t.dispatches, t.compact_ms, t.arena_alloc_ms,
      t.descriptor_ms, t.dispatch_ms, t.readback_ms, t.active_blocks,
      cells / 1e6, t.emitted_triangles,
      cells > 0.0 ? 100.0 * t.emitted_triangles / cells : 0.0,
      static_cast<double>(t.arena_bytes) / (1024.0 * 1024.0),
      t.triangle_capacity,
      t.triangle_capacity > 0
          ? 100.0 * static_cast<double>(t.emitted_triangles) /
                t.triangle_capacity
          : 0.0);
  VKC_TRY(vr::io::write_ply(opt.out, final_mesh));
  const auto t_end = std::chrono::steady_clock::now();
  const double secs = std::chrono::duration<double>(t_end - t_start).count();
  std::printf(
      "done: fused %zu frames in %.1fs (%.1f fps), final mesh %zu vertices / "
      "%zu triangles -> %s\n",
      fused, secs, fused / (secs > 0.0 ? secs : 1.0),
      final_mesh.vertices.size(), final_mesh.triangle_count(), opt.out.c_str());
  return {};
}

}  // namespace

int main(int argc, char** argv) {
  vkc::Result<Options> opt = parse_args(argc, argv);
  if (!opt) {
    std::fprintf(stderr, "%s\n", opt.status().message().c_str());
    return 2;
  }
  const vkc::Status status = run(opt.value());
  if (!status.ok()) {
    std::fprintf(stderr, "fuse_replica failed: %s\n", status.message().c_str());
    return 1;
  }
  return 0;
}
