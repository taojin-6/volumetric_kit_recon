// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// fuse_replica: the end-to-end reconstruction example. Reads a posed RGB-D
// sequence in the Replica-SLAM layout (nvblox's fuse_replica dataset), fuses
// each frame into a sparse TSDF volume (allocate the truncation band, then
// integrate depth + colour), periodically extracts a marching-cubes mesh, and
// writes the final coloured mesh to a binary PLY for inspection. This is the
// headless spine; the live-viewer variant renders the growing mesh each frame
// through the volumetric_kit_gfx sibling.
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
#include <string>
#include <utility>

#include "dataset.hpp"
#include "ply_writer.hpp"
#include "volumetric_kit/recon/core/allocator.hpp"
#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/instance.hpp"
#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/core/stage_metrics.hpp"
#include "volumetric_kit/recon/mesh/marching_cubes.hpp"
#include "volumetric_kit/recon/mesh/mesh.hpp"
#include "volumetric_kit/recon/tsdf/tsdf_integrator.hpp"
#include "volumetric_kit/recon/volume/voxel_block_grid.hpp"
#include "volumetric_kit/recon/volume/voxel_grid.hpp"
#include "volumetric_kit/recon/volume/voxel_hash_map.hpp"

namespace vr = volumetric_kit::recon;
namespace vol = volumetric_kit::recon::volume;
namespace tsdf = volumetric_kit::recon::tsdf;
namespace mesh = volumetric_kit::recon::mesh;

namespace {

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
  // Report the TRUE dirty-block fraction every N fused frames; 0 = off. Unlike
  // a frustum survey this counts only blocks the integrator actually wrote.
  int dirty_every = 0;
  // Re-mesh only the blocks a fuse changed, through
  // MarchingCubes::extract_device_incremental. Implies the integrator's dirty
  // tracking (the flags it reads), the extractor's span table (the ranges it
  // re-meshes against), and --device-extract (the only path it exists on).
  //
  // It also takes OWNERSHIP of the flags: the extract consumes them and resets
  // them immediately after, so the next window accumulates from zero. That is
  // the whole discipline the feature depends on -- the fuse kernel only ORs
  // into the flags, so without a reset every block reads dirty within a few
  // frames and the run re-meshes everything through the incremental path while
  // paying for the table, the dilation and the retirement.
  bool incremental = false;
  int num_buckets = 16384;  // initial map size; grows on overflow via resize
  bool preload = false;     // decode every frame up front (RAM for decode time)
};

const char* arg_value(int argc, char** argv, int& i) {
  if (i + 1 >= argc) {
    return nullptr;
  }
  return argv[++i];
}

vr::Result<Options> parse_args(int argc, char** argv) {
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
      if (v == nullptr) return vr::Status::invalid_argument("-o needs a path");
      opt.out = v;
    } else if (a == "--cam-params") {
      const char* v = arg_value(argc, argv, i);
      if (v == nullptr)
        return vr::Status::invalid_argument("--cam-params path");
      opt.cam_params = v;
    } else if (a == "--device-extract") {
      opt.device_extract = true;
    } else if (a == "--share-vertices") {
      opt.share_vertices = true;
    } else if (a == "--incremental") {
      opt.incremental = true;
    } else if (a == "--dirty-every") {
      if (!need_int(opt.dirty_every))
        return vr::Status::invalid_argument("--dirty-every");
    } else if (a == "--voxel") {
      if (!need(opt.voxel)) return vr::Status::invalid_argument("--voxel");
    } else if (a == "--trunc") {
      if (!need(opt.trunc)) return vr::Status::invalid_argument("--trunc");
    } else if (a == "--min-depth") {
      if (!need(opt.min_depth))
        return vr::Status::invalid_argument("--min-depth");
    } else if (a == "--max-depth") {
      if (!need(opt.max_depth))
        return vr::Status::invalid_argument("--max-depth");
    } else if (a == "--max-weight") {
      if (!need(opt.max_weight))
        return vr::Status::invalid_argument("--max-weight");
    } else if (a == "--max-frames") {
      if (!need_int(opt.max_frames))
        return vr::Status::invalid_argument("--max-frames");
    } else if (a == "--stride") {
      if (!need_int(opt.stride))
        return vr::Status::invalid_argument("--stride");
    } else if (a == "--mesh-every") {
      if (!need_int(opt.mesh_every))
        return vr::Status::invalid_argument("--mesh-every");
    } else if (a == "--buckets") {
      if (!need_int(opt.num_buckets))
        return vr::Status::invalid_argument("--buckets");
    } else if (a == "--preload") {
      opt.preload = true;
    } else if (a[0] == '-') {
      return vr::Status::invalid_argument("unknown flag: " + a);
    } else if (opt.scene_dir.empty()) {
      opt.scene_dir = a;
    } else {
      return vr::Status::invalid_argument("unexpected argument: " + a);
    }
  }
  if (opt.scene_dir.empty()) {
    return vr::Status::invalid_argument(
        "usage: fuse_replica <scene_dir> [-o out.ply] [--share-vertices] "
        "[--device-extract] [--incremental] [--dirty-every n] "
        "[--voxel m] "
        "[--max-frames n] [--stride n] [--max-depth m] [--preload]");
  }
  // --incremental only exists on the device path, so it turns it on rather than
  // being ignored beside it. Ignoring it was worse than it looks: the tracking
  // and the grid-sized span table are switched on by the flag itself, so the
  // run paid ~36 MB and a per-fuse dirty pass and then took the host extract
  // anyway -- and nothing said so.
  if (opt.incremental) opt.device_extract = true;
  // Two owners of one set of flags. --dirty-every reads the accumulated flags
  // and then resets them, which is exactly what the incremental extract does,
  // on a cadence that has nothing to do with --mesh-every: at the defaults it
  // would zero the flags at frames 10/20/30/40 and leave the frame-50 extract
  // seeing only frames 41-50, so every block changed before that reads clean
  // and keeps stale triangles. Refused rather than silently resolved -- either
  // is a legitimate thing to want, and picking one for the caller is how the
  // survey's numbers end up describing a run nobody asked for.
  if (opt.incremental && opt.dirty_every > 0) {
    return vr::Status::invalid_argument(
        "--dirty-every and --incremental both consume the integrator's dirty "
        "flags and reset them; run one or the other");
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
      return vr::Status::invalid_argument(
          "numeric options (--voxel/--trunc/--min-depth/--max-depth/"
          "--max-weight) must be finite");
    }
  }
  if (!(opt.voxel > 0.0f)) {
    return vr::Status::invalid_argument("--voxel must be > 0");
  }
  if (opt.trunc <= 0.0f) {
    opt.trunc = 4.0f * opt.voxel;  // default the band to 4 voxels
  }
  if (!(opt.max_depth > 0.0f)) {
    return vr::Status::invalid_argument("--max-depth must be > 0");
  }
  if (opt.min_depth < 0.0f || opt.min_depth >= opt.max_depth) {
    return vr::Status::invalid_argument(
        "--min-depth must be in [0, --max-depth)");
  }
  if (!(opt.max_weight > 0.0f)) {
    return vr::Status::invalid_argument("--max-weight must be > 0");
  }
  if (opt.max_frames < 1) {
    return vr::Status::invalid_argument("--max-frames must be >= 1");
  }
  // num_blocks = bucket_size (8) * num_buckets is an int32; keep the product in
  // range so it cannot overflow to a negative that still passes validate().
  constexpr std::int64_t kBucketSize = 8;
  if (opt.num_buckets < 1 ||
      static_cast<std::int64_t>(opt.num_buckets) * kBucketSize >
          std::numeric_limits<std::int32_t>::max()) {
    return vr::Status::invalid_argument(
        "--buckets must be >= 1 and small enough that 8 * buckets fits int32");
  }
  return opt;
}

vr::Status run(const Options& opt) {
  // --- Device bring-up (headless: no surface needed) ---
  VR_ASSIGN(vr::Instance instance, vr::Instance::create({}));
  VR_ASSIGN(VkPhysicalDevice gpu, instance.select_physical_device());
  VR_ASSIGN(vr::Device device, vr::Device::create(instance.handle(), gpu, {}));
  VR_ASSIGN(vr::Allocator allocator,
            vr::Allocator::create(instance.handle(), device));

  // --- Dataset ---
  VR_ASSIGN(vr_example::ReplicaDataset dataset,
            vr_example::ReplicaDataset::open(opt.scene_dir, opt.cam_params));
  const vr_example::CameraModel& cam = dataset.camera();
  std::printf(
      "dataset: %zu poses, %ux%u @ fx=%.1f fy=%.1f cx=%.1f cy=%.1f, depth "
      "scale %.1f\n",
      dataset.frame_count(), cam.width, cam.height, cam.fx, cam.fy, cam.cx,
      cam.cy, cam.depth_scale);

  // --- Volume + pipeline ---
  vol::VoxelGridParams grid{};
  grid.voxel_size = opt.voxel;
  grid.block_size = 8;
  grid.voxels_per_block = 512;
  grid.trunc_dist = opt.trunc;
  grid.bucket_size = 8;
  grid.num_buckets = opt.num_buckets;
  grid.num_blocks = grid.bucket_size * grid.num_buckets;
  grid.max_chain = 128;

  const vol::AttributeSpec attrs[] = {{"tsdf", sizeof(float)},
                                      {"weight", sizeof(float)},
                                      {"color", sizeof(std::uint32_t)}};
  VR_ASSIGN(vol::VoxelBlockGrid volume,
            vol::VoxelBlockGrid::create(device, allocator, grid, attrs, 3));
  // Dirty-block tracking is opt-in and only --dirty-every asks for it: with it
  // off the integrator allocates no per-block flag array (num_blocks * 4 bytes,
  // doubling with every map grow) and its kernel stores no flags, so the
  // default run measures the same fusion every other consumer gets.
  VR_ASSIGN(tsdf::TsdfIntegrator integrator,
            tsdf::TsdfIntegrator::create(device, allocator, [&] {
              tsdf::TsdfIntegratorConfig c;
              c.track_dirty_blocks = opt.dirty_every > 0 || opt.incremental;
              return c;
            }()));
  VR_ASSIGN(mesh::MarchingCubes extractor,
            mesh::MarchingCubes::create(device, allocator, [&] {
              mesh::MarchingCubesConfig c;
              c.share_vertices = opt.share_vertices;
              // The span table is what an incremental extract re-meshes
              // against, and it is sized by the grid rather than the surface --
              // so it stays off unless asked for.
              c.track_block_spans = opt.incremental;
              // Measured on the same runs, since the whole reason to run
              // --incremental is to find out what it costs and saves.
              c.track_retriangulation = opt.incremental;
              return c;
            }()));

  // Allocate the truncation band for a frame, growing the map (preserving the
  // per-voxel data already fused) if it overflows -- exercises the block-index-
  // preserving resize on real data.
  auto allocate_band = [&](const vr_example::RgbdFrame& frame,
                           const vr::DepthCameraParams& depth_camera,
                           vr::StageMetrics* metrics) -> vr::Status {
    for (int attempt = 0; attempt < 5; ++attempt) {
      vol::AllocFailures failures;
      VR_ASSIGN(std::uint32_t failed,
                volume.map().allocate_from_depth(
                    frame.depth.data(), depth_camera, &failures, metrics));
      if (failed == 0) {
        return {};
      }
      // Grow only for a *capacity* limit. Depth allocation is the most
      // contended entry point in the map -- adjacent pixels dilate into the
      // same block, and the kernel's bucket spin-lock gives up after a bounded
      // number of retries -- so the retry loop can hand back a residue of pure
      // lock failures over a table that is nowhere near full. Doubling on that
      // is expensive and unbounded: at this example's defaults each attribute
      // array goes 768 MiB -> 1536 MiB, and resize builds the grown buffers
      // beside the old ones, so the transient peak is ~2.3 GiB -- for pressure
      // that does not exist. Report it and retry instead; the next dispatch
      // sees less contention because the blocks that did land are now present.
      if (!failures.capacity_limited()) {
        std::printf(
            "  %u allocations lost bucket-lock races (no capacity limit) -> "
            "retrying without growing\n",
            failed);
        continue;
      }
      // Double in int64 and bail before the block index (bucket_size * buckets)
      // would overflow int32, so a growth that can no longer fit reports
      // cleanly instead of tripping the signed-overflow UB.
      const std::int64_t grown =
          static_cast<std::int64_t>(volume.grid().num_buckets) * 2;
      if (grown * volume.grid().bucket_size >
          std::numeric_limits<std::int32_t>::max()) {
        return vr::Status::out_of_memory(
            "map cannot grow further without overflowing the block index");
      }
      // Report the occupancy alongside the reason: it is a 4-byte read of the
      // heap counter (not the O(total slots) diagnostics scan), and it is what
      // says whether this grow was inevitable or premature. A capture-scale
      // consumer should poll it and grow on a threshold instead of waiting for
      // the failure -- linear probing degrades sharply past ~0.7, so growing at
      // the cliff means every insert before it ran at its slowest.
      vr::Result<float> load = volume.map().load_factor();
      std::printf(
          "  map overflow at %.3f load (%u fails: %u chain, %u heap, %u table) "
          "-> resize to %lld buckets\n",
          load.ok() ? load.value() : -1.0f, failed, failures.chain,
          failures.heap, failures.table, static_cast<long long>(grown));
      // Its own row rather than folded into "allocate" or left untimed: this is
      // by far the most expensive thing an overflowing frame does (the ~2.3 GiB
      // transient above, plus init_table and the rehash passes), and charging
      // it to "allocate" would sink that stage's device share on exactly the
      // frames where the host cost is not the kernel at all. Untimed it would
      // simply vanish -- the frames that cost the most contributing nothing to
      // the table below.
      {
        vr::StageScope resize_span(metrics, "resize");
        VR_TRY(volume.resize(static_cast<std::int32_t>(grown)));
      }
    }
    return vr::Status::out_of_memory(
        "allocation kept overflowing after resize");
  };

  // The camera intrinsics, dimensions, and depth range are identical every
  // frame -- only the pose changes -- so build both param structs once and
  // rewrite just cam_to_world per frame. Depth and colour share Replica's one
  // registered camera; keeping the shared intrinsics in a single place also
  // stops the depth and colour cameras silently drifting apart.
  vr::DepthCameraParams depth_camera{};
  depth_camera.fx = cam.fx;
  depth_camera.fy = cam.fy;
  depth_camera.cx = cam.cx;
  depth_camera.cy = cam.cy;
  depth_camera.min_depth = opt.min_depth;
  depth_camera.max_depth = opt.max_depth;
  depth_camera.width = cam.width;
  depth_camera.height = cam.height;

  vr::ColorCameraParams color_camera{};
  color_camera.fx = cam.fx;
  color_camera.fy = cam.fy;
  color_camera.cx = cam.cx;
  color_camera.cy = cam.cy;
  color_camera.width = cam.width;
  color_camera.height = cam.height;

  const std::size_t last = std::min<std::size_t>(
      dataset.frame_count(), static_cast<std::size_t>(opt.max_frames));

  // Optionally decode the whole sequence up front. Deliberately *outside* the
  // timed region below: streaming spends ~75% of the loop in JPEG/PNG decode,
  // so preloading is what makes the reported fps a measure of fusion rather
  // than of the reader.
  if (opt.preload) {
    const auto stride = static_cast<std::size_t>(opt.stride);
    // Announce the cost before spending it: --preload has no frame cap of its
    // own, so a long sequence can quietly ask for many gigabytes.
    std::printf(
        "preloading %.0f MB...\n",
        static_cast<double>(dataset.preload_bytes_projected(last, stride)) /
            (1024 * 1024));
    const auto preload_start = std::chrono::steady_clock::now();
    VR_ASSIGN(const std::size_t cached_frames, dataset.preload(last, stride));
    const double preload_seconds =
        std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                      preload_start)
            .count();
    std::printf("preloaded %zu frames (%.0f MB) in %.1fs\n", cached_frames,
                static_cast<double>(dataset.preloaded_bytes()) / (1024 * 1024),
                preload_seconds);
  }

  const auto t_start = std::chrono::steady_clock::now();
  std::size_t fused = 0;
  // Remesh accumulators; see the report below.
  std::size_t remeshes = 0;
  std::uint64_t sum_dispatches = 0;
  // Extracts that really were incremental, and what they re-meshed. Separate
  // from `remeshes` because the extractor falls back silently by design, so
  // "asked for" and "got" are different numbers and only the second one
  // explains the timings above.
  std::size_t incremental_extracts = 0;
  std::uint64_t sum_remeshed = 0;
  std::uint64_t sum_incr_active = 0;
  // Of the blocks re-meshed, the ones whose triangulation actually moved. The
  // number a per-triangle cache keyed by arena slot trades against, which the
  // re-mesh share above is only an upper bound on.
  std::uint64_t sum_retriangulated = 0;
  double sum_total = 0.0, sum_compact = 0.0, sum_arena = 0.0;
  double sum_dispatch = 0.0, sum_read = 0.0;
  mesh::ExtractTimings last_rt{};
  // Host and device spans per stage, summed across every fused frame -- rows
  // accumulate by name, so the loop below adds straight into this rather than
  // folding a per-frame set into it. The two halves are the point: the host row
  // is wall clock around a fence-blocked submit, the device row is the kernel
  // inside it, and the gap is submit overhead plus the host round trips the
  // stage makes.
  vr::StageMetrics stage_totals;
  std::size_t dirty_samples = 0;
  std::uint64_t sum_dirty = 0, sum_remesh = 0, sum_active = 0;
  std::uint32_t last_dirty = 0, last_active_blocks = 0, last_remesh = 0;
  for (std::size_t i = 0; i < last; i += static_cast<std::size_t>(opt.stride)) {
    vr::Result<vr_example::FrameView> frame_result = dataset.frame(i);
    if (!frame_result) {
      if (frame_result.status().domain() == vr::Status::Code::NotFound) {
        // Ran past the frames present on disk (we may have only a subset of the
        // trajectory): stop cleanly rather than erroring.
        std::printf("frame %zu not on disk; stopping at %zu fused frames\n", i,
                    fused);
        break;
      }
      // A frame that IS on disk but failed to decode is a real error.
      return frame_result.status();
    }
    const vr_example::FrameView view = std::move(frame_result).value();
    const vr_example::RgbdFrame& frame = *view;

    // Only the pose changes per frame; depth_camera/color_camera were built
    // once above.
    depth_camera.cam_to_world = frame.cam_to_world;
    color_camera.cam_to_world = frame.cam_to_world;
    const tsdf::ColorFrame color_frame{frame.color.data(), color_camera};

    VR_TRY(allocate_band(frame, depth_camera, &stage_totals));
    VR_TRY(integrator.integrate(volume, frame.depth.data(), depth_camera,
                                opt.max_weight, tsdf::IntegrationMode::Classic,
                                &color_frame, &stage_totals));
    ++fused;

    if (opt.dirty_every > 0 &&
        (fused % static_cast<std::size_t>(opt.dirty_every)) == 0) {
      // Read, then reset: the sample is the UNION of this window's
      // `--dirty-every` frames, which is exactly what an incremental extract
      // running at that cadence would have to redo. (An earlier comment here
      // claimed the reset came first and the sample was one frame's writes;
      // the reset is the last statement of the block, so it never was.)
      //
      // The active set is compacted once and handed to dirty_remesh_blocks,
      // which is why that takes a span: it needs the coordinates the flags do
      // not carry, and compacting a second time inside it would be a dispatch,
      // a fence wait and a full read-back per sample.
      VR_ASSIGN(std::vector<vol::BlockIndex> all,
                volume.map().compact_active_blocks());
      const std::uint32_t dirty = integrator.dirty_block_count();
      VR_ASSIGN(const std::vector<vr::Vec3i> remesh_blocks,
                integrator.dirty_remesh_blocks(volume, all.data(), all.size()));
      const auto remesh = static_cast<std::uint32_t>(remesh_blocks.size());
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
      integrator.reset_dirty();
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
        if (opt.incremental) {
          // All three fields off the same integrator in the same breath. A
          // capacity or an epoch cached across a fuse names a buffer this
          // object may already have replaced or a topology it may already have
          // left.
          const mesh::DirtyBlocks dirty{integrator.dirty_flags_buffer(),
                                        integrator.dirty_flags_capacity(),
                                        integrator.dirty_epoch()};
          VR_ASSIGN(mesh::DeviceMesh dm, extractor.extract_device_incremental(
                                             volume, 0.0f, dirty, &rt));
          tris = dm.triangle_count;
          // Consumed, so cleared -- and cleared here, immediately after the
          // extract that read them, rather than on a cadence of its own. The
          // fuse kernel only ORs into the flags, so anything else makes the
          // window they describe drift out of step with the window between
          // extracts: too long and every block reads dirty (a full re-mesh
          // wearing the incremental path's costs), too short and blocks that
          // really changed read clean and keep triangles the fuse invalidated.
          //
          // Reset even when the extract fell back to a full pass: a full pass
          // re-meshes everything, so the flags it did not read are just as
          // spent as the ones it did.
          integrator.reset_dirty();
        } else {
          VR_ASSIGN(mesh::DeviceMesh dm,
                    extractor.extract_device(volume, 0.0f, &rt));
          tris = dm.triangle_count;
        }
      } else {
        VR_ASSIGN(mesh::Mesh preview, extractor.extract(volume, 0.0f, &rt));
        tris = preview.triangle_count();
      }
      ++remeshes;
      sum_total += rt.total_ms();
      sum_compact += rt.compact_ms;
      sum_arena += rt.arena_alloc_ms;
      sum_dispatch += rt.dispatch_ms;
      sum_read += rt.readback_ms;
      sum_dispatches += rt.dispatches;
      // Counted, not assumed. Every clause the extractor decides on is
      // invisible from here, and a run that silently fell back to full
      // extracts would otherwise be reported as measuring the feature -- which
      // is the only way a benchmark of it can lie.
      if (rt.incremental) {
        ++incremental_extracts;
        sum_remeshed += rt.remeshed_blocks;
        sum_incr_active += rt.active_blocks;
        sum_retriangulated += rt.retriangulated_blocks;
      }
      last_rt = rt;
      if (fused % 100 == 0) {
        std::printf("  frame %zu: fused %zu, %zu triangles so far\n", i, fused,
                    tris);
      }
    }
  }

  if (remeshes > 0) {
    const double n = static_cast<double>(remeshes);
    const double cells = static_cast<double>(last_rt.active_blocks) *
                         static_cast<double>(grid.voxels_per_block);
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
    if (opt.incremental) {
      // Both halves, because either alone reads as success. "0 of 40
      // incremental" is a run that measured the fallback; "40 of 40, 98%
      // re-meshed" is a run that measured the feature doing all the work
      // anyway, which is what an unreset flag array produces.
      std::printf(
          "  incr    %zu of %zu extracts incremental, mean %.1f%% of blocks "
          "re-meshed\n",
          incremental_extracts, remeshes,
          sum_incr_active > 0 ? 100.0 * static_cast<double>(sum_remeshed) /
                                    static_cast<double>(sum_incr_active)
                              : 0.0);
      // And how much of THAT re-mesh actually changed shape. A block re-meshed
      // because a neighbour's TSDF drifted, whose own cells all kept their
      // marching-cubes case, re-emits the same triangles into the same arena
      // slots -- so the share below, not the one above, is what a slot-keyed
      // per-triangle cache loses per pass.
      std::printf(
          "  retri   %.1f%% of re-meshed blocks changed triangulation "
          "(%llu of %llu)\n",
          sum_remeshed > 0 ? 100.0 * static_cast<double>(sum_retriangulated) /
                                 static_cast<double>(sum_remeshed)
                           : 0.0,
          static_cast<unsigned long long>(sum_retriangulated),
          static_cast<unsigned long long>(sum_remeshed));
    }
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
    for (const vr::StageRow& row : stage_totals.rows()) {
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
    // octant, which is the set an incremental extract would have to redo.
    //
    // Deliberately NOT reported as a speedup. Only the marching-cubes dispatch
    // scales with the block count; of the phases printed above it, compact
    // walks every table slot regardless, arena alloc is sized by the whole
    // surface, and readback copies all of it. The share below is the ceiling an
    // incremental extract could aim at, not a factor anything runs faster by.
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
  VR_ASSIGN(mesh::Mesh final_mesh, extractor.extract(volume, 0.0f, &t));
  // cells is what the dispatch actually walks: one workgroup per active block,
  // striding over that block's voxels. Printed beside the triangles because the
  // RATIO is the interesting number -- a low emit rate means the kernel is
  // dominated by gathering cells that produce nothing, which points somewhere
  // completely different from a kernel dominated by its output.
  const double cells = static_cast<double>(t.active_blocks) *
                       static_cast<double>(grid.voxels_per_block);
  std::printf(
      "extract   %.1f ms in %u dispatch(es)\n"
      "  phases  compact %.2f  inputs %.2f  arena %.2f  desc %.2f  "
      "dispatch %.2f  read %.2f\n"
      "  blocks  %u active -> %.2fM cells, %u tris emitted (%.2f%% of cells)\n"
      "  arena   %.1f MB resident, %u tris planned (%.1f%% full)\n",
      t.total_ms(), t.dispatches, t.compact_ms, t.input_upload_ms,
      t.arena_alloc_ms, t.descriptor_ms, t.dispatch_ms, t.readback_ms,
      t.active_blocks, cells / 1e6, t.emitted_triangles,
      cells > 0.0 ? 100.0 * t.emitted_triangles / cells : 0.0,
      static_cast<double>(t.arena_bytes) / (1024.0 * 1024.0),
      t.triangle_capacity,
      t.triangle_capacity > 0
          ? 100.0 * static_cast<double>(t.emitted_triangles) /
                t.triangle_capacity
          : 0.0);
  VR_TRY(vr_example::write_ply(opt.out, final_mesh));
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
  vr::Result<Options> opt = parse_args(argc, argv);
  if (!opt) {
    std::fprintf(stderr, "%s\n", opt.status().message().c_str());
    return 2;
  }
  const vr::Status status = run(opt.value());
  if (!status.ok()) {
    std::fprintf(stderr, "fuse_replica failed: %s\n", status.message().c_str());
    return 1;
  }
  return 0;
}
