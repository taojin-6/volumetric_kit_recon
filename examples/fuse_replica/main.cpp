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
  int num_buckets = 16384;   // initial map size; grows on overflow via resize
  bool preload = false;  // decode every frame up front (RAM for decode time)
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
        "usage: fuse_replica <scene_dir> [-o out.ply] [--voxel m] "
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
  VR_ASSIGN(tsdf::TsdfIntegrator integrator,
            tsdf::TsdfIntegrator::create(device, allocator));
  VR_ASSIGN(mesh::MarchingCubes extractor,
            mesh::MarchingCubes::create(device, allocator));

  // Allocate the truncation band for a frame, growing the map (preserving the
  // per-voxel data already fused) if it overflows -- exercises the block-index-
  // preserving resize on real data.
  auto allocate_band =
      [&](const vr_example::RgbdFrame& frame,
          const vr::DepthCameraParams& depth_camera) -> vr::Status {
    for (int attempt = 0; attempt < 5; ++attempt) {
      vol::AllocFailures failures;
      VR_ASSIGN(std::uint32_t failed,
                volume.map().allocate_from_depth(frame.depth.data(),
                                                 depth_camera, &failures));
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
      std::printf(
          "  map overflow (%u fails: %u chain, %u heap) -> resize to %lld "
          "buckets\n",
          failed, failures.chain, failures.heap, static_cast<long long>(grown));
      VR_TRY(volume.resize(static_cast<std::int32_t>(grown)));
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

    VR_TRY(allocate_band(frame, depth_camera));
    VR_TRY(integrator.integrate(volume, frame.depth.data(), depth_camera,
                                opt.max_weight, tsdf::IntegrationMode::Classic,
                                &color_frame));
    ++fused;

    if (opt.mesh_every > 0 &&
        (fused % static_cast<std::size_t>(opt.mesh_every)) == 0) {
      VR_ASSIGN(mesh::Mesh preview, extractor.extract(volume));
      std::printf("  frame %zu: fused %zu, %zu triangles so far\n", i, fused,
                  preview.triangle_count());
    }
  }

  // --- Final mesh -> PLY ---
  VR_ASSIGN(mesh::Mesh final_mesh, extractor.extract(volume));
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
