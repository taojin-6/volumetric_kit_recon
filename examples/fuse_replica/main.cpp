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
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
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
  float trunc = 0.08f;     // truncation distance (metres); default 4 * voxel
  float max_depth = 8.0f;  // reject depth beyond this (metres)
  float max_weight = 20.0f;
  int max_frames = 1 << 30;  // process every available frame by default
  int stride = 1;            // integrate every N-th frame
  int mesh_every = 50;       // re-extract + log this often (0 = only at end)
  int num_buckets = 16384;   // initial map size; grows on overflow via resize
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
        "[--max-frames n] [--stride n] [--max-depth m]");
  }
  if (opt.cam_params.empty()) {
    opt.cam_params = opt.scene_dir + "/../cam_params.json";
  }
  if (opt.stride < 1) opt.stride = 1;
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
  auto allocate_band = [&](const vr_example::RgbdFrame& frame,
                           const vol::DepthCameraParams& dcam) -> vr::Status {
    for (int attempt = 0; attempt < 5; ++attempt) {
      VR_ASSIGN(std::uint32_t failed,
                volume.map().allocate_from_depth(frame.depth.data(), dcam));
      if (failed == 0) {
        return {};
      }
      const std::int32_t grown = volume.grid().num_buckets * 2;
      std::printf("  map overflow (%u fails) -> resize to %d buckets\n", failed,
                  grown);
      VR_TRY(volume.resize(grown));
    }
    return vr::Status::out_of_memory(
        "allocation kept overflowing after resize");
  };

  const auto t_start = std::chrono::steady_clock::now();
  std::size_t fused = 0;
  const std::size_t last = std::min<std::size_t>(
      dataset.frame_count(), static_cast<std::size_t>(opt.max_frames));
  for (std::size_t i = 0; i < last; i += static_cast<std::size_t>(opt.stride)) {
    vr::Result<vr_example::RgbdFrame> frame_result = dataset.load(i);
    if (!frame_result) {
      // Ran past the frames present on disk (we may have only a subset of the
      // trajectory): stop cleanly rather than erroring.
      std::printf("frame %zu not on disk; stopping at %zu fused frames\n", i,
                  fused);
      break;
    }
    const vr_example::RgbdFrame frame = std::move(frame_result).value();

    vol::DepthCameraParams dcam{};
    dcam.fx = cam.fx;
    dcam.fy = cam.fy;
    dcam.cx = cam.cx;
    dcam.cy = cam.cy;
    dcam.min_depth = cam.min_depth;
    dcam.max_depth = opt.max_depth;
    dcam.width = cam.width;
    dcam.height = cam.height;
    dcam.cam_to_world = frame.cam_to_world;

    tsdf::ColorCameraParams ccam{};
    ccam.fx = cam.fx;
    ccam.fy = cam.fy;
    ccam.cx = cam.cx;
    ccam.cy = cam.cy;
    ccam.width = cam.width;
    ccam.height = cam.height;
    ccam.cam_to_world = frame.cam_to_world;
    const tsdf::ColorFrame color_frame{frame.color.data(), ccam};

    VR_TRY(allocate_band(frame, dcam));
    VR_TRY(integrator.integrate(volume, frame.depth.data(), dcam,
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
