// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// Synthetic count fields test the actual bin-plan shaders against a serial
// uint64 host oracle, without billions of triangle-distance evaluations.
#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <vector>

#include "volumetric_kit/recon/core/allocator.hpp"
#include "volumetric_kit/recon/core/command_batch.hpp"
#include "volumetric_kit/recon/core/compute_util.hpp"
#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/instance.hpp"
#include "volumetric_kit/recon/core/log.hpp"
#include "volumetric_kit/recon/tsdf/mesh_integrator.hpp"

#include "mesh_bin_ranges_comp.spv.hpp"
#include "mesh_compact_bins_comp.spv.hpp"
#include "mesh_scan_add_comp.spv.hpp"
#include "mesh_scan_counts_comp.spv.hpp"
#include "mesh_scan_sums_comp.spv.hpp"

namespace vr = volumetric_kit::recon;
#define EXPECT(condition)                                                      \
  do {                                                                         \
    if (!(condition))                                                          \
      return vr::Status::invalid_argument("line " + std::to_string(__LINE__) + \
                                          ": " #condition);                    \
  } while (0)
namespace {
constexpr std::uint32_t kMaxBin = vr::tsdf::MeshIntegrator::kMaxBinTriangles;
constexpr std::uint32_t kBudget =
    vr::tsdf::MeshIntegrator::kMaxDispatchBinEntries;
constexpr std::uint32_t kWindow = kBudget - kMaxBin + 1;
struct Bin {
  std::int32_t ptr;
  std::uint32_t begin, count;
};
using Summary = std::array<std::uint32_t, 4>;
using Range = std::array<std::uint32_t, 2>;
std::uint32_t groups(std::uint32_t n) { return 1u + (n - 1u) / 256u; }

vr::Status check(vr::Device& device, vr::Allocator& allocator,
                 const std::vector<std::uint32_t>& counts, bool retry) {
  const auto size = static_cast<std::uint32_t>(counts.size());
  std::uint64_t sum = 0;
  std::uint32_t occupied = 0, largest = 0;
  std::vector<Bin> want;
  for (std::uint32_t i = 0; i < size; ++i) {
    if (counts[i]) {
      want.push_back({static_cast<std::int32_t>(i * 8u),
                      static_cast<std::uint32_t>(sum), counts[i]});
      ++occupied;
      largest = std::max(largest, counts[i]);
    }
    sum += counts[i];
  }
  const bool overflow = sum > std::numeric_limits<std::uint32_t>::max();
  const bool rejected = overflow || largest > kMaxBin;
  const auto nr = static_cast<std::uint32_t>(
      1u +
      (std::max<std::uint64_t>(1, std::min<std::uint64_t>(sum, UINT32_MAX)) -
       1u) /
          kWindow);
  vr::ComputeKernel leaf, compact, ranges, add;
  std::array<vr::ComputeKernel, 2> sums;
  vr::KernelSetBuilder kb(device);
  VkPushConstantRange scan_pc{VK_SHADER_STAGE_COMPUTE_BIT, 0, 8};
  VkPushConstantRange compact_pc{VK_SHADER_STAGE_COMPUTE_BIT, 0, 20};
  VkPushConstantRange ranges_pc{VK_SHADER_STAGE_COMPUTE_BIT, 0, 16};
  VR_TRY(kb.add(leaf, "test_mesh_scan", vr_mesh_scan_counts_comp_spv,
                vr_mesh_scan_counts_comp_spv_size, 2, &scan_pc));
  for (auto& kernel : sums) {
    VR_TRY(kb.add(kernel, "test_mesh_sums", vr_mesh_scan_sums_comp_spv,
                  vr_mesh_scan_sums_comp_spv_size, 2, &scan_pc));
  }
  VR_TRY(kb.add(add, "test_mesh_add", vr_mesh_scan_add_comp_spv,
                vr_mesh_scan_add_comp_spv_size, 2, &scan_pc));
  VR_TRY(kb.add(compact, "test_mesh_compact", vr_mesh_compact_bins_comp_spv,
                vr_mesh_compact_bins_comp_spv_size, 4, &compact_pc));
  VR_TRY(kb.add(ranges, "test_mesh_ranges", vr_mesh_bin_ranges_comp_spv,
                vr_mesh_bin_ranges_comp_spv_size, 3, &ranges_pc));
  VR_ASSIGN(auto pool, kb.build());
  VR_ASSIGN(auto counts_buf, vr::device_storage_buffer(allocator, size * 4u));
  VR_ASSIGN(auto blocks_buf,
            vr::device_storage_buffer(allocator, size * sizeof(Bin)));
  VR_ASSIGN(auto ranges_buf,
            vr::device_storage_buffer(allocator, nr * sizeof(Range)));
  std::array<vr::Buffer, 3> levels;
  const std::array<std::uint32_t, 3> ns{groups(size), groups(groups(size)), 1};
  const std::size_t root = ns[0] > 256 ? 2 : 1;
  for (std::size_t i = 0; i <= root; ++i) {
    VR_ASSIGN(levels[i],
              vr::device_storage_buffer(allocator, ns[i] * sizeof(Summary)));
  }
  const auto bind = [](vr::ComputeKernel& kernel, std::uint32_t index,
                       const vr::Buffer& b) {
    kernel.set.write_storage_buffer(index, b.handle(), 0, VK_WHOLE_SIZE);
  };
  bind(leaf, 0, counts_buf);
  bind(leaf, 1, levels[0]);
  for (std::size_t i = 0; i < root; ++i) {
    bind(sums[i], 0, levels[i]);
    bind(sums[i], 1, levels[i + 1]);
  }
  bind(add, 0, levels[0]);
  bind(add, 1, levels[1]);
  bind(compact, 0, counts_buf);
  bind(compact, 1, levels[0]);
  bind(compact, 2, levels[root]);
  bind(compact, 3, blocks_buf);
  bind(ranges, 0, blocks_buf);
  bind(ranges, 1, levels[root]);
  bind(ranges, 2, ranges_buf);
  VkPhysicalDeviceProperties props{};
  vkGetPhysicalDeviceProperties(device.physical_device(), &props);
  const auto max_groups = props.limits.maxComputeWorkGroupCount[0];
  std::vector<Bin> actual(size);
  std::vector<Range> chunks(nr);
  std::vector<std::uint32_t> cursors(size);
  Summary total{};
  const auto plan = [&](vr::CommandBatch& batch,
                        std::uint32_t cap) -> vr::Status {
    const std::uint32_t compact_push[5] = {size, 8, cap, kMaxBin, 0};
    VR_TRY(batch.dispatch(compact, compact_push, sizeof(compact_push),
                          groups(size), max_groups));
    const std::uint32_t ranges_push[4] = {nr, cap, kWindow, kMaxBin};
    VR_TRY(batch.dispatch(ranges, ranges_push, sizeof(ranges_push), groups(nr),
                          max_groups));
    VR_TRY(batch.readback(counts_buf, 0, counts_buf.size(), cursors.data()));
    VR_TRY(batch.readback(blocks_buf, 0, blocks_buf.size(), actual.data()));
    VR_TRY(batch.readback(ranges_buf, 0, ranges_buf.size(), chunks.data()));
    return {};
  };
  {
    vr::CommandBatch batch(device, allocator);
    VR_TRY(batch.upload(counts_buf, 0, counts.data(), counts_buf.size()));
    VR_TRY(batch.fill(blocks_buf, 0, blocks_buf.size(), 0xcccccccc));
    const std::uint32_t p[2] = {size, 0};
    VR_TRY(batch.dispatch(leaf, p, sizeof(p), groups(size), max_groups));
    for (std::size_t i = 0; i < root; ++i) {
      const std::uint32_t sp[2] = {ns[i], 0};
      VR_TRY(
          batch.dispatch(sums[i], sp, sizeof(sp), groups(ns[i]), max_groups));
    }
    if (root == 2) {
      const std::uint32_t sp[2] = {ns[0], 0};
      VR_TRY(batch.dispatch(add, sp, sizeof(sp), groups(ns[0]), max_groups));
    }
    VR_TRY(plan(batch, retry ? 1u : size));
    VR_TRY(batch.readback(levels[root], 0, sizeof(total), total.data()));
    VR_TRY(batch.submit());
  }
  EXPECT(total[0] == static_cast<std::uint32_t>(sum));
  EXPECT(total[1] == occupied);
  EXPECT(total[2] == largest);
  EXPECT((total[3] != 0) == overflow);
  if (rejected || retry) {
    for (const Bin& bin : actual) EXPECT(bin.count == 0xccccccccu);
    for (const Range& r : chunks) EXPECT(r[0] == 0 && r[1] == 0);
    if (rejected) return {};
    vr::CommandBatch batch(device, allocator);
    VR_TRY(plan(batch, size));
    VR_TRY(batch.submit());
  }
  std::uint64_t prefix = 0;
  for (std::uint32_t i = 0; i < size; ++i) {
    EXPECT(cursors[i] == prefix);
    prefix += counts[i];
  }
  for (std::size_t i = 0; i < want.size(); ++i) {
    EXPECT(actual[i].ptr == want[i].ptr);
    EXPECT(actual[i].begin == want[i].begin);
    EXPECT(actual[i].count == want[i].count);
  }
  std::uint32_t next = 0;
  for (const Range& r : chunks) {
    if (!r[1]) continue;
    EXPECT(r[0] == next);
    std::uint64_t work = 0;
    for (std::uint32_t i = r[0]; i < r[0] + r[1]; ++i) {
      EXPECT(i < want.size());
      work += want[i].count;
    }
    EXPECT(work <= kBudget);
    next += r[1];
  }
  EXPECT(next == occupied);
  return {};
}
vr::Status run(vr::Device& device, vr::Allocator& allocator) {
  VR_TRY(check(device, allocator, {0}, false));
  VR_TRY(check(device, allocator, {1}, false));
  std::vector<std::uint32_t> tail(257);
  tail[0] = 1;
  tail[255] = kMaxBin;
  tail[256] = 3;
  VR_TRY(check(device, allocator, tail, true));
  std::vector<std::uint32_t> sparse(65539);
  for (std::size_t i = 0; i < sparse.size(); i += 7) sparse[i] = 15;
  sparse.back() = kMaxBin;
  VR_TRY(check(device, allocator, sparse, true));
  // Exactly 16 maximum-sized bins fill one watchdog budget. Zeros around
  // them test compaction and boundary starts; 35 bins require three ranges.
  std::vector<std::uint32_t> boundaries(71);
  for (std::size_t i = 1; i < boundaries.size(); i += 2)
    boundaries[i] = kMaxBin;
  VR_TRY(check(device, allocator, boundaries, false));
  VR_TRY(check(device, allocator, {kMaxBin + 1}, false));
  // Largest legal sum, then one extra entry: carry must survive the tree.
  std::vector<std::uint32_t> edge(65539, kMaxBin);
  edge[65535] = kMaxBin - 1;
  edge[65536] = edge[65537] = edge[65538] = 0;
  VR_TRY(check(device, allocator, edge, false));
  edge.back() = 1;
  VR_TRY(check(device, allocator, edge, false));
  // The first summary group now wraps to zero; the root adds only one and
  // produces no new carry. It must preserve the child's overflow bit.
  edge[65535] = kMaxBin;
  VR_TRY(check(device, allocator, edge, false));
  VR_TRY(check(device, allocator, {UINT32_MAX, 1}, false));
  return {};
}
}  // namespace
int main() {
  std::atomic<int> errors{0};
  vr::set_log_handler([&](vr::LogLevel level, std::string_view message) {
    if (level == vr::LogLevel::Error) ++errors;
    if (level >= vr::LogLevel::Warning)
      std::fprintf(stderr, "%.*s\n", int(message.size()), message.data());
  });
  vr::InstanceConfig config;
  config.enable_validation = true;
  auto instance = vr::Instance::create(config);
  if (!instance.ok()) {
    std::puts("no Vulkan instance; skipping");
    return 0;
  }
  auto gpu = instance.value().select_physical_device();
  if (!gpu.ok()) {
    std::puts("no Vulkan device; skipping");
    return 0;
  }
  auto device = vr::Device::create(instance.value(), gpu.value(), {});
  if (!device.ok()) return 1;
  auto allocator =
      vr::Allocator::create(instance.value().handle(), device.value());
  if (!allocator.ok()) return 1;
  const auto status = run(device.value(), allocator.value());
  if (!status.ok()) std::fprintf(stderr, "%s\n", status.message().c_str());
  if (!status.ok() || errors != 0) return 1;
  std::puts("tsdf_mesh_bins: OK");
  return 0;
}
