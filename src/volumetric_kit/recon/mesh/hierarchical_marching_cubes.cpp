// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/recon/mesh/hierarchical_marching_cubes.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <new>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include "volumetric_kit/recon/core/command_batch.hpp"
#include "volumetric_kit/recon/core/compute_kernel.hpp"
#include "volumetric_kit/recon/core/compute_util.hpp"
#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/mesh/marching_cubes_tables.hpp"

#include "hierarchical_marching_cubes_comp.spv.hpp"

namespace volumetric_kit::recon::mesh {
namespace {
using Clock = std::chrono::steady_clock;
double elapsed(Clock::time_point start) {
  return std::chrono::duration<double, std::milli>(Clock::now() - start)
      .count();
}
struct Tables {
  std::int32_t triangles[4096];
  std::int32_t corners[24];
  std::int32_t edges[24];
};
struct Push {
  std::uint32_t leaf_count;
  std::uint32_t leaf_base;
  std::uint32_t node_capacity;
  std::uint32_t max_level;
  std::int32_t num_buckets;
  std::int32_t bucket_size;
  std::int32_t max_chain;
  std::uint32_t capacity;
  float voxel_size;
  float iso;
  float weight_threshold;
  std::uint32_t has_color;
};
static_assert(sizeof(Push) == 48, "adaptive mesh push ABI");
static_assert(offsetof(Push, voxel_size) == 32, "adaptive mesh push ABI");
struct Draw {
  VkDrawIndexedIndirectCommand command{0, 1, 0, 0, 0};
  std::uint32_t rejected = 0;
};
static_assert(sizeof(Draw) == 24, "adaptive mesh command ABI");
Status invalid(const char* message) {
  return Status::invalid_argument(std::string("HierarchicalMarchingCubes: ") +
                                  message);
}
}  // namespace

struct HierarchicalMarchingCubes::Impl {
  struct Slot {
    Buffer vertices;
    Buffer indices;
    Buffer indirect;
    std::uint32_t capacity = 0;
    std::uint64_t generation = 0;
    Slot() = default;
    Slot(Slot&& other) noexcept { *this = std::move(other); }
    Slot& operator=(Slot&& other) noexcept {
      if (this != &other) {
        vertices = std::move(other.vertices);
        indices = std::move(other.indices);
        indirect = std::move(other.indirect);
        capacity = std::exchange(other.capacity, 0);
        generation = std::exchange(other.generation, 0);
      }
      return *this;
    }
    Slot(const Slot&) = delete;
    Slot& operator=(const Slot&) = delete;
  };
  Device* device = nullptr;
  Allocator* allocator = nullptr;
  MarchingCubesConfig config;
  DescriptorPool pool;
  ComputeKernel kernel;
  Buffer tables;
  Buffer dummy;
  std::vector<Slot> slots;
  std::uint32_t current = 0;
  std::uint32_t max_groups = 0;
  VkDeviceSize max_range = 0;
  std::uint64_t generation = 0;
  std::uint64_t released = 0;

  Result<Buffer> output(VkDeviceSize bytes, VkBufferUsageFlags usage) {
    VR_TRY(check_storage_buffer_range("hierarchical mesh output", bytes,
                                      max_range));
    return device_storage_buffer(*allocator, bytes, usage,
                                 config.queue_families,
                                 config.queue_family_count);
  }
  std::uint64_t triangle_limit() const noexcept {
    return std::min<std::uint64_t>(
        std::numeric_limits<std::uint32_t>::max() / 3u,
        std::min(max_range / (3 * sizeof(Vertex)),
                 max_range / (3 * sizeof(std::uint32_t))));
  }
  Status grow(Slot& slot, std::uint64_t capacity) {
    capacity = std::max<std::uint64_t>(1, capacity);
    const auto limit = triangle_limit();
    if (capacity > limit) {
      return Status::out_of_memory(
          "hierarchical mesh exceeds device output capacity");
    }
    if (capacity <= slot.capacity) return {};
    // Reserve growth headroom only after the device's actual output outgrows
    // this slot. Clamp the headroom, never the required count, to device
    // limits.
    capacity = std::min(limit, std::max(capacity, std::uint64_t(slot.capacity) +
                                                      slot.capacity / 2u));
    const VkDeviceSize vertices_bytes = capacity * 3 * sizeof(Vertex);
    const VkDeviceSize indices_bytes = capacity * 3 * sizeof(std::uint32_t);
    VR_TRY(check_storage_buffer_range("hierarchical mesh vertices",
                                      vertices_bytes, max_range));
    VR_TRY(check_storage_buffer_range("hierarchical mesh indices",
                                      indices_bytes, max_range));
    VR_ASSIGN(Buffer vertices,
              output(vertices_bytes, config.extra_vertex_usage));
    VR_ASSIGN(Buffer indices, output(indices_bytes, config.extra_index_usage));
    // Unshared extraction always draws the identity index run. Initialize it
    // once per arena growth instead of rewriting it on every extraction.
    // Submit before committing either buffer so a failed upload cannot leave
    // a retained slot advertising an uninitialized index run.
    std::vector<std::uint32_t> identity(static_cast<std::size_t>(capacity) * 3);
    std::iota(identity.begin(), identity.end(), std::uint32_t(0));
    CommandBatch fill(*device, *allocator);
    VR_TRY(fill.upload(indices, 0, identity.data(), indices_bytes));
    VR_TRY(fill.submit());
    slot.vertices = std::move(vertices);
    slot.indices = std::move(indices);
    slot.capacity = static_cast<std::uint32_t>(capacity);
    return {};
  }
};

HierarchicalMarchingCubes::HierarchicalMarchingCubes() noexcept = default;
HierarchicalMarchingCubes::~HierarchicalMarchingCubes() = default;
HierarchicalMarchingCubes::HierarchicalMarchingCubes(
    HierarchicalMarchingCubes&&) noexcept = default;
HierarchicalMarchingCubes& HierarchicalMarchingCubes::operator=(
    HierarchicalMarchingCubes&& other) noexcept {
  if (this != &other) impl_ = std::move(other.impl_);
  return *this;
}
bool HierarchicalMarchingCubes::valid() const noexcept {
  return impl_ != nullptr;
}

Result<HierarchicalMarchingCubes> HierarchicalMarchingCubes::create(
    Device& device, Allocator& allocator,
    const MarchingCubesConfig& config) try {
  if (device.handle() == VK_NULL_HANDLE || !allocator.valid())
    return invalid("empty device or allocator");
  if (config.share_vertices || config.track_block_spans) {
    return Status::unsupported(
        "HierarchicalMarchingCubes: vertex sharing and block spans are not "
        "implemented");
  }
  if (config.slot_count == 0 || config.slot_count > 8 ||
      config.queue_family_count > BufferDesc::kMaxQueueFamilies ||
      ((config.extra_vertex_usage | config.extra_index_usage |
        config.extra_indirect_usage) &
       VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT) != 0)
    return invalid("invalid output configuration");
  HierarchicalMarchingCubes result;
  result.impl_ = std::make_unique<Impl>();
  Impl& p = *result.impl_;
  p.device = &device;
  p.allocator = &allocator;
  p.config = config;
  VkPhysicalDeviceProperties properties{};
  vkGetPhysicalDeviceProperties(device.physical_device(), &properties);
  p.max_groups = properties.limits.maxComputeWorkGroupCount[0];
  p.max_range = properties.limits.maxStorageBufferRange;
  VkPushConstantRange range{VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(Push)};
  KernelSetBuilder builder(device);
  VR_TRY(builder.add(p.kernel, "hierarchical_marching_cubes",
                     vr_hierarchical_marching_cubes_comp_spv,
                     vr_hierarchical_marching_cubes_comp_spv_size, 10, &range));
  VR_ASSIGN(p.pool, builder.build());
  VR_ASSIGN(p.tables, device_storage_buffer(allocator, sizeof(Tables)));
  VR_ASSIGN(p.dummy, device_storage_buffer(allocator, sizeof(std::uint32_t)));
  Tables tables;
  std::memcpy(tables.triangles, kTriTable, sizeof(kTriTable));
  std::memcpy(tables.corners, kCornerOffset, sizeof(kCornerOffset));
  std::memcpy(tables.edges, kEdgeToVert, sizeof(kEdgeToVert));
  CommandBatch batch(device, allocator);
  VR_TRY(batch.upload(p.tables, 0, &tables, sizeof(tables)));
  VR_TRY(batch.fill(p.dummy, 0, sizeof(std::uint32_t), 0));
  VR_TRY(batch.submit());
  p.slots.resize(config.slot_count);
  for (auto& slot : p.slots) {
    VR_ASSIGN(slot.indirect,
              p.output(sizeof(Draw), config.extra_indirect_usage |
                                         VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT));
  }
  return result;
} catch (const std::bad_alloc&) {
  return Status::out_of_memory(
      "HierarchicalMarchingCubes::create: host allocation failed");
}

Result<DeviceMesh> HierarchicalMarchingCubes::extract_device(
    const volume::HierarchicalFieldView& field, float iso,
    ExtractTimings* timings) try {
  if (timings) *timings = {};
  if (!valid()) return invalid("empty extractor");
  VR_TRY(field.validate());
  if (!std::isfinite(iso)) return invalid("non-finite iso-value");
  Impl& p = *impl_;
  const Buffer* inputs[] = {
      field.root_hash, field.nodes,  field.leaf_indices,
      field.tsdf,      field.weight, field.color ? field.color : &p.dummy};
  for (const Buffer* b : inputs) {
    VR_TRY(check_storage_buffer_range("hierarchical field input", b->size(),
                                      p.max_range));
  }
  // Bound the *unclamped* counting atomic before dispatch, even if the output
  // arena is tiny. Each block has at most 729 candidates and 5 triangles each.
  if (std::uint64_t(field.leaf_count) * 729 * 5 * 3 >
      std::numeric_limits<std::uint32_t>::max())
    return invalid("leaf count can overflow draw counter");
  std::uint32_t next = p.current;
  if (p.slots.size() > 1) {
    bool found = false;
    for (std::uint32_t n = 1; n <= p.slots.size(); ++n) {
      const auto candidate = (p.current + n) % std::uint32_t(p.slots.size());
      if (p.slots[candidate].generation <= p.released) {
        next = candidate;
        found = true;
        break;
      }
    }
    if (!found) return invalid("all output slots remain outstanding");
  }
  if (p.generation == std::numeric_limits<std::uint64_t>::max())
    return invalid("generation exhausted");
  p.current = next;
  ++p.generation;
  Impl::Slot& slot = p.slots[next];
  slot.generation = 0;  // A failed extract publishes no borrowed slot.
  if (timings) timings->active_blocks = field.leaf_count;
  Draw draw;
  for (unsigned attempt = 0; attempt < 2; ++attempt) {
    auto start = Clock::now();
    std::uint64_t plan = slot.capacity;
    if (attempt != 0) {
      plan = draw.command.indexCount / 3u;
    } else if (plan == 0) {
      plan = std::min(p.triangle_limit(), std::uint64_t(field.leaf_count) * 64);
    }
    VR_TRY(p.grow(slot, plan));
    if (timings) timings->arena_alloc_ms += elapsed(start);
    start = Clock::now();
    p.kernel.set.write_storage_buffer(0, p.tables.handle(), 0, p.tables.size());
    for (std::uint32_t i = 0; i < 6; ++i) {
      p.kernel.set.write_storage_buffer(i + 1, inputs[i]->handle(), 0,
                                        inputs[i]->size());
    }
    p.kernel.set.write_storage_buffer(7, slot.vertices.handle(), 0,
                                      slot.vertices.size());
    p.kernel.set.write_storage_buffer(9, slot.indirect.handle(), 0,
                                      slot.indirect.size());
    if (timings) timings->descriptor_ms += elapsed(start);
    Push push{field.leaf_count,
              0,
              field.node_capacity,
              field.max_level,
              field.root_grid.num_buckets,
              field.root_grid.bucket_size,
              field.root_grid.max_chain,
              slot.capacity,
              field.finest_voxel_size,
              iso,
              volume::kObservedWeight,
              field.color ? 1u : 0u};
    draw = {};
    start = Clock::now();
    CommandBatch batch(*p.device, *p.allocator);
    VR_TRY(batch.upload(slot.indirect, 0, &draw, sizeof(draw)));
    for (std::uint32_t base = 0; base < field.leaf_count;) {
      push.leaf_base = base;
      const auto count = std::min(p.max_groups, field.leaf_count - base);
      VR_TRY(
          batch.dispatch(p.kernel, &push, sizeof(push), count, p.max_groups));
      base += count;
    }
    VR_TRY(batch.readback(slot.indirect, 0, sizeof(draw), &draw));
    VR_TRY(batch.submit());
    if (timings) {
      timings->dispatch_ms += elapsed(start);
      ++timings->dispatches;
    }
    if (draw.rejected != 0)
      return invalid("invalid node or leaf list on device");
    if (draw.command.indexCount / 3u <= slot.capacity) break;
    if (attempt != 0)
      return Status::out_of_memory(
          "hierarchical mesh changed during extraction");
  }
  slot.generation = p.generation;
  DeviceMesh result;
  result.vertices = slot.vertices.handle();
  result.indices = slot.indices.handle();
  result.indirect = slot.indirect.handle();
  result.vertex_count = draw.command.indexCount;
  result.triangle_count = draw.command.indexCount / 3;
  result.vertex_usage = slot.vertices.usage();
  result.index_usage = slot.indices.usage();
  result.indirect_usage = slot.indirect.usage();
  result.sharing_mode = slot.vertices.sharing_mode();
  result.generation = p.generation;
  result.live_generation = &p.generation;
  if (timings) {
    timings->triangle_capacity = slot.capacity;
    timings->vertex_capacity = slot.capacity * 3;
    timings->emitted_triangles = result.triangle_count;
    timings->emitted_vertices = result.vertex_count;
    for (const auto& s : p.slots)
      timings->arena_bytes +=
          s.vertices.size() + s.indices.size() + s.indirect.size();
  }
  return result;
} catch (const std::bad_alloc&) {
  return Status::out_of_memory(
      "HierarchicalMarchingCubes::extract_device: host allocation failed");
}

Result<Mesh> HierarchicalMarchingCubes::download(const DeviceMesh& view) const
    try {
  if (!valid() || view.live_generation != &impl_->generation ||
      !view.is_current() || !view.valid()) {
    return invalid("download requires this extractor's current view");
  }
  const auto& slot = impl_->slots[impl_->current];
  if (view.vertices != slot.vertices.handle() ||
      view.indices != slot.indices.handle() ||
      view.indirect != slot.indirect.handle() ||
      view.vertex_count != view.triangle_count * std::uint64_t(3) ||
      view.triangle_count > slot.capacity)
    return invalid("invalid mesh view");
  Mesh mesh;
  mesh.vertices.resize(view.vertex_count);
  mesh.indices.resize(view.vertex_count);
  if (!mesh.vertices.empty()) {
    CommandBatch batch(*impl_->device, *impl_->allocator);
    VR_TRY(batch.readback(slot.vertices, 0,
                          VkDeviceSize(view.vertex_count) * sizeof(Vertex),
                          mesh.vertices.data()));
    VR_TRY(batch.submit());
    std::iota(mesh.indices.begin(), mesh.indices.end(), std::uint32_t(0));
  }
  return mesh;
} catch (const std::bad_alloc&) {
  return Status::out_of_memory(
      "HierarchicalMarchingCubes::download: host allocation failed");
}

Result<Mesh> HierarchicalMarchingCubes::extract_host(
    const volume::HierarchicalFieldView& field, float iso,
    ExtractTimings* timings) {
  VR_ASSIGN(DeviceMesh view, extract_device(field, iso, timings));
  const auto start = Clock::now();
  auto mesh = download(view);
  if (timings) timings->readback_ms += elapsed(start);
  impl_->slots[impl_->current].generation = 0;
  return mesh;
}

void HierarchicalMarchingCubes::release_through(
    std::uint64_t generation) noexcept {
  if (valid() && generation <= impl_->generation)
    impl_->released = std::max(impl_->released, generation);
}

}  // namespace volumetric_kit::recon::mesh
