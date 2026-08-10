// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// Move-semantics tests for the compute core's RAII handle owners. Per the
// CLAUDE.md RAII rules, every move-only owner must: move-construct (leaving the
// source empty), move-assign over a live object (freeing the old resource), and
// survive a self-move. Under the sanitizer CI job these become real
// leak/double-free detectors -- a forgotten reset or a missing self-move guard
// shows up as an ASan report here.
//
// The owners are exercised with real handles, one per mechanism:
//   - Buffer            -- the hand-written `std::function` deleter
//   (VMA-backed).
//   - DescriptorSetLayout -- the UniqueHandle member, whose defaulted moves
//   back
//                            ShaderModule / DescriptorPool / ComputePipeline
//                            too.
//   - Allocator         -- the pImpl + unique_ptr owner.
//   - ComputeKernel     -- the layout/pipeline/set bundle, whose hand-written
//                          moves reset the copyable non-owning set.
//   - GpuTimer          -- a UniqueHandle member PLUS scalar state the
//                          defaulted moves would copy and leave live in the
//                          source.
//
// All need a device, so the whole test skips (exit 0) where no driver/device
// is present, like the other Vulkan tests.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <utility>

#include "volumetric_kit/recon/core/allocator.hpp"
#include "volumetric_kit/recon/core/buffer.hpp"
#include "volumetric_kit/recon/core/compute_kernel.hpp"
#include "volumetric_kit/recon/core/descriptor.hpp"
#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/gpu_timer.hpp"
#include "volumetric_kit/recon/core/instance.hpp"
#include "volumetric_kit/recon/core/vulkan.hpp"

namespace vr = volumetric_kit::recon;

#define CHECK(cond)                                                          \
  do {                                                                       \
    if (!(cond)) {                                                           \
      std::fprintf(stderr, "FAIL (%s:%d): %s\n", __FILE__, __LINE__, #cond); \
      return 1;                                                              \
    }                                                                        \
  } while (0)

namespace {

// Buffer: the std::function-deleter owner. move-construct, move-assign over a
// live buffer, and self-move.
int test_buffer_moves(vr::Allocator& allocator) {
  vr::BufferDesc desc;
  desc.size = 256;
  desc.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;

  vr::Result<vr::Buffer> a = allocator.create_buffer(desc);
  CHECK(a.ok());
  vr::Buffer buf_a = std::move(a).value();
  CHECK(buf_a.valid());
  const VkBuffer raw = buf_a.handle();

  // move-construct: source emptied, destination adopts the handle + size.
  vr::Buffer buf_b(std::move(buf_a));
  CHECK(!buf_a.valid());
  CHECK(buf_a.handle() == VK_NULL_HANDLE);
  CHECK(buf_a.size() == 0);
  CHECK(buf_b.valid());
  CHECK(buf_b.handle() == raw);
  CHECK(buf_b.size() == 256);

  // move-assign over a live buffer: the destination's original allocation is
  // freed (ASan would flag a leak otherwise), then it adopts the source.
  vr::Result<vr::Buffer> c = allocator.create_buffer(desc);
  CHECK(c.ok());
  vr::Buffer buf_c = std::move(c).value();
  buf_c = std::move(buf_b);
  CHECK(buf_c.valid());
  CHECK(buf_c.handle() == raw);
  CHECK(!buf_b.valid());

  // self-move: the `if (this != &other)` guard makes it a no-op. Launder
  // through a pointer so the compiler can't see the self-assignment
  // (-Wself-move).
  vr::Buffer* alias = &buf_c;
  buf_c = std::move(*alias);
  CHECK(buf_c.valid());
  CHECK(buf_c.handle() == raw);
  return 0;
}

// DescriptorSetLayout: the UniqueHandle-backed owner (defaulted moves). An
// empty (zero-binding) layout is a valid handle and needs no shader.
int test_layout_moves(VkDevice device) {
  vr::Result<vr::DescriptorSetLayout> a =
      vr::DescriptorSetLayout::create(device, nullptr, 0);
  CHECK(a.ok());
  vr::DescriptorSetLayout layout_a = std::move(a).value();
  CHECK(layout_a.valid());
  const VkDescriptorSetLayout raw = layout_a.handle();

  vr::DescriptorSetLayout layout_b(std::move(layout_a));
  CHECK(!layout_a.valid());
  CHECK(layout_a.handle() == VK_NULL_HANDLE);
  CHECK(layout_b.valid());
  CHECK(layout_b.handle() == raw);

  vr::Result<vr::DescriptorSetLayout> c =
      vr::DescriptorSetLayout::create(device, nullptr, 0);
  CHECK(c.ok());
  vr::DescriptorSetLayout layout_c = std::move(c).value();
  layout_c = std::move(layout_b);
  CHECK(layout_c.valid());
  CHECK(layout_c.handle() == raw);
  CHECK(!layout_b.valid());

  vr::DescriptorSetLayout* alias = &layout_c;
  layout_c = std::move(*alias);
  CHECK(layout_c.valid());
  CHECK(layout_c.handle() == raw);
  return 0;
}

// Allocator: the pImpl + unique_ptr owner (defaulted moves) -- the one owner
// not backed by UniqueHandle, so its move path (vmaDestroyAllocator on the old
// Impl during move-assign) needs its own coverage.
int test_allocator_moves(VkInstance instance, const vr::Device& device) {
  vr::Result<vr::Allocator> a = vr::Allocator::create(instance, device);
  CHECK(a.ok());
  vr::Allocator alloc_a = std::move(a).value();
  CHECK(alloc_a.valid());

  // move-construct: source emptied, destination adopts the allocator.
  vr::Allocator alloc_b(std::move(alloc_a));
  CHECK(!alloc_a.valid());
  CHECK(alloc_b.valid());

  // move-assign over a live allocator: the destination's original VmaAllocator
  // is freed (ASan would flag a leak otherwise), then it adopts the source.
  vr::Result<vr::Allocator> c = vr::Allocator::create(instance, device);
  CHECK(c.ok());
  vr::Allocator alloc_c = std::move(c).value();
  alloc_c = std::move(alloc_b);
  CHECK(alloc_c.valid());
  CHECK(!alloc_b.valid());

  // self-move: a no-op. Launder through a pointer to dodge -Wself-move.
  vr::Allocator* alias = &alloc_c;
  alloc_c = std::move(*alias);
  CHECK(alloc_c.valid());
  return 0;
}

// ComputeKernel: a bundle of two UniqueHandle-backed owners (layout, pipeline
// -- covered above) plus a copyable, non-owning DescriptorSet. Its moves are
// hand-written to reset that `set` on the source, so a moved-from kernel is
// fully empty (no stale set handle behind valid()==false). A 0-binding layout +
// a set from a 1-slot pool exercises the moves without a shader -- the pipeline
// stays empty; the point is the set-reset and the layout move.
int test_compute_kernel_moves(VkDevice device) {
  vr::Result<vr::DescriptorSetLayout> layout =
      vr::DescriptorSetLayout::create(device, nullptr, 0);
  CHECK(layout.ok());
  VkDescriptorPoolSize size{};
  size.type = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  size.descriptorCount = 1;
  vr::Result<vr::DescriptorPool> pool =
      vr::DescriptorPool::create(device, &size, 1, 1);
  CHECK(pool.ok());
  vr::Result<vr::DescriptorSet> set =
      pool.value().allocate(layout.value().handle());
  CHECK(set.ok());
  const VkDescriptorSet raw = set.value().handle();
  CHECK(raw != VK_NULL_HANDLE);

  vr::ComputeKernel a;
  a.layout = std::move(layout).value();
  a.set = set.value();  // copyable, non-owning view
  CHECK(a.layout.valid());
  CHECK(a.set.handle() == raw);

  // move-construct: source's set is reset (not left a stale copy), layout
  // moved.
  vr::ComputeKernel b(std::move(a));
  CHECK(a.set.handle() == VK_NULL_HANDLE);  // NOLINT(bugprone-use-after-move)
  CHECK(!a.layout.valid());
  CHECK(!a.valid());
  CHECK(b.set.handle() == raw);
  CHECK(b.layout.valid());

  // move-assign over a live kernel: destination adopts the source, source
  // reset.
  //
  // The destination has to be live *and distinguishable*. Seeding it with
  // `c.set = b.set` made `c.set.handle() == raw` true before the assignment
  // ran, so the check below passed whether or not anything was transferred.
  // A second real pool + set gives c its own handle to lose.
  vr::Result<vr::DescriptorSetLayout> layout2 =
      vr::DescriptorSetLayout::create(device, nullptr, 0);
  CHECK(layout2.ok());
  vr::Result<vr::DescriptorPool> pool2 =
      vr::DescriptorPool::create(device, &size, 1, 1);
  CHECK(pool2.ok());
  vr::Result<vr::DescriptorSet> set2 =
      pool2.value().allocate(layout2.value().handle());
  CHECK(set2.ok());
  const VkDescriptorSet raw2 = set2.value().handle();
  CHECK(raw2 != VK_NULL_HANDLE);
  CHECK(raw2 != raw);  // both alive, so the driver cannot have reused one

  vr::ComputeKernel c;
  c.layout = std::move(layout2).value();
  c.set = set2.value();
  CHECK(c.set.handle() == raw2);
  c = std::move(b);
  CHECK(c.set.handle() == raw);  // adopted b's, not kept its own
  CHECK(c.layout.valid());
  CHECK(b.set.handle() == VK_NULL_HANDLE);  // NOLINT(bugprone-use-after-move)

  // self-move: the `if (this != &other)` guard keeps its set + layout. Launder
  // through a pointer to dodge -Wself-move.
  vr::ComputeKernel* alias = &c;
  c = std::move(*alias);
  CHECK(c.set.handle() == raw);
  CHECK(c.layout.valid());
  return 0;
}

// GpuTimer: a UniqueHandle member plus scalar state (device, max_spans, the
// timestamp validity and period) that the defaulted moves copied and left live
// in the source. It is the one owner here whose accessors have to stay
// consistent with valid() for a *capability* reason: a moved-from timer that
// still reports valid() is byte-indistinguishable from the supported
// "this family has no timestamps" degradation, so the bug hides as a
// capability report and the caller is told the GPU cannot do it.
int test_gpu_timer_moves(const vr::Device& device) {
  vr::Result<vr::GpuTimer> a = vr::GpuTimer::create(device);
  CHECK(a.ok());
  vr::GpuTimer timer_a = std::move(a).value();
  CHECK(timer_a.valid());
  const bool was_available = timer_a.available();

  // move-construct: the source is fully empty, not merely handle-less.
  vr::GpuTimer timer_b(std::move(timer_a));
  CHECK(!timer_a.valid());  // NOLINT(bugprone-use-after-move)
  CHECK(!timer_a.available());
  CHECK(timer_a.count() == 0);
  CHECK(timer_b.valid());
  CHECK(timer_b.available() == was_available);

  // move-assign over a live timer: the destination's own pool is freed (ASan
  // would flag the leak otherwise), then it adopts the source's.
  vr::Result<vr::GpuTimer> c = vr::GpuTimer::create(device);
  CHECK(c.ok());
  vr::GpuTimer timer_c = std::move(c).value();
  CHECK(timer_c.valid());
  timer_c = std::move(timer_b);
  CHECK(timer_c.valid());
  CHECK(timer_c.available() == was_available);
  CHECK(!timer_b.valid());  // NOLINT(bugprone-use-after-move)

  // self-move: the `if (this != &other)` guard keeps the pool AND the recorded
  // window. The defaulted operator= was memberwise, so `spans_ =
  // std::move(spans_)` silently emptied a window that had already been measured
  // and the caller read it as "nothing was measured".
  const auto record = [](VkCommandBuffer) {};
  CHECK(device.submit_single_time(record, &timer_c, "self-move"));
  const std::size_t recorded = timer_c.count();
  vr::GpuTimer* alias = &timer_c;
  timer_c = std::move(*alias);
  CHECK(timer_c.valid());
  CHECK(timer_c.available() == was_available);
  CHECK(timer_c.count() == recorded);
  return 0;
}

}  // namespace

int main() {
  vr::Result<vr::Instance> instance = vr::Instance::create({});
  if (!instance) {
    std::fprintf(stderr, "no Vulkan instance (%s); skipping\n",
                 instance.status().message().c_str());
    return 0;
  }
  vr::Result<VkPhysicalDevice> gpu = instance.value().select_physical_device();
  if (!gpu) {
    std::fprintf(stderr, "no compute-capable device (%s); skipping\n",
                 gpu.status().message().c_str());
    return 0;
  }
  vr::Result<vr::Device> device =
      vr::Device::create(instance.value().handle(), gpu.value(), {});
  if (!device) {
    std::fprintf(stderr, "device create failed: %s\n",
                 device.status().message().c_str());
    return 1;
  }
  vr::Result<vr::Allocator> allocator =
      vr::Allocator::create(instance.value().handle(), device.value());
  if (!allocator) {
    std::fprintf(stderr, "allocator create failed: %s\n",
                 allocator.status().message().c_str());
    return 1;
  }

  if (int rc = test_buffer_moves(allocator.value()); rc != 0) {
    return rc;
  }
  if (int rc = test_layout_moves(device.value().handle()); rc != 0) {
    return rc;
  }
  if (int rc = test_allocator_moves(instance.value().handle(), device.value());
      rc != 0) {
    return rc;
  }
  if (int rc = test_compute_kernel_moves(device.value().handle()); rc != 0) {
    return rc;
  }
  if (int rc = test_gpu_timer_moves(device.value()); rc != 0) {
    return rc;
  }

  std::printf("recon compute RAII move tests passed\n");
  return 0;
}
