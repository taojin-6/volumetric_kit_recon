// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// Move-semantics tests for the compute core's RAII handle owners. Per the
// CLAUDE.md RAII rules, every move-only owner must: move-construct (leaving the
// source empty), move-assign over a live object (freeing the old resource), and
// survive a self-move. Under the sanitizer CI job these become real
// leak/double-free detectors -- a forgotten reset or a missing self-move guard
// shows up as an ASan report here.
//
// Two owners are exercised with real handles, one per mechanism:
//   - Buffer            -- the hand-written `std::function` deleter
//   (VMA-backed).
//   - DescriptorSetLayout -- the UniqueHandle member, whose defaulted moves
//   back
//                            ShaderModule / DescriptorPool / ComputePipeline
//                            too.
//
// Both need a device, so the whole test skips (exit 0) where no driver/device
// is present, like the other Vulkan tests.

#include <cstdint>
#include <cstdio>
#include <utility>

#include "volumetric_kit/recon/core/allocator.hpp"
#include "volumetric_kit/recon/core/buffer.hpp"
#include "volumetric_kit/recon/core/descriptor.hpp"
#include "volumetric_kit/recon/core/device.hpp"
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

  std::printf("recon compute RAII move tests passed\n");
  return 0;
}
