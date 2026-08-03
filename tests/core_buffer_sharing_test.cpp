// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// BufferDesc::queue_families: the sharing mode a buffer the renderer reads
// directly needs (interop seam B).
//
// The mode itself is not queryable -- Vulkan gives no way to read a created
// buffer's sharingMode back -- so what this pins is the behaviour a caller
// depends on, and it leans on the validation layer for the rest. A CONCURRENT
// buffer whose indices are not unique, or which names fewer than two families,
// is a validation error; so a test that asks for those shapes and gets a
// working buffer *is* the assertion that the deduplication happened -- but only
// with the layer loaded, which is why this asks for it explicitly below rather
// than taking the default. Where the Khronos layer is not installed,
// `enable_validation` is documented as a no-op, and these cases degrade to
// smoke tests; the failure they catch is a real one either way in CI.
//
// Needs a device, so the whole test skips (exit 0) where no driver/device is
// present, like the other Vulkan tests.

#include <cstdint>
#include <cstdio>

#include "volumetric_kit/recon/core/allocator.hpp"
#include "volumetric_kit/recon/core/buffer.hpp"
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

// The reduction, pinned directly. This is the half with teeth: it needs no
// device and no validation layer, so it fails on any machine if the rule
// breaks. Sabotaging the deduplication and re-running proved the device cases
// below do *not* catch it here -- they pass without the layer installed, which
// is most machines.
int check_reduction() {
  constexpr std::uint32_t kCap = 4;
  std::uint32_t out[kCap]{};

  // Nothing in, nothing out -- the default path, and null is legal at count 0.
  CHECK(vr::detail::distinct_queue_families(nullptr, 0, out, kCap) == 0);

  // Two distinct families stay two, in first-seen order.
  const std::uint32_t two[2] = {1, 0};
  CHECK(vr::detail::distinct_queue_families(two, 2, out, kCap) == 2);
  CHECK(out[0] == 1);
  CHECK(out[1] == 0);

  // The case the whole rule exists for: a caller naming its compute and render
  // families unconditionally, on a device where they are the same one. Two
  // entries in, one distinct out -- which is what makes the buffer EXCLUSIVE
  // rather than a CONCURRENT one Vulkan rejects.
  const std::uint32_t same[2] = {2, 2};
  CHECK(vr::detail::distinct_queue_families(same, 2, out, kCap) == 1);
  CHECK(out[0] == 2);

  // Duplicates anywhere in the run, not just adjacent.
  const std::uint32_t messy[5] = {3, 1, 3, 1, 3};
  CHECK(vr::detail::distinct_queue_families(messy, 5, out, kCap) == 2);
  CHECK(out[0] == 3);
  CHECK(out[1] == 1);

  // Overflow is reported, not truncated silently: a partial reduction would
  // name fewer families than will actually touch the buffer, which is the
  // original bug wearing a different hat.
  const std::uint32_t many[5] = {0, 1, 2, 3, 4};
  CHECK(vr::detail::distinct_queue_families(many, 5, out, kCap) == kCap + 1);
  // Repeats of an already-seen family do not count toward the ceiling.
  const std::uint32_t repeats[6] = {0, 1, 2, 3, 3, 0};
  CHECK(vr::detail::distinct_queue_families(repeats, 6, out, kCap) == kCap);

  return 0;
}

int main() {
  if (check_reduction() != 0) {
    return 1;
  }

  // Validation on: without the layer, a malformed CONCURRENT buffer is created
  // happily and every case below passes whether the deduplication runs or not.
  vr::InstanceConfig config;
  config.enable_validation = true;
  vr::Result<vr::Instance> instance = vr::Instance::create(config);
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

  constexpr VkDeviceSize kBytes = 4096;
  const auto base = [] {
    vr::BufferDesc desc;
    desc.size = kBytes;
    desc.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
    return desc;
  };

  // Naming no families is the default and stays EXCLUSIVE -- every buffer this
  // library allocates for itself.
  {
    vr::Result<vr::Buffer> buffer = allocator.value().create_buffer(base());
    CHECK(buffer.ok());
    CHECK(buffer.value().handle() != VK_NULL_HANDLE);
  }

  // Two distinct families: CONCURRENT, which is the seam-B case.
  {
    const std::uint32_t families[2] = {0, 1};
    vr::BufferDesc desc = base();
    desc.queue_families = families;
    desc.queue_family_count = 2;
    vr::Result<vr::Buffer> buffer = allocator.value().create_buffer(desc);
    CHECK(buffer.ok());
    CHECK(buffer.value().handle() != VK_NULL_HANDLE);
  }

  // The same family twice. This is the call site that matters: a consumer
  // passes its compute and render families unconditionally, and off Apple they
  // are frequently the same one. Requesting CONCURRENT here would be a
  // validation error on two counts (duplicate indices, and fewer than two
  // distinct), so succeeding is the deduplication working.
  {
    const std::uint32_t families[2] = {0, 0};
    vr::BufferDesc desc = base();
    desc.queue_families = families;
    desc.queue_family_count = 2;
    vr::Result<vr::Buffer> buffer = allocator.value().create_buffer(desc);
    CHECK(buffer.ok());
    CHECK(buffer.value().handle() != VK_NULL_HANDLE);
  }

  // One family is exclusive by definition, and must not reach Vulkan as a
  // CONCURRENT buffer naming a single index.
  {
    const std::uint32_t families[1] = {0};
    vr::BufferDesc desc = base();
    desc.queue_families = families;
    desc.queue_family_count = 1;
    vr::Result<vr::Buffer> buffer = allocator.value().create_buffer(desc);
    CHECK(buffer.ok());
    CHECK(buffer.value().handle() != VK_NULL_HANDLE);
  }

  // A count with no array is the caller's mistake, and is refused rather than
  // read past.
  {
    vr::BufferDesc desc = base();
    desc.queue_families = nullptr;
    desc.queue_family_count = 2;
    vr::Result<vr::Buffer> buffer = allocator.value().create_buffer(desc);
    CHECK(!buffer.ok());
    CHECK(buffer.status().domain() == vr::Status::Code::InvalidArgument);
  }

  std::printf("PASS\n");
  return 0;
}
