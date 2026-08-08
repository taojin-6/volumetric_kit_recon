// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// BufferDesc::queue_families: the sharing mode a buffer the renderer reads
// directly needs (interop seam B).
//
// Three layers, because the obvious one asserts nothing on its own:
//
// (1) The reduction, pinned directly through the internal header. Needs no
//     device and no validation layer, so it fails on any machine if the rule
//     breaks.
// (2) The mode each case actually produced, read back off Buffer::sharing_mode.
//     Vulkan cannot be asked what a VkBuffer was created with, which is why the
//     Buffer records it -- and it is what lets "the same family twice collapses
//     to EXCLUSIVE" be an assertion rather than an inference drawn from the
//     call not failing.
// (3) A validation-error count around the device cases. The first cut of this
//     test leaned on the layer alone and claimed a malformed CONCURRENT buffer
//     would be caught wherever it was installed. It would not have been:
//     recon's debug callback returns VK_FALSE (it must -- aborting is for a
//     debugger), vmaCreateBuffer still returns VK_SUCCESS, and nothing turned
//     the diagnostic into a failure. Counting Error-level messages through the
//     log handler closes that, as a delta around the buffer cases so an
//     unrelated driver complaint during setup can neither mask nor fake it.
//
// Needs a device, so (2) and (3) skip (exit 0) where no driver/device is
// present, like the other Vulkan tests.

#include <cstdint>
#include <cstdio>
#include <string_view>
#include <utility>

#include "queue_family_set.hpp"
#include "volumetric_kit/recon/core/allocator.hpp"
#include "volumetric_kit/recon/core/buffer.hpp"
#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/instance.hpp"
#include "volumetric_kit/recon/core/log.hpp"
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

// Error-level messages seen since the handler was installed. The validation
// layer reaches this through recon's debug messenger, so a malformed buffer
// lands here rather than only on stderr.
int g_errors = 0;

}  // namespace

// The reduction, pinned directly. Sabotaging the deduplication must fail this
// on every machine, layer or no layer.
int check_reduction() {
  constexpr std::uint32_t kCap = 4;
  std::uint32_t out[kCap]{};

  // Nothing in, nothing out -- the default path, and null is legal at count 0.
  CHECK(vr::distinct_queue_families(nullptr, 0, out, kCap) == 0);

  // Two distinct families stay two, in first-seen order.
  const std::uint32_t two[2] = {1, 0};
  CHECK(vr::distinct_queue_families(two, 2, out, kCap) == 2);
  CHECK(out[0] == 1);
  CHECK(out[1] == 0);

  // The case the whole rule exists for: a caller naming its compute and render
  // families unconditionally, on a device where they are the same one. Two
  // entries in, one distinct out -- which is what makes the buffer EXCLUSIVE
  // rather than a CONCURRENT one Vulkan rejects.
  const std::uint32_t same[2] = {2, 2};
  CHECK(vr::distinct_queue_families(same, 2, out, kCap) == 1);
  CHECK(out[0] == 2);

  // Duplicates anywhere in the run, not just adjacent.
  const std::uint32_t messy[5] = {3, 1, 3, 1, 3};
  CHECK(vr::distinct_queue_families(messy, 5, out, kCap) == 2);
  CHECK(out[0] == 3);
  CHECK(out[1] == 1);

  // Overflow is reported, not truncated silently: a partial reduction would
  // name fewer families than will actually touch the buffer, which is the
  // original bug wearing a different hat.
  const std::uint32_t many[5] = {0, 1, 2, 3, 4};
  CHECK(vr::distinct_queue_families(many, 5, out, kCap) == kCap + 1);
  // Repeats of an already-seen family do not count toward the ceiling.
  const std::uint32_t repeats[6] = {0, 1, 2, 3, 3, 0};
  CHECK(vr::distinct_queue_families(repeats, 6, out, kCap) == kCap);

  return 0;
}

int main() {
  if (check_reduction() != 0) {
    return 1;
  }

  // Installed before the instance, so the layer's output reaches the counter.
  // Errors are printed as well as counted: a bare count is undiagnosable.
  vr::set_log_handler([](vr::LogLevel level, std::string_view message) {
    if (level == vr::LogLevel::Error) {
      ++g_errors;
      std::fprintf(stderr, "[vulkan error] %.*s\n",
                   static_cast<int>(message.size()), message.data());
    }
  });

  // Validation on: it is what catches a CONCURRENT buffer whose indices are not
  // unique, or which names fewer than two families. Where the Khronos layer is
  // not installed, `enable_validation` is documented as a no-op and layer (3)
  // goes quiet with it -- layers (1) and (2) still assert.
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

  // Real family indices, not hardcoded ones. Vulkan requires every index a
  // CONCURRENT buffer names be less than the device's family count, so `{0, 1}`
  // on a single-family driver -- lavapipe, which the Linux CI legs run -- is
  // itself the malformed shape this test exists to detect.
  std::uint32_t family_count = 0;
  vkGetPhysicalDeviceQueueFamilyProperties(gpu.value(), &family_count, nullptr);
  CHECK(family_count >= 1);
  std::fprintf(stderr, "device reports %u queue famil%s\n", family_count,
               family_count == 1 ? "y" : "ies");

  // Everything below this point must produce no validation error.
  const int errors_before = g_errors;

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
    CHECK(buffer.value().sharing_mode() == VK_SHARING_MODE_EXCLUSIVE);
  }

  // Two distinct families: CONCURRENT, which is the seam-B case. Needs a device
  // that actually has two; on a single-family driver the shape is unreachable
  // rather than untested -- there is no cross-family read to arrange.
  if (family_count >= 2) {
    const std::uint32_t families[2] = {0, 1};
    vr::BufferDesc desc = base();
    desc.queue_families = families;
    desc.queue_family_count = 2;
    vr::Result<vr::Buffer> buffer = allocator.value().create_buffer(desc);
    CHECK(buffer.ok());
    CHECK(buffer.value().handle() != VK_NULL_HANDLE);
    CHECK(buffer.value().sharing_mode() == VK_SHARING_MODE_CONCURRENT);
  } else {
    std::fprintf(stderr, "single queue family; skipping the CONCURRENT case\n");
  }

  // The same family twice. This is the call site that matters: a consumer
  // passes its compute and render families unconditionally, and off Apple they
  // are frequently the same one. Requesting CONCURRENT here would be a
  // validation error on two counts (duplicate indices, and fewer than two
  // distinct), so EXCLUSIVE is the deduplication working.
  {
    const std::uint32_t families[2] = {0, 0};
    vr::BufferDesc desc = base();
    desc.queue_families = families;
    desc.queue_family_count = 2;
    vr::Result<vr::Buffer> buffer = allocator.value().create_buffer(desc);
    CHECK(buffer.ok());
    CHECK(buffer.value().sharing_mode() == VK_SHARING_MODE_EXCLUSIVE);
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
    CHECK(buffer.value().sharing_mode() == VK_SHARING_MODE_EXCLUSIVE);
  }

  // More distinct families than the reduction can hold is refused, rather than
  // reaching Vulkan naming only the first few.
  {
    const std::uint32_t families[5] = {0, 1, 2, 3, 4};
    vr::BufferDesc desc = base();
    desc.queue_families = families;
    desc.queue_family_count = 5;
    vr::Result<vr::Buffer> buffer = allocator.value().create_buffer(desc);
    CHECK(!buffer.ok());
    CHECK(buffer.status().domain() == vr::Status::Code::InvalidArgument);
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

  // An index no family on this device answers to is refused, in either mode.
  // It is CONCURRENT where it matters: naming a nonexistent family violates
  // VUID-VkBufferCreateInfo-sharingMode-01419, and vmaCreateBuffer still
  // returns VK_SUCCESS, so with layers off -- the shipping configuration, and
  // the only one on iOS -- nothing at all reports it. The shape that produces
  // it is an app hardcoding the two families it saw on Apple and running
  // somewhere with fewer, so the out-of-range index is derived from this
  // device's real count rather than assumed.
  {
    const std::uint32_t past_end[] = {0, family_count};
    vr::BufferDesc desc = base();
    desc.queue_families = past_end;
    desc.queue_family_count = 2;
    vr::Result<vr::Buffer> buffer = allocator.value().create_buffer(desc);
    CHECK(!buffer.ok());
    CHECK(buffer.status().domain() == vr::Status::Code::InvalidArgument);
  }
  // Also refused when the duplicate collapses it to EXCLUSIVE, where Vulkan
  // ignores the list: an index the device does not have is a caller bug either
  // way, and reporting it where the list is given is the point.
  {
    const std::uint32_t past_end[] = {family_count, family_count};
    vr::BufferDesc desc = base();
    desc.queue_families = past_end;
    desc.queue_family_count = 2;
    vr::Result<vr::Buffer> buffer = allocator.value().create_buffer(desc);
    CHECK(!buffer.ok());
    CHECK(buffer.status().domain() == vr::Status::Code::InvalidArgument);
  }

  // A Buffer outliving the Allocator *object* is well-defined, because the
  // allocator's lifetime is what a Buffer holds a reference to -- not the
  // wrapper's. Move-assignment is the case a prose ordering rule could not
  // express: `alloc = std::move(other)` ends the resource's life while the
  // wrapper visibly lives on, so a reader who satisfied "destroy Buffers first"
  // by keeping the object alive still got a use-after-free when the Buffer
  // freed through the destroyed VmaAllocator. Under the sanitizers leg this is
  // a real detector; here it is at least a crash.
  {
    vr::Result<vr::Allocator> other =
        vr::Allocator::create(instance.value().handle(), device.value());
    CHECK(other.ok());
    vr::Result<vr::Buffer> buffer = allocator.value().create_buffer(base());
    CHECK(buffer.ok());
    allocator.value() = std::move(other).value();  // frees the old allocator
    CHECK(allocator.value().valid());
    // buffer's destructor runs here, against the allocator it was made from.
  }

  // The layer's verdict on every buffer above. Zero is the assertion; a
  // non-zero count has already printed what it was.
  CHECK(g_errors == errors_before);

  std::printf("PASS\n");
  return 0;
}
