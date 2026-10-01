// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// CommandBatch: every recording call, in orders that would expose a command
// run out of place -- an upload after a dispatch, a readback before one -- over
// a device-local and a host-visible buffer, since a buffer's memory type must
// not change what a batch does. Uploads inline, staged and packed by the
// caller, several readbacks in one batch, transfers left unordered (fills,
// uploads and copies rising through one buffer among them), image copies, the
// refusals, the moves, timed dispatches and uploads, and a throwing record.
// Skips (exit 0) where no device is present.

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <utility>
#include <vector>

#include "test_image.hpp"
#include "volumetric_kit/recon/core/allocator.hpp"
#include "volumetric_kit/recon/core/buffer.hpp"
#include "volumetric_kit/recon/core/command_batch.hpp"
#include "volumetric_kit/recon/core/compute_kernel.hpp"
#include "volumetric_kit/recon/core/compute_util.hpp"
#include "volumetric_kit/recon/core/descriptor.hpp"
#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/gpu_timer.hpp"
#include "volumetric_kit/recon/core/instance.hpp"
#include "volumetric_kit/recon/core/log.hpp"
#include "volumetric_kit/recon/core/stage_metrics.hpp"

namespace vr = volumetric_kit::recon;

#define CHECK(cond)                                                        \
  do {                                                                     \
    if (!(cond)) {                                                         \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      return 1;                                                            \
    }                                                                      \
  } while (0)

namespace {

// Validation-layer errors, counted by the log handler: the only report of a
// command the batch recorded against the spec -- an inline update past 64 KiB
// or off 4-byte alignment -- on a driver that runs it anyway.
std::atomic<int> g_errors{0};  // the layer may report from any thread

constexpr std::uint32_t kCount = 256;  // four workgroups of add.comp
constexpr VkDeviceSize kBytes = kCount * sizeof(std::uint32_t);

struct Push {
  std::uint32_t count;
  std::uint32_t delta;
};

std::vector<std::uint32_t> load_spirv(const char* path) {
  std::ifstream in(path, std::ios::binary | std::ios::ate);
  if (!in) return {};
  const auto size = static_cast<std::size_t>(in.tellg());
  std::vector<std::uint32_t> code(size / 4);
  in.seekg(0);
  in.read(reinterpret_cast<char*>(code.data()),
          static_cast<std::streamsize>(size));
  return code;
}

std::vector<std::uint32_t> pattern(std::uint32_t base, std::size_t n = kCount) {
  std::vector<std::uint32_t> v(n);
  for (std::size_t i = 0; i < n; ++i) {
    v[i] = base + static_cast<std::uint32_t>(i) * 7u;
  }
  return v;
}

std::vector<std::uint32_t> plus(std::vector<std::uint32_t> v,
                                std::uint32_t delta) {
  for (auto& x : v) x += delta;
  return v;
}

struct Rig {
  vr::Device* device;
  vr::Allocator* allocator;
  vr::ComputeKernel* add;
  std::uint32_t max_groups;
};

// Bind `buffer` and record `add` over its first kCount values.
vr::Status add_to(vr::CommandBatch& batch, const Rig& rig,
                  const vr::Buffer& buffer, std::uint32_t delta) {
  rig.add->set.write_storage_buffer(0, buffer.handle(), 0, VK_WHOLE_SIZE);
  const Push push{kCount, delta};
  return batch.dispatch(*rig.add, &push, sizeof(push),
                        vr::group_count(kCount, 64), rig.max_groups);
}

// A device-local buffer (kind 0) or a host-visible one (kind 1), with the
// usage every batch call needs.
vr::Result<vr::Buffer> make(vr::Allocator& a, int kind, VkDeviceSize bytes) {
  if (kind == 0) return vr::device_storage_buffer(a, bytes);
  return vr::storage_buffer(
      a, bytes, vr::HostAccess::Random,
      VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
}

// An image handle nothing was made for, for tests that never reach Vulkan.
VkImage fake_image() {
  VkImage handle = VK_NULL_HANDLE;
  const std::uint64_t value = 0x1000;
  static_assert(sizeof(handle) == sizeof(value), "a 64-bit handle");
  std::memcpy(&handle, &value, sizeof(handle));
  return handle;
}

// An Image hands its deleter on with it and runs it once, whoever holds it.
int test_image_moves() {
  int freed = 0;
  const auto made = [&freed] {
    return vr::Image(fake_image(), VK_FORMAT_R8_UNORM, 4, 2,
                     VK_IMAGE_USAGE_TRANSFER_SRC_BIT, VK_IMAGE_LAYOUT_GENERAL,
                     [&freed] { ++freed; });
  };
  {
    vr::Image a = made();
    vr::Image b(std::move(a));
    CHECK(!a.valid() && a.width() == 0 && a.height() == 0);      // NOLINT
    CHECK(a.format() == VK_FORMAT_UNDEFINED && a.usage() == 0);  // NOLINT
    CHECK(a.layout() == VK_IMAGE_LAYOUT_UNDEFINED);              // NOLINT
    CHECK(b.valid() && b.width() == 4 && b.height() == 2);
    CHECK(b.layout() == VK_IMAGE_LAYOUT_GENERAL);
    vr::Image c = made();
    c = std::move(b);  // over a live image, which goes
    CHECK(freed == 1 && !b.valid() && c.valid());  // NOLINT
    vr::Image* alias = &c;
    c = std::move(*alias);  // self-move
    CHECK(freed == 1 && c.valid());
  }
  CHECK(freed == 2);
  return 0;
}

int run_kind(const Rig& rig, int kind) {
  vr::Device& device = *rig.device;
  vr::Allocator& allocator = *rig.allocator;
  vr::Result<vr::Buffer> made = make(allocator, kind, kBytes);
  CHECK(made.ok());
  const vr::Buffer buffer = std::move(made).value();
  std::printf("  %s\n", kind == 0 ? "device-local" : "host-visible");

  // Round trip, inline (1 KiB, aligned).
  const std::vector<std::uint32_t> p = pattern(100);
  std::vector<std::uint32_t> got(kCount, 0);
  {
    vr::CommandBatch batch(device, allocator);
    CHECK(batch.upload(buffer, 0, p.data(), kBytes).ok());
    CHECK(batch.readback(buffer, 0, kBytes, got.data()).ok());
    CHECK(batch.submit().ok());
  }
  CHECK(got == p);

  // Upload, dispatch, readback: one submit, in order.
  {
    vr::CommandBatch batch(device, allocator);
    CHECK(batch.upload(buffer, 0, p.data(), kBytes).ok());
    CHECK(add_to(batch, rig, buffer, 5).ok());
    CHECK(batch.readback(buffer, 0, kBytes, got.data()).ok());
    CHECK(batch.submit().ok());
  }
  CHECK(got == plus(p, 5));

  // An upload recorded after a dispatch lands after it: the dispatch must not
  // add onto the uploaded values.
  const std::vector<std::uint32_t> q = pattern(9000);
  {
    vr::CommandBatch batch(device, allocator);
    CHECK(add_to(batch, rig, buffer, 1).ok());
    CHECK(batch.upload(buffer, 0, q.data(), kBytes).ok());
    CHECK(batch.readback(buffer, 0, kBytes, got.data()).ok());
    CHECK(batch.submit().ok());
  }
  CHECK(got == q);

  // A readback recorded before a dispatch reads what stood before it.
  std::vector<std::uint32_t> before(kCount, 0);
  std::vector<std::uint32_t> after(kCount, 0);
  {
    vr::CommandBatch batch(device, allocator);
    CHECK(batch.readback(buffer, 0, kBytes, before.data()).ok());
    CHECK(add_to(batch, rig, buffer, 3).ok());
    CHECK(batch.readback(buffer, 0, kBytes, after.data()).ok());
    CHECK(batch.submit().ok());
  }
  CHECK(before == q);
  CHECK(after == plus(q, 3));

  // Offsets: a fill, an inline partial upload and a staged unaligned one,
  // then several small readbacks at odd sizes out of one batch.
  const unsigned char odd[6] = {1, 2, 3, 4, 5, 6};
  std::uint32_t head[3] = {};
  unsigned char middle[6] = {};
  std::uint32_t tail = 0;
  {
    vr::CommandBatch batch(device, allocator);
    CHECK(batch.fill(buffer, 0, kBytes, 0xABCDu).ok());
    CHECK(batch.upload(buffer, 16, p.data(), 32).ok());  // inline
    CHECK(batch.upload(buffer, 102, odd, 6).ok());       // staged: unaligned
    CHECK(batch.readback(buffer, 12, 12, head).ok());
    CHECK(batch.readback(buffer, 102, 6, middle).ok());
    CHECK(batch.readback(buffer, kBytes - 4, 4, &tail).ok());
    CHECK(batch.submit().ok());
  }
  CHECK(head[0] == 0xABCDu && head[1] == p[0] && head[2] == p[1]);
  for (int i = 0; i < 6; ++i) CHECK(middle[i] == odd[i]);
  CHECK(tail == 0xABCDu);

  // A reserved upload, filled by the caller after the call, at an odd offset.
  unsigned char packed[10] = {};
  {
    vr::CommandBatch batch(device, allocator);
    vr::Result<void*> staging = batch.reserve_upload(buffer, 41, 10);
    CHECK(staging.ok() && staging.value() != nullptr);
    for (int i = 0; i < 10; ++i) {
      static_cast<unsigned char*>(staging.value())[i] =
          static_cast<unsigned char>(200 + i);
    }
    CHECK(batch.readback(buffer, 41, 10, packed).ok());
    CHECK(batch.submit().ok());
  }
  for (int i = 0; i < 10; ++i) CHECK(packed[i] == 200 + i);
  return 0;
}

}  // namespace

int main() {
  // Installed before the instance, so the layer's output reaches the counter.
  vr::set_log_handler([](vr::LogLevel level, std::string_view message) {
    if (level == vr::LogLevel::Error) {
      ++g_errors;
      std::fprintf(stderr, "[vulkan error] %.*s\n",
                   static_cast<int>(message.size()), message.data());
    }
  });
  // A no-op where the Khronos layer is not installed.
  vr::InstanceConfig config;
  config.enable_validation = true;
  vr::Result<vr::Instance> instance = vr::Instance::create(config);
  if (!instance) {
    std::fprintf(stderr, "no Vulkan instance; skipping\n");
    return 0;
  }
  vr::Result<VkPhysicalDevice> gpu = instance.value().select_physical_device();
  if (!gpu) {
    std::fprintf(stderr, "no compute-capable device; skipping\n");
    return 0;
  }
  vr::Result<vr::Device> device_result =
      vr::Device::create(instance.value(), gpu.value(), {});
  CHECK(device_result.ok());
  vr::Device device = std::move(device_result).value();
  vr::Result<vr::Allocator> allocator_result =
      vr::Allocator::create(instance.value().handle(), device);
  CHECK(allocator_result.ok());
  vr::Allocator allocator = std::move(allocator_result).value();

  const std::vector<std::uint32_t> spv = load_spirv(VR_ADD_SPV);
  CHECK(!spv.empty());
  VkPushConstantRange range{};
  range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
  range.size = sizeof(Push);
  vr::ComputeKernel add;
  vr::KernelSetBuilder builder(device);
  CHECK(builder
            .add(add, "test_add",
                 reinterpret_cast<const unsigned char*>(spv.data()),
                 spv.size() * sizeof(std::uint32_t), 1, &range)
            .ok());
  vr::Result<vr::DescriptorPool> pool = builder.build();
  CHECK(pool.ok());
  VkPhysicalDeviceProperties props{};
  vkGetPhysicalDeviceProperties(device.physical_device(), &props);
  const Rig rig{&device, &allocator, &add,
                props.limits.maxComputeWorkGroupCount[0]};

  std::printf("buffers:\n");
  for (int kind = 0; kind < 2; ++kind) {
    if (run_kind(rig, kind) != 0) return 1;
  }

  vr::Result<vr::Buffer> a_result =
      vr::device_storage_buffer(allocator, kBytes);
  vr::Result<vr::Buffer> b_result =
      vr::device_storage_buffer(allocator, kBytes);
  CHECK(a_result.ok() && b_result.ok());
  const vr::Buffer a = std::move(a_result).value();
  const vr::Buffer b = std::move(b_result).value();
  const std::vector<std::uint32_t> p = pattern(1);
  std::vector<std::uint32_t> got(kCount, 0);

  // A staged upload past the inline limit, read back whole.
  {
    const std::size_t n = (vr::CommandBatch::kMaxInlineUpload * 2) / 4;
    vr::Result<vr::Buffer> big_result = vr::device_storage_buffer(
        allocator, vr::CommandBatch::kMaxInlineUpload * 2);
    CHECK(big_result.ok());
    const vr::Buffer big = std::move(big_result).value();
    const std::vector<std::uint32_t> data = pattern(77, n);
    std::vector<std::uint32_t> back(n, 0);
    vr::CommandBatch batch(device, allocator);
    CHECK(batch.upload(big, 0, data.data(), big.size()).ok());
    CHECK(batch.readback(big, 0, big.size(), back.data()).ok());
    CHECK(batch.submit().ok());
    CHECK(back == data);
  }

  // Transfers on different buffers run without a barrier between them, and
  // each readback still waits for the upload it reads.
  {
    const std::vector<std::uint32_t> q = pattern(500);
    std::vector<std::uint32_t> got_b(kCount, 0);
    vr::CommandBatch batch(device, allocator);
    CHECK(batch.upload(a, 0, p.data(), kBytes).ok());
    CHECK(batch.upload(b, 0, q.data(), kBytes).ok());
    CHECK(batch.readback(a, 0, kBytes, got.data()).ok());
    CHECK(batch.readback(b, 0, kBytes, got_b.data()).ok());
    CHECK(batch.submit().ok());
    CHECK(got == p);
    CHECK(got_b == q);
  }

  // Device-to-device copy, and within one buffer where the ranges are apart.
  {
    vr::CommandBatch batch(device, allocator);
    CHECK(batch.upload(a, 0, p.data(), kBytes).ok());
    CHECK(batch.copy(a, 0, b, 0, kBytes).ok());
    CHECK(batch.copy(b, 0, b, kBytes / 2, kBytes / 2).ok());
    CHECK(batch.readback(b, 0, kBytes, got.data()).ok());
    CHECK(batch.submit().ok());
  }
  for (std::uint32_t i = 0; i < kCount / 2; ++i) {
    CHECK(got[i] == p[i]);
    CHECK(got[kCount / 2 + i] == p[i]);
  }

  // Zero at any alignment: inside one word, both edges off a word, aligned,
  // ending on a word -- and not a byte either side touched.
  {
    const std::vector<std::uint32_t> ones(kCount, 0xFFFFFFFFu);
    std::vector<unsigned char> bytes(kBytes, 0);
    vr::CommandBatch batch(device, allocator);
    CHECK(batch.upload(a, 0, ones.data(), kBytes).ok());
    CHECK(batch.zero(a, 1, 2).ok());
    CHECK(batch.zero(a, 7, 10).ok());
    CHECK(batch.zero(a, 32, 8).ok());
    CHECK(batch.zero(a, 45, 7).ok());
    CHECK(batch.readback(a, 0, kBytes, bytes.data()).ok());
    CHECK(batch.submit().ok());
    for (VkDeviceSize i = 0; i < kBytes; ++i) {
      const bool zeroed = (i >= 1 && i < 3) || (i >= 7 && i < 17) ||
                          (i >= 32 && i < 40) || (i >= 45 && i < 52);
      CHECK(bytes[i] == (zeroed ? 0 : 0xFF));
    }
  }

  // Fills at rising, disjoint offsets share one barrier. One that goes back
  // over them keeps its barrier and lands second, and a kernel after the run
  // reads every fill.
  {
    vr::CommandBatch batch(device, allocator);
    for (std::uint32_t i = 0; i < kCount; ++i) {
      CHECK(batch.fill(a, VkDeviceSize(i) * 4, 4, i).ok());
    }
    CHECK(batch.fill(a, 0, 16, 7u).ok());
    CHECK(add_to(batch, rig, a, 1).ok());
    CHECK(batch.readback(a, 0, kBytes, got.data()).ok());
    CHECK(batch.submit().ok());
    for (std::uint32_t i = 0; i < kCount; ++i) {
      CHECK(got[i] == (i < 4 ? 7u : i) + 1);
    }
  }

  // Staged uploads at rising, disjoint offsets share a run too. One that goes
  // back over them keeps its barrier and lands second.
  {
    const VkDeviceSize half = vr::CommandBatch::kMaxInlineUpload * 2;
    const std::size_t n = half / 4;
    vr::Result<vr::Buffer> big_result =
        vr::device_storage_buffer(allocator, half * 2);
    CHECK(big_result.ok());
    const vr::Buffer big = std::move(big_result).value();
    const std::vector<std::uint32_t> first = pattern(3, n);
    const std::vector<std::uint32_t> second = pattern(9, n);
    const std::vector<std::uint32_t> over = pattern(40, n);
    std::vector<std::uint32_t> back(2 * n, 0);
    vr::CommandBatch batch(device, allocator);
    CHECK(batch.upload(big, 0, first.data(), half).ok());
    CHECK(batch.upload(big, half, second.data(), half).ok());
    CHECK(batch.upload(big, half / 2, over.data(), half).ok());
    CHECK(batch.readback(big, 0, half * 2, back.data()).ok());
    CHECK(batch.submit().ok());
    for (std::size_t i = 0; i < 2 * n; ++i) {
      const std::size_t lo = n / 2;
      const std::uint32_t want = i < lo       ? first[i]
                                 : i < lo + n ? over[i - lo]
                                              : second[i - n];
      CHECK(back[i] == want);
    }
  }

  // Copies from other buffers into one at rising, disjoint offsets share a
  // run too. One whose source the run has written keeps its barrier, and
  // copies what was written.
  {
    const VkDeviceSize half = kBytes / 2;
    vr::Result<vr::Buffer> c_result =
        vr::device_storage_buffer(allocator, kBytes);
    CHECK(c_result.ok());
    const vr::Buffer c = std::move(c_result).value();
    {
      vr::CommandBatch batch(device, allocator);
      CHECK(batch.upload(a, 0, p.data(), kBytes).ok());
      CHECK(batch.submit().ok());
    }
    vr::CommandBatch batch(device, allocator);
    CHECK(batch.fill(b, 0, kBytes, 9u).ok());
    CHECK(batch.copy(a, 0, c, 0, half / 2).ok());
    CHECK(batch.copy(a, half / 2, c, half / 2, half / 2).ok());
    CHECK(batch.copy(b, 0, c, half, half).ok());
    CHECK(batch.readback(c, 0, kBytes, got.data()).ok());
    CHECK(batch.submit().ok());
    for (std::uint32_t i = 0; i < kCount; ++i) {
      CHECK(got[i] == (i < kCount / 2 ? p[i] : 9u));
    }
  }

  // An indirect dispatch sized by a command in a buffer: two of four groups.
  {
    vr::Result<vr::Buffer> args_result =
        vr::device_storage_buffer(allocator, sizeof(VkDispatchIndirectCommand),
                                  VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT);
    CHECK(args_result.ok());
    const vr::Buffer args = std::move(args_result).value();
    const VkDispatchIndirectCommand command{2, 1, 1};
    add.set.write_storage_buffer(0, a.handle(), 0, VK_WHOLE_SIZE);
    const Push push{kCount, 10};
    vr::CommandBatch batch(device, allocator);
    CHECK(batch.upload(a, 0, p.data(), kBytes).ok());
    CHECK(batch.upload(args, 0, &command, sizeof(command)).ok());
    CHECK(batch.dispatch_indirect(add, &push, sizeof(push), args, 0).ok());
    CHECK(batch.readback(a, 0, kBytes, got.data()).ok());
    CHECK(batch.submit().ok());
    for (std::uint32_t i = 0; i < kCount; ++i) {
      CHECK(got[i] == (i < 128 ? p[i] + 10 : p[i]));
    }
    // Misaligned, too short, and without INDIRECT_BUFFER usage.
    vr::CommandBatch misaligned(device, allocator);
    CHECK(
        !misaligned.dispatch_indirect(add, &push, sizeof(push), args, 2).ok());
    vr::CommandBatch past(device, allocator);
    CHECK(!past.dispatch_indirect(add, &push, sizeof(push), args, 4).ok());
    vr::CommandBatch plain(device, allocator);
    CHECK(!plain.dispatch_indirect(add, &push, sizeof(push), a, 0).ok());
  }

  // An acquire takes a buffer over before what reads it. From outside Vulkan
  // it records the transfer, for an EXCLUSIVE buffer and a CONCURRENT one;
  // from this device's family, from none, or from another family into a
  // CONCURRENT buffer it records nothing. Either way the kernel after it
  // sees what an earlier batch wrote, and the layer stays silent.
  {
    const auto invalid = vr::Status::Code::InvalidArgument;
    const std::uint32_t own = device.compute_family();
    std::uint32_t family_count = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(device.physical_device(),
                                             &family_count, nullptr);
    const auto taken_over = [&](const vr::Buffer& buffer, std::uint32_t from) {
      std::vector<std::uint32_t> back(kCount, 0);
      vr::CommandBatch write(device, allocator);
      if (!write.upload(buffer, 0, p.data(), kBytes).ok()) return false;
      if (!write.submit().ok()) return false;
      vr::CommandBatch batch(device, allocator);
      return batch.acquire(buffer, from).ok() &&
             add_to(batch, rig, buffer, 2).ok() &&
             batch.readback(buffer, 0, kBytes, back.data()).ok() &&
             batch.submit().ok() && back == plus(p, 2);
    };
    CHECK(taken_over(a, VK_QUEUE_FAMILY_EXTERNAL));
    CHECK(taken_over(a, VK_QUEUE_FAMILY_IGNORED));
    CHECK(taken_over(a, own));
    if (family_count > 1) {
      const std::uint32_t other = own == 0 ? 1 : 0;
      const std::uint32_t both[2] = {own, other};
      vr::Result<vr::Buffer> shared =
          vr::device_storage_buffer(allocator, kBytes, 0, both, 2);
      CHECK(shared.ok());
      CHECK(shared.value().sharing_mode() == VK_SHARING_MODE_CONCURRENT);
      CHECK(taken_over(shared.value(), VK_QUEUE_FAMILY_EXTERNAL));
      CHECK(taken_over(shared.value(), other));
    }

    // Refused, poisoning the batch: an empty buffer, and a family the device
    // does not have.
    vr::CommandBatch empty(device, allocator);
    CHECK(empty.acquire(vr::Buffer(), own).domain() == invalid);
    CHECK(empty.submit().domain() == invalid);
    vr::CommandBatch stranger(device, allocator);
    CHECK(stranger.acquire(a, family_count).domain() == invalid);
    CHECK(stranger.submit().domain() == invalid);
  }

  // An image copies into a buffer, rows packed, from its corner, after a
  // fill before it and before a kernel after it: one R8 and one R8G8, each a
  // region short of the image, rising through the buffer, then the first
  // again back over the second, which keeps its barrier and lands last.
  // Refused, poisoning the batch: an empty image or one of another format or
  // a layout a copy cannot read, a region empty, past it or past 64 bits of
  // bytes, a misaligned offset, a range past the buffer, and an image without
  // TRANSFER_SRC.
  {
    const auto invalid = vr::Status::Code::InvalidArgument;
    std::vector<std::uint8_t> luma(7 * 5);
    std::vector<std::uint8_t> chroma(4 * 3 * 2);
    for (std::size_t i = 0; i < luma.size(); ++i) {
      luma[i] = static_cast<std::uint8_t>(3 * i + 1);
    }
    for (std::size_t i = 0; i < chroma.size(); ++i) {
      chroma[i] = static_cast<std::uint8_t>(200 - i);
    }
    auto y =
        test_image::make(device, allocator, VK_FORMAT_R8_UNORM, 7, 5, luma);
    auto c =
        test_image::make(device, allocator, VK_FORMAT_R8G8_UNORM, 4, 3, chroma);
    CHECK(y.ok() && c.ok());
    std::vector<std::uint8_t> want(kBytes, 1);  // the kernel adds 1 a byte
    for (std::size_t r = 0; r < 4; ++r) {
      for (std::size_t x = 0; x < 6; ++x) {
        want[r * 6 + x] = static_cast<std::uint8_t>(luma[r * 7 + x] + 1);
      }
    }
    for (std::size_t r = 0; r < 2; ++r) {
      for (std::size_t x = 0; x < 6; ++x) {
        want[24 + r * 6 + x] = static_cast<std::uint8_t>(chroma[r * 8 + x] + 1);
      }
    }
    want[24] = static_cast<std::uint8_t>(luma[0] + 1);
    want[25] = static_cast<std::uint8_t>(luma[1] + 1);
    std::vector<std::uint8_t> got(kBytes, 0);
    vr::CommandBatch batch(device, allocator);
    CHECK(batch.fill(a, 0, kBytes, 0u).ok());
    CHECK(batch.copy(y.value(), 6, 4, a, 0).ok());
    CHECK(batch.copy(c.value(), 3, 2, a, 24).ok());
    CHECK(batch.copy(y.value(), 2, 1, a, 24).ok());
    CHECK(add_to(batch, rig, a, 0x01010101u).ok());
    CHECK(batch.readback(a, 0, kBytes, got.data()).ok());
    CHECK(batch.submit().ok());
    CHECK(got == want);

    const auto refused = [&](auto&& record) {
      vr::CommandBatch refusing(device, allocator);
      return record(refusing).domain() == invalid &&
             refusing.submit().domain() == invalid;
    };
    auto rgba = test_image::make(device, allocator, VK_FORMAT_R8G8B8A8_UNORM, 1,
                                 1, {0, 0, 0, 0});
    auto unusable =
        test_image::make(device, allocator, VK_FORMAT_R8_UNORM, 1, 1, {0}, 0);
    CHECK(rgba.ok() && unusable.ok());
    // Refused before anything reaches Vulkan, so no image need exist.
    const auto fake = [](VkFormat format, std::uint32_t width,
                         std::uint32_t height, VkImageLayout layout) {
      return vr::Image(fake_image(), format, width, height,
                       VK_IMAGE_USAGE_TRANSFER_SRC_BIT, layout, nullptr);
    };
    CHECK(refused([&](vr::CommandBatch& b) {
      return b.copy(fake(VK_FORMAT_R8_UNORM, 1, 1, VK_IMAGE_LAYOUT_UNDEFINED),
                    1, 1, a, 0);
    }));
    CHECK(refused([&](vr::CommandBatch& b) {
      return b.copy(fake(VK_FORMAT_R8_UNORM, 1, 1,
                         VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL),
                    1, 1, a, 0);
    }));
    // 2^63 + 2 texels of two bytes, which wrap to 4 bytes.
    CHECK(refused([&](vr::CommandBatch& b) {
      const std::uint32_t w = 0xFFFE0002u, h = 0x80010001u;
      return b.copy(fake(VK_FORMAT_R8G8_UNORM, w, h, VK_IMAGE_LAYOUT_GENERAL),
                    w, h, a, 0);
    }));
    CHECK(refused(
        [&](vr::CommandBatch& b) { return b.copy(vr::Image(), 1, 1, a, 0); }));
    CHECK(refused(
        [&](vr::CommandBatch& b) { return b.copy(rgba.value(), 1, 1, a, 0); }));
    CHECK(refused(
        [&](vr::CommandBatch& b) { return b.copy(y.value(), 8, 5, a, 0); }));
    CHECK(refused(
        [&](vr::CommandBatch& b) { return b.copy(y.value(), 7, 0, a, 0); }));
    CHECK(refused(
        [&](vr::CommandBatch& b) { return b.copy(y.value(), 7, 5, a, 2); }));
    CHECK(refused([&](vr::CommandBatch& b) {
      return b.copy(y.value(), 7, 5, a, kBytes - 32);
    }));
    CHECK(refused([&](vr::CommandBatch& b) {
      return b.copy(unusable.value(), 1, 1, a, 0);
    }));
  }

  // Timed dispatches keep their spans: two in one submit, each resolved
  // into its own row. Timed uploads do too, inline, staged and reserved, and
  // a timed copy.
  {
    vr::Result<vr::GpuTimer> timer = vr::GpuTimer::create(device);
    CHECK(timer.ok());
    vr::StageMetrics metrics;
    {
      vr::GpuStageScope first(&metrics, timer.value(), "first");
      vr::GpuStageScope second(&metrics, timer.value(), "second");
      vr::GpuStageScope uploads(&metrics, timer.value(), "uploads");
      vr::CommandBatch batch(device, allocator);
      CHECK(batch.upload(a, 0, p.data(), kBytes, &uploads).ok());
      CHECK(batch.upload(b, 2, p.data(), 8, &uploads).ok());
      CHECK(batch.reserve_upload(b, 16, 8, &uploads).ok());
      CHECK(batch.copy(a, 0, b, 32, 8, &uploads).ok());
      add.set.write_storage_buffer(0, a.handle(), 0, VK_WHOLE_SIZE);
      const Push push{kCount, 1};
      CHECK(batch.dispatch(add, &push, sizeof(push), 4, rig.max_groups, &first)
                .ok());
      CHECK(batch.dispatch(add, &push, sizeof(push), 4, rig.max_groups, &second)
                .ok());
      CHECK(batch.submit().ok());
      if (timer.value().available()) CHECK(timer.value().count() == 6);
    }
    for (const char* name : {"first", "second", "uploads"}) {
      const vr::StageRow* row = nullptr;
      for (const vr::StageRow& r : metrics.rows()) {
        if (std::string(r.name) == name) row = &r;
      }
      CHECK(row != nullptr);
      CHECK(row->cpu_ms > 0.0);
      if (timer.value().available()) CHECK(row->has_gpu);
    }
  }

  // A set rewritten after its dispatch was recorded is refused at submit, so
  // the first dispatch never runs on the second's binding.
  {
    const std::vector<std::uint32_t> z(kCount, 0);
    vr::CommandBatch clear(device, allocator);
    CHECK(clear.upload(a, 0, z.data(), kBytes).ok());
    CHECK(clear.upload(b, 0, z.data(), kBytes).ok());
    CHECK(clear.submit().ok());

    vr::CommandBatch batch(device, allocator);
    CHECK(add_to(batch, rig, a, 1).ok());
    CHECK(add_to(batch, rig, b, 2).ok());
    CHECK(batch.submit().domain() == vr::Status::Code::InvalidArgument);

    std::vector<std::uint32_t> got_b(kCount, 1);
    vr::CommandBatch look(device, allocator);
    CHECK(look.readback(a, 0, kBytes, got.data()).ok());
    CHECK(look.readback(b, 0, kBytes, got_b.data()).ok());
    CHECK(look.submit().ok());
    CHECK(got == z);
    CHECK(got_b == z);
  }

  // One kernel dispatched twice in one batch over different buffers, each on
  // a set of its own; and an extra set rewritten after its dispatch is refused
  // as the kernel's own is.
  {
    CHECK(vr::allocate_kernel_sets(device, add, 0, 1).status().domain() ==
          vr::Status::Code::InvalidArgument);
    CHECK(vr::allocate_kernel_sets(device, add, 1, 0).status().domain() ==
          vr::Status::Code::InvalidArgument);
    CHECK(vr::allocate_kernel_sets(device, vr::ComputeKernel{}, 1, 1)
              .status()
              .domain() == vr::Status::Code::InvalidArgument);
    vr::Result<vr::KernelSets> sets =
        vr::allocate_kernel_sets(device, add, 1, 2);
    CHECK(sets.ok() && sets.value().sets.size() == 2);
    const vr::DescriptorSet& sa = sets.value().sets[0];
    const vr::DescriptorSet& sb = sets.value().sets[1];

    const std::vector<std::uint32_t> q = pattern(900);
    vr::CommandBatch fill(device, allocator);
    CHECK(fill.upload(a, 0, p.data(), kBytes).ok());
    CHECK(fill.upload(b, 0, q.data(), kBytes).ok());
    CHECK(fill.submit().ok());

    const Push one{kCount, 1};
    const Push two{kCount, 2};
    const std::uint32_t groups = vr::group_count(kCount, 64);
    sa.write_storage_buffer(0, a.handle(), 0, VK_WHOLE_SIZE);
    sb.write_storage_buffer(0, b.handle(), 0, VK_WHOLE_SIZE);
    std::vector<std::uint32_t> got_b(kCount, 0);
    vr::CommandBatch batch(device, allocator);
    CHECK(batch.dispatch(add, sa, &one, sizeof(one), groups, rig.max_groups)
              .ok());
    CHECK(batch.dispatch(add, sb, &two, sizeof(two), groups, rig.max_groups)
              .ok());
    // The first set again, after the second: the same buffer twice in order.
    CHECK(batch.dispatch(add, sa, &two, sizeof(two), groups, rig.max_groups)
              .ok());
    CHECK(batch.readback(a, 0, kBytes, got.data()).ok());
    CHECK(batch.readback(b, 0, kBytes, got_b.data()).ok());
    CHECK(batch.submit().ok());
    CHECK(got == plus(p, 3));
    CHECK(got_b == plus(q, 2));

    vr::CommandBatch rewritten(device, allocator);
    CHECK(rewritten.dispatch(add, sb, &one, sizeof(one), groups, rig.max_groups)
              .ok());
    sb.write_storage_buffer(0, a.handle(), 0, VK_WHOLE_SIZE);
    CHECK(rewritten.submit().domain() == vr::Status::Code::InvalidArgument);

    vr::CommandBatch empty_set(device, allocator);
    CHECK(empty_set
              .dispatch(add, vr::DescriptorSet{}, &one, sizeof(one), groups,
                        rig.max_groups)
              .domain() == vr::Status::Code::InvalidArgument);
  }

  // A refusal poisons its batch: nothing it recorded runs.
  {
    const std::vector<std::uint32_t> z(kCount, 0);
    vr::CommandBatch clear(device, allocator);
    CHECK(clear.upload(a, 0, z.data(), kBytes).ok());
    CHECK(clear.submit().ok());

    vr::CommandBatch batch(device, allocator);
    CHECK(batch.upload(a, 0, p.data(), kBytes).ok());
    CHECK(batch.upload(a, 4, p.data(), kBytes).domain() ==
          vr::Status::Code::InvalidArgument);  // past the end
    CHECK(batch.fill(a, 0, 4, 0).domain() ==
          vr::Status::Code::InvalidArgument);  // poisoned
    CHECK(batch.submit().domain() == vr::Status::Code::InvalidArgument);
    CHECK(batch.submitted());
    CHECK(batch.submit().domain() == vr::Status::Code::InvalidArgument);

    vr::CommandBatch look(device, allocator);
    CHECK(look.readback(a, 0, kBytes, got.data()).ok());
    CHECK(look.submit().ok());
    for (std::uint32_t v : got) CHECK(v == 0);  // the first upload never ran
  }

  // The other refusals, each on a batch of its own.
  {
    vr::BufferDesc desc;
    desc.size = kBytes;
    desc.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
    desc.memory = vr::MemoryUsage::DeviceLocal;
    vr::Result<vr::Buffer> bare_result = allocator.create_buffer(desc);
    CHECK(bare_result.ok());
    const vr::Buffer bare = std::move(bare_result).value();
    // Host-visible without the transfer bits: refused all the same, since
    // nothing is ever written through its mapping.
    vr::Result<vr::Buffer> mapped_result =
        vr::storage_buffer(allocator, kBytes);
    CHECK(mapped_result.ok());
    const vr::Buffer mapped_bare = std::move(mapped_result).value();
    const auto refused = [&](auto&& call) {
      vr::CommandBatch batch(device, allocator);
      return call(batch).domain() == vr::Status::Code::InvalidArgument;
    };
    // Usage.
    CHECK(refused(
        [&](vr::CommandBatch& c) { return c.upload(bare, 0, p.data(), 4); }));
    CHECK(refused([&](vr::CommandBatch& c) {
      return c.reserve_upload(bare, 0, 4).status();
    }));
    CHECK(refused([&](vr::CommandBatch& c) {
      return c.upload(mapped_bare, 0, p.data(), 4);
    }));
    CHECK(refused([&](vr::CommandBatch& c) {
      return c.readback(bare, 0, 4, got.data());
    }));
    CHECK(refused([&](vr::CommandBatch& c) {
      return c.readback(mapped_bare, 0, 4, got.data());
    }));
    CHECK(refused([&](vr::CommandBatch& c) { return c.fill(bare, 0, 4, 0); }));
    CHECK(
        refused([&](vr::CommandBatch& c) { return c.copy(a, 0, bare, 0, 4); }));
    CHECK(
        refused([&](vr::CommandBatch& c) { return c.copy(bare, 0, a, 0, 4); }));
    // Ranges, alignment, overlap, nulls.
    CHECK(refused(
        [&](vr::CommandBatch& c) { return c.upload(a, kBytes, p.data(), 4); }));
    CHECK(refused(
        [&](vr::CommandBatch& c) { return c.upload(a, 0, nullptr, 4); }));
    CHECK(refused([&](vr::CommandBatch& c) {
      return c.reserve_upload(a, kBytes - 2, 4).status();
    }));
    CHECK(refused([&](vr::CommandBatch& c) {
      return c.reserve_upload(a, 0, 0).status();
    }));
    CHECK(refused([&](vr::CommandBatch& c) { return c.fill(a, 2, 4, 0); }));
    CHECK(refused([&](vr::CommandBatch& c) { return c.fill(a, 0, 6, 0); }));
    CHECK(
        refused([&](vr::CommandBatch& c) { return c.zero(a, kBytes - 2, 4); }));
    CHECK(refused([&](vr::CommandBatch& c) { return c.zero(bare, 0, 4); }));
    CHECK(refused([&](vr::CommandBatch& c) { return c.copy(a, 0, a, 8, 16); }));
    CHECK(refused([&](vr::CommandBatch& c) {
      return c.readback(a, kBytes - 2, 4, got.data());
    }));
    CHECK(refused(
        [&](vr::CommandBatch& c) { return c.readback(a, 0, 4, nullptr); }));
    if (rig.max_groups < 0xFFFFFFFFu) {
      CHECK(refused([&](vr::CommandBatch& c) {
        const Push push{kCount, 0};
        return c.dispatch(add, &push, sizeof(push), rig.max_groups + 1,
                          rig.max_groups);
      }));
    }
    CHECK(refused([&](vr::CommandBatch& c) {
      return c.dispatch(add, nullptr, sizeof(Push), 1, rig.max_groups);
    }));
    // A push off 4 bytes, or past the kernel's range.
    const std::uint32_t words[3] = {kCount, 0, 0};
    CHECK(refused([&](vr::CommandBatch& c) {
      return c.dispatch(add, words, 6, 1, rig.max_groups);
    }));
    CHECK(refused([&](vr::CommandBatch& c) {
      return c.dispatch(add, words, sizeof(words), 1, rig.max_groups);
    }));
    // Nothing at all is fine, and submits nothing.
    vr::CommandBatch empty(device, allocator);
    CHECK(empty.upload(a, 0, nullptr, 0).ok());
    CHECK(empty.readback(a, 0, 0, nullptr).ok());
    CHECK(empty.submit().ok());
  }

  // A batch with no allocator dispatches and uploads inline, and refuses
  // what would need staging.
  {
    vr::CommandBatch batch(device);
    CHECK(batch.upload(a, 0, p.data(), kBytes).ok());
    CHECK(batch.upload(a, 2, p.data(), 4).domain() ==
          vr::Status::Code::InvalidArgument);
    vr::CommandBatch reserves(device);
    CHECK(reserves.reserve_upload(a, 0, 4).status().domain() ==
          vr::Status::Code::InvalidArgument);
    vr::CommandBatch reads(device);
    CHECK(reads.readback(a, 0, 4, got.data()).domain() ==
          vr::Status::Code::InvalidArgument);
  }

  // A moved-from device poisons the batch rather than reaching Vulkan.
  {
    vr::Result<vr::Device> other_result =
        vr::Device::create(instance.value(), gpu.value(), {});
    CHECK(other_result.ok());
    vr::Device other = std::move(other_result).value();
    const vr::Device taken(std::move(other));
    vr::CommandBatch batch(other,
                           allocator);  // NOLINT(bugprone-use-after-move)
    CHECK(batch.fill(a, 0, 4, 0).domain() == vr::Status::Code::InvalidArgument);
    CHECK(batch.submit().domain() == vr::Status::Code::InvalidArgument);
  }

  // Moves: the recorded commands go with the batch, and the source is empty.
  {
    std::uint32_t word = 0;
    vr::CommandBatch source(device, allocator);
    CHECK(source.fill(a, 0, 4, 41u).ok());
    CHECK(source.readback(a, 0, 4, &word).ok());
    vr::CommandBatch moved(std::move(source));
    CHECK(!source.submitted());  // NOLINT(bugprone-use-after-move)
    CHECK(source.fill(a, 0, 4, 0).domain() ==
          vr::Status::Code::InvalidArgument);
    CHECK(source.submit().domain() == vr::Status::Code::InvalidArgument);
    CHECK(moved.submit().ok());
    CHECK(word == 41u);

    // Over a live batch, whose own commands are dropped.
    vr::CommandBatch next(device, allocator);
    CHECK(next.fill(a, 0, 4, 42u).ok());
    CHECK(next.readback(a, 0, 4, &word).ok());
    vr::CommandBatch live(device, allocator);
    CHECK(live.fill(a, 0, 4, 7u).ok());
    live = std::move(next);
    CHECK(next.submit().domain() ==  // NOLINT(bugprone-use-after-move)
          vr::Status::Code::InvalidArgument);
    CHECK(live.submit().ok());
    CHECK(word == 42u);

    // Self-move keeps the batch.
    vr::CommandBatch self(device, allocator);
    CHECK(self.fill(a, 0, 4, 43u).ok());
    CHECK(self.readback(a, 0, 4, &word).ok());
    vr::CommandBatch* alias = &self;
    self = std::move(*alias);
    CHECK(self.submit().ok());
    CHECK(word == 43u);
  }

  // A `record` that throws gives its buffer back reset: the next submit begins
  // the same one, which the layer refuses while it is still recording.
  {
    bool threw = false;
    try {
      static_cast<void>(device.submit_single_time(
          [](VkCommandBuffer) { throw std::runtime_error("record"); }));
    } catch (const std::runtime_error&) {
      threw = true;
    }
    CHECK(threw);
    CHECK(device.submit_single_time([](VkCommandBuffer) {}).ok());
  }

  // Several threads batch on one device at once, each with its own kernel,
  // buffer and timer: an inline upload, a timed one staged past the inline
  // limit, a dispatch and two readbacks a round. On the created device, and on
  // one adopted with no embedder mutex -- fuse_viewer's layout -- whose queue
  // the device must lock itself. The layer reports a race as an error; where
  // it is not installed, only the results are checked.
  {
    constexpr int kThreads = 4;
    constexpr int kRounds = 50;
    constexpr VkDeviceSize kStaged = vr::CommandBatch::kMaxInlineUpload * 2;
    std::vector<vr::ComputeKernel> kernels(kThreads);
    vr::KernelSetBuilder per_thread(device);
    for (vr::ComputeKernel& k : kernels) {
      CHECK(per_thread
                .add(k, "test_add",
                     reinterpret_cast<const unsigned char*>(spv.data()),
                     spv.size() * sizeof(std::uint32_t), 1, &range)
                .ok());
    }
    vr::Result<vr::DescriptorPool> thread_pool = per_thread.build();
    CHECK(thread_pool.ok());
    const auto run_threads = [&](vr::Device& on) {
      std::atomic<int> failed{0};
      std::vector<std::thread> threads;
      for (int t = 0; t < kThreads; ++t) {
        threads.emplace_back([&, t] {
          const auto fail = [&](int round, const std::string& why) {
            std::fprintf(stderr, "FAIL thread %d round %d: %s\n", t, round,
                         why.c_str());
            ++failed;
          };
          vr::Result<vr::Buffer> made =
              vr::device_storage_buffer(allocator, kBytes + kStaged);
          vr::Result<vr::GpuTimer> timer = vr::GpuTimer::create(on);
          if (!made.ok() || !timer.ok()) {
            fail(-1, !made.ok() ? made.status().message()
                                : timer.status().message());
            return;
          }
          const vr::Buffer& mine = made.value();
          const Rig own{&on, &allocator, &kernels[t], rig.max_groups};
          const std::vector<std::uint32_t> staged =
              pattern(7u * static_cast<std::uint32_t>(t), kStaged / 4);
          for (int r = 0; r < kRounds; ++r) {
            const std::vector<std::uint32_t> in =
                pattern(1000u * static_cast<std::uint32_t>(t) +
                        static_cast<std::uint32_t>(r));
            std::vector<std::uint32_t> out(kCount, 0);
            std::uint32_t last = 0;
            vr::StageMetrics metrics;
            vr::Status submitted;
            {
              vr::GpuStageScope stage(&metrics, timer.value(), "staged");
              // A failed call poisons the batch, so submit returns the first
              // refusal.
              vr::CommandBatch batch(on, allocator);
              batch.upload(mine, 0, in.data(), kBytes);
              batch.upload(mine, kBytes, staged.data(), kStaged, &stage);
              add_to(batch, own, mine, 3);
              batch.readback(mine, 0, kBytes, out.data());
              batch.readback(mine, kBytes + kStaged - 4, 4, &last);
              submitted = batch.submit();
            }
            if (!submitted.ok()) {
              fail(r, submitted.message());
            } else if (out != plus(in, 3) || last != staged.back()) {
              fail(r, "wrong result");
            } else if (timer.value().available() &&
                       (metrics.rows().empty() ||
                        !metrics.rows().front().has_gpu)) {
              fail(r, "no device time");
            }
          }
        });
      }
      for (std::thread& t : threads) t.join();
      return failed == 0;
    };
    CHECK(run_threads(device));

    vr::AdoptedDevice adopted;
    adopted.instance = instance.value().handle();
    adopted.physical_device = device.physical_device();
    adopted.device = device.handle();
    adopted.compute_family = device.compute_family();
    adopted.compute_queue = device.compute_queue();
    adopted.enabled_timeline_semaphore = true;
    adopted.enabled_scalar_block_layout = true;
    vr::Result<vr::Device> borrowed = vr::Device::adopt(adopted, {});
    CHECK(borrowed.ok());
    CHECK(run_threads(borrowed.value()));
  }

  CHECK(test_image_moves() == 0);
  CHECK(g_errors == 0);
  std::printf("core command batch: OK\n");
  return 0;
}
