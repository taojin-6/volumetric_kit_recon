// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// CommandBatch: every recording call, in orders that would expose a command
// run out of place -- an upload after a dispatch, a readback before one -- over
// a device-local and a host-visible buffer, since a buffer's memory type must
// not change what a batch does. Uploads inline and staged, several readbacks
// in one batch, the refusals, and timed dispatches. Skips (exit 0) where no
// device is present.

#include <cstdint>
#include <cstdio>
#include <fstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

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
int g_errors = 0;

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

  // Timed dispatches keep their spans: two in one submit, one row.
  {
    vr::Result<vr::GpuTimer> timer = vr::GpuTimer::create(device);
    CHECK(timer.ok());
    vr::StageMetrics metrics;
    {
      vr::GpuStageScope stage(&metrics, timer.value(), "batch");
      vr::CommandBatch batch(device, allocator);
      add.set.write_storage_buffer(0, a.handle(), 0, VK_WHOLE_SIZE);
      const Push push{kCount, 1};
      CHECK(batch.dispatch(add, &push, sizeof(push), 4, rig.max_groups, &stage)
                .ok());
      CHECK(batch.dispatch(add, &push, sizeof(push), 4, rig.max_groups, &stage)
                .ok());
      CHECK(batch.submit().ok());
    }
    const vr::StageRow* row = nullptr;
    for (const vr::StageRow& r : metrics.rows()) {
      if (std::string(r.name) == "batch") row = &r;
    }
    CHECK(row != nullptr);
    CHECK(row->cpu_ms > 0.0);
    if (timer.value().available()) CHECK(row->has_gpu);
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
    CHECK(refused([&](vr::CommandBatch& c) { return c.fill(a, 2, 4, 0); }));
    CHECK(refused([&](vr::CommandBatch& c) { return c.fill(a, 0, 6, 0); }));
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
    // Nothing at all is fine, and submits nothing.
    vr::CommandBatch empty(device, allocator);
    CHECK(empty.upload(a, 0, nullptr, 0).ok());
    CHECK(empty.readback(a, 0, 0, nullptr).ok());
    CHECK(empty.submit().ok());
  }

  CHECK(g_errors == 0);
  std::printf("core command batch: OK\n");
  return 0;
}
