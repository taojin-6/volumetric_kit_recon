// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// CommandBatch: every recording call, in orders that would expose a command
// run out of place -- an upload after a dispatch, a readback before one -- over
// a device-local and a host-visible buffer, since a buffer's memory type must
// not change what a batch does. Uploads inline, staged and packed by the
// caller, several readbacks in one batch, transfers left unordered, the
// refusals, the moves, and timed dispatches and uploads. Skips (exit 0) where
// no device is present.

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <string>
#include <string_view>
#include <thread>
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

  // Timed dispatches keep their spans: two in one submit, each resolved
  // into its own row. Timed uploads do too, inline, staged and reserved.
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
      add.set.write_storage_buffer(0, a.handle(), 0, VK_WHOLE_SIZE);
      const Push push{kCount, 1};
      CHECK(batch.dispatch(add, &push, sizeof(push), 4, rig.max_groups, &first)
                .ok());
      CHECK(batch.dispatch(add, &push, sizeof(push), 4, rig.max_groups, &second)
                .ok());
      CHECK(batch.submit().ok());
      if (timer.value().available()) CHECK(timer.value().count() == 5);
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

  // Several threads batch on one device at once, each with its own kernel and
  // buffer. The device locks its command pool and queue, so every result is
  // right and the layer's thread-safety checks report nothing.
  {
    constexpr int kThreads = 4;
    constexpr int kRounds = 50;
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
    std::atomic<int> wrong{0};
    std::vector<std::thread> threads;
    for (int t = 0; t < kThreads; ++t) {
      threads.emplace_back([&, t] {
        vr::Result<vr::Buffer> made =
            vr::device_storage_buffer(allocator, kBytes);
        if (!made.ok()) {
          ++wrong;
          return;
        }
        const vr::Buffer& mine = made.value();
        const Rig own{&device, &allocator, &kernels[t], rig.max_groups};
        for (int r = 0; r < kRounds; ++r) {
          const std::vector<std::uint32_t> in =
              pattern(1000u * static_cast<std::uint32_t>(t) +
                      static_cast<std::uint32_t>(r));
          std::vector<std::uint32_t> out(kCount, 0);
          vr::CommandBatch batch(device, allocator);
          const bool ran = batch.upload(mine, 0, in.data(), kBytes).ok() &&
                           add_to(batch, own, mine, 3).ok() &&
                           batch.readback(mine, 0, kBytes, out.data()).ok() &&
                           batch.submit().ok();
          if (!ran || out != plus(in, 3)) ++wrong;
        }
      });
    }
    for (std::thread& t : threads) t.join();
    CHECK(wrong == 0);
  }

  CHECK(g_errors == 0);
  std::printf("core command batch: OK\n");
  return 0;
}
