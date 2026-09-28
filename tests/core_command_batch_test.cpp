// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// CommandBatch and StagingArena.
//
// Every case runs over the two buffers the host reaches differently:
// device-local (always staged) and host-visible (always mapped). The mapped one
// takes the direct shortcuts, so the ordering cases -- an upload after a
// dispatch, a readback before one -- are the ones that would catch a shortcut
// taken where it changes the result, on every platform. Skips (exit 0) where no
// device is present.

#include <cstdint>
#include <cstdio>
#include <fstream>
#include <string>
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

std::vector<std::uint32_t> pattern(std::uint32_t base) {
  std::vector<std::uint32_t> v(kCount);
  for (std::uint32_t i = 0; i < kCount; ++i) v[i] = base + i * 7;
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

// Bind `buffer` and record `add` over all of it.
vr::Status add_to(vr::CommandBatch& batch, const Rig& rig,
                  const vr::Buffer& buffer, std::uint32_t delta) {
  rig.add->set.write_storage_buffer(0, buffer.handle(), 0, VK_WHOLE_SIZE);
  const Push push{kCount, delta};
  return batch.dispatch(*rig.add, &push, sizeof(push),
                        vr::group_count(kCount, 64), rig.max_groups);
}

// The two kinds of buffer, with the usage every batch call needs.
vr::Result<vr::Buffer> make(vr::Allocator& a, int kind) {
  if (kind == 0) return vr::device_storage_buffer(a, kBytes);
  return vr::storage_buffer(
      a, kBytes, vr::HostAccess::Random,
      VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);
}

const char* kKindName[] = {"device-local", "host-visible"};

int run_kind(const Rig& rig, int kind) {
  // An arena of its own, so what it stages is this kind's alone.
  vr::Result<vr::StagingArena> arena = vr::StagingArena::create(*rig.allocator);
  CHECK(arena.ok());
  vr::StagingArena staging = std::move(arena).value();
  vr::Result<vr::Buffer> made = make(*rig.allocator, kind);
  CHECK(made.ok());
  vr::Buffer buffer = std::move(made).value();
  CHECK((buffer.mapped() != nullptr) == (kind == 1));
  std::printf("  %s: %s\n", kKindName[kind],
              buffer.mapped() != nullptr ? "mapped" : "staged");

  // Round trip. A mapped buffer needs no staging and no submit.
  const std::vector<std::uint32_t> p = pattern(100);
  std::vector<std::uint32_t> got(kCount, 0);
  {
    vr::CommandBatch batch(*rig.device, staging);
    CHECK(batch.upload(buffer, 0, p.data(), kBytes).ok());
    CHECK(batch.readback(buffer, 0, kBytes, got.data()).ok());
    CHECK(batch.submit().ok());
  }
  CHECK(got == p);
  if (buffer.mapped() != nullptr) {
    CHECK(staging.upload_capacity() == 0 && staging.readback_capacity() == 0);
  }

  // Upload, dispatch, readback: one submit, in order.
  {
    vr::CommandBatch batch(*rig.device, staging);
    CHECK(batch.upload(buffer, 0, p.data(), kBytes).ok());
    CHECK(add_to(batch, rig, buffer, 5).ok());
    CHECK(batch.readback(buffer, 0, kBytes, got.data()).ok());
    CHECK(batch.submit().ok());
  }
  CHECK(got == plus(p, 5));

  // An upload after a command is ordered after it: the dispatch must not add
  // onto the uploaded values, as it would if the upload were written at once.
  const std::vector<std::uint32_t> q = pattern(9000);
  {
    vr::CommandBatch batch(*rig.device, staging);
    CHECK(add_to(batch, rig, buffer, 1).ok());
    CHECK(batch.upload(buffer, 0, q.data(), kBytes).ok());
    CHECK(batch.readback(buffer, 0, kBytes, got.data()).ok());
    CHECK(batch.submit().ok());
  }
  CHECK(got == q);

  // A readback before a command reads what stood before it, not after.
  std::vector<std::uint32_t> before(kCount, 0);
  std::vector<std::uint32_t> after(kCount, 0);
  {
    vr::CommandBatch batch(*rig.device, staging);
    CHECK(batch.readback(buffer, 0, kBytes, before.data()).ok());
    CHECK(add_to(batch, rig, buffer, 3).ok());
    CHECK(batch.readback(buffer, 0, kBytes, after.data()).ok());
    CHECK(batch.submit().ok());
  }
  CHECK(before == q);
  CHECK(after == plus(q, 3));

  // Offsets: a fill and a partial upload, read back at an offset.
  {
    vr::CommandBatch batch(*rig.device, staging);
    CHECK(batch.fill(buffer, 0, kBytes, 0xABCDu).ok());
    CHECK(batch.upload(buffer, 16, p.data(), 32).ok());
    CHECK(batch.readback(buffer, 8, 48, got.data()).ok());
    CHECK(batch.submit().ok());
  }
  CHECK(got[0] == 0xABCDu && got[1] == 0xABCDu);
  for (int i = 0; i < 8; ++i) CHECK(got[2 + i] == p[i]);
  CHECK(got[10] == 0xABCDu && got[11] == 0xABCDu);
  return 0;
}

}  // namespace

int main() {
  vr::Result<vr::Instance> instance = vr::Instance::create({});
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

  vr::Result<vr::StagingArena> staging_result =
      vr::StagingArena::create(allocator);
  CHECK(staging_result.ok());
  vr::StagingArena staging = std::move(staging_result).value();
  CHECK(staging.valid());
  CHECK(staging.upload_capacity() == 0);

  std::printf("buffers:\n");
  for (int kind = 0; kind < 2; ++kind) {
    if (run_kind(rig, kind) != 0) return 1;
  }

  vr::Result<vr::Buffer> a_result =
      vr::device_storage_buffer(allocator, kBytes);
  vr::Result<vr::Buffer> b_result =
      vr::device_storage_buffer(allocator, kBytes);
  CHECK(a_result.ok() && b_result.ok());
  vr::Buffer a = std::move(a_result).value();
  vr::Buffer b = std::move(b_result).value();
  const std::vector<std::uint32_t> p = pattern(1);
  std::vector<std::uint32_t> got(kCount, 0);

  // Device-to-device copy, and within one buffer where the ranges are apart.
  {
    vr::CommandBatch batch(device, staging);
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
    vr::Buffer args = std::move(args_result).value();
    const VkDispatchIndirectCommand command{2, 1, 1};
    add.set.write_storage_buffer(0, a.handle(), 0, VK_WHOLE_SIZE);
    const Push push{kCount, 10};
    vr::CommandBatch batch(device, staging);
    CHECK(batch.upload(a, 0, p.data(), kBytes).ok());
    CHECK(batch.upload(args, 0, &command, sizeof(command)).ok());
    CHECK(batch.dispatch_indirect(add, &push, sizeof(push), args, 0).ok());
    CHECK(batch.readback(a, 0, kBytes, got.data()).ok());
    CHECK(batch.submit().ok());
    for (std::uint32_t i = 0; i < kCount; ++i) {
      CHECK(got[i] == (i < 128 ? p[i] + 10 : p[i]));
    }
    // Misaligned, too short, and without INDIRECT_BUFFER usage.
    vr::CommandBatch refused(device, staging);
    CHECK(!refused.dispatch_indirect(add, &push, sizeof(push), args, 2).ok());
    vr::CommandBatch past(device, staging);
    CHECK(!past.dispatch_indirect(add, &push, sizeof(push), args, 4).ok());
    vr::CommandBatch plain(device, staging);
    CHECK(!plain.dispatch_indirect(add, &push, sizeof(push), a, 0).ok());
  }

  // Timed dispatches keep their spans: two in one submit, one row.
  {
    vr::Result<vr::GpuTimer> timer = vr::GpuTimer::create(device);
    CHECK(timer.ok());
    vr::StageMetrics metrics;
    {
      vr::GpuStageScope stage(&metrics, timer.value(), "batch");
      vr::CommandBatch batch(device, staging);
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

  // Refusals, each poisoning its batch: nothing it recorded runs.
  {
    const std::vector<std::uint32_t> z(kCount, 0);
    vr::CommandBatch clear(device, staging);
    CHECK(clear.upload(a, 0, z.data(), kBytes).ok());
    CHECK(clear.submit().ok());  // releases the arena for the next batch

    vr::CommandBatch batch(device, staging);
    CHECK(batch.upload(a, 0, p.data(), kBytes).ok());  // staged: not yet run
    const vr::Status past = batch.upload(a, 4, p.data(), kBytes);
    CHECK(past.domain() == vr::Status::Code::InvalidArgument);
    CHECK(batch.fill(a, 0, 4, 0).domain() ==
          vr::Status::Code::InvalidArgument);  // poisoned
    CHECK(batch.submit().domain() == vr::Status::Code::InvalidArgument);
    CHECK(batch.submitted());
    CHECK(batch.submit().domain() == vr::Status::Code::InvalidArgument);
  }
  {
    vr::CommandBatch batch(device, staging);
    CHECK(batch.readback(a, 0, kBytes, got.data()).ok());
    CHECK(batch.submit().ok());
    for (std::uint32_t v : got) CHECK(v == 0);  // the poisoned upload never ran
  }
  {
    vr::BufferDesc desc;
    desc.size = kBytes;
    desc.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
    desc.memory = vr::MemoryUsage::DeviceLocal;
    vr::Result<vr::Buffer> bare_result = allocator.create_buffer(desc);
    CHECK(bare_result.ok());
    const vr::Buffer bare = std::move(bare_result).value();
    const auto refused = [&](auto&& call) {
      vr::CommandBatch batch(device, staging);
      return call(batch).domain() == vr::Status::Code::InvalidArgument;
    };
    // Usage checked as if staged, whatever the path would have been.
    CHECK(refused(
        [&](vr::CommandBatch& c) { return c.upload(bare, 0, p.data(), 4); }));
    CHECK(refused([&](vr::CommandBatch& c) {
      return c.readback(bare, 0, 4, got.data());
    }));
    CHECK(refused([&](vr::CommandBatch& c) { return c.fill(bare, 0, 4, 0); }));
    // Mapped, so the direct path would work -- and is refused all the same,
    // or the bit would go missing until the buffer moved to device memory.
    vr::Result<vr::Buffer> mapped_result =
        vr::storage_buffer(allocator, kBytes);
    CHECK(mapped_result.ok());
    const vr::Buffer mapped_bare = std::move(mapped_result).value();
    CHECK(mapped_bare.mapped() != nullptr);
    CHECK(refused([&](vr::CommandBatch& c) {
      return c.upload(mapped_bare, 0, p.data(), 4);
    }));
    CHECK(refused([&](vr::CommandBatch& c) {
      return c.readback(mapped_bare, 0, 4, got.data());
    }));
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
    // Nothing at all is fine.
    vr::CommandBatch empty(device, staging);
    CHECK(empty.upload(a, 0, nullptr, 0).ok());
    CHECK(empty.submit().ok());
  }

  // One batch per arena at a time.
  {
    vr::CommandBatch first(device, staging);
    vr::CommandBatch second(device, staging);
    CHECK(second.upload(a, 0, p.data(), 4).domain() ==
          vr::Status::Code::InvalidArgument);
    CHECK(first.upload(a, 0, p.data(), 4).ok());
    CHECK(first.submit().ok());
  }
  {
    vr::CommandBatch again(device, staging);  // released by `first`
    CHECK(again.submit().ok());
  }

  // Growth: a batch needing several chunks leaves one chunk holding all of
  // them, so the same traffic next time allocates nothing.
  {
    vr::Result<vr::StagingArena> fresh_result =
        vr::StagingArena::create(allocator);
    CHECK(fresh_result.ok());
    vr::StagingArena fresh = std::move(fresh_result).value();
    vr::Result<vr::Buffer> big_result =
        vr::device_storage_buffer(allocator, 256 * 1024);
    CHECK(big_result.ok());
    const vr::Buffer big = std::move(big_result).value();
    std::vector<std::uint32_t> data(256 * 1024 / 4);
    for (std::size_t i = 0; i < data.size(); ++i) {
      data[i] = static_cast<std::uint32_t>(i * 2654435761u);
    }
    const auto upload_in_quarters = [&]() -> int {
      vr::CommandBatch batch(device, fresh);
      for (int q = 0; q < 4; ++q) {
        CHECK(batch
                  .upload(big, q * 64 * 1024, data.data() + q * 16 * 1024,
                          64 * 1024)
                  .ok());
      }
      std::vector<std::uint32_t> back(data.size());
      CHECK(batch.readback(big, 0, 256 * 1024, back.data()).ok());
      CHECK(batch.submit().ok());
      CHECK(back == data);
      return 0;
    };
    if (upload_in_quarters() != 0) return 1;
    if (upload_in_quarters() != 0) return 1;
    const VkDeviceSize settled = fresh.upload_capacity();
    CHECK(settled >= 256 * 1024);
    if (upload_in_quarters() != 0) return 1;
    CHECK(fresh.upload_capacity() == settled);
  }

  // The arena's moves leave the source empty.
  {
    vr::Result<vr::StagingArena> made = vr::StagingArena::create(allocator);
    CHECK(made.ok());
    vr::StagingArena source = std::move(made).value();
    {
      vr::CommandBatch batch(device, source);
      CHECK(batch.upload(a, 0, p.data(), kBytes).ok());
      CHECK(batch.submit().ok());
    }
    CHECK(source.upload_capacity() > 0);
    vr::StagingArena moved(std::move(source));
    CHECK(!source.valid());  // NOLINT(bugprone-use-after-move)
    CHECK(source.upload_capacity() == 0);
    CHECK(moved.valid() && moved.upload_capacity() > 0);
    vr::Result<vr::StagingArena> other = vr::StagingArena::create(allocator);
    CHECK(other.ok());
    vr::StagingArena target = std::move(other).value();
    target = std::move(moved);
    CHECK(!moved.valid());  // NOLINT(bugprone-use-after-move)
    CHECK(target.valid() && target.upload_capacity() > 0);
    vr::StagingArena* self = &target;
    target = std::move(*self);
    CHECK(target.valid() && target.upload_capacity() > 0);
    vr::CommandBatch refused(device, moved);  // moved-from arena
    CHECK(refused.upload(a, 0, p.data(), 4).domain() ==
          vr::Status::Code::InvalidArgument);
    CHECK(vr::StagingArena::create(allocator).ok());
  }

  std::printf("core command batch: OK\n");
  return 0;
}
