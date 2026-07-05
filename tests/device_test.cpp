// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// Exercises the Vulkan device create/adopt seam: standalone create (owns the
// VkDevice) and non-owning adopt (borrows one an embedder made, the shared-
// device interop case). Skips (exit 0) where no Vulkan driver or compute device
// is present, so headless CI stays green; the pure checks always run.

#include <cstdio>

#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/instance.hpp"

namespace vr = volumetric_kit::recon;

int main() {
  int failures = 0;
  auto check = [&](bool cond, const char* what) {
    if (!cond) {
      std::fprintf(stderr, "FAIL: %s\n", what);
      ++failures;
    }
  };

  // requirements() is pure -- no device needed.
  const vr::DeviceRequirements reqs = vr::Device::requirements({});
  check(reqs.api_version == VK_API_VERSION_1_2, "requirements target 1.2");
  check((reqs.queue_flags & VK_QUEUE_COMPUTE_BIT) != 0, "requirements compute");
  check((reqs.queue_flags & VK_QUEUE_TRANSFER_BIT) != 0,
        "requirements transfer");
  check(reqs.timeline_semaphore, "requirements timeline semaphore");

  // adopt rejects null handles without touching a device.
  vr::Result<vr::Device> bad = vr::Device::adopt(vr::AdoptedDevice{}, {});
  check(!bad.ok() && bad.status().domain() == vr::Status::Code::InvalidArgument,
        "adopt rejects null handles");

  // The live path needs a driver + a compute device; skip gracefully otherwise.
  vr::Result<vr::Instance> instance = vr::Instance::create({});
  if (!instance) {
    std::fprintf(stderr,
                 "no Vulkan instance (%s); skipping live device tests\n",
                 instance.status().message().c_str());
    return failures == 0 ? 0 : 1;
  }
  vr::Result<VkPhysicalDevice> physical =
      instance.value().select_physical_device();
  if (!physical) {
    std::fprintf(stderr, "no compute device (%s); skipping\n",
                 physical.status().message().c_str());
    return failures == 0 ? 0 : 1;
  }

  // Standalone create: owns the device.
  vr::Result<vr::Device> owner =
      vr::Device::create(instance.value().handle(), physical.value(), {});
  if (!owner) {
    std::fprintf(stderr, "Device::create failed: %s\n",
                 owner.status().message().c_str());
    return 1;
  }
  check(owner.value().owns_device(), "created device is owned");
  check(owner.value().handle() != VK_NULL_HANDLE, "device handle non-null");
  check(owner.value().compute_queue() != VK_NULL_HANDLE, "compute queue");
  check(owner.value().command_pool() != VK_NULL_HANDLE, "command pool");

  // Adopt the owner's device by raw handles (default config needs no
  // extensions, so no enabled-list declaration is required).
  vr::AdoptedDevice adopted;
  adopted.instance = instance.value().handle();
  adopted.physical_device = physical.value();
  adopted.device = owner.value().handle();
  adopted.compute_family = owner.value().compute_family();
  adopted.compute_queue = owner.value().compute_queue();
  {
    vr::Result<vr::Device> borrowed = vr::Device::adopt(adopted, {});
    if (!borrowed) {
      std::fprintf(stderr, "Device::adopt failed: %s\n",
                   borrowed.status().message().c_str());
      return 1;
    }
    check(!borrowed.value().owns_device(), "adopted device is not owned");
    check(borrowed.value().handle() == owner.value().handle(),
          "adopt shares the device handle");
    check(borrowed.value().command_pool() != VK_NULL_HANDLE,
          "adopt owns its own command pool");
    check(borrowed.value().command_pool() != owner.value().command_pool(),
          "adopt's pool is distinct from the owner's");
  }  // borrowed destructs here: it must NOT destroy the shared VkDevice.

  // The owner's device is still valid after the borrower is gone (a double-free
  // trips the sanitizer job; a use-after-free would corrupt this handle).
  check(owner.value().handle() != VK_NULL_HANDLE,
        "owner survives the adopted wrapper's teardown");

  if (failures == 0) {
    std::puts("recon device create/adopt test passed");
    return 0;
  }
  std::fprintf(stderr, "%d checks failed\n", failures);
  return 1;
}
