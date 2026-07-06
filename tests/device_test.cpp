// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// Exercises the Vulkan device create/adopt seam: standalone create (owns the
// VkDevice) and non-owning adopt (borrows one an embedder made, the shared-
// device interop case), plus the move-only semantics of Device and Instance.
// Skips (exit 0) where no Vulkan driver or compute device is present, so
// headless CI stays green; the pure checks always run.

#include <cstdio>
#include <utility>

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
  // Transfer is implied by a compute queue, so it is deliberately NOT demanded
  // in the published queue_flags (an embedder must not require TRANSFER_BIT).
  check((reqs.queue_flags & VK_QUEUE_TRANSFER_BIT) == 0,
        "requirements does not over-demand transfer");
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

  // Adopt the owner's device by raw handles. The default config requires
  // timelineSemaphore + scalarBlockLayout (which Device::create enabled) and no
  // extensions, so the creator declares those features and no enabled-extension
  // list.
  vr::AdoptedDevice adopted;
  adopted.instance = instance.value().handle();
  adopted.physical_device = physical.value();
  adopted.device = owner.value().handle();
  adopted.compute_family = owner.value().compute_family();
  adopted.compute_queue = owner.value().compute_queue();
  adopted.enabled_timeline_semaphore = true;
  adopted.enabled_scalar_block_layout = true;
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

  // adopt rejects a device the creator did not declare timelineSemaphore on
  // (recon's default config requires it, and it cannot be queried back).
  {
    vr::AdoptedDevice no_timeline = adopted;
    no_timeline.enabled_timeline_semaphore = false;
    vr::Result<vr::Device> r = vr::Device::adopt(no_timeline, {});
    check(!r.ok() && r.status().domain() == vr::Status::Code::Unsupported,
          "adopt rejects a device without timelineSemaphore enabled");
  }

  // adopt likewise rejects a device the creator did not declare
  // scalarBlockLayout on (the recon compute-shader buffer ABI; also
  // un-queryable post-creation).
  {
    vr::AdoptedDevice no_scalar = adopted;
    no_scalar.enabled_scalar_block_layout = false;
    vr::Result<vr::Device> r = vr::Device::adopt(no_scalar, {});
    check(!r.ok() && r.status().domain() == vr::Status::Code::Unsupported,
          "adopt rejects a device without scalarBlockLayout enabled");
  }

  // adopt rejects a device missing a core feature recon's config requires but
  // the creator never declared enabled.
  {
    vr::DeviceConfig want_feature;
    want_feature.features.shaderInt64 = VK_TRUE;
    vr::Result<vr::Device> r = vr::Device::adopt(adopted, want_feature);
    check(!r.ok() && r.status().domain() == vr::Status::Code::Unsupported,
          "adopt rejects a device missing a required feature");
  }

  // --- Move-only semantics (CLAUDE.md requires these for every move-only
  // type). Device: move-construct empties the source; move-assign over a live
  // object frees the old resources and adopts the new; self-move is a no-op.
  // Under the sanitizer CI job these become real double-free / leak detectors.
  {
    vr::Result<vr::Device> a =
        vr::Device::create(instance.value().handle(), physical.value(), {});
    vr::Result<vr::Device> b =
        vr::Device::create(instance.value().handle(), physical.value(), {});
    if (!a || !b) {
      std::fprintf(stderr, "move-test device create failed\n");
      return 1;
    }
    const VkDevice a_handle = a.value().handle();
    const VkCommandPool a_pool = a.value().command_pool();
    vr::Device moved = std::move(a.value());
    check(moved.handle() == a_handle, "device move-ctor transfers the handle");
    check(moved.command_pool() == a_pool,
          "device move-ctor transfers the pool");
    check(moved.owns_device(), "device move-ctor transfers ownership");
    check(a.value().handle() == VK_NULL_HANDLE,
          "device move-ctor empties the source handle");
    check(a.value().command_pool() == VK_NULL_HANDLE,
          "device move-ctor empties the source pool");
    check(a.value().owns_device(),
          "device move-ctor resets source ownership to true");

    const VkDevice b_handle = b.value().handle();
    moved = std::move(b.value());  // frees a's resources, then adopts b's
    check(moved.handle() == b_handle, "device move-assign adopts the source");
    check(b.value().handle() == VK_NULL_HANDLE,
          "device move-assign empties the source");

    vr::Device* self =
        &moved;  // launder through a pointer to dodge -Wself-move
    moved = std::move(*self);
    check(moved.handle() == b_handle, "device self-move leaves it intact");
    check(moved.command_pool() != VK_NULL_HANDLE,
          "device self-move keeps the pool");
  }  // moved (b's device) destructs once; a and b hold emptied devices.

  // Instance: the same three cases on the second move-only type.
  {
    vr::Result<vr::Instance> ia = vr::Instance::create({});
    vr::Result<vr::Instance> ib = vr::Instance::create({});
    if (!ia || !ib) {
      std::fprintf(stderr, "move-test instance create failed\n");
      return 1;
    }
    const VkInstance ia_handle = ia.value().handle();
    vr::Instance imoved = std::move(ia.value());
    check(imoved.handle() == ia_handle, "instance move-ctor transfers handle");
    check(ia.value().handle() == VK_NULL_HANDLE,
          "instance move-ctor empties the source");

    const VkInstance ib_handle = ib.value().handle();
    imoved = std::move(ib.value());
    check(imoved.handle() == ib_handle, "instance move-assign adopts source");
    check(ib.value().handle() == VK_NULL_HANDLE,
          "instance move-assign empties the source");

    vr::Instance* iself = &imoved;  // launder to dodge -Wself-move
    imoved = std::move(*iself);
    check(imoved.handle() == ib_handle, "instance self-move leaves it intact");
  }

  if (failures == 0) {
    std::puts("recon device create/adopt test passed");
    return 0;
  }
  std::fprintf(stderr, "%d checks failed\n", failures);
  return 1;
}
