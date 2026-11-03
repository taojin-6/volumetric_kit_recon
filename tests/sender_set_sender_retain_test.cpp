// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// What a sender's front batch reads stays alive with the batch
// (sender/set_sender.hpp, SetSender::send's `inputs`, which the front batch
// retains). The set's prep is a fill of a buffer the inputs own, as a set's
// pictures own what its prep reads, and the caller hands over the last
// reference, as the live examples do. Sent and waited for: the inputs are
// alive at the front batch's fence wait and let go by the time send returns,
// the batch still in the caller's hands. A fence wait that fails, which the
// test's vkWaitForFences makes return a driver's error without waiting: send
// returns the failure, the inputs outlive send and the caller's batch, and the
// device frees them once it has waited for the work, when it is destroyed.
// Linux and macOS, where a test can stand in for a Vulkan call. The device
// part skips with no device.

#include <dlfcn.h>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <utility>

#include "volumetric_kit/core/testing/gpu_test.hpp"
#include "volumetric_kit/core/testing/test_check.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/buffer.hpp"
#include "volumetric_kit/core/vulkan/command_batch.hpp"
#include "volumetric_kit/core/vulkan/compute_util.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/core/vulkan/vulkan.hpp"
#include "volumetric_kit/recon/core/device_requirements.hpp"
#include "volumetric_kit/recon/sender/set_sender.hpp"
#include "volumetric_kit/recon/tsdf/fuser.hpp"
#include "volumetric_kit/recon/tsdf/fusion_grid.hpp"

namespace {

// While a case watches its inputs: whether a fence wait came and whether the
// inputs were alive at the first; with `refuse_waits` every wait returns a
// driver's error without waiting.
thread_local const std::weak_ptr<volumetric_kit::core::Buffer>* watched =
    nullptr;
thread_local bool first_wait_seen = false;
thread_local bool alive_at_first_wait = false;
thread_local bool refuse_waits = false;

}  // namespace

extern "C" VKAPI_ATTR VkResult VKAPI_CALL
vkWaitForFences(VkDevice device, std::uint32_t count, const VkFence* fences,
                VkBool32 wait_all, std::uint64_t timeout) {
  static const auto real = reinterpret_cast<PFN_vkWaitForFences>(
      dlsym(RTLD_NEXT, "vkWaitForFences"));
  if (watched != nullptr && !first_wait_seen) {
    first_wait_seen = true;
    alive_at_first_wait = !watched->expired();
  }
  if (refuse_waits) return VK_ERROR_UNKNOWN;
  if (real == nullptr) std::abort();
  return real(device, count, fences, wait_all, timeout);
}

namespace {

namespace vr = volumetric_kit::recon;
namespace vkc = volumetric_kit::core;
namespace sender = volumetric_kit::recon::sender;

// Watches `inputs` from here, refusing every fence wait when `refuse`; stops
// both however the case leaves.
class Watch {
 public:
  Watch(const std::weak_ptr<vkc::Buffer>& inputs, bool refuse) {
    watched = &inputs;
    first_wait_seen = false;
    alive_at_first_wait = false;
    refuse_waits = refuse;
  }
  ~Watch() {
    watched = nullptr;
    refuse_waits = false;
  }
  Watch(const Watch&) = delete;
  Watch& operator=(const Watch&) = delete;
  Watch(Watch&&) = delete;
  Watch& operator=(Watch&&) = delete;
};

// What became of one set's inputs.
struct Outcome {
  vkc::Status status;                // send's
  bool alive_at_wait = false;        // at the front batch's fence wait
  bool alive_after_send = false;     // once send returned, its batch still here
  bool alive_after_batch = false;    // once the caller's batch was gone too
  std::weak_ptr<vkc::Buffer> alive;  // from then on
};

// One set sent on `device`, fused alone, with no frames: its prep a fill of
// the buffer its inputs own, recorded into its front batch. `refuse` fails
// every fence wait in send. What the work uses is freed only once the device
// is idle.
int send_one(vkc::Device& device, vkc::Allocator& allocator, bool refuse,
             Outcome* sent) {
  auto grid =
      vr::tsdf::create_fusion_grid(device, allocator, 0.02F, 0.08F, 1024);
  auto fuser = vr::tsdf::Fuser::create(device, allocator, {});
  CHECK(grid.ok() && fuser.ok());
  auto out = sender::SetSender::create(device, *fuser, *grid, nullptr, {});
  CHECK(out.ok());
  auto made = vkc::device_storage_buffer(allocator, 16);
  CHECK(made.ok());
  auto inputs = std::make_shared<vkc::Buffer>(std::move(made).value());
  sent->alive = inputs;
  {
    vkc::CommandBatch front(device, allocator);
    CHECK(front.fill(*inputs, 0, 16, 1U).ok());  // the set's prep
    {
      const Watch watch(sent->alive, refuse);
      sent->status = out->send(front, {}, std::move(inputs), 0, 0).status();
    }
    CHECK(first_wait_seen);
    sent->alive_at_wait = alive_at_first_wait;
    sent->alive_after_send = !sent->alive.expired();
  }
  sent->alive_after_batch = !sent->alive.expired();
  CHECK(device.wait_idle().ok());
  return 0;
}

// Sent and waited for: alive at the wait, let go by the time send returns.
int released_after_wait(vkc::test::GpuContext& gpu) {
  Outcome sent;
  CHECK(send_one(gpu.device, gpu.allocator, false, &sent) == 0);
  CHECK(sent.status.ok() && sent.alive_at_wait);
  CHECK(!sent.alive_after_send && sent.alive.expired());
  return 0;
}

// The front batch's fence wait fails: kept past send and the caller's batch,
// and freed with the device, which waits for the work as it is destroyed.
int kept_after_failed_wait(vkc::test::GpuContext& gpu) {
  Outcome sent;
  {
    auto device = vkc::Device::create(gpu.instance, gpu.physical,
                                      vr::device_requirements());
    CHECK(device.ok());
    auto allocator = vkc::Allocator::create(gpu.instance.handle(), *device);
    CHECK(allocator.ok());
    CHECK(send_one(*device, *allocator, true, &sent) == 0);
  }
  CHECK(!sent.status.ok() && sent.alive_at_wait);
  CHECK(sent.alive_after_send && sent.alive_after_batch);
  CHECK(sent.alive.expired());
  return 0;
}

int gpu_main(vkc::test::GpuContext& gpu) {
  CHECK(released_after_wait(gpu) == 0);
  CHECK(kept_after_failed_wait(gpu) == 0);
  std::puts("recon_sender_set_sender_retain: OK");
  return 0;
}

}  // namespace

int main() {
  return vkc::test::run_on_gpu(vr::device_requirements(), gpu_main);
}
