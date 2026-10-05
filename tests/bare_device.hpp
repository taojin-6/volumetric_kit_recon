// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// A device adopted with none of its optional extensions declared, so a decoder
// handed it can keep no picture on the GPU: the device path fails to open on
// any machine, which is how a test reaches the fallback to host pictures.

#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/instance.hpp"
#include "volumetric_kit/recon/core/result.hpp"

namespace vr_test {

namespace vr = volumetric_kit::recon;

// `device` again, borrowed: it must outlive the result.
inline vr::Result<vr::Device> bare_device(const vr::Instance& instance,
                                          const vr::Device& device) {
  vr::AdoptedDevice view;
  view.instance = instance.handle();
  view.instance_api_version = instance.api_version();
  view.physical_device = device.physical_device();
  view.device = device.handle();
  view.queue_family = device.queue_family();
  view.queue = device.queue();
  view.submit_mutex = device.submit_mutex();
  view.enabled_features.timeline_semaphore = true;
  view.enabled_features.scalar_block_layout = true;
  return vr::Device::adopt(view, vr::device_requirements());
}

}  // namespace vr_test
