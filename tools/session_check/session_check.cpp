// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// session_check: a session's frames checked by hand, from a file or from a
// writer that dials this host (an iPad's pattern, a test's sender). It decodes
// the pictures on this machine's GPU, joins each to its depth
// (link/rgbd_stream_sensor.hpp), checks on the device that each frame's
// picture carries its depth's pattern index, and prints counts, times and
// rgbd_breach (link/session_check.hpp). A poll's error is printed and the run
// goes on. Exit 0 when the frame contract holds, no frame shows another's
// picture and no poll failed; 2 for a command line it does not take. It
// writes nothing.
//
//   session_check <session.vrst> [--sensor <id>] [--depth-only]
//   session_check --listen <ipv4>:<port> [--wait <s>] [--frames <n>]
//                 [--seconds <s>] [--sensor <id>] [--depth-only]

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <optional>
#include <string>

#include "interrupt.hpp"
#include "link/rgbd_stream_sensor.hpp"
#include "link/session_check.hpp"
#include "volumetric_kit/core/base/result.hpp"
#include "volumetric_kit/core/vulkan/allocator.hpp"
#include "volumetric_kit/core/vulkan/device.hpp"
#include "volumetric_kit/core/vulkan/instance.hpp"
#include "volumetric_kit/sensor/frame/device_requirements.hpp"

namespace {

namespace vkc = volumetric_kit::core;
namespace sensor = volumetric_kit::sensor;
namespace link = vr_example::link;

struct Args {
  std::string file;
  std::optional<link::Address> listen;
  std::string sensor;
  bool depth_only = false;
  std::optional<long> wait_s;  // --listen only: 300 unless set
  long frames = 0;             // --listen only; 0: until the stream ends
  long seconds = 0;            // --listen only; 0: as frames
};

int usage() {
  std::fprintf(stderr,
               "usage: session_check <session.vrst> [--sensor <id>] "
               "[--depth-only]\n"
               "       session_check --listen <ipv4>:<port> [--wait <s>] "
               "[--frames <n>] [--seconds <s>] [--sensor <id>] "
               "[--depth-only]\n");
  return 2;
}

std::optional<Args> parse(int argc, char** argv) {
  Args a;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const bool has_value = i + 1 < argc;
    if (arg == "--depth-only") {
      a.depth_only = true;
    } else if (arg == "--listen" && has_value) {
      a.listen = link::parse_address(argv[++i]);
      if (!a.listen) return std::nullopt;
    } else if (arg == "--sensor" && has_value) {
      a.sensor = argv[++i];
    } else if ((arg == "--wait" || arg == "--frames" || arg == "--seconds") &&
               has_value) {
      const std::optional<long> n = link::count_of(argv[++i]);
      if (!n) return std::nullopt;
      if (arg == "--wait") {
        if (*n == 0) return std::nullopt;
        a.wait_s = *n;
      } else {
        (arg == "--frames" ? a.frames : a.seconds) = *n;
      }
    } else if (arg.rfind("--", 0) != 0 && a.file.empty()) {
      a.file = arg;
    } else {
      return std::nullopt;
    }
  }
  if (a.file.empty() == !a.listen) return std::nullopt;  // one source
  if (!a.listen && (a.wait_s || a.frames != 0 || a.seconds != 0)) {
    return std::nullopt;  // a file's options are its own
  }
  return a;
}

// Checks every frame; a poll's error is printed and counted, and the run
// goes on, as the sensor does past one (the polls after an end hand out
// what is whole, then exhausted() says so).
vkc::Status run(const Args& a, bool* ok) {
  VKC_ASSIGN(vkc::Instance instance, vkc::Instance::create({}));
  const vkc::DeviceRequirements reqs = sensor::device_requirements();
  VKC_ASSIGN(vkc::PhysicalDeviceInfo gpu,
             instance.select_physical_device(reqs));
  VKC_ASSIGN(vkc::Device device, vkc::Device::create(instance, gpu, reqs));
  VKC_ASSIGN(vkc::Allocator allocator,
             vkc::Allocator::create(instance.handle(), device));

  link::RgbdStreamSensor::Options options;
  options.sensor = a.sensor;
  if (!a.depth_only) {
    options.device = &device;  // decode the pictures here
    options.allocator = &allocator;
  }
  if (a.listen) {
    std::printf("session_check: listening on %s:%u\n", a.listen->host.c_str(),
                static_cast<unsigned>(a.listen->port));
  }
  VKC_ASSIGN(
      std::unique_ptr<link::RgbdStreamSensor> source,
      a.listen ? link::RgbdStreamSensor::listen(
                     a.listen->host, a.listen->port, std::move(options), {},
                     std::chrono::seconds(a.wait_s.value_or(300)))
               : link::RgbdStreamSensor::open_file(a.file, std::move(options)));
  VKC_ASSIGN(std::unique_ptr<link::SessionCheck> check,
             link::SessionCheck::create(device, allocator, source->info()));
  vgrid_example::catch_interrupts();
  const auto start = std::chrono::steady_clock::now();
  const auto out_of_time = [&] {
    return a.seconds != 0 && std::chrono::steady_clock::now() - start >=
                                 std::chrono::seconds(a.seconds);
  };
  VKC_TRY(source->start());
  std::uint64_t taken = 0;
  std::uint64_t errors = 0;
  while (!source->exhausted() && !vgrid_example::interrupted() &&
         !out_of_time() &&
         (a.frames == 0 || taken < static_cast<std::uint64_t>(a.frames))) {
    vkc::Result<std::optional<sensor::RgbdFrame>> frame = source->poll();
    if (!frame.ok()) {
      ++errors;
      std::fprintf(stderr, "session_check: %s\n",
                   frame.status().message().c_str());
      continue;
    }
    if (!*frame) continue;  // not yet, or a reconnect under way
    VKC_TRY(check->take(**frame));
    ++taken;
  }
  const link::SessionReport report = check->report(*source);
  std::fputs(link::format(report).c_str(), stdout);
  std::printf("errors: %llu\n", static_cast<unsigned long long>(errors));
  *ok = report.ok() && errors == 0;
  return {};
}

}  // namespace

int main(int argc, char** argv) {
  const std::optional<Args> args = parse(argc, argv);
  if (!args) return usage();
  bool ok = false;
  const vkc::Status ran = run(*args, &ok);
  if (!ran.ok()) {
    std::fprintf(stderr, "session_check: %s\n", ran.message().c_str());
    return 1;
  }
  return ok ? 0 : 1;
}
