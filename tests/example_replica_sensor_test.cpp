// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// The Replica example's sensor (examples/common/replica_sensor.hpp) reads its
// intrinsics through the core's read_file: a missing cam_params.json is
// NotFound, named as this sensor's; one without a key is refused; and a scene
// with a pose and no images opens with nothing to play. Host only.

#include <cstdio>
#include <string>

#include "replica_sensor.hpp"
#include "scratch_dir.hpp"
#include "test_check.hpp"
#include "volumetric_kit/core/base/file_io.hpp"
#include "volumetric_kit/core/base/result.hpp"

namespace vkc = volumetric_kit::core;

namespace {

using Code = vkc::Status::Code;
using vr_example::ReplicaSensor;

int test_open() {
  const vr_test::Scratch scratch("recon_replica_sensor_");
  const std::string scene = scratch.path.string();
  const std::string params = scene + "/cam_params.json";
  const ReplicaSensor::Options options;

  const auto missing = ReplicaSensor::open(scene, params, options);
  CHECK(!missing.ok() && missing.status().domain() == Code::NotFound);
  CHECK(missing.status().message().rfind("ReplicaSensor::open: cam params: ",
                                         0) == 0);

  CHECK(vkc::write_file(
      params, R"({"camera": {"w": 64, "h": 48, "fx": 50.0, "fy": 50.0}})",
      vkc::WriteMode::CreateNew));
  const auto keyless = ReplicaSensor::open(scene, params, options);
  CHECK(!keyless.ok() && keyless.status().domain() == Code::InvalidArgument &&
        keyless.status().message().find("missing key 'cx'") !=
            std::string::npos);

  CHECK(vkc::write_file(params,
                        R"({"camera": {"w": 64, "h": 48, "fx": 50.0,
                            "fy": 50.0, "cx": 31.5, "cy": 23.5,
                            "scale": 1000.0}})",
                        vkc::WriteMode::Replace));
  CHECK(vkc::write_file(scene + "/traj.txt",
                        "1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1\n",
                        vkc::WriteMode::CreateNew));
  const auto opened = ReplicaSensor::open(scene, params, options);
  CHECK(opened.ok() && opened.value().frame_count() == 0);
  CHECK(opened.value().info().color->size.width == 64 &&
        opened.value().info().color->intrinsics.cy == 23.5);
  return 0;
}

}  // namespace

int main() {
  if (test_open() != 0) return 1;
  std::printf("replica sensor tests passed\n");
  return 0;
}
