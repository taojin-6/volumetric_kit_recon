// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// A test's own directory for the files it writes, and those files read back.

#include <stdlib.h>  // mkdtemp: POSIX and glibc declare it here,
#include <unistd.h>  // and Apple here.

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <system_error>
#include <vector>

#include "volumetric_kit/core/base/file_io.hpp"
#include "volumetric_kit/core/base/result.hpp"

namespace vr_test {

// A new directory under the system's temporary one, `prefix` and six random
// characters (mkdtemp), removed with everything in it when the scratch goes. A
// test that cannot have one stops here.
struct Scratch {
  explicit Scratch(const std::string& prefix) {
    std::string name =
        (std::filesystem::temp_directory_path() / (prefix + "XXXXXX")).string();
    if (mkdtemp(name.data()) == nullptr) {
      std::fprintf(stderr, "FAIL: cannot make a scratch directory %s\n",
                   name.c_str());
      std::abort();
    }
    path = name;
  }
  ~Scratch() {
    std::error_code ignored;
    std::filesystem::remove_all(path, ignored);
  }
  Scratch(const Scratch&) = delete;
  Scratch& operator=(const Scratch&) = delete;

  std::filesystem::path path;
};

// The file's bytes (`read_file`); none when it cannot be read.
inline std::vector<std::uint8_t> read_bytes(const std::string& path) {
  const volumetric_kit::core::Result<std::string> bytes =
      volumetric_kit::core::read_file(path);
  if (!bytes.ok()) return {};
  return {bytes->begin(), bytes->end()};
}

}  // namespace vr_test
