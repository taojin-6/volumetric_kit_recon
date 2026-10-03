// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// Error Status helpers shared by the io implementation files.

#include <exception>
#include <new>
#include <string>

#include "volumetric_kit/recon/core/result.hpp"

namespace volumetric_kit::recon::io::detail {

// Reject an empty path or one with an embedded NUL before any file access.
inline Status check_path(const char* api, const std::string& path) {
  if (path.empty() || path.find('\0') != std::string::npos) {
    return Status::invalid_argument(std::string(api) +
                                    ": empty path or embedded NUL");
  }
  return {};
}

// Translate the exception being handled into a Status; call only from a catch
// block. The fallback messages fit the small-string buffer, so reporting an
// allocation failure does not allocate.
inline Status exception_status(const char* api) noexcept {
  try {
    try {
      throw;
    } catch (const std::bad_alloc&) {
      return Status::out_of_memory("out of memory");
    } catch (const std::exception& e) {
      return Status::io_error(std::string(api) + ": " + e.what());
    }
  } catch (...) {
  }
  return Status::io_error("I/O failure");
}

}  // namespace volumetric_kit::recon::io::detail
