// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/recon/core/check.hpp"

#include <cstdlib>
#include <string>

#include "volumetric_kit/recon/core/log.hpp"

namespace volumetric_kit::recon::detail {

void check_failed(const char* file, int line, const char* expr,
                  std::string_view msg) {
  std::string text = "contract check failed: ";
  text.append(msg.data(), msg.size());
  text += " [";
  text += expr;
  text += "] at ";
  text += file;
  text += ':';
  text += std::to_string(line);
  log_message(LogLevel::Error, text);
  std::abort();
}

}  // namespace volumetric_kit::recon::detail
