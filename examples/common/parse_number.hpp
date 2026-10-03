// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

// Numeric flag values for the codec examples' command lines.

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <string>

#include "volumetric_kit/recon/core/result.hpp"

namespace vr_example {

namespace vr = volumetric_kit::recon;

/// Parse @p text, the value of @p flag, as a finite number.
inline vr::Status parse_number(const std::string& flag, const char* text,
                               double& out) {
  char* end = nullptr;
  errno = 0;
  const double value = std::strtod(text, &end);
  if (end == text || *end != '\0' || errno != 0 || !std::isfinite(value)) {
    return vr::Status::invalid_argument(flag +
                                        ": not a finite number: " + text);
  }
  out = value;
  return {};
}

/// As the double overload, refusing a value past float's range.
inline vr::Status parse_number(const std::string& flag, const char* text,
                               float& out) {
  double value = 0.0;
  VR_TRY(parse_number(flag, text, value));
  if (std::abs(value) > std::numeric_limits<float>::max()) {
    return vr::Status::invalid_argument(flag + ": number too large: " + text);
  }
  out = static_cast<float>(value);
  return {};
}

/// Parse @p text, the value of @p flag, as a decimal int.
inline vr::Status parse_number(const std::string& flag, const char* text,
                               int& out) {
  char* end = nullptr;
  errno = 0;
  const long long value = std::strtoll(text, &end, 10);
  if (end == text || *end != '\0' || errno != 0 ||
      value < std::numeric_limits<int>::min() ||
      value > std::numeric_limits<int>::max()) {
    return vr::Status::invalid_argument(flag + ": not an integer: " + text);
  }
  out = static_cast<int>(value);
  return {};
}

}  // namespace vr_example
