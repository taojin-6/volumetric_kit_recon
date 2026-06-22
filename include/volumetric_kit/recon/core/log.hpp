// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file log.hpp
/// @brief A pluggable logging seam -- a caller-installable diagnostic handler.
///
/// The library imposes no logging framework on consumers: it emits through a
/// handler they can install, defaulting to stderr for warnings and errors. This
/// is the deliberate replacement for the glog dependency the salvaged code
/// carried.

#include <functional>
#include <string_view>

#include "volumetric_kit/recon/core/export.hpp"

namespace volumetric_kit::recon {

enum class LogLevel { Debug, Info, Warning, Error };

using LogHandler = std::function<void(LogLevel, std::string_view)>;

/// Install the diagnostic sink. Pass a default-constructed (empty) handler to
/// restore the built-in default (warnings + errors to stderr). Thread-safe.
VR_CORE_API void set_log_handler(LogHandler handler);

/// Emit a diagnostic through the current handler (or the default sink).
/// Thread-safe.
VR_CORE_API void log_message(LogLevel level, std::string_view message);

}  // namespace volumetric_kit::recon
