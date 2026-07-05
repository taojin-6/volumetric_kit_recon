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

/// @brief Severity of a diagnostic passed to a @ref LogHandler.
///
/// The built-in default sink emits @ref LogLevel::Warning and @ref
/// LogLevel::Error to stderr and drops @ref LogLevel::Debug and @ref
/// LogLevel::Info; an installed handler receives every level and decides for
/// itself.
enum class LogLevel {
  Debug,    ///< Verbose developer tracing; dropped by the default sink.
  Info,     ///< Normal progress information; dropped by the default sink.
  Warning,  ///< A recoverable problem; emitted to stderr by the default sink.
  Error,    ///< A failure; emitted to stderr by the default sink.
};

using LogHandler = std::function<void(LogLevel, std::string_view)>;

/// Install the diagnostic sink. Pass a default-constructed (empty) handler to
/// restore the built-in default (warnings + errors to stderr). Thread-safe.
VR_CORE_API void set_log_handler(LogHandler handler);

/// Emit a diagnostic through the current handler (or the default sink).
/// Thread-safe.
VR_CORE_API void log_message(LogLevel level, std::string_view message);

}  // namespace volumetric_kit::recon
