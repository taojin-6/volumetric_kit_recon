// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file log.hpp
/// @brief recon's side of the family's one log sink.
///
/// The sink is volumetric_kit_core's: one process-wide handler for calib,
/// recon and gfx alike, defaulting to stderr for warnings and errors. The
/// library imposes no logging framework on consumers; an application installs
/// its own handler with @ref set_log_handler (the core's, named here) and
/// receives each message with its level and source. recon's messages carry the
/// source @ref kLogSource, so the default sink keeps printing `[vr <level>]`.

#include <string_view>

#include "volumetric_kit/core/base/log.hpp"

namespace volumetric_kit::recon {

using core::LogHandler;
using core::LogLevel;
using core::set_log_handler;

/// @brief The source recon's diagnostics carry; the default sink prints
///        `[vr <level>]`.
inline constexpr std::string_view kLogSource = "vr";

/// @brief Emit a recon diagnostic through the family's sink, with source
///        @ref kLogSource. Thread-safe.
/// @param level    The message's severity.
/// @param message  The message; it need not be NUL-terminated.
inline void log_message(LogLevel level, std::string_view message) {
  core::log_message(level, kLogSource, message);
}

}  // namespace volumetric_kit::recon
