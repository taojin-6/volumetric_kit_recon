// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/recon/core/log.hpp"

#include <cstdio>
#include <mutex>
#include <utility>

namespace volumetric_kit::recon {
namespace {

std::mutex g_mutex;
LogHandler g_handler;  // Empty => use default_sink.

const char* level_name(LogLevel level) {
  switch (level) {
    case LogLevel::Debug:
      return "debug";
    case LogLevel::Info:
      return "info";
    case LogLevel::Warning:
      return "warning";
    case LogLevel::Error:
      return "error";
  }
  return "?";
}

// Default sink: warnings and errors to stderr, quieter levels dropped.
void default_sink(LogLevel level, std::string_view message) {
  if (level == LogLevel::Warning || level == LogLevel::Error) {
    // fwrite the body by its exact length rather than printf's %.*s: a
    // string_view need not be NUL-terminated, and its size can exceed INT_MAX
    // (a negative %.*s precision would scan for a NUL that isn't there).
    std::fprintf(stderr, "[vr %s] ", level_name(level));
    std::fwrite(message.data(), 1, message.size(), stderr);
    std::fputc('\n', stderr);
  }
}

}  // namespace

void set_log_handler(LogHandler handler) {
  std::lock_guard<std::mutex> lock(g_mutex);
  g_handler = std::move(handler);
}

void log_message(LogLevel level, std::string_view message) {
  // Copy the handler out under the lock, then call it unlocked so a handler is
  // free to log re-entrantly without deadlocking.
  LogHandler handler;
  {
    std::lock_guard<std::mutex> lock(g_mutex);
    handler = g_handler;
  }
  if (handler) {
    handler(level, message);
  } else {
    default_sink(level, message);
  }
}

}  // namespace volumetric_kit::recon
