// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/recon/core/log.hpp"

#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

namespace volumetric_kit::recon {
namespace {

// State for the logging seam: the installed handler plus the mutex guarding it.
// Heap-allocated and deliberately never destroyed so the sink outlives every
// consumer -- a static object may log from its destructor during process
// teardown, and a function-local static here would risk locking an
// already-destroyed mutex (static destruction-order UB).
struct LogState {
  std::mutex mutex;
  // A shared_ptr (not a std::function held by value) so log_message can extend
  // the handler's lifetime across an unlocked call with a cheap refcount bump,
  // rather than copying the std::function (which heap-allocates for non-SBO
  // targets) on every diagnostic.
  std::shared_ptr<const LogHandler> handler;
};

LogState& state() {
  static LogState* s = new LogState();
  return *s;
}

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
    // Compose the whole line and emit it with a single fwrite so concurrent log
    // calls don't interleave (a prefix from one thread between another's body
    // and newline). Append the body by its exact length rather than printf's
    // %.*s: a string_view need not be NUL-terminated, and its size can exceed
    // INT_MAX (a negative %.*s precision would scan for a NUL that isn't there).
    std::string line = "[vr ";
    line += level_name(level);
    line += "] ";
    line.append(message.data(), message.size());
    line += '\n';
    std::fwrite(line.data(), 1, line.size(), stderr);
  }
}

}  // namespace

void set_log_handler(LogHandler handler) {
  // Wrap a non-empty handler; an empty one stores a null pointer so log_message
  // falls back to the default sink.
  auto next = handler ? std::make_shared<const LogHandler>(std::move(handler))
                      : std::shared_ptr<const LogHandler>{};
  LogState& s = state();
  std::lock_guard<std::mutex> lock(s.mutex);
  s.handler = std::move(next);
}

void log_message(LogLevel level, std::string_view message) {
  // Take a reference (refcount bump) to the handler under the lock, then call it
  // unlocked so a handler may log re-entrantly without deadlocking and a
  // concurrent set_log_handler cannot free the handler mid-call.
  std::shared_ptr<const LogHandler> handler;
  {
    LogState& s = state();
    std::lock_guard<std::mutex> lock(s.mutex);
    handler = s.handler;
  }
  if (handler && *handler) {
    (*handler)(level, message);
  } else {
    default_sink(level, message);
  }
}

}  // namespace volumetric_kit::recon
