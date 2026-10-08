// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

#include <cstddef>

namespace vr_test {

// Link test_allocation_failure.cpp only into tests that inject failures. The
// replacement operators are compiled separately from their callers so GCC
// does not inline delete's free and mistake it for a mismatched deallocation.
struct AllocationFailure {
  // While set, each throwing allocation records its size in `bytes`.
  bool measure = false;
  std::size_t bytes = 0;
  // While set, the next nothrow allocation of `bytes` fails, once.
  bool armed = false;
  bool injected = false;
};

// Limit injection to the calling thread, so a driver's worker cannot consume
// it.
extern thread_local AllocationFailure allocation_failure;

}  // namespace vr_test
