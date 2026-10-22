// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

#include <cstdio>

// For test functions returning an exit code; preserve the failing call site.
#define CHECK(cond)                                                        \
  do {                                                                     \
    if (!(cond)) {                                                         \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      return 1;                                                            \
    }                                                                      \
  } while (0)

// Defined under AddressSanitizer or ThreadSanitizer, which slow a host loop
// many times over and keep freed memory a while.
#if defined(__has_feature)
#if __has_feature(address_sanitizer) || __has_feature(thread_sanitizer)
#define VR_TEST_SANITIZED 1
#endif
#endif
#if defined(__SANITIZE_ADDRESS__) || defined(__SANITIZE_THREAD__)
#define VR_TEST_SANITIZED 1
#endif
