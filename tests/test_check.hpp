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
