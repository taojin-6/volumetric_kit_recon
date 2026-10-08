// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "test_allocation_failure.hpp"

#include <cstdlib>
#include <new>

namespace vr_test {
thread_local AllocationFailure allocation_failure;
}  // namespace vr_test

// Every form allocates and frees through malloc and free, so a sanitizer sees
// matched pairs whichever form a library calls. No form throws: a test built
// with -fno-exceptions links this too, and one whose host is out of memory for
// real cannot go on.
void* operator new(std::size_t bytes) {
  auto& failure = vr_test::allocation_failure;
  if (failure.measure) failure.bytes = bytes;
  if (void* p = std::malloc(bytes == 0 ? 1 : bytes)) return p;
  std::abort();
}

void* operator new(std::size_t bytes, const std::nothrow_t&) noexcept {
  auto& failure = vr_test::allocation_failure;
  if (failure.armed && bytes == failure.bytes) {
    failure.armed = false;
    failure.injected = true;
    return nullptr;
  }
  return std::malloc(bytes == 0 ? 1 : bytes);
}

void operator delete(void* p) noexcept { std::free(p); }
void* operator new[](std::size_t bytes) { return ::operator new(bytes); }
void operator delete[](void* p) noexcept { ::operator delete(p); }
// A linked library may use sized delete even when this compiler does not
// emit it, so both forms must free through the replacement above.
void operator delete(void* p, std::size_t) noexcept { ::operator delete(p); }
void operator delete[](void* p, std::size_t) noexcept {
  ::operator delete[](p);
}
void* operator new[](std::size_t bytes, const std::nothrow_t&) noexcept {
  return ::operator new(bytes, std::nothrow);
}
void operator delete(void* p, const std::nothrow_t&) noexcept {
  ::operator delete(p);
}
void operator delete[](void* p, const std::nothrow_t&) noexcept {
  ::operator delete[](p);
}
