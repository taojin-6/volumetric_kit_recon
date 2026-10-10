// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "test_allocation_failure.hpp"

#include <cstdlib>
#include <new>

namespace vr_test {
thread_local AllocationFailure allocation_failure;
}  // namespace vr_test

void* operator new(std::size_t bytes) {
  auto& failure = vr_test::allocation_failure;
  if (failure.measure) failure.bytes = bytes;
  if (failure.armed && bytes == failure.bytes) {
    failure.armed = false;  // The error Status can allocate its message.
    failure.injected = true;
    throw std::bad_alloc();
  }
  if (void* p = std::malloc(bytes == 0 ? 1 : bytes)) return p;
  throw std::bad_alloc();
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

// The default nothrow operators can use the sanitizer's allocator directly.
// Route them through the same allocation/deallocation pair as throwing new.
void* operator new(std::size_t bytes, const std::nothrow_t&) noexcept {
  try {
    return ::operator new(bytes);
  } catch (const std::bad_alloc&) {
    return nullptr;
  }
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
