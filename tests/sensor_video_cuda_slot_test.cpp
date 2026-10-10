// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// No GPU or libcuda: exercise the picture slot's ownership through the same
// host allocations that can fail after CUDA import and mapping succeed.
#include <cstdio>
#include <memory>
#include <new>
#include <type_traits>
#include <utility>
#include <vector>

#include "cuda_pictures.hpp"
#include "test_allocation_failure.hpp"
#include "volumetric_kit/core/vulkan/buffer.hpp"

namespace vkc = volumetric_kit::core;
namespace video = volumetric_kit::recon::sensor::video;

static_assert(!std::is_copy_constructible_v<video::CudaPictureSlot>);
static_assert(!std::is_copy_assignable_v<video::CudaPictureSlot>);
static_assert(std::is_nothrow_move_constructible_v<video::CudaPictureSlot>);
static_assert(std::is_nothrow_move_assignable_v<video::CudaPictureSlot>);

#define CHECK(cond)                                                        \
  do {                                                                     \
    if (!(cond)) {                                                         \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
      return 1;                                                            \
    }                                                                      \
  } while (0)

namespace {
struct Releases {
  int mappings = 0;
  int imports = 0;
  int buffers = 0;
  bool ordered = true;
  bool import_only = false;
} releases;

CUresult CUDAAPI free_mapping(CUdeviceptr pointer) {
  releases.ordered =
      releases.ordered && pointer != 0 && releases.mappings == releases.imports;
  ++releases.mappings;
  return CUDA_SUCCESS;
}

CUresult CUDAAPI destroy_import(CUexternalMemory memory) {
  releases.ordered = releases.ordered && memory != nullptr &&
                     releases.mappings + (releases.import_only ? 1 : 0) ==
                         releases.imports + 1;
  ++releases.imports;
  return CUDA_SUCCESS;
}

video::CudaPictureSlot mapped_slot() {
  video::CudaPictureSlot slot;
  slot.memory = reinterpret_cast<CUexternalMemory>(std::uintptr_t{1});
  slot.pointer = 256;
  slot.bytes = 4096;
  return slot;
}

vkc::Buffer exported_buffer() {
  return vkc::Buffer(
      reinterpret_cast<VkBuffer>(std::uintptr_t{1}), 4096,
      VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, VK_SHARING_MODE_EXCLUSIVE, nullptr,
      [] {
        releases.ordered =
            releases.ordered && releases.imports == releases.buffers + 1;
        ++releases.buffers;
      },
      std::nullopt);
}

int allocation_failure(bool fail_vector_growth) {
  auto& failure = vr_test::allocation_failure;
  // Measure the allocation's size, then fail that same allocation. No driver
  // work or unrelated host allocation is involved in this ownership test.
  failure = {};
  failure.measure = true;
  if (fail_vector_growth) {
    std::vector<video::CudaPictureSlot> slots;
    slots.emplace_back();
  } else {
    auto buffer = std::make_shared<vkc::Buffer>();
  }
  failure.measure = false;
  releases = {};
  bool caught = false;
  try {
    auto exported = exported_buffer();
    auto slot = mapped_slot();
    if (!fail_vector_growth) failure.armed = true;
    slot.buffer = std::make_shared<vkc::Buffer>(std::move(exported));
    std::vector<video::CudaPictureSlot> slots;
    if (fail_vector_growth) failure.armed = true;
    slots.push_back(std::move(slot));
  } catch (const std::bad_alloc&) {
    caught = true;
  }
  failure.armed = false;
  CHECK(caught && failure.injected);
  CHECK(releases.mappings == 1);
  CHECK(releases.imports == 1);
  CHECK(releases.buffers == 1);
  CHECK(releases.ordered);
  return 0;
}

bool empty(const video::CudaPictureSlot& slot) {
  return !slot.buffer && slot.memory == nullptr && slot.pointer == 0 &&
         slot.bytes == 0;
}

video::CudaPictureSlot backed_slot() {
  auto slot = mapped_slot();
  slot.buffer = std::make_shared<vkc::Buffer>(exported_buffer());
  return slot;
}

int moves() {
  releases = {};
  {
    auto first = backed_slot();
    auto second = std::move(first);
    CHECK(empty(first));
    CHECK(second.buffer && second.memory && second.pointer && second.bytes);
    CHECK(releases.imports == 0 && releases.buffers == 0);
    auto third = backed_slot();
    third = std::move(second);  // assignment releases a live destination
    CHECK(empty(second));
    CHECK(third.buffer && third.memory && third.pointer && third.bytes);
    CHECK(releases.mappings == 1 && releases.imports == 1 &&
          releases.buffers == 1);
    auto* alias = &third;
    third = std::move(*alias);
    CHECK(third.buffer && third.memory && third.pointer && third.bytes);
    CHECK(releases.imports == 1 && releases.buffers == 1);
  }
  CHECK(releases.mappings == 2 && releases.imports == 2 &&
        releases.buffers == 2);
  CHECK(releases.ordered);
  return 0;
}

int ring_operations() {
  releases = {};
  {
    std::vector<video::CudaPictureSlot> slots;
    slots.reserve(1);
    slots.push_back(backed_slot());
    const auto old_capacity = slots.capacity();
    slots.push_back(backed_slot());  // relocation must not release the imports
    CHECK(slots.capacity() > old_capacity);
    CHECK(releases.mappings == 0 && releases.imports == 0 &&
          releases.buffers == 0);
    slots.erase(slots.begin());  // move assignment over a live slot
    CHECK(releases.mappings == 1 && releases.imports == 1 &&
          releases.buffers == 1);
    CHECK(slots.size() == 1 && slots.front().buffer);
    slots.clear();
    CHECK(releases.mappings == 2 && releases.imports == 2 &&
          releases.buffers == 2);
  }
  CHECK(releases.imports == 2 && releases.buffers == 2 && releases.ordered);
  return 0;
}

int partial_acquisition_and_retained_picture() {
  releases = {};
  {
    video::CudaPictureSlot empty_slot;
  }
  CHECK(releases.mappings == 0 && releases.imports == 0 &&
        releases.buffers == 0);
  releases.import_only = true;
  {
    auto exported = exported_buffer();
    auto slot = mapped_slot();
    slot.pointer = 0;  // import succeeded, mapping failed
  }
  CHECK(releases.mappings == 0 && releases.imports == 1 &&
        releases.buffers == 1);
  CHECK(releases.ordered);

  releases = {};
  std::shared_ptr<const vkc::Buffer> picture;
  {
    auto slot = backed_slot();
    picture = slot.buffer;
  }
  CHECK(releases.mappings == 1 && releases.imports == 1 &&
        releases.buffers == 0);
  picture.reset();
  CHECK(releases.buffers == 1 && releases.ordered);
  return 0;
}
}  // namespace

namespace volumetric_kit::recon::sensor::video {
const CudaDriver* cuda_driver() {
  static const CudaDriver driver = [] {
    CudaDriver d;
    d.cuMemFree = free_mapping;
    d.cuDestroyExternalMemory = destroy_import;
    return d;
  }();
  return &driver;
}
}  // namespace volumetric_kit::recon::sensor::video

int main() {
  CHECK(allocation_failure(false) == 0);
  CHECK(allocation_failure(true) == 0);
  CHECK(moves() == 0);
  CHECK(ring_operations() == 0);
  CHECK(partial_acquisition_and_retained_picture() == 0);
  std::puts("sensor video CUDA slot tests passed");
  return 0;
}
