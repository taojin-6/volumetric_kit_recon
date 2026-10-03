// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file descriptor.hpp
/// @brief Descriptor-set layout, pool, and set wrappers for binding compute
///        resources (storage buffers).

#include <cstdint>
#include <memory>

#include "volumetric_kit/recon/core/export.hpp"
#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/core/unique_handle.hpp"
#include "volumetric_kit/recon/core/vulkan.hpp"

namespace volumetric_kit::recon {

class DescriptorSet;

/// @brief Owns a `VkDescriptorSetLayout` -- the binding shape a compute
/// shader's
///        descriptor set must satisfy.
class VR_CORE_API DescriptorSetLayout {
 public:
  /// @brief Create a layout from @p count bindings.
  /// @param device    The device to create it on.
  /// @param bindings  The binding array (may be null iff @p count is 0).
  /// @param count     Number of bindings.
  static Result<DescriptorSetLayout> create(
      VkDevice device, const VkDescriptorSetLayoutBinding* bindings,
      std::uint32_t count);

  DescriptorSetLayout() noexcept = default;
  DescriptorSetLayout(DescriptorSetLayout&&) noexcept = default;
  DescriptorSetLayout& operator=(DescriptorSetLayout&&) noexcept = default;
  DescriptorSetLayout(const DescriptorSetLayout&) = delete;
  DescriptorSetLayout& operator=(const DescriptorSetLayout&) = delete;

  /// @return The layout handle (`VK_NULL_HANDLE` when empty).
  VkDescriptorSetLayout handle() const noexcept { return layout_.get(); }
  /// @return `true` if this owns a layout.
  bool valid() const noexcept { return layout_.valid(); }

 private:
  UniqueHandle<VkDescriptorSetLayout, vkDestroyDescriptorSetLayout> layout_;
};

/// @brief Owns a `VkDescriptorPool` and allocates @ref DescriptorSet from it.
class VR_CORE_API DescriptorPool {
 public:
  /// @brief Create a pool sized by @p sizes with room for @p max_sets sets.
  /// @param device      The device to create it on.
  /// @param sizes       Per-type capacity array (non-null, @p size_count > 0).
  /// @param size_count  Number of entries in @p sizes.
  /// @param max_sets    Maximum sets allocatable from the pool (> 0).
  static Result<DescriptorPool> create(VkDevice device,
                                       const VkDescriptorPoolSize* sizes,
                                       std::uint32_t size_count,
                                       std::uint32_t max_sets);

  DescriptorPool() noexcept = default;
  DescriptorPool(DescriptorPool&&) noexcept = default;
  DescriptorPool& operator=(DescriptorPool&&) noexcept = default;
  DescriptorPool(const DescriptorPool&) = delete;
  DescriptorPool& operator=(const DescriptorPool&) = delete;

  /// @brief Allocate one descriptor set with @p layout.
  ///
  /// The set is owned by the pool and freed when the pool is destroyed (there
  /// is no per-set free); the returned @ref DescriptorSet is a non-owning view.
  /// @param layout  The set layout to allocate against.
  Result<DescriptorSet> allocate(VkDescriptorSetLayout layout);

  /// @return The pool handle (`VK_NULL_HANDLE` when empty).
  VkDescriptorPool handle() const noexcept { return pool_.get(); }
  /// @return `true` if this owns a pool.
  bool valid() const noexcept { return pool_.valid(); }

 private:
  UniqueHandle<VkDescriptorPool, vkDestroyDescriptorPool> pool_;
};

/// @brief A non-owning `VkDescriptorSet` (owned by its @ref DescriptorPool)
/// plus
///        typed update helpers.
///
/// Freely copyable -- it borrows the set handle rather than owning it.
/// Copies share the write counter used to reject stale command batches.
class VR_CORE_API DescriptorSet {
 public:
  DescriptorSet() noexcept = default;

  /// @brief Wrap an already-allocated set. Called by @ref
  /// DescriptorPool::allocate.
  DescriptorSet(VkDevice device, VkDescriptorSet set) noexcept
      : state_(std::make_shared<State>(State{device, set, 0})) {}

  /// @brief Bind a storage buffer (SSBO) at @p binding.
  /// @param binding  The `layout(binding=)` slot in the shader.
  /// @param buffer   The buffer to bind.
  /// @param offset   Byte offset into @p buffer.
  /// @param range    Bound byte range (`VK_WHOLE_SIZE` for the rest).
  /// @pre @ref valid is true (else aborts via `VR_CHECK`).
  void write_storage_buffer(std::uint32_t binding, VkBuffer buffer,
                            VkDeviceSize offset, VkDeviceSize range) const;

  /// @return The set handle (`VK_NULL_HANDLE` when empty).
  VkDescriptorSet handle() const noexcept {
    return state_ != nullptr ? state_->set : VK_NULL_HANDLE;
  }
  /// @return `true` if this refers to a set.
  bool valid() const noexcept { return handle() != VK_NULL_HANDLE; }
  /// @return How many writes this view and its copies have made, so a
  ///         @ref CommandBatch can refuse a set rewritten through any alias.
  std::uint64_t writes() const noexcept {
    return state_ != nullptr ? state_->writes : 0;
  }

 private:
  struct State {
    VkDevice device;
    VkDescriptorSet set;
    std::uint64_t writes;
  };
  std::shared_ptr<State> state_;
};

}  // namespace volumetric_kit::recon
