// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#pragma once

/// @file shader.hpp
/// @brief A compute SPIR-V shader module.

#include <cstddef>
#include <cstdint>

#include "volumetric_kit/recon/core/export.hpp"
#include "volumetric_kit/recon/core/result.hpp"
#include "volumetric_kit/recon/core/unique_handle.hpp"
#include "volumetric_kit/recon/core/vulkan.hpp"

namespace volumetric_kit::recon {

/// @brief Owns a `VkShaderModule` built from SPIR-V words.
///
/// Takes SPIR-V as raw 32-bit words plus a byte length -- it does not read
/// files. Callers embed the compiled `.spv` at build time (a `constexpr` byte
/// array) or read it at runtime (see the compute smoke test), then hand the
/// words here. recon does no SPIR-V reflection: descriptor-set layouts and
/// push-constant ranges are declared explicitly at @ref
/// ComputePipeline::create, so this wrapper carries only the module handle.
class VR_CORE_API ShaderModule {
 public:
  /// @brief Create a shader module from SPIR-V.
  /// @param device      The device to create it on.
  /// @param code        SPIR-V as 32-bit words.
  /// @param size_bytes  Byte length of @p code (a non-zero multiple of 4).
  /// @return The module, or a non-OK @ref Status:
  ///         @ref Status::Code::InvalidArgument for a null device/code or a
  ///         size that is zero or not a multiple of 4; @ref
  ///         Status::Code::Backend if `vkCreateShaderModule` fails.
  static Result<ShaderModule> create(VkDevice device, const std::uint32_t* code,
                                     std::size_t size_bytes);

  ShaderModule() noexcept = default;
  ShaderModule(ShaderModule&&) noexcept = default;
  ShaderModule& operator=(ShaderModule&&) noexcept = default;
  ShaderModule(const ShaderModule&) = delete;
  ShaderModule& operator=(const ShaderModule&) = delete;

  /// @return The module handle (`VK_NULL_HANDLE` when empty).
  VkShaderModule handle() const noexcept { return module_.get(); }
  /// @return `true` if this owns a module.
  bool valid() const noexcept { return module_.valid(); }

 private:
  UniqueHandle<VkShaderModule, vkDestroyShaderModule> module_;
};

}  // namespace volumetric_kit::recon
