// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/recon/core/shader.hpp"

#include "volumetric_kit/recon/core/vk_result.hpp"

namespace volumetric_kit::recon {

Result<ShaderModule> ShaderModule::create(VkDevice device,
                                          const std::uint32_t* code,
                                          std::size_t size_bytes) {
  if (device == VK_NULL_HANDLE) {
    return Status::invalid_argument("ShaderModule::create: device is null");
  }
  // codeSize is a byte count that Vulkan requires be a multiple of 4 (SPIR-V is
  // a stream of 32-bit words); reject a malformed blob before the device call.
  if (code == nullptr || size_bytes == 0 || size_bytes % 4 != 0) {
    return Status::invalid_argument(
        "ShaderModule::create: SPIR-V must be non-null and a non-zero multiple "
        "of 4 bytes");
  }

  VkShaderModuleCreateInfo info{};
  info.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
  info.codeSize = size_bytes;
  info.pCode = code;

  VkShaderModule handle = VK_NULL_HANDLE;
  VR_VK_TRY(vkCreateShaderModule(device, &info, nullptr, &handle));

  ShaderModule shader;
  shader.module_ =
      UniqueHandle<VkShaderModule, vkDestroyShaderModule>(device, handle);
  return shader;
}

}  // namespace volumetric_kit::recon
