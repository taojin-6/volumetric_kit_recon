// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

#include "volumetric_kit/recon/core/compute_pipeline.hpp"

#include <utility>

#include "volumetric_kit/recon/core/shader.hpp"
#include "volumetric_kit/recon/core/vk_result.hpp"

namespace volumetric_kit::recon {

Result<ComputePipeline> ComputePipeline::create(
    VkDevice device, const ComputePipelineDesc& desc) {
  if (desc.shader == nullptr || !desc.shader->valid()) {
    return Status::invalid_argument(
        "ComputePipeline::create: shader is null or invalid");
  }
  if (desc.entry_point == nullptr) {
    return Status::invalid_argument(
        "ComputePipeline::create: entry_point is null");
  }
  if (desc.set_layout_count > 0 && desc.set_layouts == nullptr) {
    return Status::invalid_argument(
        "ComputePipeline::create: null set_layouts with a non-zero count");
  }
  if (desc.push_range_count > 0 && desc.push_ranges == nullptr) {
    return Status::invalid_argument(
        "ComputePipeline::create: null push_ranges with a non-zero count");
  }
  if (device == VK_NULL_HANDLE) {
    return Status::invalid_argument("ComputePipeline::create: device is null");
  }

  VkPipelineLayoutCreateInfo layout_info{};
  layout_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
  layout_info.setLayoutCount = desc.set_layout_count;
  layout_info.pSetLayouts = desc.set_layouts;
  layout_info.pushConstantRangeCount = desc.push_range_count;
  layout_info.pPushConstantRanges = desc.push_ranges;

  VkPipelineLayout layout = VK_NULL_HANDLE;
  VR_VK_TRY(vkCreatePipelineLayout(device, &layout_info, nullptr, &layout));
  // Own the layout immediately so an early return from the pipeline create
  // below still frees it.
  UniqueHandle<VkPipelineLayout, vkDestroyPipelineLayout> owned_layout(device,
                                                                       layout);

  VkPipelineShaderStageCreateInfo stage{};
  stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
  stage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
  stage.module = desc.shader->handle();
  stage.pName = desc.entry_point;

  VkComputePipelineCreateInfo pipeline_info{};
  pipeline_info.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
  pipeline_info.stage = stage;
  pipeline_info.layout = layout;

  VkPipeline pipeline = VK_NULL_HANDLE;
  VR_VK_TRY(vkCreateComputePipelines(device, VK_NULL_HANDLE, 1, &pipeline_info,
                                     nullptr, &pipeline));

  ComputePipeline result;
  result.layout_ = std::move(owned_layout);
  result.pipeline_ =
      UniqueHandle<VkPipeline, vkDestroyPipeline>(device, pipeline);
  return result;
}

}  // namespace volumetric_kit::recon
