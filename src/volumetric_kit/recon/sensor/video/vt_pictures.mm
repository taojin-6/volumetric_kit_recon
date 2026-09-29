// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Tao Jin

// Before anything includes Vulkan: VK_EXT_metal_objects' structures.
#define VK_USE_PLATFORM_METAL_EXT

#include "vt_pictures.hpp"

#include <IOSurface/IOSurface.h>
#include <Metal/Metal.h>

#include <string>
#include <utility>

#include "volumetric_kit/recon/core/device.hpp"
#include "volumetric_kit/recon/core/image.hpp"
#include "volumetric_kit/recon/core/vulkan.hpp"

namespace volumetric_kit::recon::sensor::video {

struct VtPictures::Impl {
  const Device* device = nullptr;
  const char* who = nullptr;
  id<MTLDevice> metal = nil;  // the one MoltenVK runs the device on

  Status error(const std::string& what) const {
    return Status::io_error(std::string(who) + ": " + what);
  }

  // Plane @p plane of @p surface as an image of @p format, holding @p pixels.
  Result<std::shared_ptr<const Image>> plane_image(CVPixelBufferRef pixels,
                                                   IOSurfaceRef surface,
                                                   int plane, VkFormat format,
                                                   MTLPixelFormat metal_format);
};

Result<std::shared_ptr<const Image>> VtPictures::Impl::plane_image(
    CVPixelBufferRef pixels, IOSurfaceRef surface, int plane, VkFormat format,
    MTLPixelFormat metal_format) {
  const auto width =
      static_cast<std::uint32_t>(CVPixelBufferGetWidthOfPlane(pixels, plane));
  const auto height =
      static_cast<std::uint32_t>(CVPixelBufferGetHeightOfPlane(pixels, plane));
  MTLTextureDescriptor* desc =
      [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:metal_format
                                                         width:width
                                                        height:height
                                                     mipmapped:NO];
  desc.usage = MTLTextureUsageShaderRead;
  desc.storageMode = MTLStorageModeShared;
  id<MTLTexture> texture =
      [metal newTextureWithDescriptor:desc
                            iosurface:surface
                                plane:static_cast<NSUInteger>(plane)];
  if (texture == nil) return error("wrapping a picture plane in a texture");

  VkImportMetalTextureInfoEXT import{};
  import.sType = VK_STRUCTURE_TYPE_IMPORT_METAL_TEXTURE_INFO_EXT;
  import.plane = VK_IMAGE_ASPECT_PLANE_0_BIT;
  import.mtlTexture = texture;
  VkImageCreateInfo info{};
  info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
  info.pNext = &import;
  info.imageType = VK_IMAGE_TYPE_2D;
  info.format = format;
  info.extent = {width, height, 1};
  info.mipLevels = 1;
  info.arrayLayers = 1;
  info.samples = VK_SAMPLE_COUNT_1_BIT;
  info.tiling = VK_IMAGE_TILING_OPTIMAL;
  info.usage = VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
  info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  const VkDevice dev = device->handle();
  VkImage image = VK_NULL_HANDLE;
  if (vkCreateImage(dev, &info, nullptr, &image) != VK_SUCCESS) {
    return error("importing a picture plane");
  }
  // MoltenVK binds the texture's own storage; the memory is its formality.
  VkMemoryRequirements needs{};
  vkGetImageMemoryRequirements(dev, image, &needs);
  VkMemoryAllocateInfo alloc{};
  alloc.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
  alloc.allocationSize = needs.size;
  alloc.memoryTypeIndex = static_cast<std::uint32_t>(
      __builtin_ctz(needs.memoryTypeBits != 0 ? needs.memoryTypeBits : 1));
  VkDeviceMemory memory = VK_NULL_HANDLE;
  if (vkAllocateMemory(dev, &alloc, nullptr, &memory) != VK_SUCCESS ||
      vkBindImageMemory(dev, image, memory, 0) != VK_SUCCESS) {
    vkFreeMemory(dev, memory, nullptr);
    vkDestroyImage(dev, image, nullptr);
    return error("binding a picture plane");
  }
  // The image holds the texture and the pixel buffer, so VideoToolbox does
  // not reuse the surface while anything reads it.
  CVPixelBufferRetain(pixels);
  return std::make_shared<const Image>(
      image, format, width, height, info.usage, VK_IMAGE_LAYOUT_UNDEFINED,
      [dev, image, memory, texture, pixels] {
        vkDestroyImage(dev, image, nullptr);
        vkFreeMemory(dev, memory, nullptr);
        static_cast<void>(texture);  // released with the capture
        CVPixelBufferRelease(pixels);
      });
}

Result<std::unique_ptr<VtPictures>> VtPictures::create(const Device& device,
                                                       const char* who) {
  if (!device.imports_metal_textures()) {
    return Status::unsupported(std::string(who) +
                               ": the device imports no Metal textures "
                               "(VK_EXT_metal_objects)");
  }
  const auto export_objects = reinterpret_cast<PFN_vkExportMetalObjectsEXT>(
      vkGetDeviceProcAddr(device.handle(), "vkExportMetalObjectsEXT"));
  VkExportMetalDeviceInfoEXT metal{};
  metal.sType = VK_STRUCTURE_TYPE_EXPORT_METAL_DEVICE_INFO_EXT;
  VkExportMetalObjectsInfoEXT objects{};
  objects.sType = VK_STRUCTURE_TYPE_EXPORT_METAL_OBJECTS_INFO_EXT;
  objects.pNext = &metal;
  if (export_objects != nullptr) export_objects(device.handle(), &objects);
  if (metal.mtlDevice == nil) {
    return Status::unsupported(std::string(who) +
                               ": the device names no Metal device");
  }
  auto impl = std::make_unique<Impl>();
  impl->device = &device;
  impl->who = who;
  impl->metal = metal.mtlDevice;
  return std::unique_ptr<VtPictures>(new VtPictures(std::move(impl)));
}

VtPictures::VtPictures(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
VtPictures::~VtPictures() = default;

Result<bool> VtPictures::import(CVPixelBufferRef pixels, std::uint32_t width,
                                std::uint32_t height, DecodedPicture& out) {
  const OSType format = CVPixelBufferGetPixelFormatType(pixels);
  IOSurfaceRef surface = CVPixelBufferGetIOSurface(pixels);
  if ((format != kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange &&
       format != kCVPixelFormatType_420YpCbCr8BiPlanarFullRange) ||
      surface == nullptr || CVPixelBufferGetPlaneCount(pixels) != 2 ||
      CVPixelBufferGetWidthOfPlane(pixels, 0) < width ||
      CVPixelBufferGetHeightOfPlane(pixels, 0) < height ||
      CVPixelBufferGetWidthOfPlane(pixels, 1) < (width + 1) / 2 ||
      CVPixelBufferGetHeightOfPlane(pixels, 1) < (height + 1) / 2) {
    return false;
  }
  VR_ASSIGN(auto luma,
            impl_->plane_image(pixels, surface, 0, VK_FORMAT_R8_UNORM,
                               MTLPixelFormatR8Unorm));
  VR_ASSIGN(auto chroma,
            impl_->plane_image(pixels, surface, 1, VK_FORMAT_R8G8_UNORM,
                               MTLPixelFormatRG8Unorm));
  out.width = width;
  out.height = height;
  out.layout = VideoPixelLayout::Nv12;
  out.plane[0] = out.plane[1] = out.plane[2] = nullptr;
  out.stride[0] = out.stride[1] = out.stride[2] = 0;
  out.device = nullptr;
  out.image[0] = std::move(luma);
  out.image[1] = std::move(chroma);
  return true;
}

}  // namespace volumetric_kit::recon::sensor::video
