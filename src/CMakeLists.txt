# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# Orchestrator for the component tiers. Each subdirectory owns its own target
# and build settings; the strict dependency rule (a tier may depend only on
# tiers to its left) is expressed by what each links.

# camera is the family's camera vocabulary -- the camera model, rigid
# transforms, the sensor array's calibration file -- over the family core's base
# tier and GLM alone, linking no Vulkan: calib and a capture driver use it as
# well as the fusion tiers (the 2026-10-06 decision). VR_CAMERA_ONLY builds it
# alone.
add_subdirectory(volumetric_kit/recon/camera)
set(_vr_tiers recon_camera)
if(NOT VR_CAMERA_ONLY)
  add_subdirectory(volumetric_kit/recon/core)
  # tsdf fuses into grid's VoxelBlockGrid (volumetric_kit_grid).
  add_subdirectory(volumetric_kit/recon/tsdf)
  # Host image I/O, on the family core's base tier alone.
  add_subdirectory(volumetric_kit/recon/io)
  # texture fills the mesh uv0 by projecting a posed camera onto it; it depends
  # on grid's mesh (the Mesh it textures) and recon_core's DepthCameraParams.
  add_subdirectory(volumetric_kit/recon/texture)
  # sensor is the sensor interface: IRgbdSensor + RgbdFrame + the camera and
  # colour convention conversions. It depends on camera and core alone -- the
  # camera types it trades in are camera::CameraModel and core vocabulary, not
  # voxels or fusion -- so it sits beside the fusion tiers rather than on top of
  # them, and stays implementable from outside this repo, with drivers
  # deliberately left to whoever can build and test them (the 2026-08-02
  # decision).
  add_subdirectory(volumetric_kit/recon/sensor)
  # A sender's set step: it links the frame prep's types, the fuser and grid's
  # codec stream.
  add_subdirectory(volumetric_kit/recon/sender)
  # TODO: add_subdirectory() for interop as that tier lands.
  list(
    APPEND
    _vr_tiers
    recon_core
    recon_tsdf
    recon_texture
    recon_sensor
    recon_sensor_utils
    recon_sensor_array
    recon_sensor_video_syntax
    recon_image_io
    recon_sender)
endif()

# --- recon: umbrella alias --------------------------------------------------
# Consumers link `volumetric_kit::recon` and get whatever tiers are built. It
# carries recon's version, which a consumer's vkc_require_sibling reads.
add_library(recon INTERFACE)
add_library(volumetric_kit::recon ALIAS recon)
set_target_properties(recon PROPERTIES VKC_PACKAGE_VERSION "${PROJECT_VERSION}")
target_link_libraries(recon INTERFACE ${_vr_tiers})
if(TARGET recon_sensor_apple)
  target_link_libraries(recon INTERFACE recon_sensor_apple)
endif()
if(TARGET recon_sensor_video)
  target_link_libraries(recon INTERFACE recon_sensor_video)
endif()
if(TARGET recon_sensor_orbbec)
  target_link_libraries(recon INTERFACE recon_sensor_orbbec)
endif()
