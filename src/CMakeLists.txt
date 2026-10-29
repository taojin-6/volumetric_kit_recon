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
  add_subdirectory(volumetric_kit/recon/volume)
  # A triangle mesh's distance field, on volume alone: no depth fusion (the
  # 2026-10-28 decision).
  add_subdirectory(volumetric_kit/recon/voxelize)
  add_subdirectory(volumetric_kit/recon/tsdf)
  # mesh reads the volume voxel payload directly (proven against a dense
  # analytic SDF), so it depends on volume, not tsdf.
  add_subdirectory(volumetric_kit/recon/mesh)
  # Host file I/O: images on the family core's base tier alone, mesh files off
  # mesh for the public Mesh serialization API.
  add_subdirectory(volumetric_kit/recon/io)
  # texture fills the mesh uv0 by projecting a posed camera onto it; it depends
  # on mesh (the Mesh it textures) + volume (DepthCameraParams + the projection
  # ABI), both to its left, so the strict dependency rule holds.
  add_subdirectory(volumetric_kit/recon/texture)
  # sensor is the sensor interface: IRgbdSensor + RgbdFrame + the camera and
  # colour convention conversions. It depends on camera and core alone -- the
  # camera types it trades in are camera::CameraModel and core vocabulary, not
  # voxels or fusion -- so it sits beside the fusion tiers rather than on top of
  # them, and stays implementable from outside this repo, with drivers
  # deliberately left to whoever can build and test them (the 2026-08-02
  # decision).
  add_subdirectory(volumetric_kit/recon/sensor)
  add_subdirectory(volumetric_kit/recon/codec)
  # Quality measurement for the tests and examples that judge a reconstruction
  # or a codec: mesh-to-mesh distance, accuracy / coverage, F-score (the
  # 2026-09-27 eval decision). Off recon_mesh.
  add_subdirectory(volumetric_kit/recon/eval)
  # The .vrst session container: recorded tracks of records, stored verbatim. On
  # the family core's base tier alone, so it depends on no tier here.
  add_subdirectory(volumetric_kit/recon/stream)
  # A session served and read live over TCP: the stream's chunks on sockets, off
  # stream (and recon's header-only log seam).
  add_subdirectory(volumetric_kit/recon/networking)
  # A codec stream, after networking: grids coded as frames into a .vrst file
  # and to live viewers, decoded back. It links the codec, the container and the
  # server, and no fuser or sensor tier.
  add_subdirectory(volumetric_kit/recon/codec_stream)
  # A sender's set step, after codec_stream: it links the frame prep's types,
  # the fuser and the codec stream.
  add_subdirectory(volumetric_kit/recon/sender)
  # TODO: add_subdirectory() for interop as that tier lands.
  list(
    APPEND
    _vr_tiers
    recon_core
    recon_volume
    recon_voxelize
    recon_tsdf
    recon_mesh
    recon_texture
    recon_sensor
    recon_sensor_utils
    recon_sensor_array
    recon_sensor_video_syntax
    recon_codec
    recon_eval
    recon_image_io
    recon_mesh_io
    recon_stream
    recon_networking
    recon_codec_stream
    recon_sender)
endif()

# --- recon: umbrella alias --------------------------------------------------
# Consumers link `volumetric_kit::recon` and get whatever tiers are built.
add_library(recon INTERFACE)
add_library(volumetric_kit::recon ALIAS recon)
target_link_libraries(recon INTERFACE ${_vr_tiers})
if(TARGET recon_mesh_io_assimp)
  target_link_libraries(recon INTERFACE recon_mesh_io_assimp)
endif()
if(TARGET recon_sensor_apple)
  target_link_libraries(recon INTERFACE recon_sensor_apple)
endif()
if(TARGET recon_sensor_video)
  target_link_libraries(recon INTERFACE recon_sensor_video)
endif()
if(TARGET recon_sensor_orbbec)
  target_link_libraries(recon INTERFACE recon_sensor_orbbec)
endif()
