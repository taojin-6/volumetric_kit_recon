# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# Orchestrator for the component tiers. Each subdirectory owns its own target
# and build settings; the strict dependency rule (a tier may depend only on
# tiers to its left) is expressed by what each links.

# The camera and sensor tiers are the sensor package's (sensor/), which
# third_party/ adds first: recon's libraries link its frame contract,
# sensor_frame, alone.
add_subdirectory(volumetric_kit/recon/core)
add_subdirectory(volumetric_kit/recon/volume)
# A triangle mesh's distance field, on volume alone: no depth fusion (the
# 2026-10-28 decision).
add_subdirectory(volumetric_kit/recon/voxelize)
add_subdirectory(volumetric_kit/recon/tsdf)
# mesh reads the volume voxel payload directly (proven against a dense analytic
# SDF), so it depends on volume, not tsdf.
add_subdirectory(volumetric_kit/recon/mesh)
# Host mesh files, off mesh for the public Mesh serialization API.
add_subdirectory(volumetric_kit/recon/io)
# texture fills the mesh uv0 by projecting a posed camera onto it; it depends on
# mesh (the Mesh it textures) + volume, both to its left, and the sensor
# package's frame contract (DepthCameraParams + the projection GLSL).
add_subdirectory(volumetric_kit/recon/texture)
add_subdirectory(volumetric_kit/recon/codec)
# Quality measurement for the tests and examples that judge a reconstruction or
# a codec: mesh-to-mesh distance, accuracy / coverage, F-score (the 2026-09-27
# eval decision). Off recon_mesh.
add_subdirectory(volumetric_kit/recon/eval)
# The .vrst session container: recorded tracks of records, stored verbatim. On
# the family core's base tier alone, so it depends on no tier here.
add_subdirectory(volumetric_kit/recon/stream)
# A session served and read live over TCP: the stream's chunks on sockets, off
# stream (and recon's header-only log seam).
add_subdirectory(volumetric_kit/recon/networking)
# A codec stream, after networking: grids coded as frames into a .vrst file and
# to live viewers, decoded back. It links the codec, the container and the
# server, and no fuser or sensor tier.
add_subdirectory(volumetric_kit/recon/codec_stream)
# A sender's set step, after codec_stream: it links the frame contract, the
# fuser and the codec stream.
add_subdirectory(volumetric_kit/recon/sender)
# TODO: add_subdirectory() for interop as that tier lands.
set(_vr_tiers
    recon_core
    recon_volume
    recon_voxelize
    recon_tsdf
    recon_mesh
    recon_texture
    recon_codec
    recon_eval
    recon_mesh_io
    recon_stream
    recon_networking
    recon_codec_stream
    recon_sender)

# --- recon: umbrella alias --------------------------------------------------
# Consumers link `volumetric_kit::recon` and get whatever tiers are built. It
# carries recon's version, which a consumer's vkc_require_sibling reads.
add_library(recon INTERFACE)
add_library(volumetric_kit::recon ALIAS recon)
set_target_properties(recon PROPERTIES VKC_PACKAGE_VERSION "${PROJECT_VERSION}")
target_link_libraries(recon INTERFACE ${_vr_tiers})
if(TARGET recon_mesh_io_assimp)
  target_link_libraries(recon INTERFACE recon_mesh_io_assimp)
endif()
