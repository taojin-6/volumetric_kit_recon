# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# Orchestrator for the component tiers. Each subdirectory owns its own target
# and build settings and installs itself into the shared
# volumetric_kit_reconTargets export set; the strict dependency rule (a tier may
# depend only on tiers to its left) is expressed by what each links.
include(GNUInstallDirs)

add_subdirectory(volumetric_kit/recon/core)
add_subdirectory(volumetric_kit/recon/volume)
add_subdirectory(volumetric_kit/recon/tsdf)
# mesh reads the volume voxel payload directly (proven against a dense analytic
# SDF), so it depends on volume, not tsdf.
add_subdirectory(volumetric_kit/recon/mesh)
# Host asset I/O branches off mesh for the public Mesh serialization API.
add_subdirectory(volumetric_kit/recon/io)
# texture fills the mesh uv0 by projecting a posed camera onto it; it depends on
# mesh (the Mesh it textures) + volume (DepthCameraParams + the projection ABI),
# both to its left, so the strict dependency rule holds.
add_subdirectory(volumetric_kit/recon/texture)
# sensor is the capture contract: ICameraCapture + CapturedFrame + the camera
# convention conversions. It depends on core alone -- the camera-parameter
# structs it trades in are core vocabulary (core/camera_params.hpp), not voxels
# or fusion -- so it sits beside the fusion tiers rather than on top of them,
# and stays implementable from outside this repo, with drivers deliberately left
# to whoever can build and test them (the 2026-08-02 decision).
add_subdirectory(volumetric_kit/recon/sensor)
add_subdirectory(volumetric_kit/recon/codec)
# Quality measurement for the tests and examples that judge a reconstruction or
# a codec: mesh-to-mesh distance, accuracy / coverage, F-score (the 2026-09-27
# eval decision). Off recon_mesh.
add_subdirectory(volumetric_kit/recon/eval)
# TODO: add_subdirectory() for interop as that tier lands.

# --- recon: umbrella alias --------------------------------------------------
# Consumers link `volumetric_kit::recon` and get whatever tiers are built.
add_library(recon INTERFACE)
add_library(volumetric_kit::recon ALIAS recon)
target_link_libraries(
  recon
  INTERFACE recon_core
            recon_volume
            recon_tsdf
            recon_mesh
            recon_texture
            recon_sensor
            recon_sensor_utils
            recon_codec
            recon_eval
            recon_io)
if(TARGET recon_io_assimp)
  target_link_libraries(recon INTERFACE recon_io_assimp)
endif()
if(TARGET recon_sensor_video)
  target_link_libraries(recon INTERFACE recon_sensor_video)
endif()
if(TARGET recon_sensor_orbbec)
  target_link_libraries(recon INTERFACE recon_sensor_orbbec)
endif()

# --- Install / export -------------------------------------------------------
# Tiers install their own targets into volumetric_kit_reconTargets and their own
# headers (so a partial build never ships a header for a tier it didn't build);
# here we add the umbrella and finalize the export set and package config.
if(VR_INSTALL)
  install(TARGETS recon EXPORT volumetric_kit_reconTargets)

  install(
    EXPORT volumetric_kit_reconTargets
    NAMESPACE volumetric_kit::
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/volumetric_kit_recon")

  # The installed package re-finds the core at the MAJOR.MINOR this build used:
  # the core's version file accepts only the same minor before 1.0. A core built
  # here carries its version on core_base; an installed one that a parent
  # project found carries none, but find_package set its version.
  get_target_property(_vr_core_version volumetric_kit::core_base VERSION)
  if(NOT _vr_core_version)
    set(_vr_core_version "${volumetric_kit_core_VERSION}")
  endif()
  string(REGEX MATCH "^[0-9]+\\.[0-9]+" VR_CORE_MAJOR_MINOR
               "${_vr_core_version}")
  if(NOT VR_CORE_MAJOR_MINOR)
    message(FATAL_ERROR "volumetric_kit_core's version is unknown")
  endif()

  include(CMakePackageConfigHelpers)
  configure_package_config_file(
    "${PROJECT_SOURCE_DIR}/cmake/volumetric_kit_reconConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/volumetric_kit_reconConfig.cmake"
    INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/volumetric_kit_recon")
  # SameMinorVersion while in 0.x: under semver a 0.MINOR bump may break, so a
  # consumer that requests 0.MINOR.PATCH must not be handed a different 0.MINOR.
  # Relax to SameMajorVersion once the API stabilizes at 1.0.
  write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/volumetric_kit_reconConfigVersion.cmake"
    COMPATIBILITY SameMinorVersion)
  install(
    FILES "${CMAKE_CURRENT_BINARY_DIR}/volumetric_kit_reconConfig.cmake"
          "${CMAKE_CURRENT_BINARY_DIR}/volumetric_kit_reconConfigVersion.cmake"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/volumetric_kit_recon")
endif()
