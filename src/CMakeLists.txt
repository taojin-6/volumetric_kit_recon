# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# Orchestrator for the component tiers. Each subdirectory owns its own target
# and build settings and installs itself into the shared
# volumetric_kit_reconTargets export set; the strict dependency rule (a tier may
# depend only on tiers to its left) is expressed by what each links.
include(GNUInstallDirs)

add_subdirectory(volumetric_kit/recon/core)
add_subdirectory(volumetric_kit/recon/volume)
# TODO: add_subdirectory() for tsdf / mesh / interop as those tiers land.

# --- recon: umbrella alias --------------------------------------------------
# Consumers link `volumetric_kit::recon` and get whatever tiers are built.
add_library(recon INTERFACE)
add_library(volumetric_kit::recon ALIAS recon)
target_link_libraries(recon INTERFACE recon_core recon_volume)

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
