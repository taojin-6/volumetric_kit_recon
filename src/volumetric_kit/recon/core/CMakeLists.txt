# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# --- recon_core: Tier 1 -----------------------------------------------------
# Foundation tier: version API, logging, contract checks, backend-neutral
# Status/Result error handling, plus header-only POD math and device macros.
# Every other tier depends on this; it has NO external dependencies and -- by
# design -- no GPU-API or Vulkan include (those belong to the backend tiers and
# to volumetric_kit_gfx, respectively).
include(GenerateExportHeader)
include(GNUInstallDirs)

# log.cpp uses std::mutex, so recon_core needs the platform threading runtime.
# Link it explicitly rather than relying on the compiler's implicit pthread, so
# the static archive resolves on toolchains that require it -- GCC/libstdc++ on
# Linux with --as-needed, Android, and static links. (macOS folds pthread into
# libc, which would otherwise hide the gap.)
find_package(Threads REQUIRED)

set(_vr_generated_inc "${CMAKE_CURRENT_BINARY_DIR}/generated/include")

add_library(recon_core)
add_library(volumetric_kit::recon_core ALIAS recon_core)

target_sources(recon_core PRIVATE version.cpp log.cpp check.cpp result.cpp)

# Generated headers: the VR_CORE_API export macro + the configured version
# header. The export header is per-tier (core/export.hpp + VR_CORE_API +
# VR_CORE_STATIC_DEFINE) so each future tier generates its own isolated
# macro/guard and they never collide on one path.
generate_export_header(
  recon_core
  BASE_NAME
  VR_CORE
  EXPORT_MACRO_NAME
  VR_CORE_API
  EXPORT_FILE_NAME
  "${_vr_generated_inc}/volumetric_kit/recon/core/export.hpp")
configure_file(
  "${PROJECT_SOURCE_DIR}/include/volumetric_kit/recon/version.hpp.in"
  "${_vr_generated_inc}/volumetric_kit/recon/version.hpp" @ONLY)

set_target_properties(
  recon_core
  PROPERTIES CXX_VISIBILITY_PRESET hidden
             VISIBILITY_INLINES_HIDDEN ON
             POSITION_INDEPENDENT_CODE ON)

# With a static archive, VR_CORE_API must expand to nothing for consumers.
get_target_property(_vr_type recon_core TYPE)
if(_vr_type STREQUAL "STATIC_LIBRARY")
  target_compile_definitions(recon_core PUBLIC VR_CORE_STATIC_DEFINE)
endif()

target_compile_features(recon_core PUBLIC cxx_std_17)

# PUBLIC: a static-archive consumer must also link the threading runtime, so the
# exported target carries this requirement (the Config.cmake re-finds Threads).
target_link_libraries(recon_core PUBLIC Threads::Threads)

target_include_directories(
  recon_core
  PUBLIC "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
         "$<BUILD_INTERFACE:${_vr_generated_inc}>"
         "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>")

vr_target_warnings(recon_core)

# Install the target into the shared export set + the headers this tier
# generates. The umbrella target, public-header tree, and package config are
# finalized by the parent (src/CMakeLists.txt).
if(VR_INSTALL)
  install(
    TARGETS recon_core
    EXPORT volumetric_kit_reconTargets
    ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
  install(
    DIRECTORY "${_vr_generated_inc}/"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
    FILES_MATCHING
    PATTERN "*.hpp")
endif()
