# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# --- recon_core: Tier 1 -----------------------------------------------------
# Foundation tier: what is recon's own beneath every other tier -- the
# requirements recon's kernels place on a device, recon's log source, the camera
# and colour-space vocabulary, and the GLM-backed vector/matrix types -- over
# volumetric_kit_core's error handling, log sink and contract checks (base tier)
# and its Vulkan foundation (vulkan tier), which recon's tiers use under the
# core's names. Every other tier depends on this. Vulkan is reached only through
# the core's core/vulkan/vulkan.hpp umbrella. Deps: volumetric_kit_core,
# header-only GLM and the Vulkan loader/headers.
include(GenerateExportHeader)
include(GNUInstallDirs)

# The core's device serializes submits with std::mutex and recon's sensor tier
# runs capture threads, so everything above recon_core needs the platform
# threading runtime. Link it explicitly rather than relying on the compiler's
# implicit pthread, so the static archive resolves on toolchains that require it
# -- GCC/libstdc++ on Linux with --as-needed, Android, and static links. (macOS
# folds pthread into libc, which would otherwise hide the gap.)
find_package(Threads REQUIRED)

# GLM: header-only vector/matrix math. PUBLIC -- it appears in public headers
# (math/vector_types.hpp aliases glm::vec*/glm::mat4), so consumers need it too
# (the package Config re-finds it). An imported CONFIG target brings glm's
# headers in as SYSTEM includes, so glm internals don't trip -Werror. We link
# the canonical glm::glm target, which every glm packaging provides -- the name
# we link here is baked into the exported target and must resolve on every
# consumer's platform, and the newer glm::glm-header-only target does not exist
# on older packagings (e.g. Ubuntu's libglm-dev), which would break their
# find_package(volumetric_kit_recon). glm is header-only in our use, so linking
# glm::glm carries no runtime library. If a target platform lacks a system glm,
# supply it via the package manager (brew/apt/vcpkg/Conan) or a FetchContent
# pin.
find_package(glm CONFIG REQUIRED)

# The Vulkan loader + headers. PUBLIC: the public headers reach Vulkan through
# the core's core/vulkan/vulkan.hpp and so include <vulkan/vulkan.h>, meaning
# every consumer compiles against it and links the loader (the package Config
# re-finds Vulkan). MoltenVK is the loader/driver on Apple. The core's vulkan
# tier links it too; naming it here keeps recon's own use explicit.
find_package(Vulkan REQUIRED)

set(_vr_generated_inc "${CMAKE_CURRENT_BINARY_DIR}/generated/include")

add_library(recon_core)
add_library(volumetric_kit::recon_core ALIAS recon_core)

# The Vulkan foundation is the core's (core_vulkan); what recon compiles of its
# own here is only the version query. The rest of the tier is header-only.
target_sources(recon_core PRIVATE version.cpp)

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

# PUBLIC: consumers compile our public headers (which include GLM and the core's
# base and vulkan tiers) and, for a static archive, must also link the threading
# runtime -- so the exported target carries these requirements (the Config.cmake
# re-finds Threads, glm, Vulkan and volumetric_kit_core).
target_link_libraries(
  recon_core PUBLIC volumetric_kit::core_base volumetric_kit::core_vulkan
                    Threads::Threads glm::glm Vulkan::Vulkan)

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
  # This tier ships only its own headers: the core/ source subtree plus the
  # generated export.hpp/version.hpp. Scoping to core/ (rather than the whole
  # include/ tree) keeps a core-only build from installing a later tier's
  # headers, which would pull deps this package's Config does not declare. Each
  # future tier adds its own header install the same way.
  install(
    DIRECTORY "${PROJECT_SOURCE_DIR}/include/volumetric_kit/recon/core"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/volumetric_kit/recon"
    FILES_MATCHING
    PATTERN "*.hpp")
  install(
    DIRECTORY "${_vr_generated_inc}/"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
    FILES_MATCHING
    PATTERN "*.hpp")
endif()
