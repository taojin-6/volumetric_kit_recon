# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# Pinned dependencies (FetchContent). recon takes Vulkan and glm from the system
# (find_package in the root + core CMakeLists). In order: the family's
# volumetric_kit_core, the in-tree sensor package (sensor/), and tinyply,
# header-only with no system package to rely on across our targets
# (Linux/Android/macOS/iOS). The fetches run on the first configure (a network
# is required then; afterwards the FetchContent cache serves it).

include(FetchContent)

# --- volumetric_kit_core: the family's shared foundation ---------------------
# recon's Status / Result / VKC_CHECK / log sink come from its base tier, and
# its Vulkan foundation -- instance, device, VMA allocator, buffers, images,
# descriptors, compute pipelines, command batches, GPU timers, external memory
# and the SharedDevice -- from its vulkan tier (DECISIONS.md, 2026-10-03 and
# 2026-10-04), so they are the same types calib and gfx use, and one VkDevice
# can serve all three. Unlike the header-only vendored libraries below, it is a
# compiled library and a PUBLIC dependency of recon_core. VMA comes with it: the
# core compiles VMA's implementation privately in core_vulkan, so recon no
# longer fetches or compiles its own (a second VMA_IMPLEMENTATION in the same
# link would collide). Pinned by commit, never main; an application that also
# fetches calib or gfx declares the core first, and FetchContent keeps the first
# declaration of a name, so this pin then yields to the application's, and
# vkc_require_core refuses that core if it is older than VR_CORE_MIN_VERSION or
# lacks the vulkan tier. Work on both checkouts at once with
# -DFETCHCONTENT_SOURCE_DIR_VOLUMETRIC_KIT_CORE=<core checkout>. The core's
# tests and -Werror stay off in a subproject.
#
# The pin is core 0.29.0 (a4a6fff, core #65's merge), as is the Homebrew key
# action in .github/workflows/_build.yml; the vkc_require_core checks below say
# which core each part needs. The tests are main()s of their own on the core's
# test support, which the sensor package's tests (VS_BUILD_TESTS) need too.
FetchContent_Declare(
  volumetric_kit_core
  GIT_REPOSITORY https://github.com/taojin-6/volumetric_kit_core.git
  GIT_TAG a4a6fff4af65b26b25df27c8ed8c9eecbfbfcd87)
set(VKC_WITH_VULKAN ON)
if(VR_BUILD_TESTS OR VS_BUILD_TESTS)
  set(VKC_BUILD_TEST_SUPPORT ON)
endif()
# Scenario parsing is optional and used only by executable examples and their
# host-side adapter tests. Library consumers need no parser.
if(VR_BUILD_EXAMPLES OR VR_BUILD_TESTS)
  set(VKC_WITH_CONFIG ON)
endif()
FetchContent_MakeAvailable(volumetric_kit_core)
if(VR_BUILD_EXAMPLES OR VR_BUILD_TESTS)
  # The examples read their scenarios, and the tests check them, through the
  # config tier; the libraries' floor covers every core version either needs.
  vkc_require_core(${VR_CORE_MIN_VERSION} CONFIG)
endif()

# The set()s above are lost when calib or the application made the core
# available first, without its vulkan tier or its test support.
vkc_require_core(${VR_CORE_MIN_VERSION} VULKAN)
if(VR_BUILD_TESTS AND NOT (TARGET volumetric_kit::core_test_base
                           AND TARGET volumetric_kit::core_test_policy))
  message(
    FATAL_ERROR
      "recon's tests need volumetric_kit_core's test support: set "
      "VKC_BUILD_TEST_SUPPORT ON before the first FetchContent_MakeAvailable "
      "of the core")
endif()

# --- volumetric_kit_sensor: cameras, capture and the frame contract ---------
# The in-tree package (sensor/): its own project, on the core alone, which recon
# adds as a sibling. recon's libraries link its frame contract (sensor_frame)
# alone; the examples assemble its drivers, array and prep with recon's tiers.
# Declared after the core, so the package's own core pin yields to the one
# above, and before anything of recon's. An application that also fetches the
# sensor package declares it first (by URL + SHA + SOURCE_SUBDIR sensor), and
# FetchContent keeps that declaration. Its options (VS_WITH_ORBBEC,
# VS_WITH_FFMPEG, VS_WITH_CUDA, VS_BUILD_TESTS) are given on recon's configure.
FetchContent_Declare(volumetric_kit_sensor SOURCE_DIR
                                           "${PROJECT_SOURCE_DIR}/sensor")
FetchContent_MakeAvailable(volumetric_kit_sensor)
vkc_require_sibling(volumetric_kit::sensor ${VR_SENSOR_MIN_VERSION} TARGETS
                    volumetric_kit::sensor_frame)

# (volumetric_kit_grid comes here, after the sensor package and before tinyply.)

# --- Asset I/O implementation headers (never exported to consumers) --------
# tinyply supplies PLY writing. Its implementation TU lives in recon_mesh_io; no
# helper target leaks through its link interface.
FetchContent_Declare(
  tinyply
  GIT_REPOSITORY https://github.com/ddiakopoulos/tinyply.git
  GIT_TAG c9bb690dfe5e9105961e9e28120c48c9ae084bc6 # 3.0
  GIT_SHALLOW FALSE
  SOURCE_SUBDIR do-not-add-tinyply-cmake)
FetchContent_MakeAvailable(tinyply)
set(VR_TINYPLY_INCLUDE_DIR
    "${tinyply_SOURCE_DIR}/source"
    PARENT_SCOPE)
