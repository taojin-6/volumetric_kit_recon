# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# Pinned dependencies (FetchContent). recon takes Vulkan and glm from the system
# (find_package in the root + core CMakeLists). In order: the family's
# volumetric_kit_core, volumetric_kit_sensor and volumetric_kit_grid. The
# fetches run on the first configure (a network is required then; afterwards the
# FetchContent cache serves it).

include(FetchContent)

# --- volumetric_kit_core: the family's shared foundation ---------------------
# recon's Status / Result / VKC_CHECK / log sink come from its base tier, and
# its Vulkan foundation -- instance, device, VMA allocator, buffers, images,
# descriptors, compute pipelines, command batches, GPU timers, external memory
# and the SharedDevice -- from its vulkan tier (DECISIONS.md, 2026-10-03 and
# 2026-10-04), so they are the same types calib and gfx use, and one VkDevice
# can serve all three. It is a compiled library and a PUBLIC dependency of
# recon_core. VMA comes with it: the core compiles VMA's implementation
# privately in core_vulkan, so recon no longer fetches or compiles its own (a
# second VMA_IMPLEMENTATION in the same link would collide). Pinned by commit,
# never main; an application that also fetches calib or gfx declares the core
# first, and FetchContent keeps the first declaration of a name, so this pin
# then yields to the application's, and vkc_require_core refuses that core if it
# is older than VR_CORE_MIN_VERSION or lacks the vulkan tier. Work on both
# checkouts at once with -DFETCHCONTENT_SOURCE_DIR_VOLUMETRIC_KIT_CORE=<core
# checkout>. The core's tests and -Werror stay off in a subproject.
#
# The pin is core 0.31.0 (7850255, the head of core's
# feat/fuse-realsense-profile, whose FuseRealsense profile fuse_realsense
# loads), as are the Homebrew key action in .github/workflows/_build.yml and the
# workflow-trigger check in .github/workflows/lint.yml; the vkc_require_core
# checks below say which core each part needs. The tests are main()s of their
# own on the core's test support. TODO: once that core PR merges, and before
# this pin reaches main, re-pin to its merge commit and set VR_CORE_MIN_VERSION,
# with its comment in the root CMakeLists.txt and the version CHANGELOG and
# DECISIONS' 2026-10-29 entry name, to the version it lands as: another open
# core PR also claims 0.31.0, so it may land as 0.32.0, and a floor left at
# 0.31.0 would then accept a core without the profile.
FetchContent_Declare(
  volumetric_kit_core
  GIT_REPOSITORY https://github.com/taojin-6/volumetric_kit_core.git
  GIT_TAG 78502552393f4f2cac0ce0409fb1c8950d5c3e5d)
set(VKC_WITH_VULKAN ON)
if(VR_BUILD_TESTS)
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
# A repository of its own, on the core alone. recon's libraries link its frame
# contract (sensor_frame) alone; the examples assemble its drivers, array and
# prep with recon's tiers. Declared after the core, so the package's own core
# pin yields to the one above, and before anything of recon's. Pinned by commit,
# never main; an application that also fetches sensor declares it first, and
# FetchContent keeps that declaration, as with the core. Work on both checkouts
# at once with -DFETCHCONTENT_SOURCE_DIR_VOLUMETRIC_KIT_SENSOR=<sensor
# checkout>. Its drivers (VS_WITH_ORBBEC, VS_WITH_REALSENSE, VS_WITH_FFMPEG,
# VS_WITH_CUDA) are given on recon's configure. sensor's own CI tests sensor, so
# its tests and -Werror stay off in this build; recon's examples take its test
# fixtures (VS_TEST_DATA_DIR, which it sets).
#
# The pin is sensor 0.5.0 (ffa20ea, sensor #5's merge), as are its Orbbec SDK
# and librealsense actions in .github/workflows (the sibling-actions-at-pins
# hook).
FetchContent_Declare(
  volumetric_kit_sensor
  GIT_REPOSITORY https://github.com/taojin-6/volumetric_kit_sensor.git
  GIT_TAG ffa20eac6da8fcfbbfc2475e1f8bd8b4fbbec76a)
FetchContent_MakeAvailable(volumetric_kit_sensor)
vkc_require_sibling(volumetric_kit::sensor ${VR_SENSOR_MIN_VERSION} TARGETS
                    volumetric_kit::sensor_frame)

# --- volumetric_kit_grid: voxel hashing, meshing and the codec ----------------
# A repository of its own, on the core alone, declared after the core and before
# recon's tiers, which link it. Pinned by commit, never main; an application
# that also fetches grid declares it first, and FetchContent keeps that
# declaration, as with the core; vkc_require_sibling then refuses a grid older
# than VR_GRID_MIN_VERSION. Work on both checkouts at once with
# -DFETCHCONTENT_SOURCE_DIR_VOLUMETRIC_KIT_GRID=<grid checkout>. grid's own CI
# tests grid, so its tests and -Werror stay off in this build. recon's tests
# turn on grid's test support, and recon's examples and tests its example
# helpers (grid_example_common), which grid builds with its examples.
#
# The pin is grid 0.3.1 (d1fcbc4, grid #8's merge).
FetchContent_Declare(
  volumetric_kit_grid
  GIT_REPOSITORY https://github.com/taojin-6/volumetric_kit_grid.git
  GIT_TAG d1fcbc4faa46f0cfe1be146bdb9eb7e1732ca30d)
set(VGRID_BUILD_TEST_SUPPORT ${VR_BUILD_TESTS})
if(VR_BUILD_EXAMPLES OR VR_BUILD_TESTS)
  set(VGRID_BUILD_EXAMPLES ON)
endif()
FetchContent_MakeAvailable(volumetric_kit_grid)
vkc_require_sibling(volumetric_kit::grid ${VR_GRID_MIN_VERSION})
if(VR_BUILD_TESTS AND NOT TARGET volumetric_kit::grid_test_support)
  message(
    FATAL_ERROR
      "recon's tests need volumetric_kit_grid's test support: set "
      "VGRID_BUILD_TEST_SUPPORT ON before the first FetchContent_MakeAvailable "
      "of grid")
endif()
if((VR_BUILD_EXAMPLES OR VR_BUILD_TESTS) AND NOT TARGET grid_example_common)
  message(
    FATAL_ERROR
      "recon's examples and their tests need volumetric_kit_grid's example "
      "helpers: set VGRID_BUILD_EXAMPLES ON before the "
      "first FetchContent_MakeAvailable of grid")
endif()
