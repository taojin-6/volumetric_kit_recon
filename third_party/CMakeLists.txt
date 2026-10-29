# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# Vendored dependencies (pinned FetchContent). recon takes Vulkan and glm from
# the system (find_package in the root + core CMakeLists). Besides the family's
# volumetric_kit_core and grid, what is vendored here is header-only with no
# system package to rely on across our targets (Linux/Android/macOS/iOS):
# nlohmann/json and stb below. The root CMakeLists adds this subdirectory
# unconditionally and the fetches below run on the first configure (a network is
# required then; afterwards the FetchContent cache serves it); VR_CAMERA_ONLY
# stops after the core and nlohmann/json.

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
# lacks the vulkan tier outside VR_CAMERA_ONLY. Work on both checkouts at once
# with -DFETCHCONTENT_SOURCE_DIR_VOLUMETRIC_KIT_CORE=<core checkout>. The core's
# tests and -Werror stay off in a subproject. VR_CAMERA_ONLY needs the base tier
# alone. It leaves VKC_WITH_VULKAN unset rather than OFF, as calib does, so an
# application that turns the tier on keeps it.
#
# The pin is core 0.29.0 (a4a6fff, core #65's merge), as is the Homebrew key
# action in .github/workflows/_build.yml; the vkc_require_core checks below say
# which core each part needs. The tests are main()s of their own on the core's
# test support, whose Vulkan-free part a camera-only build's tests use too.
FetchContent_Declare(
  volumetric_kit_core
  GIT_REPOSITORY https://github.com/taojin-6/volumetric_kit_core.git
  GIT_TAG a4a6fff4af65b26b25df27c8ed8c9eecbfbfcd87)
if(NOT VR_CAMERA_ONLY)
  set(VKC_WITH_VULKAN ON)
endif()
if(VR_BUILD_TESTS)
  set(VKC_BUILD_TEST_SUPPORT ON)
endif()
# Scenario parsing is optional and used only by executable examples and their
# host-side adapter tests. Camera-only and library consumers need no parser.
if(NOT VR_CAMERA_ONLY AND (VR_BUILD_EXAMPLES OR VR_BUILD_TESTS))
  set(VKC_WITH_CONFIG ON)
endif()
FetchContent_MakeAvailable(volumetric_kit_core)
if(NOT VR_CAMERA_ONLY AND (VR_BUILD_EXAMPLES OR VR_BUILD_TESTS))
  # The examples read their scenarios, and the tests check them, through the
  # config tier; the libraries' floor covers every core version either needs.
  vkc_require_core(${VR_CORE_MIN_VERSION} CONFIG)
endif()

# The set()s above are lost when calib or the application made the core
# available first, without its vulkan tier or its test support.
if(VR_CAMERA_ONLY)
  vkc_require_core(${VR_CORE_MIN_VERSION})
else()
  vkc_require_core(${VR_CORE_MIN_VERSION} VULKAN)
endif()
if(VR_BUILD_TESTS
   AND NOT (TARGET volumetric_kit::core_test_base
            AND (VR_CAMERA_ONLY OR TARGET volumetric_kit::core_test_policy)))
  message(
    FATAL_ERROR
      "recon's tests need volumetric_kit_core's test support: set "
      "VKC_BUILD_TEST_SUPPORT ON before the first FetchContent_MakeAvailable "
      "of the core")
endif()

# --- nlohmann/json (header-only; private to the targets that parse JSON) -----
# recon_camera parses the sensor array's calibration file with it, so a
# camera-only build fetches it too, and recon_sensor_orbbec its sync file. The
# release's source archive, hash-pinned; only its include/ tree is used, so its
# CMake is not run (SOURCE_SUBDIR names a directory the archive does not have).
# It is a PRIVATE SYSTEM include of each target that parses, never part of its
# interface. volumetric_kit_gfx carries the same library through tinygltf.
if(POLICY CMP0135)
  cmake_policy(SET CMP0135 NEW) # extracted files get the extraction time
endif()
FetchContent_Declare(
  nlohmann_json
  URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz
  URL_HASH
    SHA256=42f6e95cad6ec532fd372391373363b62a14af6d771056dbfc86160e6dfff7aa
  SOURCE_SUBDIR do-not-add-json-cmake)
FetchContent_MakeAvailable(nlohmann_json)

set(VR_JSON_INCLUDE_DIR
    "${nlohmann_json_SOURCE_DIR}/include"
    PARENT_SCOPE)

# The camera tier needs nothing below.
if(VR_CAMERA_ONLY)
  return()
endif()

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
# The pin is grid 0.1.0 (cbf7a6e, its repository's first main).
FetchContent_Declare(
  volumetric_kit_grid
  GIT_REPOSITORY https://github.com/taojin-6/volumetric_kit_grid.git
  GIT_TAG cbf7a6ea2b8729645b24503063ff7a076706f698)
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

# --- Asset I/O implementation headers (never exported to consumers) --------
# stb supplies image decoding and PNG writing. Its implementation TU lives in
# recon_image_io; no helper target leaks through its link interface.
FetchContent_Declare(
  stb
  GIT_REPOSITORY https://github.com/nothings/stb.git
  GIT_TAG 2c980bb59875b0d32144a71867fbdebb2f77cd20
  GIT_SHALLOW FALSE)
FetchContent_MakeAvailable(stb)
set(VR_STB_INCLUDE_DIR
    "${stb_SOURCE_DIR}"
    PARENT_SCOPE)
