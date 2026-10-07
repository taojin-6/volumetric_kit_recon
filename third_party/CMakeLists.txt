# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# Vendored dependencies (pinned FetchContent). recon takes Vulkan and glm from
# the system (find_package in the root + core CMakeLists). Besides the family's
# volumetric_kit_core, what is vendored here is header-only with no system
# package to rely on across our targets (Linux/Android/macOS/iOS):
# nlohmann/json, stb and tinyply below. The root CMakeLists adds this
# subdirectory unconditionally and the fetches below run on the first configure
# (a network is required then; afterwards the FetchContent cache serves it).

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
# declaration of a name, so this pin then yields to the application's. Work on
# both checkouts at once with
# -DFETCHCONTENT_SOURCE_DIR_VOLUMETRIC_KIT_CORE=<core checkout>. The core's
# tests and -Werror stay off in a subproject. Its install rules follow
# VR_INSTALL: recon's install must carry the core its exported targets link, and
# an application that embeds recon without installing it gets none.
#
# The pin is main after core PR #15. Its PR #11 added Device::check_enabled,
# which recon's kernels call; PR #12 bounds vk_result to VkResult's range; PR
# #13 sets the oldest Vulkan headers the tier builds on (1.3.204, 1.3.208 on
# Apple) and adds format.hpp; PR #15 adds CommandBatch::submit_async, which the
# pipelined stages will submit through (DESIGN.md's Next work).
#
# TODO: pin a release tag once the core tags one (v0.1.0).
FetchContent_Declare(
  volumetric_kit_core
  GIT_REPOSITORY https://github.com/taojin-6/volumetric_kit_core.git
  GIT_TAG a8b63d1dd1b29c4ddd8cf0701a4278ee994c2982)
set(VKC_INSTALL ${VR_INSTALL})
set(VKC_WITH_VULKAN ON)
FetchContent_MakeAvailable(volumetric_kit_core)
# The set() above is lost when calib or the application made the core available
# first, without its vulkan tier.
if(NOT TARGET volumetric_kit::core_vulkan)
  message(
    FATAL_ERROR
      "recon needs volumetric_kit_core's vulkan tier: set VKC_WITH_VULKAN ON "
      "before the first FetchContent_MakeAvailable of the core")
endif()

# --- nlohmann/json (header-only; parsed in one recon_sensor TU) ---------------
# The rig pose file (sensor/rig_poses.hpp) is JSON. The release's source
# archive, hash-pinned; only its include/ tree is used, so its CMake is not run
# (SOURCE_SUBDIR names a directory the archive does not have). It is a PRIVATE
# SYSTEM include of the one target that parses, never part of the installed
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

# --- Asset I/O implementation headers (never exported to consumers) --------
# stb supplies image decoding and PNG writing. tinyply supplies PLY writing.
# Their implementation TUs live in recon_io; no helper target leaks through the
# installed static-library link interface.
FetchContent_Declare(
  stb
  GIT_REPOSITORY https://github.com/nothings/stb.git
  GIT_TAG 2c980bb59875b0d32144a71867fbdebb2f77cd20
  GIT_SHALLOW FALSE)
FetchContent_MakeAvailable(stb)
set(VR_STB_INCLUDE_DIR
    "${stb_SOURCE_DIR}"
    PARENT_SCOPE)

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
