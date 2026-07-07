# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# Example programs: the end-to-end dataset fuse pipeline (and, later, its live
# gfx viewer). Built when VR_BUILD_EXAMPLES is on. They link the recon tiers
# plus stb_image for RGB/depth decode -- an examples-only dependency; the
# shipped tiers take no image dependency, so this fetch is scoped here (not in
# third_party/, which is compiled into recon_core unconditionally).

include(FetchContent)

# stb: single-header image loaders (JPEG colour + 16-bit PNG depth). No CMake or
# tagged releases upstream, so pin a commit and expose the header directory as
# an INTERFACE target; SYSTEM include so the header stays off the tiers' -Werror
# surface wherever it is pulled in for its declarations.
FetchContent_Declare(
  stb
  GIT_REPOSITORY https://github.com/nothings/stb.git
  GIT_TAG 31c1ad37456438565541f4919958214b6e762fb4
  GIT_SHALLOW FALSE)
FetchContent_MakeAvailable(stb)

add_library(vr_stb_image INTERFACE)
target_include_directories(vr_stb_image SYSTEM INTERFACE "${stb_SOURCE_DIR}")

add_subdirectory(common)
add_subdirectory(fuse_replica)
