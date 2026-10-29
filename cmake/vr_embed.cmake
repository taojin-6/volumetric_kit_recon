# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# vr_embed_shaders(<target> [SYMBOL_PREFIX <prefix>] [INCLUDE_TARGETS
# <target>...] [EXPORTED_FROM <target>...] SHADERS <file>...)
#
# The core's vkc_embed_shaders with spirv-val's --scalar-block-layout, the
# buffer ABI the kernels declare (2026-07-05), and the vr_ symbol prefix unless
# another is given: hash.comp embeds as hash_comp.spv.hpp, declaring
# `vr_hash_comp_spv[]` and `vr_hash_comp_spv_size`. The shaders include through
# recon's src/, so a cross-tier include reads like its C++ header path
# ("volumetric_kit/recon/tsdf/shaders/tsdf_common.glsl"), and, with
# EXPORTED_FROM, through those targets' exported GLSL too (a sibling package's,
# such as sensor_frame's pinhole.glsl); with INCLUDE_TARGETS, through those
# targets' exported GLSL alone instead, as a project that fetches recon does.
function(vr_embed_shaders target)
  cmake_parse_arguments(ARG "" "SYMBOL_PREFIX"
                        "INCLUDE_TARGETS;EXPORTED_FROM;SHADERS" ${ARGN})
  if(NOT ARG_SYMBOL_PREFIX)
    set(ARG_SYMBOL_PREFIX vr_)
  endif()
  if(ARG_INCLUDE_TARGETS)
    set(_includes INCLUDE_TARGETS ${ARG_INCLUDE_TARGETS})
  else()
    set(_includes INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/src")
    if(ARG_EXPORTED_FROM)
      list(APPEND _includes INCLUDE_TARGETS ${ARG_EXPORTED_FROM})
    endif()
  endif()
  vkc_embed_shaders(
    ${target}
    SYMBOL_PREFIX
    ${ARG_SYMBOL_PREFIX}
    ${_includes}
    SPIRV_VAL_ARGS
    --scalar-block-layout
    ${ARG_UNPARSED_ARGUMENTS}
    SHADERS
    ${ARG_SHADERS})
endfunction()
