# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# vr_embed_shaders(<target> [SYMBOL_PREFIX <prefix>] SHADERS <file>...)
#
# The core's vkc_embed_shaders with vr_compile_shaders' options, and the vr_
# symbol prefix unless another is given: hash.comp embeds as hash_comp.spv.hpp,
# declaring `vr_hash_comp_spv[]` and `vr_hash_comp_spv_size`.
function(vr_embed_shaders target)
  cmake_parse_arguments(ARG "" "SYMBOL_PREFIX" "" ${ARGN})
  if(NOT ARG_SYMBOL_PREFIX)
    set(ARG_SYMBOL_PREFIX vr_)
  endif()
  vkc_embed_shaders(
    ${target}
    SYMBOL_PREFIX
    ${ARG_SYMBOL_PREFIX}
    INCLUDE_DIRS
    "${PROJECT_SOURCE_DIR}/src"
    SPIRV_VAL_ARGS
    --scalar-block-layout
    ${ARG_UNPARSED_ARGUMENTS})
endfunction()
