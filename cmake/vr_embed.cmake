# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# vr_embed_shaders(<target> [SYMBOL_PREFIX <prefix>] SHADERS <file>...)
#
# Compiles each GLSL shader to SPIR-V (via vr_compile_shaders) and embeds the
# result into <target> as a constexpr byte-array header on a PRIVATE,
# uninstalled include path -- so <target> ships the shader technique with no
# runtime files. For a shader <stem>.<stage> (e.g. hash.comp) the generated
# header is <stem>_<stage>.spv.hpp, declaring `<prefix><stem>_<stage>_spv[]`
# (4-byte aligned, reinterpret-able to const uint32_t*) and `..._spv_size` (byte
# count) at global scope; <target>'s sources #include that header by basename.
# SYMBOL_PREFIX defaults to `vr_`. Safe to call more than once per target.
# Mirrors volumetric_kit_gfx's vg_embed_shaders.
include(vr_shaders)

function(vr_embed_shaders target)
  cmake_parse_arguments(ARG "" "SYMBOL_PREFIX" "SHADERS" ${ARGN})
  if(NOT ARG_SHADERS)
    message(FATAL_ERROR "vr_embed_shaders(${target}): no SHADERS given")
  endif()
  if(NOT ARG_SYMBOL_PREFIX)
    set(ARG_SYMBOL_PREFIX "vr_")
  endif()

  # The embedder cmake -P script sits next to this module.
  set(_embed_script "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/embed_spirv.cmake")

  # Compile the GLSL to SPIR-V on a private scratch dir, then turn each .spv
  # into a header. Both dirs live in the build tree and are never installed.
  set(_spv_dir "${CMAKE_CURRENT_BINARY_DIR}/${target}_embed_spv")
  set(_inc_dir "${CMAKE_CURRENT_BINARY_DIR}/${target}_embed_inc")
  vr_compile_shaders(${target} OUTPUT_DIR "${_spv_dir}" SHADERS ${ARG_SHADERS})
  # The custom target vr_compile_shaders just created to produce the .spv files;
  # captured so the embed target can be serialized after it (see below).
  set(_compile_target "${_vr_compile_shaders_target}")

  set(_headers)
  foreach(_src IN LISTS ARG_SHADERS)
    get_filename_component(_name "${_src}" NAME) # e.g. hash_init.comp
    string(REPLACE "." "_" _stem "${_name}") # e.g. hash_init_comp
    set(_spv "${_spv_dir}/${_name}.spv")
    set(_header "${_inc_dir}/${_stem}.spv.hpp")
    # The .spv is an OUTPUT of the vr_compile_shaders custom command above (same
    # directory scope), so this file-level DEPENDS orders compile before embed
    # under both Make and Ninja.
    add_custom_command(
      OUTPUT "${_header}"
      COMMAND ${CMAKE_COMMAND} -E make_directory "${_inc_dir}"
      COMMAND
        ${CMAKE_COMMAND} "-DSPV=${_spv}"
        "-DSYMBOL=${ARG_SYMBOL_PREFIX}${_stem}_spv" "-DOUT=${_header}" -P
        "${_embed_script}"
      DEPENDS "${_spv}" "${_embed_script}"
      COMMENT "Embedding ${_name}.spv"
      VERBATIM)
    list(APPEND _headers "${_header}")
  endforeach()

  # An aggregate target carrying the generated headers; <target> depends on it
  # so they exist before any of <target>'s sources compile. The name is
  # uniquified via a build-global counter so repeated calls don't collide on one
  # name.
  get_property(_seq GLOBAL PROPERTY _vr_embed_set_seq)
  if(NOT _seq)
    set(_seq 0)
  endif()
  math(EXPR _seq "${_seq} + 1")
  set_property(GLOBAL PROPERTY _vr_embed_set_seq "${_seq}")

  add_custom_target(${target}_embedded_shaders_${_seq} DEPENDS ${_headers})
  add_dependencies(${target} ${target}_embedded_shaders_${_seq})

  # Serialize the embed target after the compile target. The .spv files are
  # outputs of ${_compile_target} (a dependency of ${target}) AND dependencies
  # of the embed headers above, so both custom targets can reach the same .spv
  # rule. With no ordering between them, `make -j` runs that glslc recipe from
  # both targets' makefiles concurrently and truncates the file -- spirv-val
  # then reports "Missing OpFunctionEnd", and MoltenVK "Function was not
  # terminated" at MSL-conversion time. Building compile fully first makes each
  # .spv appear exactly once. (Ninja dedups on a global graph and is unaffected;
  # this is a Makefile-generator hazard, and it grows with the shader count.)
  add_dependencies(${target}_embedded_shaders_${_seq} ${_compile_target})
  target_include_directories(${target} PRIVATE "${_inc_dir}")
endfunction()
