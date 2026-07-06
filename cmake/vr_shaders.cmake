# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# vr_compile_shaders(<target> [OUTPUT_DIR <dir>] SHADERS <file>...)
#
# Compiles each GLSL shader to <dir>/<name>.spv (e.g. fill.comp ->
# fill.comp.spv) and makes <target> depend on the results, so building <target>
# (re)compiles any stale shader. Uses the SPIR-V compiler that
# find_package(Vulkan) located -- glslc (shaderc) preferred, glslangValidator as
# fallback -- and only errors when invoked, so a build compiling no shaders
# needs no compiler installed. Safe to call more than once per target.
#
# Targets Vulkan 1.2 -- recon's device floor (timeline semaphores; scalar block
# layout, the buffer ABI the compute shaders read, is 1.2 core). Mirrors
# volumetric_kit_gfx's vg_compile_shaders (which targets 1.3 for its renderer).
function(vr_compile_shaders target)
  cmake_parse_arguments(ARG "" "OUTPUT_DIR" "SHADERS" ${ARGN})
  if(NOT ARG_SHADERS)
    message(FATAL_ERROR "vr_compile_shaders(${target}): no SHADERS given")
  endif()
  if(NOT ARG_OUTPUT_DIR)
    set(ARG_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/shaders")
  endif()

  if(Vulkan_GLSLC_EXECUTABLE)
    set(_compiler "${Vulkan_GLSLC_EXECUTABLE}")
    set(_mode glslc)
  elseif(Vulkan_GLSLANG_VALIDATOR_EXECUTABLE)
    set(_compiler "${Vulkan_GLSLANG_VALIDATOR_EXECUTABLE}")
    set(_mode glslang)
  else()
    message(
      FATAL_ERROR
        "vr_compile_shaders(${target}): no GLSL->SPIR-V compiler found. Install "
        "shaderc (glslc) or glslang -- both ship with the Vulkan SDK (macOS: "
        "`brew install shaderc`).")
  endif()

  # spirv-val (SPIRV-Tools) is an optional post-compile validation pass, run
  # only when found on PATH so a toolchain without it still builds.
  find_program(VR_SPIRV_VAL NAMES spirv-val)

  set(_spv_outputs)
  set(_seen_names)
  foreach(_src IN LISTS ARG_SHADERS)
    get_filename_component(_name "${_src}" NAME)
    get_filename_component(_abs "${_src}" ABSOLUTE)
    # Outputs are keyed by basename, so two sources that share one (from
    # different directories) would clobber each other -- reject that up front.
    if(_name IN_LIST _seen_names)
      message(
        FATAL_ERROR
          "vr_compile_shaders(${target}): duplicate shader name '${_name}'")
    endif()
    list(APPEND _seen_names "${_name}")

    set(_out "${ARG_OUTPUT_DIR}/${_name}.spv")

    # The per-call _seen_names check above only sees one invocation; also track
    # outputs build-globally so two vr_compile_shaders() calls writing the same
    # path (shared basename + OUTPUT_DIR) are rejected instead of silently
    # clobbering under Make / erroring under Ninja.
    get_property(_all_outputs GLOBAL PROPERTY _vr_shader_outputs)
    if(_out IN_LIST _all_outputs)
      message(
        FATAL_ERROR
          "vr_compile_shaders(${target}): output '${_out}' is already produced "
          "by another vr_compile_shaders() call; pass a distinct OUTPUT_DIR")
    endif()
    set_property(GLOBAL APPEND PROPERTY _vr_shader_outputs "${_out}")

    # Emit a depfile alongside the .spv so an edited GLSL #include triggers a
    # recompile (the top-level source mtime alone would miss it).
    set(_dep "${_out}.d")
    if(_mode STREQUAL glslc)
      set(_cmd
          "${_compiler}"
          --target-env=vulkan1.2
          -MD
          -MF
          "${_dep}"
          -o
          "${_out}"
          "${_abs}")
    else()
      set(_cmd
          "${_compiler}"
          -V
          --target-env
          vulkan1.2
          --depfile
          "${_dep}"
          -o
          "${_out}"
          "${_abs}")
    endif()

    # Optional: validate the emitted SPIR-V against Vulkan 1.2 rules.
    set(_validate)
    if(VR_SPIRV_VAL)
      # --scalar-block-layout: recon's compute shaders read POD structs through
      # scalar block layout (2026-07-05 ABI), which spirv-val rejects by
      # default.
      set(_validate COMMAND "${VR_SPIRV_VAL}" --target-env vulkan1.2
                    --scalar-block-layout "${_out}")
    endif()

    # make_directory runs at build time (not just configure), so the compile
    # still works if the output dir was cleaned without re-running CMake.
    add_custom_command(
      OUTPUT "${_out}"
      COMMAND ${CMAKE_COMMAND} -E make_directory "${ARG_OUTPUT_DIR}"
      COMMAND ${_cmd} ${_validate}
      DEPENDS "${_abs}"
      DEPFILE "${_dep}"
      COMMENT "Compiling shader ${_name}"
      VERBATIM)
    list(APPEND _spv_outputs "${_out}")
  endforeach()

  # A custom target carrying the .spv outputs; <target> depends on it so the
  # shaders are (re)built ahead of the consumer. The name is uniquified via a
  # build-global counter so repeated calls don't collide on one target name.
  get_property(_seq GLOBAL PROPERTY _vr_shader_set_seq)
  if(NOT _seq)
    set(_seq 0)
  endif()
  math(EXPR _seq "${_seq} + 1")
  set_property(GLOBAL PROPERTY _vr_shader_set_seq "${_seq}")

  add_custom_target(${target}_shaders_${_seq} DEPENDS ${_spv_outputs})
  add_dependencies(${target} ${target}_shaders_${_seq})
endfunction()
