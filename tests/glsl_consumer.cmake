# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# What a project whose kernels include recon's exported GLSL does, for the
# subproject/ consumer: compile through the core's INCLUDE_TARGETS, with
# vkc_compile_shaders, a kernel that includes core/shaders/pinhole.glsl
# (volumetric_kit::recon_core; shaders/pinhole_cases.comp). The
# recon_subproject_glsl test builds the target, vr_glsl_consumer. The include
# root holds exactly the GLSL recon_core exports, none of recon's own. grid's
# consumer checks grid's roots the same way (grid/tests/glsl_consumer.cmake).
get_target_property(_roots volumetric_kit::recon_core VKC_SHADER_INCLUDE_DIRS)
set(_held "")
foreach(_root IN LISTS _roots)
  file(
    GLOB_RECURSE _files
    LIST_DIRECTORIES false
    RELATIVE "${_root}"
    "${_root}/*")
  list(APPEND _held ${_files})
endforeach()
set(_want volumetric_kit/recon/core/shaders/pinhole.glsl)
if(NOT _held STREQUAL _want)
  message(FATAL_ERROR "recon_core's GLSL include root holds [${_held}], "
                      "not exactly what it exports, [${_want}]")
endif()

add_custom_target(vr_glsl_consumer)
vkc_compile_shaders(
  vr_glsl_consumer INCLUDE_TARGETS volumetric_kit::recon_core SHADERS
  "${CMAKE_CURRENT_LIST_DIR}/shaders/pinhole_cases.comp")
