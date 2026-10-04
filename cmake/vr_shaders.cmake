# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# vr_compile_shaders(<target> [OUTPUT_DIR <dir>] SHADERS <file>...)
#
# The core's vkc_compile_shaders with recon's options: the calling project's
# src/ as the #include root, so a cross-tier include reads like its C++ header
# path ("volumetric_kit/recon/core/shaders/color_common.glsl"), and spirv-val's
# --scalar-block-layout, the buffer ABI the kernels declare (2026-07-05).
function(vr_compile_shaders target)
  vkc_compile_shaders(${target} INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/src"
                      SPIRV_VAL_ARGS --scalar-block-layout ${ARGN})
endfunction()
