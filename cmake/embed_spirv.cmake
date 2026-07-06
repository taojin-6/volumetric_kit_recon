# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# Generate a C++ header embedding a compiled SPIR-V binary as a constexpr byte
# array, so a library tier can ship a shader technique without runtime files.
#
# Run as a script step: cmake -DSPV=<in.spv> -DSYMBOL=<name> -DOUT=<out.hpp> -P
# embed_spirv.cmake
#
# Emits (at global scope, header-guarded): an `alignas(4) inline constexpr
# unsigned char <SYMBOL>[]` (4-byte aligned so it reinterprets to const
# uint32_t* for VkShaderModuleCreateInfo) and `inline constexpr size_t
# <SYMBOL>_size` (the byte count). Mirrors volumetric_kit_gfx's
# embed_spirv.cmake.
if(NOT SPV
   OR NOT SYMBOL
   OR NOT OUT)
  message(FATAL_ERROR "embed_spirv.cmake: SPV, SYMBOL, and OUT are required")
endif()

file(READ "${SPV}" _hex HEX)
string(LENGTH "${_hex}" _hexlen)
math(EXPR _bytes "${_hexlen} / 2")
# An empty input would emit `unsigned char <SYMBOL>[] = {}` -- a zero-size array
# that is ill-formed under -Wpedantic -Werror. Fail loudly at codegen, naming
# the offending file, instead of cryptically at compile.
if(_bytes EQUAL 0)
  message(
    FATAL_ERROR "embed_spirv.cmake: '${SPV}' is empty -- nothing to embed")
endif()
# "07230203..." -> "0x07,0x23,0x02,0x03,..." (trailing comma is a valid
# initializer).
string(REGEX REPLACE "(..)" "0x\\1," _arr "${_hex}")

file(
  WRITE "${OUT}"
  "// Generated from ${SPV} by embed_spirv.cmake -- do not edit.\n"
  "#pragma once\n"
  "#include <cstddef>\n"
  "alignas(4) inline constexpr unsigned char ${SYMBOL}[] = {${_arr}};\n"
  "inline constexpr size_t ${SYMBOL}_size = ${_bytes};\n")
