# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# Sanitizer flags, applied GLOBALLY (compile + link) when VR_SANITIZE is set.
#
# Unlike vr_warnings (per-target PRIVATE), sanitizers are applied globally on
# purpose: the flags must reach the link line, and ASan's ABI is contagious --
# mixing a sanitized library with an un-sanitized executable is broken.
# Including this before add_subdirectory(third_party) also instruments any
# vendored static deps, which otherwise raise alloc/dealloc-mismatch false
# positives when linked into a sanitized test binary. Off by default, so normal
# builds are unaffected.
#
# Example: cmake -B build -DCMAKE_BUILD_TYPE=Debug
# -DVR_SANITIZE="address;undefined"
if(VR_SANITIZE)
  if(MSVC)
    message(
      FATAL_ERROR
        "VR_SANITIZE is not supported with MSVC; use a Clang or GCC build.")
  endif()

  # address;undefined -> address,undefined (the -fsanitize= argument form).
  list(JOIN VR_SANITIZE "," _vr_sanitize_list)
  set(_vr_sanitize_flags -fsanitize=${_vr_sanitize_list}
                         -fno-omit-frame-pointer -fno-sanitize-recover=all)

  add_compile_options(${_vr_sanitize_flags})
  add_link_options(${_vr_sanitize_flags})

  message(STATUS "Sanitizers enabled (VR_SANITIZE): ${VR_SANITIZE}")
endif()
