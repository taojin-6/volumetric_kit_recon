# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# recon's libraries link the sensor package's frame contract alone: each line of
# LINKS, "<library>: <links>" as tests/CMakeLists.txt wrote them for every
# library target under src/, names no sensor target but sensor_frame -- no other
# tier and not the umbrella. Run as `cmake -DLINKS=<file> -P
# library_sensor_links.cmake`; fails naming every such link.
if(NOT EXISTS "${LINKS}")
  message(FATAL_ERROR "no ${LINKS}: the check reads nothing")
endif()
file(STRINGS "${LINKS}" _vr_lines)
set(_vr_found "")
set(_vr_count 0)
foreach(_vr_line IN LISTS _vr_lines)
  if(NOT _vr_line MATCHES "^([^:]+): ?(.*)$")
    message(FATAL_ERROR "unreadable line in ${LINKS}: ${_vr_line}")
  endif()
  set(_vr_target "${CMAKE_MATCH_1}")
  string(REPLACE " " ";" _vr_links "${CMAKE_MATCH_2}")
  math(EXPR _vr_count "${_vr_count} + 1")
  foreach(_vr_link IN LISTS _vr_links)
    # A static library's private links read $<LINK_ONLY:...> in its interface.
    string(REGEX REPLACE "^\\$<LINK_ONLY:(.*)>$" "\\1" _vr_link "${_vr_link}")
    string(REGEX REPLACE "^volumetric_kit::" "" _vr_name "${_vr_link}")
    if(_vr_name MATCHES "^sensor(_.*)?$" AND NOT _vr_name STREQUAL
                                             "sensor_frame")
      string(APPEND _vr_found "\n  ${_vr_target} links ${_vr_link}")
    endif()
  endforeach()
endforeach()
if(_vr_count EQUAL 0)
  message(FATAL_ERROR "${LINKS} lists no library: the check reads nothing")
endif()
if(_vr_found)
  message(FATAL_ERROR "a recon library links a sensor tier other than "
                      "sensor_frame:${_vr_found}")
endif()
message(STATUS "${_vr_count} recon libraries link sensor_frame alone")
