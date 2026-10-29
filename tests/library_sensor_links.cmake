# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# recon's libraries link the sensor package's frame contract alone: each line of
# LINKS, "<library>: <links>" as tests/CMakeLists.txt wrote them for every
# library target under src/, names no sensor target but sensor_frame -- no other
# tier and not the umbrella. Every name in an item counts, so no generator
# expression ($<LINK_ONLY:...>, $<BUILD_INTERFACE:...>, a condition) hides one.
# Direct links suffice: every recon library is read, and the other first-party
# packages recon links (the core, grid) sit beside or below the sensor package
# and link none of it. Run as `cmake -DLINKS=<file> -P
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
    string(REGEX MATCHALL "[A-Za-z0-9_.+-]+(::[A-Za-z0-9_.+-]+)*" _vr_names
                 "${_vr_link}")
    foreach(_vr_name IN LISTS _vr_names)
      string(REGEX REPLACE "^volumetric_kit::" "" _vr_name "${_vr_name}")
      if(_vr_name MATCHES "^sensor(_.*)?$" AND NOT _vr_name STREQUAL
                                               "sensor_frame")
        string(APPEND _vr_found "\n  ${_vr_target} links ${_vr_link}")
        break()
      endif()
    endforeach()
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
