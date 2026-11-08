# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# fuse_render on a device: two frames of a wall 1 m away (DEPTH and COLOR, as
# example_fuse_replica_run.cmake's), textured with the second, crossed to gfx's
# own device and rendered to a 320x200 PNG. The PNG's header carries that size,
# and the pixels fuse_render says it drew are a fair part of the frame, not none
# and not all. Under the core's test policy (VKC_TEST_VALIDATION, or
# VKC_TEST_SYNC_VALIDATION, which this run takes as plain validation) the loader
# forces the validation layer on, and the run fails unless the loader's log
# shows it in both instances, recon's and gfx's; with no messenger installed the
# layer writes to stdout, and an error it reports fails the run. Skips with no
# Vulkan device unless VKC_REQUIRE_VULKAN_DEVICE is set. A run that hangs is
# ended after 240 s, inside the test's 300, and fails with what it printed.

file(REMOVE_RECURSE "${WORK_DIR}")
file(MAKE_DIRECTORY "${WORK_DIR}/wall/results")
file(
  WRITE "${WORK_DIR}/cam_params.json"
  "{\"camera\": {\"w\": 256, \"h\": 144, \"fx\": 200.0, \"fy\": 200.0, \"cx\": 127.5, \"cy\": 71.5, \"scale\": 1000.0}}\n"
)
file(WRITE "${WORK_DIR}/wall/traj.txt"
     "1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1\n1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1\n")
foreach(frame 0 1)
  configure_file("${COLOR}" "${WORK_DIR}/wall/results/frame00000${frame}.jpg"
                 COPYONLY)
  configure_file("${DEPTH}" "${WORK_DIR}/wall/results/depth00000${frame}.png"
                 COPYONLY)
endforeach()
file(
  WRITE "${WORK_DIR}/run.json"
  "{\"version\":1,\"operation\":\"fuse\",\"source\":{\"kind\":\"dataset\",\"path\":\"wall\"},\"fusion\":{\"voxel_size_m\":0.02},\"window\":{\"width\":320,\"height\":200},\"output\":{\"path\":\"wall.png\"}}\n"
)

# The loader skips a layer it cannot find, and one older than 1.3.234 ignores
# VK_LOADER_LAYERS_ENABLE, so its log is what shows the layer on.
set(validation OFF)
foreach(name VKC_TEST_VALIDATION VKC_TEST_SYNC_VALIDATION)
  if(NOT "$ENV{${name}}" MATCHES "^0?$")
    set(validation ON)
  endif()
endforeach()
if(validation)
  set(ENV{VK_LOADER_LAYERS_ENABLE} "*validation")
  set(ENV{VK_LOADER_DEBUG} "layer")
endif()
execute_process(
  COMMAND "${PROGRAM}" --config "${WORK_DIR}/run.json"
  TIMEOUT 240
  RESULT_VARIABLE result
  OUTPUT_VARIABLE output
  ERROR_VARIABLE error)
string(REGEX MATCHALL "Insert instance layer \"?VK_LAYER_KHRONOS_validation"
             inserted "${error}")
list(LENGTH inserted validated)
# The loader's lines name Vulkan, which the skip rule looks for.
string(REGEX REPLACE "\\[Vulkan Loader\\][^\n]*\n" "" error "${error}")
if(NOT result EQUAL 0)
  if(error MATCHES "Vulkan|vkCreate|physical device|VK_ERROR"
     AND "$ENV{VKC_REQUIRE_VULKAN_DEVICE}" STREQUAL "")
    message(STATUS "skipping: no Vulkan device (${error})")
    return()
  endif()
  message(FATAL_ERROR "The run failed: ${result}: ${output}${error}")
endif()
if(validation AND validated LESS 2)
  message(FATAL_ERROR "The loader put the validation layer in ${validated} of "
                      "the 2 instances: is it on VK_LAYER_PATH?")
endif()
if(output MATCHES "Validation Error")
  message(FATAL_ERROR "The validation layer reported an error: ${output}")
endif()
foreach(expected "fused 2 frames\n" "textured with frame 1 \\(256x144 atlas\\)")
  if(NOT output MATCHES "${expected}")
    message(FATAL_ERROR "No \"${expected}\": ${output}")
  endif()
endforeach()

# The PNG's IHDR: the signature, the chunk's length and name, then the width and
# height as big-endian 32-bit words, 8-bit RGBA.
file(
  READ "${WORK_DIR}/wall.png" header
  LIMIT 26
  HEX)
if(NOT header MATCHES
   "^89504e470d0a1a0a0000000d49484452(........)(........)0806$")
  message(FATAL_ERROR "Not an 8-bit RGBA PNG: ${header}")
endif()
math(EXPR width "0x${CMAKE_MATCH_1}")
math(EXPR height "0x${CMAKE_MATCH_2}")
math(EXPR pixels "${width} * ${height}")
if(NOT width EQUAL 320 OR NOT height EQUAL 200)
  message(FATAL_ERROR "The PNG is ${width}x${height}, not 320x200")
endif()

# The wall fills about a seventh of the frame from the default orbit; at least a
# twentieth catches a blank or sliver frame, and fewer than all a missing clear.
if(NOT output MATCHES "drew ([0-9]+) of ${pixels} pixels")
  message(FATAL_ERROR "No count of the ${pixels} pixels drawn: ${output}")
endif()
set(drawn ${CMAKE_MATCH_1})
math(EXPR floor "${pixels} / 20")
if(drawn LESS floor OR NOT drawn LESS pixels)
  message(FATAL_ERROR "Drew ${drawn} of ${pixels} pixels: ${output}")
endif()
file(REMOVE_RECURSE "${WORK_DIR}")
