# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# codec_replica on a device: four frames of a wall 1 m away (DEPTH and COLOR, as
# example_fuse_replica_run.cmake's), each coded into a .vrst, an I-frame and
# three P-frames. It prints the per-set table, P-frame and I-frame sets apart,
# and writes the stream. Skips with no Vulkan device unless
# VKC_REQUIRE_VULKAN_DEVICE is set.

file(REMOVE_RECURSE "${WORK_DIR}")
file(MAKE_DIRECTORY "${WORK_DIR}/wall/results")
file(
  WRITE "${WORK_DIR}/cam_params.json"
  "{\"camera\": {\"w\": 256, \"h\": 144, \"fx\": 200.0, \"fy\": 200.0, \"cx\": 127.5, \"cy\": 71.5, \"scale\": 1000.0}}\n"
)
set(pose "1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1\n")
file(WRITE "${WORK_DIR}/wall/traj.txt" "${pose}${pose}${pose}${pose}")
foreach(frame 0 1 2 3)
  configure_file("${COLOR}" "${WORK_DIR}/wall/results/frame00000${frame}.jpg"
                 COPYONLY)
  configure_file("${DEPTH}" "${WORK_DIR}/wall/results/depth00000${frame}.png"
                 COPYONLY)
endforeach()
file(
  WRITE "${WORK_DIR}/run.json"
  "{\"version\":1,\"operation\":\"codec\",\"source\":{\"kind\":\"dataset\",\"path\":\"wall\",\"preload\":true},\"fusion\":{\"voxel_size_m\":0.02},\"codec\":{\"intra_period\":30,\"encode_every\":1,\"sweep\":false},\"output\":{\"stream\":\"wall.vrst\"}}\n"
)

execute_process(
  COMMAND "${PROGRAM}" --config "${WORK_DIR}/run.json"
  WORKING_DIRECTORY "${WORK_DIR}"
  RESULT_VARIABLE result
  OUTPUT_VARIABLE output
  ERROR_VARIABLE error)
if(NOT result EQUAL 0)
  if(error MATCHES "Vulkan|vkCreate|physical device|VK_ERROR"
     AND "$ENV{VKC_REQUIRE_VULKAN_DEVICE}" STREQUAL "")
    message(STATUS "skipping: no Vulkan device (${error})")
    return()
  endif()
  message(FATAL_ERROR "The run failed: ${result}: ${output}${error}")
endif()
message(STATUS "${output}")
foreach(expected "per set: 3 P-frame sets" "per set: 1 I-frame sets"
                 "\n  codec encode +3 " "\n  write +3 ")
  if(NOT output MATCHES "${expected}")
    message(FATAL_ERROR "No \"${expected}\" in the per-set table: ${output}")
  endif()
endforeach()
# A warm P-frame set is two batches: two submits and no staging. (A wait that
# finds its batch done counts none, so the waits are not pinned.)
string(FIND "${output}" "per set: 3 P-frame sets" p_start)
string(FIND "${output}" "per set: 1 I-frame sets" i_start)
math(EXPR p_length "${i_start} - ${p_start}")
string(SUBSTRING "${output}" ${p_start} ${p_length} p_sets)
foreach(expected "submits +2.000 +2.000 +2.000 +2.000"
                 "staging buffers +0.000 +0.000 +0.000 +0.000")
  if(NOT p_sets MATCHES "${expected}")
    message(FATAL_ERROR "No \"${expected}\" for the P-frame sets: ${p_sets}")
  endif()
endforeach()
if(NOT EXISTS "${WORK_DIR}/wall.vrst")
  message(FATAL_ERROR "No stream: ${output}")
endif()
file(REMOVE_RECURSE "${WORK_DIR}")
