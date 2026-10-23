# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# fuse_replica on a device: two frames of a wall 1 m away, the JPEG fixture's
# patches over a constant depth, meshed after each. The periodic remesh is the
# device extract, with nothing read back in the timed loop, and the final mesh
# is extracted to the PLY. Skips with no Vulkan device unless
# VKC_REQUIRE_VULKAN_DEVICE is set.
#
# DEPTH is 256x144 16-bit samples of 1000, 1 m at a scale of 1000:
# ~~~
# ffmpeg -f lavfi -i "color=c=black:s=256x144:d=1,format=gray16be,geq=lum=1000" \
#   -frames:v 1 -pix_fmt gray16be depth_1m_256x144.png
# ~~~

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
  "{\"version\":1,\"operation\":\"fuse\",\"source\":{\"kind\":\"dataset\",\"path\":\"wall\"},\"fusion\":{\"voxel_size_m\":0.02,\"mesh_every\":1},\"output\":{\"path\":\"wall.ply\"}}\n"
)

execute_process(
  COMMAND "${PROGRAM}" --config "${WORK_DIR}/run.json"
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
if(NOT output MATCHES "remesh +2 extracts, [^\n]*\\(extract_device\\)")
  message(FATAL_ERROR "The periodic remesh read the mesh back: ${output}")
endif()
string(CONCAT final "done: fused 2 frames [^\n]*, final mesh [1-9][0-9]* "
              "vertices / [1-9][0-9]* triangles")
if(NOT output MATCHES "${final}" OR NOT EXISTS "${WORK_DIR}/wall.ply")
  message(FATAL_ERROR "No final mesh: ${output}")
endif()
file(REMOVE_RECURSE "${WORK_DIR}")
