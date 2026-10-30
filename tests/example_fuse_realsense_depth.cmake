# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# fuse_realsense's depth source against its build, with no Vulkan driver to find
# and no camera opened. FFS=OFF, a build without sensor_ffs: an FFS scenario is
# refused as a scenario is (exit 2), saying how to build FFS, before any GPU
# work. FFS=ON: it passes that check and fails only at the GPU, which it reaches
# first. Either way the camera's own depth passes it too.

file(REMOVE_RECURSE "${WORK_DIR}")
file(MAKE_DIRECTORY "${WORK_DIR}")
set(live "{\"version\":1,\"operation\":\"fuse\",\"source\":{\"kind\":\"live\"")
file(
  WRITE "${WORK_DIR}/ffs.json"
  "${live},\"depth\":\"ffs\",\"ffs_model\":\"ffs.onnx\",\"ffs_plan_cache\":\"plans\"}}\n"
)
file(WRITE "${WORK_DIR}/camera.json" "${live}}}\n")

foreach(depth ffs camera)
  execute_process(
    COMMAND
      "${CMAKE_COMMAND}" -E env VK_DRIVER_FILES=/nonexistent
      VK_ICD_FILENAMES=/nonexistent "${PROGRAM}" --config
      "${WORK_DIR}/${depth}.json"
    RESULT_VARIABLE result
    OUTPUT_VARIABLE output
    ERROR_VARIABLE error)
  if(depth STREQUAL "ffs" AND NOT FFS)
    if(NOT result EQUAL 2
       OR NOT error MATCHES "source.depth ffs needs .*VS_WITH_FFS=ON"
       OR error MATCHES "vkCreate|Vulkan|physical device"
       OR output MATCHES "opening the cameras")
      message(FATAL_ERROR "FFS depth was not refused before the GPU in a build "
                          "without it: ${result}: ${output}${error}")
    endif()
  elseif(
    NOT result EQUAL 1
    OR error MATCHES "VS_WITH_FFS"
    OR NOT error MATCHES "fuse_realsense failed"
    OR output MATCHES "opening the cameras")
    message(
      FATAL_ERROR "${depth} depth did not reach the GPU before its camera: "
                  "${result}: ${output}${error}")
  endif()
endforeach()
file(REMOVE_RECURSE "${WORK_DIR}")
