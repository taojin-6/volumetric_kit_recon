# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# fuse_decklink against its build, with no Vulkan driver to find and no camera
# opened. Without sensor's DeckLink driver (DECKLINK=OFF) or its FFS depth
# (FFS=OFF), a valid scenario is refused as a scenario is (exit 2), naming each
# missing part and how to build it, before any GPU work. With both, it passes
# that check and fails only at the GPU, which it reaches first.

file(REMOVE_RECURSE "${WORK_DIR}")
file(MAKE_DIRECTORY "${WORK_DIR}")
file(
  WRITE "${WORK_DIR}/decklink.json"
  "{\"version\":1,\"operation\":\"fuse\",\"source\":{\"kind\":\"live\",\"driver\":\"decklink\","
  "\"cameras\":[{\"id\":\"bm-1\",\"input\":\"SDI 2\"},{\"id\":\"bm-2\",\"input\":\"SDI 1\"}],"
  "\"pairs\":[{\"left\":\"bm-1\",\"right\":\"bm-2\"}],\"calibration\":\"rig.json\","
  "\"ffs_model\":\"ffs.onnx\",\"ffs_plan_cache\":\"plans\"}}\n")

execute_process(
  COMMAND
    "${CMAKE_COMMAND}" -E env VK_DRIVER_FILES=/nonexistent
    VK_ICD_FILENAMES=/nonexistent "${PROGRAM}" --config
    "${WORK_DIR}/decklink.json"
  RESULT_VARIABLE result
  OUTPUT_VARIABLE output
  ERROR_VARIABLE error)
if(DECKLINK AND FFS)
  if(NOT result EQUAL 1
     OR error MATCHES "DeckLink stereo camera needs"
     OR NOT error MATCHES "fuse_decklink failed"
     OR output MATCHES "opening the cameras")
    message(FATAL_ERROR "fuse_decklink did not reach the GPU before its "
                        "camera: ${result}: ${output}${error}")
  endif()
else()
  set(refused TRUE)
  if(NOT result EQUAL 2
     OR NOT error MATCHES "DeckLink stereo camera needs"
     OR error MATCHES "vkCreate|Vulkan|physical device"
     OR output MATCHES "opening the cameras")
    set(refused FALSE)
  endif()
  # Each missing part named, and only those.
  foreach(part DECKLINK FFS)
    if(${part})
      if(error MATCHES "VS_WITH_${part}=ON")
        set(refused FALSE)
      endif()
    elseif(NOT error MATCHES "VS_WITH_${part}=ON")
      set(refused FALSE)
    endif()
  endforeach()
  if(NOT refused)
    message(
      FATAL_ERROR
        "fuse_decklink was not refused before the GPU in a "
        "build without the DeckLink driver or FFS: "
        "${result}: ${output}${error}")
  endif()
endif()
file(REMOVE_RECURSE "${WORK_DIR}")
