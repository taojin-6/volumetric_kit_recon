# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# Invalid CLI values must fail before opening a capture or creating a device.
# The output collision check also protects an already existing artifact.
function(expect_invalid expected)
  execute_process(
    COMMAND "${PROGRAM}" missing-scene ${ARGN}
    RESULT_VARIABLE result
    OUTPUT_VARIABLE output
    ERROR_VARIABLE error)
  if(NOT result EQUAL 2 OR NOT error MATCHES "${expected}")
    message(
      FATAL_ERROR
        "CLI ${ARGN}: expected parse failure '${expected}', got ${result}: ${output}${error}"
    )
  endif()
endfunction()

expect_invalid("invalid or missing value" --voxel nan)
expect_invalid("invalid or missing value" --noise-floor inf)
expect_invalid("invalid or missing value" --max-nodes 999999999999999999)
expect_invalid("invalid or missing value" --max-splits 5garbage)
expect_invalid("invalid or missing value" --timings-csv --preload)
expect_invalid("invalid level count" --levels 5)
expect_invalid("invalid level count" --refine-every 0)
expect_invalid("invalid level count" --pixel-stride 0)
expect_invalid("invalid resolution" --trunc 0)
expect_invalid("invalid resolution" --depth-jump 0)
expect_invalid("invalid or missing value" --depth-jump nan)
expect_invalid("invalid resolution" --min-depth 2 --max-depth 1)
expect_invalid("multiple of 8" --max-nodes 9)
expect_invalid("multiple of 8" --max-nodes 8192 --buckets 2048)
file(WRITE "${SCRATCH}/hierarchical-room-output-test.ply" "preserved\n")
expect_invalid(
  "must differ" --out "${SCRATCH}/hierarchical-room-output-test.ply"
  --timings-csv "${SCRATCH}/./hierarchical-room-output-test.ply")
file(READ "${SCRATCH}/hierarchical-room-output-test.ply" preserved)
if(NOT preserved STREQUAL "preserved\n")
  message(FATAL_ERROR "CLI modified colliding output before rejecting it")
endif()
file(REMOVE "${SCRATCH}/hierarchical-room-output-test.ply")
