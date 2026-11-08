# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# session_check's checks (session_check.hpp), shared by the tool and the link
# tests (tests/): the frame contract and each frame's pattern index, read with
# sensor_prep's PatternReader, on the examples' link (vr_example_link, whose
# vr_example_common brings sensor_prep), and core_net, whose parse_host_port
# reads the address a check listens on or dials. Defined after tests/, whose
# link tests name it; the name resolves when the build is generated.
add_library(vr_session_check INTERFACE)
target_include_directories(vr_session_check
                           INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(vr_session_check INTERFACE vr_example_link
                                                 volumetric_kit::core_net)

# session_check: a session's frames checked by hand, from a file or a writer
# that dials (session_check.cpp). Built with the tests, so CI compiles it and a
# test runs its command line.
add_executable(recon_session_check session_check.cpp)
set_target_properties(recon_session_check PROPERTIES OUTPUT_NAME session_check)
target_link_libraries(recon_session_check PRIVATE vr_session_check)
vr_target_warnings(recon_session_check)
