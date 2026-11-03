# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# session_check: a session's frames checked by hand, from a file or a writer
# that dials (session_check.cpp). Built with the tests, so CI compiles it and a
# test runs its command line; the checks are link/session_check.hpp's, which the
# link tests run.
add_executable(recon_session_check session_check.cpp)
set_target_properties(recon_session_check PROPERTIES OUTPUT_NAME session_check)
target_link_libraries(recon_session_check PRIVATE vr_example_link)
vr_target_warnings(recon_session_check)
