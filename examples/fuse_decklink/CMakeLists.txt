# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# fuse_decklink: live reconstruction from a DeckLink card's two SDI cameras as
# one stereo camera, its depth from FFS, to a PLY, on fuse_orbbec's run
# (vr_example_live), with the DeckLink camera (vr_example_decklink) and FFS
# depth (vr_example_ffs). Built on every build, so each checks the scenario and,
# lacking sensor's DeckLink driver or FFS, refuses it before anything opens.
add_executable(fuse_decklink main.cpp)

target_link_libraries(fuse_decklink PRIVATE vr_example_live vr_example_decklink
                                            vr_example_ffs)
vr_target_warnings(fuse_decklink)
