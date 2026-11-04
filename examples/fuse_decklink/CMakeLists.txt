# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# fuse_decklink: live reconstruction from a DeckLink card's two SDI cameras as
# one stereo camera, its depth from FFS, to a PLY, on fuse_orbbec's run
# (examples/common/live_fusion.hpp). It links what that run links (fuse_orbbec's
# CMakeLists.txt says why each), sensor's stereo tier (the calibration file's
# stereo camera) and, where sensor builds them, its DeckLink driver
# (VS_WITH_DECKLINK) and FFS depth (VS_WITH_FFS), which VR_EXAMPLE_DECKLINK and
# VR_EXAMPLE_FFS turn on in decklink_camera.hpp. Built on every build, so each
# checks the scenario and, lacking either, refuses it before anything opens.
add_executable(fuse_decklink main.cpp)

target_link_libraries(
  fuse_decklink
  PRIVATE vr_example_live_audio
          vr_example_mobile
          volumetric_kit::grid_eval
          volumetric_kit::grid_mesh_io
          volumetric_kit::recon_sender
          volumetric_kit::grid_voxelize
          volumetric_kit::sensor_stereo
          volumetric_kit::sensor_array
          Vulkan::Vulkan)
if(TARGET volumetric_kit::sensor_decklink)
  target_link_libraries(fuse_decklink PRIVATE volumetric_kit::sensor_decklink)
  target_compile_definitions(fuse_decklink PRIVATE VR_EXAMPLE_DECKLINK=1)
endif()
if(TARGET volumetric_kit::sensor_ffs)
  target_link_libraries(fuse_decklink PRIVATE volumetric_kit::sensor_ffs)
  target_compile_definitions(fuse_decklink PRIVATE VR_EXAMPLE_FFS=1)
endif()
vr_target_warnings(fuse_decklink)
