# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# The Replica sequence is a sensor (sensor::IRgbdSensor) and loads its assets
# through the production IO tier. Dataset layout stays example-local. The fusing
# examples' helpers are here; the scaffolding and the codec helpers they reach
# too are grid's (grid_example_common).
add_library(vr_example_common STATIC replica_sensor.cpp)
target_include_directories(vr_example_common
                           PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(
  vr_example_common
  PUBLIC grid_example_common volumetric_kit::core_config
         volumetric_kit::sensor_interface volumetric_kit::sensor_prep
         volumetric_kit::sensor_image_io)
target_compile_features(vr_example_common PUBLIC cxx_std_17)
vr_target_warnings(vr_example_common)

# The live examples' mic (live_audio.hpp): sensor's microphone driver where
# sensor builds it (VS_WITH_PORTAUDIO), which VR_EXAMPLE_MICROPHONE turns on;
# elsewhere source.audio is refused as Unsupported.
add_library(vr_example_live_audio INTERFACE)
target_link_libraries(vr_example_live_audio INTERFACE vr_example_common)
if(TARGET volumetric_kit::sensor_microphone)
  target_link_libraries(vr_example_live_audio
                        INTERFACE volumetric_kit::sensor_microphone)
  target_compile_definitions(vr_example_live_audio
                             INTERFACE VR_EXAMPLE_MICROPHONE=1)
endif()

# FFS stereo depth (ffs_depth.hpp) for the cameras that offer it: sensor_ffs
# where sensor builds it (VS_WITH_FFS), which VR_EXAMPLE_FFS turns on; elsewhere
# a scenario that needs it is refused before anything opens.
add_library(vr_example_ffs INTERFACE)
if(TARGET volumetric_kit::sensor_ffs)
  target_link_libraries(vr_example_ffs INTERFACE volumetric_kit::sensor_ffs)
  target_compile_definitions(vr_example_ffs INTERFACE VR_EXAMPLE_FFS=1)
endif()
# The DeckLink camera (decklink_camera.hpp): sensor's stereo tier, for the
# calibration file's stereo camera, and its DeckLink driver where sensor builds
# it (VS_WITH_DECKLINK), which VR_EXAMPLE_DECKLINK turns on. The stereo tier
# rides here, as the driver and sensor_ffs link it (vr_example_live's rule,
# below).
add_library(vr_example_decklink INTERFACE)
target_link_libraries(vr_example_decklink
                      INTERFACE volumetric_kit::sensor_stereo)
if(TARGET volumetric_kit::sensor_decklink)
  target_link_libraries(vr_example_decklink
                        INTERFACE volumetric_kit::sensor_decklink)
  target_compile_definitions(vr_example_decklink
                             INTERFACE VR_EXAMPLE_DECKLINK=1)
endif()

# One sensor of a .vrst session, from a file or a live link its writer dials,
# read back as a sensor::IRgbdSensor (link/rgbd_stream_sensor.hpp): its depth
# track's records, in sensor's wire form (sensor_wire), through grid's
# StreamReader or, live, a core::net::Listener's connections read by grid's
# StreamClient, and its pictures on sensor_video where sensor builds it. The key
# gate is sensor_video_syntax's, which sensor_wire brings. grid_networking is
# PUBLIC for the ViewerNote its header names.
add_library(vr_example_link STATIC link/rgbd_stream_sensor.cpp)
target_link_libraries(
  vr_example_link
  PUBLIC vr_example_common volumetric_kit::sensor_wire
         volumetric_kit::grid_stream volumetric_kit::grid_networking
  PRIVATE volumetric_kit::core_net)
if(TARGET volumetric_kit::sensor_video)
  target_link_libraries(vr_example_link PRIVATE volumetric_kit::sensor_video)
  target_compile_definitions(vr_example_link PRIVATE VR_EXAMPLE_LINK_VIDEO=1)
endif()
vr_target_warnings(vr_example_link)

# The live runs' phone lane (live_mobile.hpp) and the tracking log it writes
# (track_log.hpp): the phone's session read back (vr_example_link), prepared
# (sensor_prep), tracked (recon_track) and handed to the sender as fill
# (recon_sender).
add_library(vr_example_mobile STATIC live_mobile.cpp track_log.cpp)
target_link_libraries(
  vr_example_mobile PUBLIC vr_example_link volumetric_kit::recon_track
                           volumetric_kit::recon_sender)
# The origin tag's watch (live_tag.hpp): on calib's detector and solve and
# sensor's picture readback with VR_WITH_CALIB, a refusal without. OpenCV's core
# for the one call that keeps the detector on the watch's thread.
if(VR_WITH_CALIB)
  target_sources(vr_example_mobile PRIVATE live_tag.cpp)
  target_link_libraries(
    vr_example_mobile PRIVATE volumetric_kit::calib_detect
                              volumetric_kit::calib_solve opencv_core)
else()
  target_sources(vr_example_mobile PRIVATE live_tag_off.cpp)
endif()
vr_target_warnings(vr_example_mobile)

# The live runs' loop (live_fusion.hpp), which fuse_orbbec, fuse_realsense and
# fuse_decklink each link with their camera's driver. vr_example_live_audio
# brings vr_example_common, with the sensor package's GPU pass (sensor_prep),
# and any microphone driver source.audio records with; vr_example_mobile the
# phone lane source.mobile runs (its tracker and log) and recon_sender, the set
# step, with its codec stream, the fuser and the codec; eval is the codec
# sweep's, mesh_io the PLY's, voxelize the mesh settings codec_config.hpp maps,
# and sensor_array the cameras' array. A program names these bundles and its
# driver alone: a library they bring, named beside them, goes on the link line
# ahead of what links it and repeats there, which Apple's linker warns about.
add_library(vr_example_live INTERFACE)
target_link_libraries(
  vr_example_live
  INTERFACE vr_example_live_audio
            vr_example_mobile
            volumetric_kit::grid_eval
            volumetric_kit::grid_mesh_io
            volumetric_kit::grid_voxelize
            volumetric_kit::sensor_array
            Vulkan::Vulkan)
