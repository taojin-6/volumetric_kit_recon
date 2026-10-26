# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# --- recon_sender: a sender's set step and its codec stream ------------------
# What a sender does with a set once its prep is recorded: fuse it, free stale
# blocks on schedule and code its frame in two batches (SetSender), the frame
# written to a .vrst file, served to live viewers and decoded back as a viewer
# would (CodecStream), and the fuser's inputs for prepared frames (fuse_inputs).
# The tier after networking: it links the GPU frame prep's types
# (recon_sensor_utils), the fuser (recon_tsdf), the codec, the container and the
# server. The frame's halves around a set's fusion are internal
# (stream_coder.hpp), reached by SetSender alone.
include(GenerateExportHeader)

set(_vr_sender_generated_inc "${CMAKE_CURRENT_BINARY_DIR}/generated/include")

add_library(recon_sender)
add_library(volumetric_kit::recon_sender ALIAS recon_sender)

target_sources(recon_sender PRIVATE codec_stream.cpp fuse_inputs.cpp
                                    set_sender.cpp)

generate_export_header(
  recon_sender
  BASE_NAME
  VR_SENDER
  EXPORT_MACRO_NAME
  VR_SENDER_API
  EXPORT_FILE_NAME
  "${_vr_sender_generated_inc}/volumetric_kit/recon/sender/export.hpp")

set_target_properties(
  recon_sender
  PROPERTIES CXX_VISIBILITY_PRESET hidden
             VISIBILITY_INLINES_HIDDEN ON
             POSITION_INDEPENDENT_CODE ON)

get_target_property(_vr_sender_type recon_sender TYPE)
if(_vr_sender_type STREQUAL "STATIC_LIBRARY")
  target_compile_definitions(recon_sender PUBLIC VR_SENDER_STATIC_DEFINE)
endif()

target_compile_features(recon_sender PUBLIC cxx_std_17)

# PUBLIC: the headers take the prep's DeviceFrame, the fuser and its report, the
# encoder's settings, the session and the server's options.
target_link_libraries(
  recon_sender PUBLIC recon_sensor_utils recon_tsdf recon_codec recon_stream
                      recon_networking)

target_include_directories(
  recon_sender PUBLIC "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
                      "$<BUILD_INTERFACE:${_vr_sender_generated_inc}>")

vr_target_warnings(recon_sender)
