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
