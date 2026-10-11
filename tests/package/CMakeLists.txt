# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Tao Jin

# A stand-in for a project that finds an installed recon, configured by the
# recon_package_* tests with the core and recon packages they name. Linking the
# umbrella target makes generation check that every imported target in its link
# interface exists.
cmake_minimum_required(VERSION 3.21)
project(vr_package_consumer LANGUAGES CXX)

find_package(volumetric_kit_recon REQUIRED CONFIG)
add_executable(vr_package_consumer consumer.cpp)
target_link_libraries(vr_package_consumer PRIVATE volumetric_kit::recon)
