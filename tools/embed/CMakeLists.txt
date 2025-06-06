# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2021 Intel Corporation. All Rights Reserved.

cmake_minimum_required(VERSION 3.1.0)

project(RealsenseToolsEmbed)

# Save the command line compile commands in the build output
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

add_executable(rs-embed 
   rs-embed.cpp
   ${LZ4_DIR}/lz4.h
   ${LZ4_DIR}/lz4.c
)

set_property(TARGET rs-embed PROPERTY CXX_STANDARD 11)

include_directories(rs-embed ${LZ4_DIR} ../../third-party ../../third-party/tclap/include)
set_target_properties (rs-embed PROPERTIES FOLDER Tools)

if (BUILD_FRAMOS_CODE)
    install(
        TARGETS
        rs-embed
        RUNTIME DESTINATION
        ${CMAKE_INSTALL_INCLUDEDIR}/bin
    )
else()
    install(
        TARGETS
        rs-embed
        RUNTIME DESTINATION
        ${CMAKE_INSTALL_PREFIX}/bin
    )
endif()
