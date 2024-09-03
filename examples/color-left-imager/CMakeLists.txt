# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(RealsenseExamplesColorLeftImager)

if(BUILD_GRAPHICAL_EXAMPLES)
    add_executable(rs-color-left-imager rs-color-left-imager.cpp ../example.hpp)
    set_property(TARGET rs-color-left-imager PROPERTY CXX_STANDARD 11)
    include_directories(../)
    target_link_libraries(rs-color-left-imager ${DEPENDENCIES})
    set_target_properties (rs-color-left-imager PROPERTIES FOLDER Examples)
    install(TARGETS rs-color-left-imager RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
