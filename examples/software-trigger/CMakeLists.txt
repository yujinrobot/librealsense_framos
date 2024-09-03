# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(RealsenseExamplesSoftwareTrigger)

if(BUILD_GRAPHICAL_EXAMPLES)
    add_executable(rs-software-trigger api_how_to.h rs-software-trigger.cpp ../example.hpp)
    set_property(TARGET rs-software-trigger PROPERTY CXX_STANDARD 11)
    include_directories(rs-software-trigger ../)
    target_link_libraries(rs-software-trigger ${DEPENDENCIES})
    set_target_properties (rs-software-trigger PROPERTIES FOLDER Examples)
    install(TARGETS rs-software-trigger RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
