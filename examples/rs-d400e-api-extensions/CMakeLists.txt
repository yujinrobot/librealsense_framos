# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(RealsenseExamplesD400eAPIExtensions)

add_executable(rs-d400e-api-extensions rs-d400e-api-extensions.cpp)
set_property(TARGET rs-d400e-api-extensions PROPERTY CXX_STANDARD 11)
target_link_libraries(rs-d400e-api-extensions ${DEPENDENCIES})
include_directories(rs-d400e-api-extensions)
set_target_properties (rs-d400e-api-extensions PROPERTIES FOLDER "Examples")

install(TARGETS rs-d400e-api-extensions RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
