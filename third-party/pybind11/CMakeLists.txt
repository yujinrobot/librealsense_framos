#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(pybind11-download NONE)

include(ExternalProject)
ExternalProject_Add(
        pybind11
        PREFIX .
        GIT_REPOSITORY  "https://github.com/pybind/pybind11.git"
        GIT_TAG         v2.12.0
        GIT_CONFIG      advice.detachedHead=false  # otherwise we'll get "You are in 'detached HEAD' state..."
        SOURCE_DIR 		"${CMAKE_BINARY_DIR}/third-party/pybind11"
        # Override default steps with no action, we just want the clone step.
        CONFIGURE_COMMAND ""
        BUILD_COMMAND ""
        INSTALL_COMMAND ""
)


