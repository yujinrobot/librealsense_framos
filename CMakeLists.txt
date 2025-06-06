#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)
project(librealsense2 LANGUAGES CXX C)

include(CMake/lrs_options.cmake)
include(CMake/connectivity_check.cmake)
#Deprecation message, should be removed in future releases
if(${FORCE_LIBUVC} OR ${FORCE_WINUSB_UVC} OR ${ANDROID_USB_HOST_UVC})
    MESSAGE(DEPRECATION "FORCE_LIBUVC, FORCE_WINUSB_UVC and ANDROID_USB_HOST_UVC are deprecated, use FORCE_RSUSB_BACKEND instead")
    set(FORCE_RSUSB_BACKEND ON)
endif()

set( BUILD_FRAMOS_CODE ON )

# Checking Internet connection, as TM2 needs to download the FW from amazon cloud
if(BUILD_WITH_TM2 AND NOT INTERNET_CONNECTION)
    message(WARNING "No internet connection, disabling BUILD_WITH_TM2")
    set(BUILD_WITH_TM2 OFF)
endif()

# Checking Internet connection, as DEPTH CAM needs to download the FW from amazon cloud
if(IMPORT_DEPTH_CAM_FW AND NOT INTERNET_CONNECTION)
    message(WARNING "No internet connection, disabling IMPORT_DEPTH_CAM_FW")
    set(IMPORT_DEPTH_CAM_FW OFF)
endif()

if (BUILD_PC_STITCHING AND NOT BUILD_GLSL_EXTENSIONS)
    MESSAGE(STATUS "BUILD_PC_STITCHING explicitely depends on BUILD_GLSL_EXTENSIONS, set it ON")
    SET(BUILD_GLSL_EXTENSIONS ON)
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/CMake)

# include librealsense general configuration
include(CMake/global_config.cmake)

if(BUILD_FRAMOS_CODE)
    # include librealsense camerasuite configuration
    include(CMake/camera_suite.cmake)

    # include dynamic calibrator
    include(CMake/dynamic_calibrator.cmake)

    # #include ROS wrapper
    # include(CMake/ros_wrapper.cmake)

    # #include ROS2 wrapper
    # include(CMake/ros2_wrapper.cmake)
endif()

config_cxx_flags()
# include os specific macros
# macro definition located at "CMake/global_config.cmake"
include(CMake/include_os.cmake)

# set os specific configuration flags
# macro definition located at "CMake/<OS>_config.cmake"
os_set_flags()

# set global configuration flags
# macro definition located at "CMake/global_config.cmake"
global_set_flags()

if(BUILD_SHARED_LIBS)
    add_library(${LRS_TARGET} SHARED "")
else()
    add_library(${LRS_TARGET} STATIC "")
endif()

# set library version
set_target_properties(${LRS_TARGET} PROPERTIES VERSION ${REALSENSE_VERSION_STRING} SOVERSION "${REALSENSE_VERSION_MAJOR}.${REALSENSE_VERSION_MINOR}")

include(include/CMakeLists.txt)
include(src/CMakeLists.txt)
include(third-party/CMakeLists.txt)
include(common/utilities/CMakeLists.txt)

# configure the project according to OS specific requirments
# macro definition located at "CMake/<OS>_config.cmake"
os_target_config()

# global project configuration
# macro definition located at "CMake/global_config.cmake"
global_target_config()

# add camera suite configuration flags
# macro definition located at "CMake/camera_suite.cmake"
if(BUILD_FRAMOS_CODE)
    add_camerasuite(${LRS_TARGET})
endif()

include(CMake/install_config.cmake)

if(BUILD_FRAMOS_CODE)
    #add_compile_definitons is not compatible with cmake version: 3.1.0
    #add_compile_definitions(FRAMOS)
    add_definitions(-DFRAMOS)
    #DISABLE_LIBUSB flag added for FRAMOS librealsense as a workaround for a rare seg fault issue
    if(FRAMOS_DISABLE_LIBUSB)
        add_definitions(-DFRAMOS_DISABLE_LIBUSB)
    endif()
endif()

add_subdirectory(wrappers)
if ( (BUILD_FRAMOS_CODE) AND ( BUILD_EXAMPLES OR BUILD_PC_STITCHING ) AND BUILD_GLSL_EXTENSIONS )
    find_package(glfw3 3.3 QUIET)
    if(NOT TARGET glfw)
        message(STATUS "GLFW 3.3 not found; using internal version")
        #this forces installation of glfw3 headers, conflicting with libglfw3-dev package
        #set(GLFW_INSTALL ON CACHE BOOL "" FORCE)
        add_subdirectory(third-party/glfw)
    endif()
    add_subdirectory(src/gl)
endif()

if ( (NOT BUILD_FRAMOS_CODE) AND ( BUILD_EXAMPLES OR BUILD_PC_STITCHING ) AND BUILD_GLSL_EXTENSIONS )
    find_package(glfw3 3.3 QUIET)
    if(NOT TARGET glfw)
        message(STATUS "GLFW 3.3 not found; using internal version")
        set(GLFW_INSTALL ON CACHE BOOL "" FORCE)
        add_subdirectory(third-party/glfw)
    endif()
    add_subdirectory(src/gl)
endif()

if(BUILD_EXAMPLES)
    add_subdirectory(examples)
    add_subdirectory(tools)
elseif(BUILD_TOOLS)
    add_subdirectory(tools)
endif()

if(BUILD_UNIT_TESTS OR BUILD_LEGACY_LIVE_TEST)
    add_subdirectory(unit-tests)
endif()

if(BUILD_NETWORK_DEVICE)
    add_subdirectory(src/ethernet)
    add_subdirectory(src/compression)
endif()

if(BUILD_WITH_TM2)
    add_tm2()
endif()

if(IMPORT_DEPTH_CAM_FW)
    add_subdirectory(common/fw)
endif()

include(CMake/embedd_udev_rules.cmake)
