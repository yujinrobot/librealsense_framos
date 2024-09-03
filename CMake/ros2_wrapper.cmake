if (UNIX)
    set(ROS2_WRAPPER_PATH "OFF" CACHE FILEPATH "Path to ROS2 wrapper to include in the installer")
    if (NOT ${ROS2_WRAPPER_PATH} STREQUAL "OFF")
        if (IS_DIRECTORY ${ROS2_WRAPPER_PATH})
            install(DIRECTORY
                ${ROS2_WRAPPER_PATH}/
                DESTINATION src/librealsense2/wrappers/ros2/
                PATTERN ".git" EXCLUDE
            )
        else()
            message(FATAL_ERROR "ROS2_WRAPPER_PATH does not point to a valid directory")
        endif()
    endif()
endif()
