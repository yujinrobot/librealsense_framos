if (UNIX)
    set(ROS_WRAPPER_PATH "OFF" CACHE FILEPATH "Path to ROS wrapper to include in the installer")
    if (NOT ${ROS_WRAPPER_PATH} STREQUAL "OFF")
        if (IS_DIRECTORY ${ROS_WRAPPER_PATH})
            install(DIRECTORY
                ${ROS_WRAPPER_PATH}/
                DESTINATION src/librealsense2/wrappers/ros/
                PATTERN ".git" EXCLUDE
            )
        else()
            message(FATAL_ERROR "ROS_WRAPPER_PATH does not point to a valid directory")
        endif()
    endif()
endif()
