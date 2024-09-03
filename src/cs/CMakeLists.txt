# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 FRAMOS GmbH.
target_sources(${LRS_TARGET}
        PRIVATE
        "${CMAKE_CURRENT_LIST_DIR}/cs-device.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/cs-device.h"
        "${CMAKE_CURRENT_LIST_DIR}/cs-event-capture.h"
        "${CMAKE_CURRENT_LIST_DIR}/cs-sensor.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/cs-sensor.h"
        "${CMAKE_CURRENT_LIST_DIR}/cs-sensor-motion.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/cs-factory.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/cs-factory.h"
        "${CMAKE_CURRENT_LIST_DIR}/cs-motion.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/cs-motion.h"
        "${CMAKE_CURRENT_LIST_DIR}/cs-motion-transform.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/cs-motion-transform.h"
        "${CMAKE_CURRENT_LIST_DIR}/cs-timestamp.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/cs-timestamp.h"
        "${CMAKE_CURRENT_LIST_DIR}/cs-options.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/cs-options.h"
        "${CMAKE_CURRENT_LIST_DIR}/cs-private.h"
        "${CMAKE_CURRENT_LIST_DIR}/cs-options-default.h"
        )