set(DYNAMIC_CALIBRATOR_PATH "OFF" CACHE FILEPATH "Path to DynamicCalibrator build to include in the installer")
if (NOT ${DYNAMIC_CALIBRATOR_PATH} STREQUAL "OFF")
    if (IS_DIRECTORY ${DYNAMIC_CALIBRATOR_PATH})
        if (UNIX)
            install(PROGRAMS
                ${DYNAMIC_CALIBRATOR_PATH}/build/DynamicCalibrator
                ${DYNAMIC_CALIBRATOR_PATH}/build/CalibrationTables
                DESTINATION ${CMAKE_INSTALL_BINDIR}
            )
            install(FILES
                ${DYNAMIC_CALIBRATOR_PATH}/lib/libDSDynamicCalibrationAPI.so
                DESTINATION ${CMAKE_INSTALL_LIBDIR}
            )
        elseif (WIN32)
            string (REPLACE "\\" "/" DYNAMIC_CALIBRATOR_DIR "${DYNAMIC_CALIBRATOR_PATH}")
            install(FILES 
                ${DYNAMIC_CALIBRATOR_DIR}/build/Release/DynamicCalibrator.exe
                ${DYNAMIC_CALIBRATOR_DIR}/build/Release/CalibrationTables.exe
                ${DYNAMIC_CALIBRATOR_DIR}/lib/DSDynamicCalibrationAPI.dll
                ${DYNAMIC_CALIBRATOR_DIR}/3rdparty/glut/lib/freeglut.dll
                DESTINATION ${CMAKE_INSTALL_BINDIR}
            )
        endif()
    else()
        message(FATAL_ERROR "DYNAMIC_CALIBRATOR_PATH does not point to a valid directory")
    endif()
endif()