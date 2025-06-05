# Set CMAKE_INSTALL_* if not defined
set(CMAKECONFIG_INSTALL_DIR "${CMAKE_INSTALL_LIBDIR}/cmake/${LRS_TARGET}")

add_custom_target(uninstall "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")

include(CMakePackageConfigHelpers)

write_basic_package_version_file("${CMAKE_CURRENT_BINARY_DIR}/realsense2ConfigVersion.cmake"
    VERSION ${REALSENSE_VERSION_STRING} COMPATIBILITY AnyNewerVersion)

configure_package_config_file(CMake/realsense2Config.cmake.in realsense2Config.cmake
    INSTALL_DESTINATION ${CMAKECONFIG_INSTALL_DIR}
    INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX}/bin
    PATH_VARS CMAKE_INSTALL_INCLUDEDIR
)

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake" "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake" IMMEDIATE @ONLY)
configure_file(config/librealsense.pc.in config/realsense2.pc @ONLY)

install(TARGETS ${LRS_TARGET}
    EXPORT realsense2Targets
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_PREFIX}/include/librealsense2"
)

if(OFF)
    if (UNIX)
        set(SOURCE_DEST src/librealsense2)
    elseif (WIN32)
        set(SOURCE_DEST src)
    else()
        set(SOURCE_DEST .)
    endif()

    install(DIRECTORY ${PROJECT_SOURCE_DIR}/.github DESTINATION ${SOURCE_DEST})
    install(DIRECTORY ${PROJECT_SOURCE_DIR}/CMake DESTINATION ${SOURCE_DEST})
    install(DIRECTORY ${PROJECT_SOURCE_DIR}/common DESTINATION ${SOURCE_DEST})
    install(DIRECTORY ${PROJECT_SOURCE_DIR}/config DESTINATION ${SOURCE_DEST})
    install(DIRECTORY ${PROJECT_SOURCE_DIR}/doc DESTINATION ${SOURCE_DEST})
    install(DIRECTORY ${PROJECT_SOURCE_DIR}/examples DESTINATION ${SOURCE_DEST})
    install(DIRECTORY ${PROJECT_SOURCE_DIR}/include DESTINATION ${SOURCE_DEST})
    install(DIRECTORY ${PROJECT_SOURCE_DIR}/scripts DESTINATION ${SOURCE_DEST})
    install(DIRECTORY ${PROJECT_SOURCE_DIR}/src DESTINATION ${SOURCE_DEST})
    install(DIRECTORY ${PROJECT_SOURCE_DIR}/third-party DESTINATION ${SOURCE_DEST})
    install(DIRECTORY ${PROJECT_SOURCE_DIR}/tools DESTINATION ${SOURCE_DEST})
    install(DIRECTORY ${PROJECT_SOURCE_DIR}/unit-tests DESTINATION ${SOURCE_DEST})
    install(DIRECTORY ${PROJECT_SOURCE_DIR}/wrappers DESTINATION ${SOURCE_DEST})
    install(FILES 
        ${PROJECT_SOURCE_DIR}/.gitignore 
        ${PROJECT_SOURCE_DIR}/.travis.yml 
        ${PROJECT_SOURCE_DIR}/appveyor.yml
        ${PROJECT_SOURCE_DIR}/cmake_uninstall.cmake
        ${PROJECT_SOURCE_DIR}/CMakeLists.txt
        ${PROJECT_SOURCE_DIR}/code-of-conduct.md
        ${PROJECT_SOURCE_DIR}/CONTRIBUTING.md
        ${PROJECT_SOURCE_DIR}/LICENSE
        ${PROJECT_SOURCE_DIR}/NOTICE
        ${PROJECT_SOURCE_DIR}/package.xml
        ${PROJECT_SOURCE_DIR}/readme.md
        ${PROJECT_SOURCE_DIR}/d400e_api_extensions.md
        DESTINATION ${SOURCE_DEST}
    )

    # if (UNIX)
    #     install(FILES
    #         ${PROJECT_SOURCE_DIR}/config/99-realsense-libusb.rules 
    #         DESTINATION /etc/udev/rules.d/
    #     )
    # endif()
endif()

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/librealsense2
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT realsense2Targets
        FILE realsense2Targets.cmake
        NAMESPACE ${LRS_TARGET}::
        DESTINATION ${CMAKECONFIG_INSTALL_DIR}
)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/realsense2Config.cmake"
        DESTINATION ${CMAKECONFIG_INSTALL_DIR}
)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/realsense2ConfigVersion.cmake"
        DESTINATION ${CMAKECONFIG_INSTALL_DIR}
)

# Set library pkgconfig file for facilitating 3rd party integration
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/config/realsense2.pc"
        DESTINATION "${CMAKE_INSTALL_LIBDIR}/pkgconfig"
)

install(CODE "execute_process(COMMAND ldconfig)")

    if (BUILD_FRAMOS_CODE)
    set(CPACK_RESOURCE_FILE_LICENSE "${PROJECT_SOURCE_DIR}/LICENSE")
    set(CPACK_NSIS_MUI_ICON "${PROJECT_SOURCE_DIR}/tools/realsense-viewer/res/icon.ico")
    set(CPACK_NSIS_MUI_UNIICON "${PROJECT_SOURCE_DIR}/tools/realsense-viewer/res/icon.ico")
    set(CPACK_NSIS_INSTALLED_ICON_NAME "bin\\\\realsense-viewer.exe")
    set(CPACK_NSIS_DISPLAY_NAME "FRAMOS librealsense2")

    set(CPACK_DEBIAN_PACKAGE_RECOMMENDS "freeglut3")
    set(CPACK_DEBIAN_PACKAGE_MAINTAINER "FRAMOS GmbH")

    set(CPACK_PACKAGE_EXECUTABLES "realsense-viewer;FRAMOS Realsense Viewer")
    set(CPACK_PACKAGE_NAME "FRAMOS-librealsense2")
    set(CPACK_PACKAGE_VENDOR "FRAMOS GmbH")
    set(CPACK_PACKAGE_ICON "${PROJECT_SOURCE_DIR}\\\\installer.bmp")
    set(CPACK_PACKAGE_VERSION_MAJOR 2)
    set(CPACK_PACKAGE_VERSION_MINOR 50)
    set(CPACK_PACKAGE_VERSION_PATCH 13)

    # Workaround for path length limitation of NSIS (260 characters)
    if (WIN32)
        set(CPACK_PACKAGE_DIRECTORY C:/temp)
        set(CPACK_OUTPUT_FILE_PREFIX ${PROJECT_BINARY_DIR})
    endif()

    set(CPACK_DEBIAN_PACKAGE_CONTROL_EXTRA ${PROJECT_SOURCE_DIR}/postinst)

    set(CPACK_PROJECT_CONFIG_FILE "${PROJECT_SOURCE_DIR}/CPackOptions.cmake")

    include(CPack)
endif()
