# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

cmake_minimum_required(VERSION 3.23)
project(navkit VERSION 0.1.0 LANGUAGES CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(NAVKIT_BUILD_TESTS "Build NavKit tests" ON)
set(NAVKIT_CONFIG "apps/navkit_sim/variants/ecef_ins_gnss_lc/EcefInsGnssLcGyroAccelBiasDefault.hpp"
    CACHE STRING "Compile-time NavKit config header relative to config/compiletime")

set(NAVKIT_CONFIG_ROOT "${PROJECT_SOURCE_DIR}/config/compiletime")
if(IS_ABSOLUTE "${NAVKIT_CONFIG}")
    message(FATAL_ERROR "NAVKIT_CONFIG must be relative to ${NAVKIT_CONFIG_ROOT}")
endif()

if(NAVKIT_CONFIG MATCHES "\\\\")
    message(FATAL_ERROR "NAVKIT_CONFIG must use forward slashes, for example apps/navkit_sim/variants/ecef_ins_gnss_lc/EcefInsGnssLcGyroAccelBiasDefault.hpp")
endif()

if(NAVKIT_CONFIG MATCHES "(^|/)\\.\\.($|/)")
    message(FATAL_ERROR "NAVKIT_CONFIG must not escape ${NAVKIT_CONFIG_ROOT}")
endif()

set(NAVKIT_CONFIG_PATH "${NAVKIT_CONFIG_ROOT}/${NAVKIT_CONFIG}")
if(NOT EXISTS "${NAVKIT_CONFIG_PATH}")
    message(FATAL_ERROR
        "NAVKIT_CONFIG='${NAVKIT_CONFIG}' does not exist under ${NAVKIT_CONFIG_ROOT}")
endif()

set(NAVKIT_GENERATED_INCLUDE_DIR "${PROJECT_BINARY_DIR}/generated")
configure_file(
    "${PROJECT_SOURCE_DIR}/cmake/NavKitSelectedConfig.hpp.in"
    "${NAVKIT_GENERATED_INCLUDE_DIR}/navkit/SelectedConfig.hpp"
    @ONLY
)

find_package(Eigen3 REQUIRED CONFIG)
find_package(nlohmann_json REQUIRED CONFIG)

include(cmake/NavKitWarnings.cmake)
include(cmake/targets/NavKitCore.cmake)
include(cmake/targets/NavKitIo.cmake)
add_subdirectory(src/sim)
add_subdirectory(src/app_support)

add_subdirectory(apps/navkit_sim)
add_subdirectory(apps/navkit_replay)

if(NAVKIT_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

set_target_properties(navkit_core PROPERTIES EXPORT_NAME core)
set_target_properties(navkit_io PROPERTIES EXPORT_NAME io)
set_target_properties(navkit_sim PROPERTIES EXPORT_NAME sim)
set_target_properties(navkit_app_support PROPERTIES EXPORT_NAME app_support)
set_target_properties(navkit_sim_app PROPERTIES EXPORT_NAME sim_app)
set_target_properties(navkit_replay PROPERTIES EXPORT_NAME replay)

install(
    DIRECTORY "${PROJECT_SOURCE_DIR}/include/navkit"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
)

install(
    TARGETS
        navkit_core
        navkit_io
        navkit_sim
        navkit_app_support
        navkit_sim_app
        navkit_replay
    EXPORT NavKitTargets
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
    LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    INCLUDES DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
)

configure_package_config_file(
    "${PROJECT_SOURCE_DIR}/cmake/NavKitConfig.cmake.in"
    "${PROJECT_BINARY_DIR}/NavKitConfig.cmake"
    INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/NavKit"
)

write_basic_package_version_file(
    "${PROJECT_BINARY_DIR}/NavKitConfigVersion.cmake"
    VERSION "${PROJECT_VERSION}"
    COMPATIBILITY SameMajorVersion
)

install(
    EXPORT NavKitTargets
    FILE NavKitTargets.cmake
    NAMESPACE navkit::
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/NavKit"
)

install(
    FILES
        "${PROJECT_BINARY_DIR}/NavKitConfig.cmake"
        "${PROJECT_BINARY_DIR}/NavKitConfigVersion.cmake"
    DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/NavKit"
)
