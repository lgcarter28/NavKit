# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

cmake_minimum_required(VERSION 3.23)
project(navkit VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(NAVKIT_BUILD_TESTS "Build NavKit tests" ON)
set(NAVKIT_CONFIG "navkit_sim/StationaryGnss.hpp"
    CACHE STRING "Compile-time NavKit config header relative to config/compiletime")

set(NAVKIT_CONFIG_ROOT "${PROJECT_SOURCE_DIR}/config/compiletime")
if(IS_ABSOLUTE "${NAVKIT_CONFIG}")
    message(FATAL_ERROR "NAVKIT_CONFIG must be relative to ${NAVKIT_CONFIG_ROOT}")
endif()

if(NAVKIT_CONFIG MATCHES "\\\\")
    message(FATAL_ERROR "NAVKIT_CONFIG must use forward slashes, for example navkit_sim/StationaryGnss.hpp")
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

add_subdirectory(apps/navkit_sim)
add_subdirectory(apps/navkit_replay)

if(NAVKIT_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
