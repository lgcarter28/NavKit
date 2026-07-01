# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

cmake_minimum_required(VERSION 3.23)
project(navkit VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(NAVKIT_BUILD_TESTS "Build NavKit tests" ON)

find_package(Eigen3 REQUIRED CONFIG)
find_package(nlohmann_json REQUIRED CONFIG)

include(cmake/targets/NavKitCore.cmake)
include(cmake/targets/NavKitIo.cmake)
add_subdirectory(src/sim)

add_subdirectory(apps/navkit_sim)
add_subdirectory(apps/navkit_replay)

if(NAVKIT_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
