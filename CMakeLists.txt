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

add_library(navkit_core
    src/core/dummy.cpp
    src/sim/TrajectoryGenerator.cpp
    src/sim/GnssSimulator.cpp
    src/sim/ImuSimulator.cpp
)

add_library(navkit::core ALIAS navkit_core)

target_include_directories(navkit_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(navkit_core
    PUBLIC
        Eigen3::Eigen
        nlohmann_json::nlohmann_json
)

target_compile_options(navkit_core PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>
)

add_subdirectory(apps/navkit_sim)
add_subdirectory(apps/navkit_replay)

if(NAVKIT_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
