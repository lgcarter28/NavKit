# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

add_library(navkit_core INTERFACE)
add_library(navkit::core ALIAS navkit_core)

target_include_directories(navkit_core
    INTERFACE
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(navkit_core
    INTERFACE
        Eigen3::Eigen
)

target_compile_features(navkit_core
    INTERFACE
        cxx_std_23
)
