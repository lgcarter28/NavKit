# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

add_library(navkit_app_support INTERFACE)
add_library(navkit::app_support ALIAS navkit_app_support)

target_include_directories(navkit_app_support
    INTERFACE
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/config/compiletime
        ${NAVKIT_GENERATED_INCLUDE_DIR}
)

target_link_libraries(navkit_app_support
    INTERFACE
        navkit::core
        navkit::io
        navkit::sim
)
