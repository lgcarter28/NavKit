# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

add_library(navkit_app_support_common INTERFACE)
add_library(navkit::app_support_common ALIAS navkit_app_support_common)

target_include_directories(navkit_app_support_common
    INTERFACE
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/config/compiletime>
        $<BUILD_INTERFACE:${NAVKIT_GENERATED_INCLUDE_DIR}>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(navkit_app_support_common
    INTERFACE
        navkit::core
        navkit::io
)

get_target_property(navkit_app_support_common_dependencies
    navkit_app_support_common
    INTERFACE_LINK_LIBRARIES
)
if("navkit::sim" IN_LIST navkit_app_support_common_dependencies)
    message(FATAL_ERROR "navkit_app_support_common must not depend on navkit::sim")
endif()

add_library(navkit_swil_support INTERFACE)
add_library(navkit::swil_support ALIAS navkit_swil_support)

target_link_libraries(navkit_swil_support
    INTERFACE
        navkit::app_support_common
        navkit::sim
)
