# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

add_executable(navkit_swil main.cpp)
target_include_directories(navkit_swil
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
)
target_link_libraries(navkit_swil PRIVATE navkit::swil_support)
navkit_configure_target(navkit_swil)
