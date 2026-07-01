# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

add_executable(navkit_sim_app main.cpp)
set_target_properties(navkit_sim_app PROPERTIES OUTPUT_NAME navkit_sim)
target_include_directories(navkit_sim_app
    PRIVATE
        ${PROJECT_SOURCE_DIR}/config/compiletime
        ${NAVKIT_GENERATED_INCLUDE_DIR}
)
target_link_libraries(navkit_sim_app PRIVATE navkit::core navkit::sim navkit::io)
