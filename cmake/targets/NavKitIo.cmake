# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

add_library(navkit_io INTERFACE)
add_library(navkit::io ALIAS navkit_io)

target_link_libraries(navkit_io
    INTERFACE
        navkit::core
        nlohmann_json::nlohmann_json
)
