# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

add_executable(navkit_replay main.cpp)
target_link_libraries(navkit_replay PRIVATE navkit::core)
navkit_configure_target(navkit_replay)
