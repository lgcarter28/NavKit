# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

add_executable(navkit_sim main.cpp)
target_link_libraries(navkit_sim PRIVATE navkit::core)
