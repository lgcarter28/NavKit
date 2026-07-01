# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

add_library(navkit_sim
    BaroSimulator.cpp
    TrajectoryGenerator.cpp
    GnssSimulator.cpp
    ImuSimulator.cpp
)

add_library(navkit::sim ALIAS navkit_sim)

target_link_libraries(navkit_sim
    PUBLIC
        navkit::core
)

target_compile_options(navkit_sim PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>
)
