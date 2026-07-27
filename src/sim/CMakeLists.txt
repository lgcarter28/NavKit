# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

add_library(navkit_sim
    TruthTrajectory.cpp
    TabulatedTrajectorySource.cpp
    BaroSimulator.cpp
    StationaryTrajectorySource.cpp
    GnssSimulator.cpp
    ImuSimulator.cpp
)

add_library(navkit::sim ALIAS navkit_sim)

target_link_libraries(navkit_sim
    PUBLIC
        navkit::core
)

navkit_configure_target(navkit_sim)
