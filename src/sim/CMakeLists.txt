# Copyright (c) 2026 William Gordon Carter.
# All Rights Reserved.

add_library(navkit_sim
    trajectory/TruthTrajectory.cpp
    trajectory/TrajectoryProfiles.cpp
    trajectory/GeneratedTrajectorySource.cpp
    guidance/GuidanceCommandFilter.cpp
    guidance/GuidanceAlgorithms.cpp
    guidance/GuidanceBlocks.cpp
    guidance/GuidanceStateMachine.cpp
    autopilot/FirstOrderAutopilotModel.cpp
    trajectory/FirstOrderVehicleResponseModel.cpp
    trajectory/TabulatedTrajectorySource.cpp
    trajectory/StationaryTrajectorySource.cpp
    sensors/BaroSimulator.cpp
    sensors/GnssSimulator.cpp
    sensors/ImuSimulator.cpp
    math/FirstOrderResponse.cpp
)

add_library(navkit::sim ALIAS navkit_sim)

target_link_libraries(navkit_sim
    PUBLIC
        navkit::core
)

navkit_configure_target(navkit_sim)
