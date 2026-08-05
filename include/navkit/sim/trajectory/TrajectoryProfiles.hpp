// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/math/Types.hpp"
#include "navkit/core/time/RationalRate.hpp"
#include "navkit/sim/autopilot/FirstOrderAutopilotModel.hpp"
#include "navkit/sim/guidance/GuidanceStateMachine.hpp"
#include "navkit/sim/trajectory/FirstOrderVehicleResponseModel.hpp"
#include "navkit/sim/trajectory/GeneratedTrajectorySource.hpp"
#include "navkit/sim/trajectory/TrajectoryIntegration.hpp"
#include "navkit/sim/trajectory/TruthTrajectory.hpp"

#include <Eigen/Geometry>
#include <memory>
#include <numbers>
#include <vector>

namespace navkit::sim
{

/** Common ECEF initial condition and native cadence for generated truth profiles. */
struct TrajectoryProfileConfig
{
    core::Time_t duration_s{60.0};
    core::RationalRate rate{.samples = 1U, .s = 1U};
    core::RationalRate guidance_rate{.samples = 1U, .s = 1U};
    core::RationalRate autopilot_rate{.samples = 1U, .s = 1U};
    core::Timestamp t_epoch{};
    core::Vec3 p_e_m{6378137.0, 0.0, 0.0};
    core::Vec3 v_e_mps{core::Vec3::Zero()};
    bool initial_velocity_configured{false};
    Eigen::Quaternion<core::Scalar_t> q_b2e{Eigen::Quaternion<core::Scalar_t>::Identity()};
    core::Scalar_t maximum_bank_angle_rad{std::numbers::pi_v<core::Scalar_t> / 3.0};
    GuidanceCommandFilterConfig guidance_command_filter{};
    TranslationalIntegrationMethod translational_integration{
        TranslationalIntegrationMethod::TrapezoidalPredictorCorrector};
    AutopilotModelType autopilot_model{AutopilotModelType::FirstOrder};
    FirstOrderAutopilotConfig autopilot{};
    VehicleResponseModelType vehicle_response_model{VehicleResponseModelType::FirstOrder};
    FirstOrderVehicleResponseConfig vehicle_response{};
};

/** Programmatic compatibility selection used by the ballistic convenience factory. */
enum class BallisticCoastMode
{
    GravityTurn,
    FreeInertial,
};

/**
 * Programmatic ballistic convenience configuration.
 *
 * Runtime JSON scenarios use StateMachineTrajectoryConfig; this wrapper constructs the same
 * generic Guidance graph for focused tests and direct C++ callers.
 */
struct BallisticTrajectoryConfig
{
    TrajectoryProfileConfig profile{};
    core::Time_t launch_pad_duration_s{0.0};
    core::Time_t boost_duration_s{5.0};
    core::Scalar_t boost_acceleration_b_x_mps2{20.0};
    BallisticCoastMode coast_mode{BallisticCoastMode::GravityTurn};
};

/** Programmatic constant-altitude wrapper that constructs the generic Guidance graph. */
struct ConstantAltitudeTrajectoryConfig
{
    TrajectoryProfileConfig profile{};
    core::Scalar_t speed_mps{100.0};
    core::Vec3 velocity_error_gain_n_1ps{core::Vec3::Constant(0.5)};
    core::Scalar_t altitude_error_p_gain_1ps2{0.05};
    core::Scalar_t altitude_error_d_gain_1ps{0.1};
};

enum class CalibrationManeuver
{
    HorizontalSTurn,
    VerticalSTurn,
    DutchRoll,
};

/** Programmatic calibration wrapper that constructs the generic Guidance graph. */
struct CalibrationTrajectoryConfig
{
    TrajectoryProfileConfig profile{};
    CalibrationManeuver maneuver{CalibrationManeuver::HorizontalSTurn};
    core::Scalar_t speed_mps{100.0};
    core::Scalar_t horizontal_amplitude_rad{0.2};
    core::Scalar_t vertical_amplitude_rad{0.1};
    core::Time_t period_s{20.0};
    core::Vec3 velocity_error_gain_n_1ps{core::Vec3::Constant(0.5)};
    core::Scalar_t altitude_error_p_gain_1ps2{0.05};
    core::Scalar_t altitude_error_d_gain_1ps{0.1};
    bool bank_to_turn_enabled{false};
    bool body_y_specific_force_enabled{true};
};

/** Programmatic waypoint wrapper that constructs the generic Guidance graph. */
struct WaypointTrajectoryConfig
{
    TrajectoryProfileConfig profile{};
    std::vector<core::Vec3> waypoint_e_m{};
    core::Scalar_t speed_mps{100.0};
    core::Scalar_t acceptance_radius_m{25.0};
    core::Vec3 velocity_error_gain_n_1ps{core::Vec3::Constant(0.5)};
    core::Scalar_t altitude_error_p_gain_1ps2{0.05};
    core::Scalar_t altitude_error_d_gain_1ps{0.1};
    core::Scalar_t heading_error_gain_mps2_per_rad{5.0};
    bool body_y_specific_force_enabled{true};
};

/** Fully runtime-composed generated trajectory and Guidance graph. */
struct StateMachineTrajectoryConfig
{
    TrajectoryProfileConfig profile{};
    GuidanceStateMachineDefinition state_machine{};
    TrajectoryTerminationMode termination_mode{TrajectoryTerminationMode::ConfiguredDuration};
};

/** Derives body inertial angular-rate truth from adjacent body-to-ECEF attitudes. */
void populate_truth_angular_rates(std::vector<TruthSample>& samples);

class TrajectorySource;

/** Creates an incrementally generated ballistic trajectory source. */
[[nodiscard]] std::unique_ptr<TrajectorySource>
ballistic_trajectory_source(const BallisticTrajectoryConfig& cfg);

/** Creates an incrementally generated constant-altitude trajectory source. */
[[nodiscard]] std::unique_ptr<TrajectorySource>
constant_altitude_trajectory_source(const ConstantAltitudeTrajectoryConfig& cfg);

/** Creates an incrementally generated calibration-maneuver trajectory source. */
[[nodiscard]] std::unique_ptr<TrajectorySource>
calibration_trajectory_source(const CalibrationTrajectoryConfig& cfg);

/** Creates an incrementally generated waypoint trajectory source. */
[[nodiscard]] std::unique_ptr<TrajectorySource>
waypoint_trajectory_source(const WaypointTrajectoryConfig& cfg);

/** Creates a generated trajectory from a validated runtime Guidance graph. */
[[nodiscard]] std::unique_ptr<TrajectorySource>
state_machine_trajectory_source(StateMachineTrajectoryConfig cfg);

/** Generates a tabulated simple boost/coast ballistic truth profile integrated in ECI. */
[[nodiscard]] TruthTrajectory ballistic_trajectory(const BallisticTrajectoryConfig& cfg);

/** Generates a tabulated constant-altitude, constant-speed curved-Earth profile. */
[[nodiscard]] TruthTrajectory
constant_altitude_trajectory(const ConstantAltitudeTrajectoryConfig& cfg);

/** Generates a tabulated horizontal, vertical, or coupled calibration excitation profile. */
[[nodiscard]] TruthTrajectory calibration_trajectory(const CalibrationTrajectoryConfig& cfg);

/** Generates a tabulated simple bank-limited trajectory through configured waypoints. */
[[nodiscard]] TruthTrajectory waypoint_trajectory(const WaypointTrajectoryConfig& cfg);

} // namespace navkit::sim
