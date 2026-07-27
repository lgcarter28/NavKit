// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/math/Types.hpp"
#include "navkit/core/time/RationalRate.hpp"
#include "navkit/sim/TruthTrajectory.hpp"

#include <Eigen/Geometry>
#include <vector>

namespace navkit::sim
{

/** Common ECEF initial condition and native cadence for generated truth profiles. */
struct TrajectoryProfileConfig
{
    core::Time_t duration_s{60.0};
    core::RationalRate rate{.samples = 1U, .s = 1U};
    core::Timestamp t_epoch{};
    core::Vec3 p_e_m{6378137.0, 0.0, 0.0};
    core::Vec3 v_e_mps{core::Vec3::Zero()};
    Eigen::Quaternion<core::Scalar_t> q_b2e{Eigen::Quaternion<core::Scalar_t>::Identity()};
};

/** A launch-pad dwell, fixed-body axial boost, and unpowered ECEF ballistic coast. */
struct BallisticTrajectoryConfig
{
    TrajectoryProfileConfig profile{};
    core::Time_t launch_pad_duration_s{0.0};
    core::Time_t boost_duration_s{5.0};
    core::Scalar_t boost_acceleration_b_x_mps2{20.0};
};

/** A level, constant-speed great-circle profile at constant WGS-84 ellipsoid height. */
struct ConstantAltitudeTrajectoryConfig
{
    TrajectoryProfileConfig profile{};
    core::Scalar_t speed_mps{100.0};
};

enum class CalibrationManeuver
{
    HorizontalSTurn,
    VerticalSTurn,
    BankLeftRight,
};

/** A constant-speed local-level calibration excitation about an initial ECEF state. */
struct CalibrationTrajectoryConfig
{
    TrajectoryProfileConfig profile{};
    CalibrationManeuver maneuver{CalibrationManeuver::HorizontalSTurn};
    core::Scalar_t speed_mps{100.0};
    core::Scalar_t amplitude_rad{0.2};
    core::Time_t period_s{20.0};
};

/** A simple bank-limited waypoint follower over a local-level curved-Earth path. */
struct WaypointTrajectoryConfig
{
    TrajectoryProfileConfig profile{};
    std::vector<core::Vec3> waypoint_e_m{};
    core::Scalar_t speed_mps{100.0};
    core::Scalar_t bank_limit_rad{0.35};
    core::Scalar_t acceptance_radius_m{25.0};
};

/** Derives body inertial angular-rate truth from adjacent body-to-ECEF attitudes. */
void populate_truth_angular_rates(std::vector<TruthSample>& samples);

/** Generates a tabulated simple boost/coast ballistic truth profile. */
[[nodiscard]] TruthTrajectory ballistic_trajectory(const BallisticTrajectoryConfig& cfg);

/** Generates a tabulated constant-altitude, constant-speed curved-Earth profile. */
[[nodiscard]] TruthTrajectory
constant_altitude_trajectory(const ConstantAltitudeTrajectoryConfig& cfg);

/** Generates a tabulated horizontal/vertical/bank calibration excitation profile. */
[[nodiscard]] TruthTrajectory calibration_trajectory(const CalibrationTrajectoryConfig& cfg);

/** Generates a tabulated simple bank-limited trajectory through configured waypoints. */
[[nodiscard]] TruthTrajectory waypoint_trajectory(const WaypointTrajectoryConfig& cfg);

} // namespace navkit::sim
