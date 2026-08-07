// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/Types.hpp"
#include "navkit/sim/trajectory/TrajectoryState.hpp"

namespace navkit::sim::guidance
{

/** Frame-explicit kinematic reference assembled by Guidance blocks. */
struct KinematicReference
{
    core::Scalar_t speed_mps{};
    core::Scalar_t pitch_rad{};
    core::Scalar_t heading_rad{};
    core::Scalar_t pitch_rate_radps{};
    core::Scalar_t heading_rate_radps{};
};

/** Convert speed, flight-path elevation, and heading into NED velocity. */
[[nodiscard]] core::Vec3 velocity_n_mps(const KinematicReference& reference);

/** Analytic NED acceleration associated with a kinematic reference. */
[[nodiscard]] core::Vec3 feedforward_acceleration_n_mps2(const KinematicReference& reference);

/** Express a local velocity reference in ECI without changing the current position. */
[[nodiscard]] core::Vec3 velocity_reference_i_mps(const TrajectoryControlState& state,
                                                  const TrajectoryEnvironment& environment,
                                                  const core::Vec3& velocity_reference_n_mps);

/** Convert commanded NED velocity derivative into total ECI acceleration. */
[[nodiscard]] bool
velocity_derivative_n_to_acceleration_i(const core::Vec3& velocity_derivative_command_n_mps2,
                                        const TrajectoryEnvironment& environment,
                                        core::Vec3& acceleration_command_n_mps2,
                                        core::Vec3& acceleration_command_i_mps2);

/** Derive local body attitude from an ECI attitude and current environment. */
[[nodiscard]] bool local_rpy_b2n_rad(const TrajectoryControlState& state,
                                     const TrajectoryEnvironment& environment,
                                     core::Vec3& rpy_b2n_rad);

/** Recover WGS-84 ellipsoid height from an ECEF position. */
[[nodiscard]] bool altitude_m(const core::Vec3& position_e_m, core::Scalar_t& altitude_output_m);

/** Wrap an angle to the principal interval centered on zero. */
[[nodiscard]] core::Scalar_t wrap_angle_rad(core::Scalar_t angle_rad);

/** Calculate a bank-limited coordinated-turn command about the velocity direction. */
[[nodiscard]] bool coordinated_bank_command_rad(const core::Vec3& acceleration_command_i_mps2,
                                                core::Scalar_t reference_heading_rad,
                                                core::Scalar_t maximum_bank_angle_rad,
                                                const TrajectoryEnvironment& environment,
                                                core::Scalar_t& bank_command_rad);

} // namespace navkit::sim::guidance
