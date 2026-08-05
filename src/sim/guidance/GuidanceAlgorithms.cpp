// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/sim/guidance/GuidanceAlgorithms.hpp"

#include "navkit/core/environment/RotatingPlanetKinematics.hpp"
#include "navkit/core/environment/planet/Wgs84.hpp"
#include "navkit/core/frames/Geodetic.hpp"
#include "navkit/core/frames/LocalLevel.hpp"
#include "navkit/core/frames/RotatingFrame.hpp"
#include "navkit/core/math/Quaternion.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace navkit::sim::guidance
{

namespace
{

using Planet = core::environment::Wgs84;

}

core::Vec3 velocity_n_mps(const KinematicReference& reference)
{
    const core::Scalar_t horizontal_speed_mps = reference.speed_mps * std::cos(reference.pitch_rad);
    return core::Vec3{
        horizontal_speed_mps * std::cos(reference.heading_rad),
        horizontal_speed_mps * std::sin(reference.heading_rad),
        -reference.speed_mps * std::sin(reference.pitch_rad),
    };
}

core::Vec3 feedforward_acceleration_n_mps2(const KinematicReference& reference)
{
    const core::Scalar_t sin_pitch = std::sin(reference.pitch_rad);
    const core::Scalar_t cos_pitch = std::cos(reference.pitch_rad);
    const core::Scalar_t sin_heading = std::sin(reference.heading_rad);
    const core::Scalar_t cos_heading = std::cos(reference.heading_rad);
    return core::Vec3{
        reference.speed_mps * ((-sin_pitch * reference.pitch_rate_radps * cos_heading) -
                               (cos_pitch * sin_heading * reference.heading_rate_radps)),
        reference.speed_mps * ((-sin_pitch * reference.pitch_rate_radps * sin_heading) +
                               (cos_pitch * cos_heading * reference.heading_rate_radps)),
        -reference.speed_mps * cos_pitch * reference.pitch_rate_radps,
    };
}

core::Vec3 velocity_reference_i_mps(const TrajectoryControlState& state,
                                    const TrajectoryEnvironment& environment,
                                    const core::Vec3& velocity_reference_n_mps)
{
    return state.v_i_mps +
           (environment.C_n2i * (velocity_reference_n_mps - environment.v_eb_n_mps));
}

bool velocity_derivative_n_to_acceleration_i(const core::Vec3& velocity_derivative_command_n_mps2,
                                             const TrajectoryEnvironment& environment,
                                             core::Vec3& acceleration_command_n_mps2,
                                             core::Vec3& acceleration_command_i_mps2)
{
    core::Vec3 transport_rate_en_n_radps{};
    if (!velocity_derivative_command_n_mps2.allFinite() ||
        !core::frames::transport_rate_en_n_radps(
            environment.p_e_m, environment.v_eb_e_mps, transport_rate_en_n_radps)) {
        return false;
    }
    const core::Vec3 acceleration_command_e_mps2 =
        environment.C_e2n.transpose() * (velocity_derivative_command_n_mps2 +
                                         transport_rate_en_n_radps.cross(environment.v_eb_n_mps));
    if (!core::frames::fixed_to_inertial_acceleration<Planet>(environment.p_e_m,
                                                              environment.v_eb_e_mps,
                                                              acceleration_command_e_mps2,
                                                              environment.elapsed_s,
                                                              acceleration_command_i_mps2)) {
        return false;
    }
    acceleration_command_n_mps2 = environment.C_i2n * acceleration_command_i_mps2;
    return acceleration_command_n_mps2.allFinite() && acceleration_command_i_mps2.allFinite();
}

bool local_rpy_b2n_rad(const TrajectoryControlState& state,
                       const TrajectoryEnvironment& environment,
                       core::Vec3& rpy_b2n_rad)
{
    const Eigen::Quaternion<core::Scalar_t> q_i2n{environment.C_i2n};
    rpy_b2n_rad = core::math::rpy_rad_from_quaternion(
        core::math::normalized_with_positive_scalar(q_i2n * state.q_b2i));
    return rpy_b2n_rad.allFinite();
}

bool altitude_m(const core::Vec3& position_e_m, core::Scalar_t& altitude_output_m)
{
    core::Vec3 position_lla_deg_m{};
    if (!core::frames::ecef_m_to_lla_deg_m(position_e_m, position_lla_deg_m)) {
        return false;
    }
    altitude_output_m = position_lla_deg_m.z();
    return std::isfinite(altitude_output_m);
}

core::Scalar_t wrap_angle_rad(const core::Scalar_t angle_rad)
{
    return std::remainder(angle_rad, 2.0 * std::numbers::pi_v<core::Scalar_t>);
}

bool coordinated_bank_command_rad(const core::Vec3& acceleration_command_i_mps2,
                                  const core::Scalar_t reference_heading_rad,
                                  const core::Scalar_t maximum_bank_angle_rad,
                                  const TrajectoryEnvironment& environment,
                                  core::Scalar_t& bank_command_rad)
{
    core::Vec3 forward_n{std::cos(reference_heading_rad), std::sin(reference_heading_rad), 0.0};
    if (environment.v_eb_n_mps.squaredNorm() > 1.0e-12) {
        forward_n = environment.v_eb_n_mps.normalized();
    }
    const core::Vec3 ned_down_n{0.0, 0.0, 1.0};
    core::Vec3 zero_bank_down_n = ned_down_n - (forward_n.dot(ned_down_n) * forward_n);
    if (!forward_n.allFinite() || !zero_bank_down_n.allFinite() ||
        zero_bank_down_n.squaredNorm() <= 1.0e-12 || !std::isfinite(maximum_bank_angle_rad) ||
        maximum_bank_angle_rad <= 0.0) {
        return false;
    }
    zero_bank_down_n.normalize();
    core::Vec3 zero_bank_right_n = zero_bank_down_n.cross(forward_n);
    if (!zero_bank_right_n.allFinite() || zero_bank_right_n.squaredNorm() <= 1.0e-12) {
        return false;
    }
    zero_bank_right_n.normalize();

    const core::Vec3 specific_force_command_n_mps2 =
        (environment.C_i2n * acceleration_command_i_mps2) - environment.gravity_n_mps2;
    const core::Scalar_t lateral_specific_force_mps2 =
        zero_bank_right_n.dot(specific_force_command_n_mps2);
    const core::Scalar_t upward_specific_force_mps2 =
        -zero_bank_down_n.dot(specific_force_command_n_mps2);
    if (!specific_force_command_n_mps2.allFinite() || !std::isfinite(upward_specific_force_mps2) ||
        upward_specific_force_mps2 <= 0.0) {
        return false;
    }
    const core::Scalar_t unlimited_bank_rad =
        wrap_angle_rad(std::atan2(lateral_specific_force_mps2, upward_specific_force_mps2));
    bank_command_rad =
        std::clamp(unlimited_bank_rad, -maximum_bank_angle_rad, maximum_bank_angle_rad);
    return std::isfinite(bank_command_rad);
}

} // namespace navkit::sim::guidance
