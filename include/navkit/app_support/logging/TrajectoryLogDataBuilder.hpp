// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/environment/RotatingPlanetKinematics.hpp"
#include "navkit/core/environment/planet/PlanetPolicy.hpp"
#include "navkit/core/frames/Geodetic.hpp"
#include "navkit/core/frames/LocalLevel.hpp"
#include "navkit/core/frames/RotatingFrame.hpp"
#include "navkit/core/math/Quaternion.hpp"
#include "navkit/core/time/Duration.hpp"
#include "navkit/core/time/Timestamp.hpp"
#include "navkit/io/log_payloads/TrajectoryLogPayload.hpp"
#include "navkit/sim/trajectory/TrajectoryDynamics.hpp"
#include "navkit/sim/trajectory/TruthSample.hpp"

#include <cmath>

namespace navkit::app_support
{

/**
 * Resolve canonical trajectory truth into the frame-specific inspection contract.
 *
 * ECEF acceleration is the rotating-frame derivative of ECEF-relative velocity.
 * NED products resolve that vector in the instantaneous local-level frame. Body
 * products resolve inertial velocity and acceleration in body, matching the
 * physical quantities used by the vehicle dynamics and IMU.
 * The inertial/fixed frames are coincident at `source_epoch` and are related by
 * the selected planet's constant rotation rate.
 */
template<core::environment::EllipsoidPlanetPolicy Planet>
[[nodiscard]] bool trajectory_log_data_from_truth(const sim::TruthSample& truth,
                                                  const sim::TrajectoryDiagnostics& diagnostics,
                                                  const core::Timestamp& source_epoch,
                                                  io::TrajectoryLogData& output)
{
    static_assert(core::environment::RotatingPlanetPolicy<Planet>);

    core::Duration elapsed{};
    if (!core::elapsed_time(truth.t, source_epoch, elapsed)) {
        return false;
    }
    const core::Time_t elapsed_s = core::duration_seconds(elapsed);
    core::Vec3 a_e_mps2{};
    core::Vec3 p_lla_deg_m{};
    core::Mat3 C_e2n{};
    core::Vec3 w_en_n_radps{};
    if (!core::frames::inertial_to_fixed_acceleration<Planet>(
            diagnostics.p_i_m, diagnostics.v_i_mps, diagnostics.a_i_mps2, elapsed_s, a_e_mps2) ||
        !core::frames::fixed_m_to_lla_deg_m<Planet>(truth.p_e, p_lla_deg_m) ||
        !core::frames::fixed_to_ned_matrix<Planet>(truth.p_e, C_e2n) ||
        !core::frames::transport_rate_fixed_to_ned_radps<Planet>(
            truth.p_e, truth.v_e, w_en_n_radps)) {
        return false;
    }

    const Eigen::Quaternion<core::Scalar_t> q_b2e =
        core::math::normalized_with_positive_scalar(truth.q_b2e);
    const Eigen::Quaternion<core::Scalar_t> q_b2i =
        core::math::normalized_with_positive_scalar(diagnostics.q_b2i);
    const Eigen::Quaternion<core::Scalar_t> q_e2n{C_e2n};
    const Eigen::Quaternion<core::Scalar_t> q_b2n =
        core::math::normalized_with_positive_scalar(q_e2n * q_b2e);
    const Eigen::Quaternion<core::Scalar_t> q_i2e =
        core::math::normalized_with_positive_scalar(q_b2e * q_b2i.conjugate());
    const core::Vec3 w_if_f_radps = core::environment::planet_rate_fixed_radps<Planet>();
    const core::Vec3 w_if_b_radps = q_b2e.conjugate() * w_if_f_radps;
    const core::Vec3 w_fn_b_radps = q_b2n.conjugate() * w_en_n_radps;

    output.t = truth.t;

    output.p_e_m = truth.p_e;
    output.v_e_mps = truth.v_e;
    output.a_e_mps2 = a_e_mps2;
    output.q_b2e = q_b2e;
    output.w_eb_b_radps = diagnostics.w_ib_b_radps - w_if_b_radps;

    output.p_i_m = diagnostics.p_i_m;
    output.v_i_mps = diagnostics.v_i_mps;
    output.a_i_mps2 = diagnostics.a_i_mps2;
    output.q_b2i = q_b2i;
    output.w_ib_b_radps = diagnostics.w_ib_b_radps;

    output.p_lla_deg_m = p_lla_deg_m;
    output.v_n_mps = C_e2n * truth.v_e;
    output.a_n_mps2 = C_e2n * a_e_mps2;
    output.q_b2n = q_b2n;
    output.w_nb_b_radps = diagnostics.w_ib_b_radps - w_if_b_radps - w_fn_b_radps;

    output.v_ib_b_mps = q_b2i.conjugate() * diagnostics.v_i_mps;
    output.a_ib_b_mps2 = q_b2i.conjugate() * diagnostics.a_i_mps2;
    output.v_eb_b_mps = q_b2e.conjugate() * truth.v_e;
    output.a_eb_b_mps2 = q_b2e.conjugate() * a_e_mps2;
    output.specific_force_ib_b_mps2 = diagnostics.specific_force_ib_b_mps2;

    output.guidance_velocity_reference_i_mps = diagnostics.guidance_velocity_reference_i_mps;
    output.guidance_acceleration_command_i_mps2 = diagnostics.guidance_acceleration_command_i_mps2;
    output.guidance_acceleration_command_n_mps2 = diagnostics.guidance_acceleration_command_n_mps2;
    output.guidance_acceleration_command_b_mps2 = diagnostics.guidance_acceleration_command_b_mps2;
    output.guidance_acceleration_response_i_mps2 =
        diagnostics.guidance_acceleration_response_i_mps2;
    output.guidance_acceleration_response_n_mps2 =
        diagnostics.guidance_acceleration_response_n_mps2;
    output.guidance_acceleration_response_b_mps2 =
        diagnostics.guidance_acceleration_response_b_mps2;
    output.guidance_specific_force_command_b_mps2 =
        diagnostics.guidance_specific_force_command_b_mps2;
    output.guidance_specific_force_filtered_b_mps2 =
        diagnostics.guidance_specific_force_filtered_b_mps2;
    output.guidance_bank_command_n_rad = diagnostics.guidance_bank_command_n_rad;
    output.guidance_bank_filtered_n_rad = diagnostics.guidance_bank_filtered_n_rad;
    output.guidance_bank_response_n_rad = diagnostics.guidance_bank_response_n_rad;
    output.guidance_reference_position_e_m = diagnostics.guidance_reference_position_e_m;
    output.guidance_reference_index = diagnostics.guidance_reference_index;
    output.guidance_state_index = diagnostics.guidance_state_index;
    output.guidance_active = diagnostics.guidance_active;
    output.pad_constraint_active = diagnostics.pad_constraint_active;
    output.guidance_reference_position_valid = diagnostics.guidance_reference_position_valid;

    output.autopilot_q_command_b2i =
        core::math::normalized_with_positive_scalar(diagnostics.autopilot_q_command_b2i);
    output.autopilot_q_response_b2i =
        core::math::normalized_with_positive_scalar(diagnostics.autopilot_q_response_b2i);
    output.autopilot_q_command_b2n =
        core::math::normalized_with_positive_scalar(q_e2n * q_i2e * output.autopilot_q_command_b2i);
    output.autopilot_q_response_b2n = core::math::normalized_with_positive_scalar(
        q_e2n * q_i2e * output.autopilot_q_response_b2i);
    output.autopilot_angular_rate_command_b_radps =
        diagnostics.autopilot_angular_rate_command_b_radps;
    output.autopilot_angular_rate_feedforward_b_radps =
        diagnostics.autopilot_angular_rate_feedforward_b_radps;
    output.autopilot_angular_rate_controller_response_b_radps =
        diagnostics.autopilot_angular_rate_controller_response_b_radps;
    output.autopilot_gyro_observation_b_radps = diagnostics.autopilot_gyro_observation_b_radps;
    output.autopilot_active = diagnostics.autopilot_active;

    output.vehicle_velocity_ib_b_mps = diagnostics.vehicle_velocity_ib_b_mps;
    output.vehicle_acceleration_ib_b_mps2 = diagnostics.vehicle_acceleration_ib_b_mps2;
    output.vehicle_velocity_eb_b_mps = diagnostics.vehicle_velocity_eb_b_mps;
    output.vehicle_acceleration_eb_b_mps2 = diagnostics.vehicle_acceleration_eb_b_mps2;
    output.vehicle_specific_force_command_b_mps2 =
        diagnostics.vehicle_specific_force_command_b_mps2;
    output.vehicle_specific_force_command_response_b_mps2 =
        diagnostics.vehicle_specific_force_command_response_b_mps2;
    output.vehicle_specific_force_response_b_mps2 =
        diagnostics.vehicle_specific_force_response_b_mps2;
    output.vehicle_angular_rate_command_b_radps = diagnostics.vehicle_angular_rate_command_b_radps;
    output.vehicle_angular_rate_response_b_radps =
        diagnostics.vehicle_angular_rate_response_b_radps;

    output.velocity_tracking_error_b_mps = diagnostics.velocity_tracking_error_b_mps;
    output.acceleration_tracking_error_b_mps2 = diagnostics.acceleration_tracking_error_b_mps2;
    output.attitude_tracking_error_b_rad = diagnostics.attitude_tracking_error_b_rad;
    output.angular_rate_tracking_error_b_radps = diagnostics.angular_rate_tracking_error_b_radps;
    output.specific_force_tracking_error_b_mps2 = diagnostics.specific_force_tracking_error_b_mps2;
    output.angular_rate_limited = diagnostics.angular_rate_limited;
    output.specific_force_limited = diagnostics.specific_force_limited;

    return output.p_e_m.allFinite() && output.v_e_mps.allFinite() && output.a_e_mps2.allFinite() &&
           output.q_b2e.coeffs().allFinite() && output.p_i_m.allFinite() &&
           output.v_i_mps.allFinite() && output.a_i_mps2.allFinite() &&
           output.q_b2i.coeffs().allFinite() && output.p_lla_deg_m.allFinite() &&
           output.v_n_mps.allFinite() && output.a_n_mps2.allFinite() &&
           output.q_b2n.coeffs().allFinite() && output.v_ib_b_mps.allFinite() &&
           output.a_ib_b_mps2.allFinite() && output.v_eb_b_mps.allFinite() &&
           output.a_eb_b_mps2.allFinite() && output.specific_force_ib_b_mps2.allFinite() &&
           output.velocity_tracking_error_b_mps.allFinite() &&
           output.acceleration_tracking_error_b_mps2.allFinite() &&
           output.attitude_tracking_error_b_rad.allFinite() &&
           output.angular_rate_tracking_error_b_radps.allFinite() &&
           output.specific_force_tracking_error_b_mps2.allFinite() &&
           output.guidance_velocity_reference_i_mps.allFinite() &&
           output.guidance_acceleration_command_i_mps2.allFinite() &&
           output.guidance_acceleration_command_n_mps2.allFinite() &&
           output.guidance_acceleration_command_b_mps2.allFinite() &&
           output.guidance_acceleration_response_i_mps2.allFinite() &&
           output.guidance_acceleration_response_n_mps2.allFinite() &&
           output.guidance_acceleration_response_b_mps2.allFinite() &&
           output.guidance_specific_force_command_b_mps2.allFinite() &&
           output.guidance_specific_force_filtered_b_mps2.allFinite() &&
           std::isfinite(output.guidance_bank_command_n_rad) &&
           std::isfinite(output.guidance_bank_filtered_n_rad) &&
           std::isfinite(output.guidance_bank_response_n_rad) &&
           output.guidance_reference_position_e_m.allFinite() &&
           output.autopilot_q_command_b2i.coeffs().allFinite() &&
           output.autopilot_q_response_b2i.coeffs().allFinite() &&
           output.autopilot_q_command_b2n.coeffs().allFinite() &&
           output.autopilot_q_response_b2n.coeffs().allFinite() &&
           output.autopilot_angular_rate_command_b_radps.allFinite() &&
           output.autopilot_angular_rate_feedforward_b_radps.allFinite() &&
           output.autopilot_angular_rate_controller_response_b_radps.allFinite() &&
           output.autopilot_gyro_observation_b_radps.allFinite() &&
           output.vehicle_velocity_ib_b_mps.allFinite() &&
           output.vehicle_acceleration_ib_b_mps2.allFinite() &&
           output.vehicle_velocity_eb_b_mps.allFinite() &&
           output.vehicle_acceleration_eb_b_mps2.allFinite() &&
           output.vehicle_specific_force_command_b_mps2.allFinite() &&
           output.vehicle_specific_force_command_response_b_mps2.allFinite() &&
           output.vehicle_specific_force_response_b_mps2.allFinite() &&
           output.vehicle_angular_rate_command_b_radps.allFinite() &&
           output.vehicle_angular_rate_response_b_radps.allFinite();
}

} // namespace navkit::app_support
