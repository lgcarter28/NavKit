// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/sim/trajectory/TruthTrajectory.hpp"

#include "navkit/core/environment/gravity/J2.hpp"
#include "navkit/core/environment/planet/Wgs84.hpp"
#include "navkit/core/frames/LocalLevel.hpp"
#include "navkit/core/frames/RotatingFrame.hpp"
#include "navkit/core/math/Quaternion.hpp"
#include "navkit/core/time/Duration.hpp"

#include <algorithm>
#include <utility>

namespace navkit::sim
{

namespace
{

[[nodiscard]] core::Vec3 ecef_acceleration_at(const std::vector<TruthSample>& samples,
                                              const std::size_t index)
{
    if (samples.size() < 2U) {
        return core::Vec3::Zero();
    }
    const std::size_t first_index = index + 1U < samples.size() ? index : index - 1U;
    const std::size_t second_index = index + 1U < samples.size() ? index + 1U : index;
    core::Duration duration{};
    if (!core::elapsed_time(samples.at(second_index).t, samples.at(first_index).t, duration)) {
        return core::Vec3::Zero();
    }
    const core::Time_t dt_s = core::duration_seconds(duration);
    if (dt_s <= 0.0) {
        return core::Vec3::Zero();
    }
    return (samples.at(second_index).v_e - samples.at(first_index).v_e) / dt_s;
}

[[nodiscard]] bool derive_diagnostics(const std::vector<TruthSample>& samples,
                                      std::vector<TrajectoryDiagnostics>& diagnostics)
{
    diagnostics.clear();
    diagnostics.reserve(samples.size());
    if (samples.empty()) {
        return true;
    }
    using Planet = core::environment::Wgs84;
    using Gravity = core::environment::J2<Planet>;
    for (std::size_t index = 0U; index < samples.size(); ++index) {
        const TruthSample& sample = samples.at(index);
        core::Duration elapsed{};
        if (!core::elapsed_time(sample.t, samples.front().t, elapsed)) {
            return false;
        }
        const core::Time_t elapsed_s = core::duration_seconds(elapsed);
        const core::Vec3 a_e_mps2 = ecef_acceleration_at(samples, index);
        core::Mat3 dcm_e2i{};
        core::Mat3 dcm_e2n{};
        TrajectoryDiagnostics value{};
        if (!core::frames::fixed_to_inertial_matrix<Planet>(elapsed_s, dcm_e2i) ||
            !core::frames::ecef_to_ned_matrix(sample.p_e, dcm_e2n) ||
            !core::frames::fixed_to_inertial_position<Planet>(sample.p_e, elapsed_s, value.p_i_m) ||
            !core::frames::fixed_to_inertial_velocity<Planet>(
                sample.p_e, sample.v_e, elapsed_s, value.v_i_mps) ||
            !core::frames::fixed_to_inertial_acceleration<Planet>(
                sample.p_e, sample.v_e, a_e_mps2, elapsed_s, value.a_i_mps2)) {
            return false;
        }
        value.q_b2i = core::math::normalized_with_positive_scalar(
            Eigen::Quaternion<core::Scalar_t>{dcm_e2i} * sample.q_b2e);
        value.w_ib_b_radps = sample.w_ib_b_radps;
        const core::Vec3 gravity_i_mps2 = dcm_e2i * Gravity::acceleration(sample.p_e);
        value.specific_force_ib_b_mps2 =
            value.q_b2i.conjugate() * (value.a_i_mps2 - gravity_i_mps2);
        value.guidance_velocity_reference_i_mps = value.v_i_mps;
        value.guidance_acceleration_command_i_mps2 = value.a_i_mps2;
        value.guidance_acceleration_command_n_mps2 = dcm_e2n * dcm_e2i.transpose() * value.a_i_mps2;
        value.guidance_acceleration_command_b_mps2 = value.q_b2i.conjugate() * value.a_i_mps2;
        value.guidance_acceleration_response_i_mps2 = value.a_i_mps2;
        value.guidance_acceleration_response_n_mps2 = value.guidance_acceleration_command_n_mps2;
        value.guidance_acceleration_response_b_mps2 = value.guidance_acceleration_command_b_mps2;
        value.autopilot_q_command_b2i = value.q_b2i;
        value.autopilot_q_response_b2i = value.q_b2i;
        value.autopilot_angular_rate_command_b_radps = sample.w_ib_b_radps;
        value.autopilot_angular_rate_controller_response_b_radps = sample.w_ib_b_radps;
        value.autopilot_gyro_observation_b_radps = sample.w_ib_b_radps;
        value.vehicle_velocity_ib_b_mps = value.q_b2i.conjugate() * value.v_i_mps;
        value.vehicle_acceleration_ib_b_mps2 = value.q_b2i.conjugate() * value.a_i_mps2;
        value.vehicle_specific_force_command_b_mps2 = value.specific_force_ib_b_mps2;
        value.vehicle_specific_force_command_response_b_mps2 = value.specific_force_ib_b_mps2;
        value.vehicle_specific_force_response_b_mps2 = value.specific_force_ib_b_mps2;
        value.vehicle_angular_rate_command_b_radps = sample.w_ib_b_radps;
        value.vehicle_angular_rate_response_b_radps = sample.w_ib_b_radps;
        diagnostics.push_back(value);
    }
    return true;
}

} // namespace

TruthTrajectory::TruthTrajectory(std::vector<TruthSample> samples,
                                 std::vector<TrajectoryDiagnostics> diagnostics)
    : m_samples(std::move(samples))
    , m_diagnostics(std::move(diagnostics))
{
    if (m_diagnostics.empty()) {
        if (!derive_diagnostics(m_samples, m_diagnostics)) {
            m_samples.clear();
            m_diagnostics.clear();
        }
    }
    if (!m_diagnostics.empty() && m_diagnostics.size() != m_samples.size()) {
        m_samples.clear();
        m_diagnostics.clear();
    }
}

bool TruthTrajectory::empty() const
{
    return m_samples.empty();
}

std::size_t TruthTrajectory::size() const
{
    return m_samples.size();
}

const TruthSample& TruthTrajectory::first() const
{
    return m_samples.front();
}

const TruthSample& TruthTrajectory::last() const
{
    return m_samples.back();
}

const std::vector<TruthSample>& TruthTrajectory::samples() const
{
    return m_samples;
}

bool TruthTrajectory::append(const TruthSample& sample, const TrajectoryDiagnostics& diagnostics)
{
    if (!core::timestamp_is_valid(sample.t) ||
        (!m_samples.empty() && (sample.t.scale != m_samples.front().t.scale ||
                                !core::timestamp_less(m_samples.back().t, sample.t)))) {
        return false;
    }
    m_samples.push_back(sample);
    m_diagnostics.push_back(diagnostics);
    return true;
}

bool TruthTrajectory::update_last_diagnostics(const TrajectoryDiagnostics& diagnostics)
{
    if (m_samples.empty() || m_diagnostics.size() != m_samples.size()) {
        return false;
    }
    m_diagnostics.back() = diagnostics;
    return true;
}

bool TruthTrajectory::sample_at(const core::Timestamp& t, TruthSample& sample) const
{
    sample = {};
    if (m_samples.empty() || !core::timestamp_is_valid(t) || t.scale != m_samples.front().t.scale ||
        core::timestamp_less(t, m_samples.front().t) ||
        core::timestamp_less(m_samples.back().t, t)) {
        return false;
    }

    const auto upper = std::ranges::lower_bound(
        m_samples,
        t,
        [](const core::Timestamp& lhs, const core::Timestamp& rhs) {
            return core::timestamp_less(lhs, rhs);
        },
        &TruthSample::t);
    if (upper == m_samples.end()) {
        return false;
    }
    if (upper->t == t) {
        sample = *upper;
        return true;
    }
    if (upper == m_samples.begin()) {
        return false;
    }

    const TruthSample& previous = *std::prev(upper);
    const TruthSample& current = *upper;
    core::Duration full_duration{};
    core::Duration query_duration{};
    if (!core::elapsed_time(current.t, previous.t, full_duration) ||
        !core::elapsed_time(t, previous.t, query_duration)) {
        return false;
    }
    const core::Time_t full_dt_s = core::duration_seconds(full_duration);
    if (full_dt_s <= 0.0) {
        return false;
    }
    const core::Scalar_t alpha = core::duration_seconds(query_duration) / full_dt_s;

    sample.t = t;
    sample.p_e = previous.p_e + (alpha * (current.p_e - previous.p_e));
    sample.v_e = previous.v_e + (alpha * (current.v_e - previous.v_e));
    sample.q_b2e =
        core::math::normalized_with_positive_scalar(previous.q_b2e.slerp(alpha, current.q_b2e));
    sample.w_ib_b_radps =
        previous.w_ib_b_radps + (alpha * (current.w_ib_b_radps - previous.w_ib_b_radps));
    return true;
}

bool TruthTrajectory::diagnostics_at(const core::Timestamp& t,
                                     TrajectoryDiagnostics& diagnostics) const
{
    diagnostics = {};
    if (m_samples.empty() || m_diagnostics.size() != m_samples.size() ||
        !core::timestamp_is_valid(t) || t.scale != m_samples.front().t.scale ||
        core::timestamp_less(t, m_samples.front().t) ||
        core::timestamp_less(m_samples.back().t, t)) {
        return false;
    }

    const auto upper = std::ranges::lower_bound(
        m_samples,
        t,
        [](const core::Timestamp& lhs, const core::Timestamp& rhs) {
            return core::timestamp_less(lhs, rhs);
        },
        &TruthSample::t);
    if (upper == m_samples.end()) {
        return false;
    }
    const std::size_t upper_index =
        static_cast<std::size_t>(std::distance(m_samples.cbegin(), upper));
    if (upper->t == t) {
        diagnostics = m_diagnostics.at(upper_index);
        return true;
    }
    if (upper == m_samples.begin()) {
        return false;
    }

    const std::size_t previous_index = upper_index - 1U;
    const TruthSample& previous_sample = m_samples.at(previous_index);
    const TruthSample& current_sample = m_samples.at(upper_index);
    core::Duration full_duration{};
    core::Duration query_duration{};
    if (!core::elapsed_time(current_sample.t, previous_sample.t, full_duration) ||
        !core::elapsed_time(t, previous_sample.t, query_duration)) {
        return false;
    }
    const core::Time_t full_dt_s = core::duration_seconds(full_duration);
    if (full_dt_s <= 0.0) {
        return false;
    }
    const core::Scalar_t alpha = core::duration_seconds(query_duration) / full_dt_s;
    const TrajectoryDiagnostics& previous = m_diagnostics.at(previous_index);
    const TrajectoryDiagnostics& current = m_diagnostics.at(upper_index);

    diagnostics.p_i_m = previous.p_i_m + alpha * (current.p_i_m - previous.p_i_m);
    diagnostics.v_i_mps = previous.v_i_mps + alpha * (current.v_i_mps - previous.v_i_mps);
    diagnostics.a_i_mps2 = previous.a_i_mps2 + alpha * (current.a_i_mps2 - previous.a_i_mps2);
    diagnostics.q_b2i =
        core::math::normalized_with_positive_scalar(previous.q_b2i.slerp(alpha, current.q_b2i));
    diagnostics.w_ib_b_radps =
        previous.w_ib_b_radps + alpha * (current.w_ib_b_radps - previous.w_ib_b_radps);
    diagnostics.specific_force_ib_b_mps2 =
        previous.specific_force_ib_b_mps2 +
        alpha * (current.specific_force_ib_b_mps2 - previous.specific_force_ib_b_mps2);
    diagnostics.guidance_velocity_reference_i_mps = previous.guidance_velocity_reference_i_mps;
    diagnostics.guidance_acceleration_command_i_mps2 =
        previous.guidance_acceleration_command_i_mps2;
    diagnostics.guidance_acceleration_command_n_mps2 =
        previous.guidance_acceleration_command_n_mps2;
    diagnostics.guidance_acceleration_command_b_mps2 =
        previous.guidance_acceleration_command_b_mps2;
    diagnostics.guidance_acceleration_response_i_mps2 =
        previous.guidance_acceleration_response_i_mps2 +
        alpha * (current.guidance_acceleration_response_i_mps2 -
                 previous.guidance_acceleration_response_i_mps2);
    diagnostics.guidance_acceleration_response_n_mps2 =
        previous.guidance_acceleration_response_n_mps2 +
        alpha * (current.guidance_acceleration_response_n_mps2 -
                 previous.guidance_acceleration_response_n_mps2);
    diagnostics.guidance_acceleration_response_b_mps2 =
        previous.guidance_acceleration_response_b_mps2 +
        alpha * (current.guidance_acceleration_response_b_mps2 -
                 previous.guidance_acceleration_response_b_mps2);
    diagnostics.guidance_specific_force_command_b_mps2 =
        previous.guidance_specific_force_command_b_mps2;
    diagnostics.guidance_specific_force_filtered_b_mps2 =
        previous.guidance_specific_force_filtered_b_mps2 +
        alpha * (current.guidance_specific_force_filtered_b_mps2 -
                 previous.guidance_specific_force_filtered_b_mps2);
    diagnostics.guidance_reference_position_e_m = previous.guidance_reference_position_e_m;
    diagnostics.guidance_bank_command_n_rad = previous.guidance_bank_command_n_rad;
    diagnostics.guidance_bank_filtered_n_rad =
        previous.guidance_bank_filtered_n_rad +
        (alpha * (current.guidance_bank_filtered_n_rad - previous.guidance_bank_filtered_n_rad));
    diagnostics.guidance_bank_response_n_rad =
        previous.guidance_bank_response_n_rad +
        (alpha * (current.guidance_bank_response_n_rad - previous.guidance_bank_response_n_rad));
    diagnostics.autopilot_q_command_b2i = previous.autopilot_q_command_b2i;
    diagnostics.autopilot_q_response_b2i = core::math::normalized_with_positive_scalar(
        previous.autopilot_q_response_b2i.slerp(alpha, current.autopilot_q_response_b2i));
    diagnostics.autopilot_angular_rate_command_b_radps =
        previous.autopilot_angular_rate_command_b_radps;
    diagnostics.autopilot_angular_rate_controller_response_b_radps =
        previous.autopilot_angular_rate_controller_response_b_radps +
        alpha * (current.autopilot_angular_rate_controller_response_b_radps -
                 previous.autopilot_angular_rate_controller_response_b_radps);
    diagnostics.autopilot_gyro_observation_b_radps = previous.autopilot_gyro_observation_b_radps;
    diagnostics.vehicle_velocity_ib_b_mps =
        previous.vehicle_velocity_ib_b_mps +
        alpha * (current.vehicle_velocity_ib_b_mps - previous.vehicle_velocity_ib_b_mps);
    diagnostics.vehicle_acceleration_ib_b_mps2 =
        previous.vehicle_acceleration_ib_b_mps2 +
        alpha * (current.vehicle_acceleration_ib_b_mps2 - previous.vehicle_acceleration_ib_b_mps2);
    diagnostics.vehicle_specific_force_command_b_mps2 =
        previous.vehicle_specific_force_command_b_mps2;
    diagnostics.vehicle_specific_force_command_response_b_mps2 =
        previous.vehicle_specific_force_command_response_b_mps2 +
        alpha * (current.vehicle_specific_force_command_response_b_mps2 -
                 previous.vehicle_specific_force_command_response_b_mps2);
    diagnostics.vehicle_specific_force_response_b_mps2 =
        previous.vehicle_specific_force_response_b_mps2 +
        alpha * (current.vehicle_specific_force_response_b_mps2 -
                 previous.vehicle_specific_force_response_b_mps2);
    diagnostics.vehicle_angular_rate_command_b_radps =
        previous.vehicle_angular_rate_command_b_radps;
    diagnostics.vehicle_angular_rate_response_b_radps =
        previous.vehicle_angular_rate_response_b_radps +
        alpha * (current.vehicle_angular_rate_response_b_radps -
                 previous.vehicle_angular_rate_response_b_radps);
    diagnostics.velocity_tracking_error_b_mps =
        diagnostics.q_b2i.conjugate() *
        (diagnostics.guidance_velocity_reference_i_mps - diagnostics.v_i_mps);
    diagnostics.acceleration_tracking_error_b_mps2 =
        diagnostics.q_b2i.conjugate() *
        (diagnostics.guidance_acceleration_command_i_mps2 - diagnostics.a_i_mps2);
    diagnostics.attitude_tracking_error_b_rad =
        core::math::rotvec_rad_from_quaternion(core::math::normalized_with_positive_scalar(
            diagnostics.q_b2i.conjugate() * diagnostics.autopilot_q_command_b2i));
    diagnostics.angular_rate_tracking_error_b_radps =
        diagnostics.autopilot_angular_rate_command_b_radps - diagnostics.w_ib_b_radps;
    diagnostics.specific_force_tracking_error_b_mps2 =
        diagnostics.vehicle_specific_force_command_b_mps2 - diagnostics.specific_force_ib_b_mps2;
    diagnostics.angular_rate_limited = previous.angular_rate_limited;
    diagnostics.specific_force_limited = previous.specific_force_limited;
    diagnostics.guidance_state_index = previous.guidance_state_index;
    diagnostics.guidance_reference_index = previous.guidance_reference_index;
    diagnostics.guidance_reference_position_valid = previous.guidance_reference_position_valid;
    diagnostics.guidance_active = previous.guidance_active;
    diagnostics.pad_constraint_active = previous.pad_constraint_active;
    diagnostics.autopilot_active = previous.autopilot_active;
    return true;
}

} // namespace navkit::sim
