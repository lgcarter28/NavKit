// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/sim/trajectory/StationaryTrajectorySource.hpp"

#include "navkit/core/environment/gravity/J2.hpp"
#include "navkit/core/environment/planet/Wgs84.hpp"
#include "navkit/core/frames/RotatingFrame.hpp"
#include "navkit/core/time/Duration.hpp"

namespace navkit::sim
{

StationaryTrajectorySource::StationaryTrajectorySource(const StationaryTrajectoryConfig& cfg)
    : m_cfg(cfg)
    , m_valid(core::timestamp_is_valid(cfg.t_epoch) && core::rational_rate_is_valid(cfg.rate) &&
              cfg.duration_s >= 0.0)
{}

bool StationaryTrajectorySource::advance_to(const core::Timestamp& t)
{
    if (!m_valid || !timestamp_is_in_source_range(t)) {
        if (m_valid && core::timestamp_less(t_end(), t)) {
            m_complete = true;
        }
        return false;
    }

    m_t_available = t;
    m_initialized = true;
    core::Duration elapsed{};
    if (!core::elapsed_time(t, m_cfg.t_epoch, elapsed)) {
        return false;
    }
    m_complete = core::duration_seconds(elapsed) >= m_cfg.duration_s;
    return true;
}

bool StationaryTrajectorySource::query(const core::Timestamp& t, TruthSample& sample) const
{
    sample = {};
    if (!m_initialized || !timestamp_is_in_source_range(t) ||
        core::timestamp_less(m_t_available, t)) {
        return false;
    }

    sample.t = t;
    sample.p_e = m_cfg.p_e;
    sample.v_e = m_cfg.v_e;
    sample.q_b2e = m_cfg.q_b2e;
    sample.w_ib_b_radps = m_cfg.w_ib_b_radps;
    return true;
}

bool StationaryTrajectorySource::query_diagnostics(const core::Timestamp& t,
                                                   TrajectoryDiagnostics& diagnostics) const
{
    diagnostics = {};
    TruthSample sample{};
    if (!query(t, sample)) {
        return false;
    }
    core::Duration elapsed{};
    if (!core::elapsed_time(t, m_cfg.t_epoch, elapsed)) {
        return false;
    }
    const core::Time_t elapsed_s = core::duration_seconds(elapsed);
    using Planet = core::environment::Wgs84;
    using Gravity = core::environment::J2<Planet>;
    core::Mat3 dcm_e2i{};
    if (!core::frames::fixed_to_inertial_matrix<Planet>(elapsed_s, dcm_e2i) ||
        !core::frames::fixed_to_inertial_position<Planet>(
            sample.p_e, elapsed_s, diagnostics.p_i_m) ||
        !core::frames::fixed_to_inertial_velocity<Planet>(
            sample.p_e, sample.v_e, elapsed_s, diagnostics.v_i_mps) ||
        !core::frames::fixed_to_inertial_acceleration<Planet>(
            sample.p_e, sample.v_e, core::Vec3::Zero(), elapsed_s, diagnostics.a_i_mps2)) {
        return false;
    }
    diagnostics.q_b2i = Eigen::Quaternion<core::Scalar_t>{dcm_e2i} * sample.q_b2e;
    diagnostics.w_ib_b_radps = sample.w_ib_b_radps;
    const core::Vec3 gravity_i_mps2 = dcm_e2i * Gravity::acceleration(sample.p_e);
    diagnostics.specific_force_ib_b_mps2 =
        diagnostics.q_b2i.conjugate() * (diagnostics.a_i_mps2 - gravity_i_mps2);
    diagnostics.guidance_velocity_reference_i_mps = diagnostics.v_i_mps;
    diagnostics.guidance_acceleration_command_i_mps2 = diagnostics.a_i_mps2;
    diagnostics.guidance_acceleration_command_b_mps2 =
        diagnostics.q_b2i.conjugate() * diagnostics.a_i_mps2;
    diagnostics.guidance_acceleration_response_i_mps2 = diagnostics.a_i_mps2;
    diagnostics.guidance_acceleration_response_b_mps2 =
        diagnostics.guidance_acceleration_command_b_mps2;
    diagnostics.autopilot_q_command_b2i = diagnostics.q_b2i;
    diagnostics.autopilot_q_response_b2i = diagnostics.q_b2i;
    diagnostics.autopilot_angular_rate_command_b_radps = sample.w_ib_b_radps;
    diagnostics.autopilot_angular_rate_controller_response_b_radps = sample.w_ib_b_radps;
    diagnostics.autopilot_gyro_observation_b_radps = sample.w_ib_b_radps;
    diagnostics.vehicle_velocity_ib_b_mps = diagnostics.q_b2i.conjugate() * diagnostics.v_i_mps;
    diagnostics.vehicle_acceleration_ib_b_mps2 =
        diagnostics.q_b2i.conjugate() * diagnostics.a_i_mps2;
    diagnostics.vehicle_specific_force_command_b_mps2 = diagnostics.specific_force_ib_b_mps2;
    diagnostics.vehicle_specific_force_command_response_b_mps2 =
        diagnostics.specific_force_ib_b_mps2;
    diagnostics.vehicle_specific_force_response_b_mps2 = diagnostics.specific_force_ib_b_mps2;
    diagnostics.vehicle_angular_rate_command_b_radps = sample.w_ib_b_radps;
    diagnostics.vehicle_angular_rate_response_b_radps = sample.w_ib_b_radps;
    return true;
}

core::Timestamp StationaryTrajectorySource::t_start() const
{
    return m_cfg.t_epoch;
}

core::Timestamp StationaryTrajectorySource::t_end() const
{
    if (!m_valid) {
        return {};
    }

    core::Timestamp t{};
    if (!core::timestamp_from_seconds(
            core::timestamp_seconds(m_cfg.t_epoch) + m_cfg.duration_s, m_cfg.t_epoch.scale, t)) {
        return {};
    }
    return t;
}

bool StationaryTrajectorySource::is_complete() const
{
    return m_complete;
}

bool StationaryTrajectorySource::timestamp_is_in_source_range(const core::Timestamp& t) const
{
    return core::timestamp_is_valid(t) && t.scale == m_cfg.t_epoch.scale &&
           !core::timestamp_less(t, m_cfg.t_epoch) && !core::timestamp_less(t_end(), t);
}

} // namespace navkit::sim
