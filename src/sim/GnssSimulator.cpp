// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/sim/GnssSimulator.hpp"

#include "navkit/core/environment/RotatingPlanetKinematics.hpp"
#include "navkit/core/environment/planet/Wgs84.hpp"
#include "navkit/core/frames/LocalLevel.hpp"
#include "navkit/core/time/Duration.hpp"
#include "navkit/sim/RandomDraw.hpp"

#include <algorithm>
#include <cmath>

namespace navkit::sim
{

GnssSimulator::GnssSimulator(const GnssSimulatorConfig& cfg)
    : m_cfg(cfg)
    , m_rng(cfg.seed)
{}

bool GnssSimulator::should_generate(const TruthSample& truth) const
{
    if (!m_schedule_initialized) {
        if (!m_schedule.reset(truth.t, m_cfg.rate)) {
            return false;
        }
        m_schedule_epoch = truth.t;
        m_schedule_initialized = true;
    }
    return m_schedule.due(truth.t) && availability_active(truth.t);
}

bool GnssSimulator::availability_active(const Timestamp& t) const
{
    if (m_cfg.active_windows.empty()) {
        return true;
    }

    core::Duration elapsed{};
    if (!core::elapsed_time(t, m_schedule_epoch, elapsed)) {
        return false;
    }

    const Time_t elapsed_s = core::duration_seconds(elapsed);
    return std::ranges::any_of(m_cfg.active_windows, [elapsed_s](const GnssActiveWindow& window) {
        return elapsed_s >= window.start_s && elapsed_s < window.end_s;
    });
}

Measurement<3> GnssSimulator::generate_position(const TruthSample& truth)
{
    Measurement<3> meas;
    meas.t = truth.t;
    meas.z = truth.p_e + (truth.q_b2e * m_cfg.p_b_ant_b_m);
    if (m_cfg.noise_enabled) {
        meas.z += draw_normal_cov<3>(position_cov_e_m2(truth), m_rng);
    }
    return meas;
}

Measurement<3> GnssSimulator::generate_velocity(const TruthSample& truth)
{
    Measurement<3> meas;
    meas.t = truth.t;
    const Vec3 omega_ie_b =
        truth.q_b2e.conjugate() *
        navkit::core::environment::planet_rate_fixed_radps<navkit::core::environment::Wgs84>();
    const Vec3 omega_eb_b = truth.w_ib_b_radps - omega_ie_b;
    meas.z = truth.v_e + (truth.q_b2e * omega_eb_b.cross(m_cfg.p_b_ant_b_m));
    if (m_cfg.noise_enabled) {
        meas.z += draw_normal_cov<3>(velocity_cov_e_m2ps2(truth), m_rng);
    }
    return meas;
}

Measurement<3> GnssSimulator::generate(const TruthSample& truth)
{
    return generate_position(truth);
}

Mat3 GnssSimulator::position_cov_e_m2(const TruthSample& truth) const
{
    if (m_cfg.position_covariance_frame == GnssCovarianceFrame::Ecef) {
        return m_cfg.position_cov_m2;
    }

    const Mat3 dcm_n2e = navkit::core::frames::ned_to_ecef_matrix(truth.p_e);
    return dcm_n2e * m_cfg.position_cov_m2 * dcm_n2e.transpose();
}

Mat3 GnssSimulator::velocity_cov_e_m2ps2(const TruthSample& truth) const
{
    if (m_cfg.velocity_covariance_frame == GnssCovarianceFrame::Ecef) {
        return m_cfg.velocity_cov_m2ps2;
    }

    const Mat3 dcm_n2e = navkit::core::frames::ned_to_ecef_matrix(truth.p_e);
    return dcm_n2e * m_cfg.velocity_cov_m2ps2 * dcm_n2e.transpose();
}

} // namespace navkit::sim
