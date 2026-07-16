// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/sim/GnssSimulator.hpp"

#include <cmath>

namespace navkit::sim
{

GnssSimulator::GnssSimulator(const GnssSimulatorConfig& cfg)
    : m_cfg(cfg)
    , m_rng(cfg.seed)
{}

bool GnssSimulator::should_generate(const TruthSample& truth) const
{
    if (m_cfg.dt_s <= 0.0) {
        return false;
    }
    const auto nearest_index = std::round(truth.time / m_cfg.dt_s);
    const auto nearest_time = nearest_index * m_cfg.dt_s;
    return std::abs(truth.time - nearest_time) <= (0.5e-9 + (1.0e-9 * std::abs(nearest_time)));
}

Measurement<3> GnssSimulator::generate(const TruthSample& truth)
{
    Measurement<3> meas;
    meas.time = truth.time;
    meas.z = truth.p_e;
    if (m_cfg.noise_enabled) {
        meas.z(0) += m_cfg.sigma_h_m * m_unit_normal(m_rng);
        meas.z(1) += m_cfg.sigma_h_m * m_unit_normal(m_rng);
        meas.z(2) += m_cfg.sigma_v_m * m_unit_normal(m_rng);
    }
    return meas;
}

} // namespace navkit::sim
