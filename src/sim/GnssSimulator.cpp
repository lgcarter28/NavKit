#include "navkit/sim/GnssSimulator.hpp"

namespace navkit::sim {

GnssSimulator::GnssSimulator(const GnssSimulatorConfig& cfg)
    : m_cfg(cfg), m_rng(cfg.seed) {}

Measurement<3> GnssSimulator::generate(const TruthSample& truth) {
    Measurement<3> meas;
    meas.time = truth.time;
    meas.z = truth.p_e;
    meas.z(0) += m_cfg.sigma_h_m * m_unit_normal(m_rng);
    meas.z(1) += m_cfg.sigma_h_m * m_unit_normal(m_rng);
    meas.z(2) += m_cfg.sigma_v_m * m_unit_normal(m_rng);
    return meas;
}

} // namespace navkit::sim
