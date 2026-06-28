#pragma once

#include <random>
#include "navkit/core/Measurement.hpp"
#include "navkit/sim/TruthSample.hpp"

namespace navkit::sim {

struct GnssSimulatorConfig {
    Time_t dt_s{1.0};
    Scalar_t sigma_h_m{3.0};
    Scalar_t sigma_v_m{5.0};
    unsigned int seed{42U};
};

class GnssSimulator {
public:
    explicit GnssSimulator(const GnssSimulatorConfig& cfg);

    Measurement<3> generate(const TruthSample& truth);
    [[nodiscard]] Time_t dt_s() const { return m_cfg.dt_s; }

private:
    GnssSimulatorConfig m_cfg;
    std::mt19937 m_rng;
    std::normal_distribution<Scalar_t> m_unit_normal{0.0, 1.0};
};

} // namespace navkit::sim
