// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/estimation/measurement/Measurement.hpp"
#include "navkit/core/math/Types.hpp"
#include "navkit/core/time/RationalSchedule.hpp"
#include "navkit/sim/TruthSample.hpp"

#include <random>

namespace navkit::sim
{

using navkit::core::Mat3;
using navkit::core::RationalRate;
using navkit::core::RationalSchedule;
using navkit::core::Scalar_t;
using navkit::core::Time_t;
using navkit::core::Timestamp;
using navkit::core::Vec3;
using navkit::core::estimation::Measurement;

enum class GnssCovarianceFrame
{
    Ecef,
    Ned
};

struct GnssSimulatorConfig
{
    RationalRate rate{.samples = 1U, .s = 1U};
    GnssCovarianceFrame position_covariance_frame{GnssCovarianceFrame::Ecef};
    Mat3 position_cov_m2{(Vec3{9.0, 9.0, 25.0}).asDiagonal()};
    GnssCovarianceFrame velocity_covariance_frame{GnssCovarianceFrame::Ecef};
    Mat3 velocity_cov_m2ps2{(Vec3{0.04, 0.04, 0.04}).asDiagonal()};
    Vec3 p_b_ant_b_m{Vec3::Zero()};
    unsigned int seed{42U};
    bool noise_enabled{true};
};

class GnssSimulator
{
public:
    explicit GnssSimulator(const GnssSimulatorConfig& cfg);

    [[nodiscard]] bool should_generate(const TruthSample& truth) const;
    Measurement<3> generate_position(const TruthSample& truth);
    Measurement<3> generate_velocity(const TruthSample& truth);
    Measurement<3> generate(const TruthSample& truth);
    [[nodiscard]] Mat3 position_cov_e_m2(const TruthSample& truth) const;
    [[nodiscard]] Mat3 velocity_cov_e_m2ps2(const TruthSample& truth) const;
    [[nodiscard]] const RationalRate& rate() const
    {
        return m_cfg.rate;
    }

    [[nodiscard]] const GnssSimulatorConfig& config() const
    {
        return m_cfg;
    }

private:
    GnssSimulatorConfig m_cfg;
    mutable RationalSchedule m_schedule{};
    mutable bool m_schedule_initialized{false};
    std::mt19937 m_rng;
};

} // namespace navkit::sim
