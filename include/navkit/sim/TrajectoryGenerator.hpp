// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/math/Types.hpp"
#include "navkit/core/time/RationalRate.hpp"
#include "navkit/sim/TruthSample.hpp"

#include <vector>

namespace navkit::sim
{

using navkit::core::RationalRate;
using navkit::core::Scalar_t;
using navkit::core::Time_t;
using navkit::core::Timestamp;
using navkit::core::Vec3;

struct StationaryTrajectoryConfig
{
    Time_t duration_s{60.0};
    RationalRate rate{.samples = 1U, .s = 1U};
    Timestamp t_epoch{};
    Vec3 p_e{6378137.0, 0.0, 0.0};
    Vec3 v_e{Vec3::Zero()};
    Eigen::Quaternion<Scalar_t> q_b2e{Eigen::Quaternion<Scalar_t>::Identity()};
    Vec3 w_ib_b_radps{Vec3::Zero()};
};

class TrajectoryGenerator
{
public:
    static std::vector<TruthSample> stationary(const StationaryTrajectoryConfig& cfg);
};

} // namespace navkit::sim
