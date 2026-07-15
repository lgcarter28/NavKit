// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/math/Types.hpp"
#include "navkit/sim/TruthSample.hpp"

#include <vector>

namespace navkit::sim
{

using navkit::core::Scalar_t;
using navkit::core::Time_t;
using navkit::core::Vec3;

struct StationaryTrajectoryConfig
{
    Time_t duration_s{60.0};
    Time_t dt_s{1.0};
    Vec3 p_e{6378137.0, 0.0, 0.0};
    Vec3 v_e{Vec3::Zero()};
    Eigen::Quaternion<Scalar_t> q_b2e{Eigen::Quaternion<Scalar_t>::Identity()};
};

class TrajectoryGenerator
{
public:
    static std::vector<TruthSample> stationary(const StationaryTrajectoryConfig& cfg);
};

} // namespace navkit::sim
