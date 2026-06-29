// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/sim/TruthSample.hpp"

#include <vector>

namespace navkit::sim
{

struct StationaryTrajectoryConfig
{
    Time_t duration_s{60.0};
    Time_t dt_s{1.0};
    Eigen::Matrix<Scalar_t, 3, 1> p_e{6378137.0, 0.0, 0.0};
};

class TrajectoryGenerator
{
public:
    static std::vector<TruthSample> stationary(const StationaryTrajectoryConfig& cfg);
};

} // namespace navkit::sim
