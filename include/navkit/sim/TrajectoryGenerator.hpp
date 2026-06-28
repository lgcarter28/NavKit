#pragma once

#include <vector>
#include "navkit/sim/TruthSample.hpp"

namespace navkit::sim {

struct StationaryTrajectoryConfig {
    Time_t duration_s{60.0};
    Time_t dt_s{1.0};
    Eigen::Matrix<Scalar_t, 3, 1> p_e{6378137.0, 0.0, 0.0};
};

class TrajectoryGenerator {
public:
    static std::vector<TruthSample> stationary(const StationaryTrajectoryConfig& cfg);
};

} // namespace navkit::sim
