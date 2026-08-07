// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/sim/math/FirstOrderResponse.hpp"

#include <cmath>

namespace navkit::sim
{

core::Scalar_t exact_first_order_step(const core::Scalar_t previous,
                                      const core::Scalar_t command,
                                      const core::Time_t time_constant_s,
                                      const core::Time_t dt_s)
{
    if (time_constant_s == 0.0) {
        return command;
    }
    return command + (std::exp(-dt_s / time_constant_s) * (previous - command));
}

core::Vec3 exact_first_order_step(const core::Vec3& previous,
                                  const core::Vec3& command,
                                  const core::Vec3& time_constant_s,
                                  const core::Time_t dt_s)
{
    core::Vec3 result{};
    for (Eigen::Index axis = 0; axis < result.size(); ++axis) {
        result(axis) =
            exact_first_order_step(previous(axis), command(axis), time_constant_s(axis), dt_s);
    }
    return result;
}

} // namespace navkit::sim
