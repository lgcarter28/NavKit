// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/estimation/navigator/propagation/ImuBiasDynamics.hpp"

namespace navkit::config::navkit::products::components
{

struct GaussMarkovImuBiasDynamicsDefault
{
    using ImuBiasDynamics_t = core::estimation::GaussMarkovImuBiasDynamics;

    inline static const ImuBiasDynamics_t imu_bias_dynamics{
        .gyro_bias_correlation_rate_1ps = core::Vec3::Constant(1.0 / 3600.0),
        .accel_bias_correlation_rate_1ps = core::Vec3::Constant(1.0 / 3600.0),
    };
};

static_assert(core::estimation::ImuBiasDynamicsConfigPolicy<GaussMarkovImuBiasDynamicsDefault>);

} // namespace navkit::config::navkit::products::components
