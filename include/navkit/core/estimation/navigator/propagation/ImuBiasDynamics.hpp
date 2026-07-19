// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/math/Types.hpp"

#include <concepts>
#include <type_traits>

namespace navkit::core::estimation
{

struct GaussMarkovImuBiasDynamics
{
    Vec3 gyro_bias_correlation_rate_1ps{Vec3::Zero()};
    Vec3 accel_bias_correlation_rate_1ps{Vec3::Zero()};
};

template<typename Candidate>
concept ImuBiasDynamicsConfigPolicy = requires {
    typename Candidate::ImuBiasDynamics_t;
    requires std::same_as<typename Candidate::ImuBiasDynamics_t, GaussMarkovImuBiasDynamics>;
    requires std::same_as<std::remove_cvref_t<decltype(Candidate::imu_bias_dynamics)>,
                          GaussMarkovImuBiasDynamics>;
};

struct ZeroGaussMarkovImuBiasDynamics
{
    using ImuBiasDynamics_t = GaussMarkovImuBiasDynamics;

    inline static const ImuBiasDynamics_t imu_bias_dynamics{};
};

} // namespace navkit::core::estimation
