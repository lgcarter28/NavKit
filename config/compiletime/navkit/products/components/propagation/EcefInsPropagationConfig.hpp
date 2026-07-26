// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/api/config/ConfigApi.hpp"
#include "navkit/core/environment/gravity/J2.hpp"
#include "navkit/core/environment/planet/Wgs84.hpp"
#include "navkit/core/estimation/navigator/propagation/EcefInsPropagation.hpp"
#include "navkit/products/components/propagation/GaussMarkovImuBiasDynamicsDefault.hpp"
#include "navkit/products/components/propagation/ImuProcessNoiseDefault.hpp"

#include <cstddef>

namespace navkit::config::navkit::products::components
{

/// ECEF single-IMU strapdown propagation selection.
///
/// This slice owns mechanization choices only. GNSS aiding belongs to the
/// enclosing product sensor graph, not to this configuration.
struct EcefInsPropagationConfig
{
    using Planet = core::environment::Wgs84;
    using Gravity = core::environment::J2<Planet>;
    using ProcessNoise = ImuProcessNoiseDefault;
    using ImuBiasDynamics = GaussMarkovImuBiasDynamicsDefault;

    static constexpr std::size_t imu_buffer_capacity = 1024U;
    static constexpr std::size_t covariance_history_capacity = 256U;
    static constexpr core::Time_t covariance_update_rate_hz = 100.0;
    static constexpr bool apply_coning_sculling_compensation = true;

    using Propagation = core::estimation::EcefInsPropagation<Planet,
                                                             Gravity,
                                                             ProcessNoise,
                                                             ImuBiasDynamics,
                                                             imu_buffer_capacity,
                                                             covariance_history_capacity,
                                                             covariance_update_rate_hz,
                                                             apply_coning_sculling_compensation>;
};

} // namespace navkit::config::navkit::products::components
