// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/api/config/ConfigApi.hpp"
#include "navkit/core/environment/gravity/J2.hpp"
#include "navkit/core/environment/planet/Wgs84.hpp"
#include "navkit/core/estimation/navigator/propagation/EcefInsPropagation.hpp"

#include <cstddef>

namespace navkit::config::navkit::products::components
{

struct EcefInsGnssPropagation
{
    using Planet = core::environment::Wgs84;
    using Gravity = core::environment::J2<Planet>;

    static constexpr std::size_t imu_buffer_capacity = 1024U;
    static constexpr std::size_t covariance_history_capacity = 256U;
    static constexpr core::Time_t covariance_update_rate_hz = 100.0;
    static constexpr bool apply_coning_sculling_compensation = true;

    using Propagation =
        core::estimation::EcefInsPropagation<Planet,
                                             Gravity,
                                             core::estimation::EcefInsZeroProcessNoise,
                                             imu_buffer_capacity,
                                             covariance_history_capacity,
                                             covariance_update_rate_hz,
                                             apply_coning_sculling_compensation>;
};

} // namespace navkit::config::navkit::products::components
