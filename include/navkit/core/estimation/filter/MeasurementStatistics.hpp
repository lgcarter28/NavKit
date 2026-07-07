// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/Types.hpp"
#include "navkit/core/estimation/sensor/SensorId.hpp"

#include <Eigen/Dense>

namespace navkit::core::estimation
{

template<typename Sensor>
struct MeasurementStatistics
{
    static constexpr SensorId Id = Sensor::Id;

    using Sensor_t = Sensor;
    using MeasurementModel_t = typename Sensor_t::MeasurementModel_t;

    using O_t = typename MeasurementModel_t::O_t;
    using R_t = typename MeasurementModel_t::R_t;
    using H_t = typename MeasurementModel_t::H_t;
    using K_t = typename MeasurementModel_t::K_t;

    bool valid{false};
    bool accepted{false};
    Time_t time{0.0};

    O_t innovation{O_t::Zero()};
    R_t innovation_covariance{R_t::Zero()};
    R_t measurement_covariance{R_t::Zero()};
    H_t jacobian_h{H_t::Zero()};
    K_t kalman_gain{K_t::Zero()};

    Scalar_t nis{0.0};
};

} // namespace navkit::core::estimation
