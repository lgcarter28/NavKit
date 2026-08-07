// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/Types.hpp"
#include "navkit/core/estimation/sensor/SensorId.hpp"
#include "navkit/core/time/Timestamp.hpp"

#include <Eigen/Dense>
#include <cstdint>

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
    bool innovation_covariance_valid{false};
    bool gate_enabled{false};
    Timestamp t{};

    O_t innovation{O_t::Zero()};
    R_t innovation_covariance{R_t::Zero()};
    R_t measurement_covariance{R_t::Zero()};
    H_t jacobian_h{H_t::Zero()};
    K_t kalman_gain{K_t::Zero()};

    Scalar_t nis{0.0};
    Scalar_t gate_probability{1.0};
    Scalar_t gate_threshold{0.0};
    std::uint32_t gate_dof{static_cast<std::uint32_t>(MeasurementModel_t::M)};
};

} // namespace navkit::core::estimation
