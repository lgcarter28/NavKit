// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/api/config/ConfigApi.hpp"
#include "navkit/core/models/GnssPosModel.hpp"
#include "navkit/core/models/GnssVelModel.hpp"

#include <cstddef>
#include <tuple>

namespace navkit::config::navkit::products::components
{

template<core::estimation::StateSpaceDefPolicy StateDef>
struct PrimaryGnssPosVelSensors
{
    using PrimaryGnssPositionMeasurementModel = core::models::GnssPosModel<StateDef>;
    using PrimaryGnssVelocityMeasurementModel = core::models::GnssVelModel<StateDef>;

    static constexpr core::estimation::SensorId primary_gnss_position_sensor_id = 0U;
    static constexpr core::estimation::SensorId primary_gnss_velocity_sensor_id = 1U;
    static constexpr std::size_t primary_gnss_buffer_size = 16U;

    using PrimaryGnssDiagnostics = core::estimation::DefaultSensorDiagnostics;
    using PrimaryGnssPositionSensor =
        core::estimation::Sensor<primary_gnss_position_sensor_id,
                                 PrimaryGnssPositionMeasurementModel,
                                 primary_gnss_buffer_size,
                                 core::estimation::GnssFixedNoisePolicy,
                                 PrimaryGnssDiagnostics>;
    using PrimaryGnssVelocitySensor =
        core::estimation::Sensor<primary_gnss_velocity_sensor_id,
                                 PrimaryGnssVelocityMeasurementModel,
                                 primary_gnss_buffer_size,
                                 core::estimation::GnssFixedNoisePolicy,
                                 PrimaryGnssDiagnostics>;

    using Sensors = std::tuple<PrimaryGnssPositionSensor, PrimaryGnssVelocitySensor>;
};

} // namespace navkit::config::navkit::products::components
