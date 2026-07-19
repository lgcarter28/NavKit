// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/api/config/ConfigApi.hpp"
#include "navkit/products/components/DefaultInsCovarianceFloor.hpp"
#include "navkit/products/components/DefaultInsInitialCovariance.hpp"
#include "navkit/products/components/DefaultNumericConfig.hpp"
#include "navkit/products/components/EcefInsGnssPropagation.hpp"
#include "navkit/products/components/PrimaryGnssPosVelSensors.hpp"
#include "navkit/products/components/Profilers.hpp"

namespace navkit::config::navkit::products::ecef_ins_gnss
{

struct ProductConfig
{
    using Numeric = components::DefaultNumericConfig;
    using StateDef = core::estimation::DefaultInsStateDef;
    using SensorGraph = components::PrimaryGnssPosVelSensors<StateDef>;
    using InitialCovariance = components::DefaultInsInitialCovariance;
    using CovarianceFloor = components::DefaultInsCovarianceFloor;
    using PropagationConfig = components::EcefInsGnssPropagation;
    using Profiling = components::NullProfiling;

    using PrimaryGnssDiagnostics = typename SensorGraph::PrimaryGnssDiagnostics;
    using PrimaryGnssPositionSensor = typename SensorGraph::PrimaryGnssPositionSensor;
    using PrimaryGnssVelocitySensor = typename SensorGraph::PrimaryGnssVelocitySensor;
    using Sensors = typename SensorGraph::Sensors;

    using Profiler = typename Profiling::Profiler;
    using Propagation = typename PropagationConfig::Propagation;

    using Filter =
        core::estimation::KalmanFilter<StateDef,
                                       core::estimation::DefaultInjectionPolicy<StateDef>,
                                       core::estimation::DefaultResetPolicy<StateDef>,
                                       Sensors,
                                       Profiler>;
    using NavigatorUpdate = core::estimation::UpdatePostFilter<Filter>;
    using Navigator =
        core::estimation::Navigator<Filter, Sensors, Propagation, NavigatorUpdate, Profiler>;
};

static_assert(api::config::NavKitProductConfigPolicy<ProductConfig>);

} // namespace navkit::config::navkit::products::ecef_ins_gnss

namespace navkit::config::navkit
{

using EcefInsGnssConfig = products::ecef_ins_gnss::ProductConfig;

} // namespace navkit::config::navkit
