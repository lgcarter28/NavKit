// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/api/config/ConfigApi.hpp"
#include "navkit/products/components/foundation/NumericConfigDefault.hpp"
#include "navkit/products/components/sensors/PrimaryGnssPosVelSensors.hpp"

namespace navkit::config::navkit::products
{

/// Reusable ECEF strapdown INS product with loosely coupled GNSS aiding.
///
/// Concrete product selections supply the state-space and the state-layout-dependent
/// initialization/covariance components explicitly. This keeps the mechanism and
/// aiding architecture reusable while making each selected state contract visible.
template<core::estimation::StateSpaceDefPolicy StateDefT,
         core::estimation::InitialCovarianceConfigPolicy<StateDefT> InitialCovarianceConfig,
         core::estimation::CovarianceFloorConfigPolicy<StateDefT> CovarianceFloorConfig,
         core::estimation::PropagationConfigPolicy<StateDefT> PropagationConfigT,
         core::profiling::ProfilingConfigPolicy ProfilingConfig>
struct EcefInsGnssLc
{
    using Numeric = components::NumericConfigDefault;
    using StateDef_t = StateDefT;
    using SensorGraph = components::PrimaryGnssPosVelSensors<StateDef_t>;
    using StateDef = StateDef_t;
    using InitialCovariance = InitialCovarianceConfig;
    using CovarianceFloor = CovarianceFloorConfig;
    using PropagationConfig_t = PropagationConfigT;
    using PropagationConfig = PropagationConfig_t;
    using Profiling = ProfilingConfig;

    using PrimaryGnssDiagnostics = typename SensorGraph::PrimaryGnssDiagnostics;
    using PrimaryGnssPositionSensor = typename SensorGraph::PrimaryGnssPositionSensor;
    using PrimaryGnssVelocitySensor = typename SensorGraph::PrimaryGnssVelocitySensor;
    using Sensors = typename SensorGraph::Sensors;

    using Profiler = typename Profiling::Profiler;
    using Propagation = typename PropagationConfig_t::Propagation;

    using Filter =
        core::estimation::KalmanFilter<StateDef_t,
                                       core::estimation::DefaultInjectionPolicy<StateDef_t>,
                                       core::estimation::DefaultResetPolicy<StateDef_t>,
                                       Sensors,
                                       Profiler>;
    using NavigatorUpdate = core::estimation::UpdateAfterEachSensor<Filter>;
    using Navigator =
        core::estimation::Navigator<Filter, Sensors, Propagation, NavigatorUpdate, Profiler>;
};

} // namespace navkit::config::navkit::products
