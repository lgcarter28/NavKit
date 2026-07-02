// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/ProfileExport.hpp"
#include "navkit/core/estimation/filter/KalmanFilter.hpp"
#include "navkit/core/estimation/filter/injection/InjectionPolicies.hpp"
#include "navkit/core/estimation/filter/reset/ResetPolicies.hpp"
#include "navkit/core/estimation/navigator/Navigator.hpp"
#include "navkit/core/estimation/navigator/update/UpdatePolicies.hpp"
#include "navkit/core/estimation/sensor/Sensor.hpp"

#include <cstddef>

namespace navkit::app_support
{

template<typename Config>
using ConfigProfiler_t = typename ConfigProfiler<Config>::type;

template<typename Model, std::size_t BufferSize, typename NoisePolicy>
using Sensor_t = core::estimation::Sensor<Model, BufferSize, NoisePolicy>;

template<typename StateDef, typename MeasurementModels, typename Profiler>
using DefaultKalmanFilter_t =
    core::estimation::KalmanFilter<StateDef,
                                   core::estimation::DefaultInjectionPolicy<StateDef>,
                                   core::estimation::DefaultResetPolicy<StateDef>,
                                   MeasurementModels,
                                   Profiler>;

template<typename Filter, typename Sensors, typename Profiler>
using UpdatePostFilterNavigator_t =
    core::estimation::Navigator<Filter, Sensors, core::estimation::UpdatePostFilter, Profiler>;

} // namespace navkit::app_support
