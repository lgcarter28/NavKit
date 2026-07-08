// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/estimation/filter/FilterPolicy.hpp"
#include "navkit/core/estimation/filter/MeasurementStatistics.hpp"
#include "navkit/core/estimation/sensor/SensorPolicy.hpp"
#include "navkit/core/estimation/sensor/SensorTuplePolicy.hpp"
#include "navkit/io/LoggerPolicy.hpp"
#include "navkit/io/log_payloads/MeasurementStatisticsLogPayload.hpp"

#include <tuple>

namespace navkit::app_support
{

template<navkit::core::estimation::SensorPolicy Sensor,
         navkit::core::estimation::FilterSensorPolicy<Sensor> Filter,
         navkit::io::LoggerPayloadPolicy<navkit::io::MeasurementStatisticsLogPayload<
             navkit::core::estimation::MeasurementStatistics<Sensor>>> Logger>
void log_sensor_measurement_statistics(Logger& logger, const Filter& filter)
{
    if (filter.template measurement_statistics_available<Sensor>()) {
        logger.log(navkit::io::MeasurementStatisticsLogPayload<
                   navkit::core::estimation::MeasurementStatistics<Sensor>>{
            .statistics = filter.template measurement_statistics<Sensor>(),
        });
    }
}

template<typename Logger, typename Filter, typename... Sensors>
void log_measurement_statistics_impl(Logger& logger, const Filter& filter, std::tuple<Sensors...>*)
{
    (log_sensor_measurement_statistics<Sensors>(logger, filter), ...);
}

template<navkit::core::estimation::SensorTuplePolicy Sensors, typename Logger, typename Filter>
void log_measurement_statistics(Logger& logger, const Filter& filter)
{
    log_measurement_statistics_impl(logger, filter, static_cast<Sensors*>(nullptr));
}

} // namespace navkit::app_support
