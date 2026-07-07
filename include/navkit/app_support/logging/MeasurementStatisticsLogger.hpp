// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/estimation/filter/FilterPolicy.hpp"
#include "navkit/core/estimation/sensor/SensorPolicy.hpp"

#include <tuple>

namespace navkit::app_support
{

template<navkit::core::estimation::SensorPolicy Sensor,
         typename Logger,
         navkit::core::estimation::FilterPolicy Filter>
void log_measurement_statistics_for_sensor(Logger& logger, const Filter& filter)
{
    if constexpr (requires {
                      logger.log_gnss_pos_statistics(
                          filter.template measurement_statistics<Sensor>());
                  }) {
        if (filter.template has_measurement_statistics<Sensor>()) {
            logger.log_gnss_pos_statistics(filter.template measurement_statistics<Sensor>());
        }
    }
}

template<typename Logger, typename Filter, typename... Statistics>
void log_measurement_statistics_impl(Logger& logger,
                                     const Filter& filter,
                                     std::tuple<Statistics...>*)
{
    (log_measurement_statistics_for_sensor<typename Statistics::Sensor_t>(logger, filter), ...);
}

template<typename StatisticsTuple, typename Logger, typename Filter>
void log_measurement_statistics(Logger& logger, const Filter& filter)
{
    log_measurement_statistics_impl(logger, filter, static_cast<StatisticsTuple*>(nullptr));
}

} // namespace navkit::app_support
