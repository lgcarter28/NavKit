// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include <tuple>

namespace navkit::app_support
{

template<typename Sensor, typename Logger, typename Filter>
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
void log_measurement_statistics(Logger& logger,
                                const Filter& filter,
                                std::tuple<Statistics...> statistics_tuple)
{
    static_cast<void>(statistics_tuple);
    (log_measurement_statistics_for_sensor<typename Statistics::Sensor_t>(logger, filter), ...);
}

} // namespace navkit::app_support
