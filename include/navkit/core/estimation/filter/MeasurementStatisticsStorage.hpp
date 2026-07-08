// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/estimation/filter/MeasurementStatistics.hpp"
#include "navkit/core/estimation/sensor/SensorPolicy.hpp"

#include <tuple>

namespace navkit::core::estimation
{

template<typename Sensors>
struct MeasurementStatisticsStorage;

template<SensorPolicy... Sensors>
struct MeasurementStatisticsStorage<std::tuple<Sensors...>>
{
    using type = std::tuple<MeasurementStatistics<Sensors>...>;
};

template<typename Sensors>
using MeasurementStatisticsStorage_t = typename MeasurementStatisticsStorage<Sensors>::type;

} // namespace navkit::core::estimation
