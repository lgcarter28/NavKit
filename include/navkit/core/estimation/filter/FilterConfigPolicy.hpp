// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include <concepts>

namespace navkit::core::estimation
{

template<typename Candidate>
concept MeasurementStatisticsConfigPolicy = requires {
    { Candidate::EnableMeasurementStatistics } -> std::convertible_to<bool>;
};

} // namespace navkit::core::estimation
