// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include <concepts>

namespace navkit::core::estimation
{

struct DefaultSensorDiagnostics
{
    static constexpr bool enable_measurement_statistics = true;
};

template<typename Candidate>
concept SensorDiagnosticsPolicy = requires {
    { Candidate::enable_measurement_statistics } -> std::convertible_to<bool>;
};

} // namespace navkit::core::estimation
