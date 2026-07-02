// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include <cstdint>

namespace navkit::core::profiling
{

enum class ProfilePoint : std::uint16_t
{
    NavigatorProcessMeasurements,
    KalmanObservationUpdate,
    PropagationUpdate
};

} // namespace navkit::core::profiling
