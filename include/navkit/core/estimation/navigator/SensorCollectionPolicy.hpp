// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/estimation/sensor/SensorTuplePolicy.hpp"

namespace navkit::core::estimation
{

template<typename Candidate>
concept SensorCollectionPolicy = SensorTuplePolicy<Candidate>;

} // namespace navkit::core::estimation
