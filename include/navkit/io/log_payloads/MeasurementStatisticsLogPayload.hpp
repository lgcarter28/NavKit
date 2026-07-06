// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

namespace navkit::io
{

template<typename Statistics>
struct MeasurementStatisticsLogPayload
{
    const Statistics& statistics;
};

} // namespace navkit::io
