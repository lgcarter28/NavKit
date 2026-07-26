// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/profiling/ProfilerPolicy.hpp"

namespace navkit::core::profiling
{

/// Compile-time selection that supplies the profiler used by a product.
template<typename Candidate>
concept ProfilingConfigPolicy = requires {
    typename Candidate::Profiler;
    requires ProfilerPolicy<typename Candidate::Profiler>;
};

} // namespace navkit::core::profiling
