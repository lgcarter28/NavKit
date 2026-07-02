// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/ConfigPolicy.hpp"
#include "navkit/core/config/Types.hpp"
#include "navkit/core/estimation/sensor/SensorConfigPolicy.hpp"

#include <cstddef>

namespace navkit::config::navkit
{

struct MinimalNumericConfig
{
    using Scalar_t = core::Scalar_t;
    using Time_t = core::Time_t;
};

struct MinimalGnssBufferConfig
{
    static constexpr std::size_t BufferSize = 16;
};

struct MinimalConfig
{
    using Numeric = MinimalNumericConfig;
    using GnssBuffer = MinimalGnssBufferConfig;
};

static_assert(core::config::NumericConfigPolicy<MinimalNumericConfig>);
static_assert(core::estimation::BufferConfigPolicy<MinimalGnssBufferConfig>);
static_assert(core::config::ConfigPolicy<MinimalConfig>);
static_assert(core::estimation::BufferConfigPolicy<MinimalConfig::GnssBuffer>);

} // namespace navkit::config::navkit
