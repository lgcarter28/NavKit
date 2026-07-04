// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/api/config/ConfigApi.hpp"

#include <cstddef>

namespace navkit::config::navkit::products::minimal
{

struct NumericConfig
{
    using Scalar_t = core::Scalar_t;
    using Time_t = core::Time_t;
};

struct GnssBufferConfig
{
    static constexpr std::size_t BufferSize = 16;
};

struct ProductConfig
{
    using Numeric = NumericConfig;
    using GnssBuffer = GnssBufferConfig;
};

static_assert(core::config::NumericConfigPolicy<NumericConfig>);
static_assert(core::estimation::BufferConfigPolicy<GnssBufferConfig>);
static_assert(core::config::ConfigPolicy<ProductConfig>);
static_assert(core::estimation::BufferConfigPolicy<ProductConfig::GnssBuffer>);

} // namespace navkit::config::navkit::products::minimal

namespace navkit::config::navkit
{

using MinimalConfig = products::minimal::ProductConfig;

} // namespace navkit::config::navkit
