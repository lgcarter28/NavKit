// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include <cstddef>

namespace navkit
{

struct Config
{
    using Scalar_t = double;
    using Time_t = double;

    static constexpr std::size_t IMU_BUFF_SIZE = 256;
    static constexpr std::size_t GNSS_BUFF_SIZE = 16;
    static constexpr std::size_t BARO_BUFF_SIZE = 64;
};

using Scalar_t = Config::Scalar_t;
using Time_t = Config::Time_t;

} // namespace navkit
