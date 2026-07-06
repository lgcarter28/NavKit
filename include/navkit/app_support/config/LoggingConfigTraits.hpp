// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/io/RunLogger.hpp"

#include <type_traits>

namespace navkit::app_support
{

template<typename Config, typename = void>
struct LoggerConfig
{
    using type = io::RunLogger;
};

template<typename Config>
struct LoggerConfig<Config, std::void_t<typename Config::Logger>>
{
    using type = typename Config::Logger;
};

template<typename Config>
using LoggerConfig_t = typename LoggerConfig<Config>::type;

} // namespace navkit::app_support
