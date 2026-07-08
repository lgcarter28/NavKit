// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

namespace navkit::app_support
{

template<typename Config>
struct LoggerConfig
{
    using type = typename Config::Logger;
};

template<typename Config>
using LoggerConfig_t = typename LoggerConfig<Config>::type;

} // namespace navkit::app_support
