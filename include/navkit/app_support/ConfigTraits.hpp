// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include <type_traits>

namespace navkit::app_support
{

template<typename Config, typename = void>
struct NavKitConfig
{
    using type = Config;
};

template<typename Config>
struct NavKitConfig<Config, std::void_t<typename Config::NavKit>>
{
    using type = typename Config::NavKit;
};

template<typename Config>
using NavKitConfig_t = typename NavKitConfig<Config>::type;

} // namespace navkit::app_support
