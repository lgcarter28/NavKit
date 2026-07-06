// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include <string_view>
#include <tuple>
#include <vector>

namespace navkit::app_support
{

template<typename BindingTuple>
struct EmulatorRuntimeKeys;

template<typename... Bindings>
struct EmulatorRuntimeKeys<std::tuple<Bindings...>>
{
    static std::vector<std::string_view> values()
    {
        return {Bindings::Emulator_t::RuntimeKey...};
    }
};

} // namespace navkit::app_support
