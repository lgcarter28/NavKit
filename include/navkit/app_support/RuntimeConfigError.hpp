// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include <stdexcept>
#include <string>
#include <string_view>

namespace navkit::app_support::detail
{

[[noreturn]] inline void throw_runtime_config_error(const std::string& message)
{
    throw std::runtime_error("navkit_sim runtime config error: " + message);
}

[[nodiscard]] inline std::string quoted_path(std::string_view path)
{
    std::string result;
    result.reserve(path.size() + 2U);
    result.push_back('\'');
    result.append(path);
    result.push_back('\'');
    return result;
}

} // namespace navkit::app_support::detail
