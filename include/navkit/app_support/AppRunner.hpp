// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include <concepts>
#include <filesystem>

namespace navkit::app_support
{

template<typename Config>
concept SelectedAppConfigPolicy = requires(const std::filesystem::path& config_path) {
    typename Config::NavKit;
    typename Config::App;
    { Config::App::run(config_path) } -> std::same_as<int>;
};

template<SelectedAppConfigPolicy Config>
int run_selected_app(const std::filesystem::path& config_path)
{
    return Config::App::run(config_path);
}

} // namespace navkit::app_support
