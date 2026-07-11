// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include <filesystem>
#include <nlohmann/json.hpp>
#include <string>

namespace navkit::app_support
{

struct RunSettings
{
    std::string run_name;
    std::filesystem::path output_dir;
};

inline RunSettings run_settings_from_json(const nlohmann::json& cfg)
{
    const std::string run_name = cfg.value("run_name", "stationary_gnss_demo");
    const std::filesystem::path output_dir =
        cfg.value("output_dir", std::string("output/logs/") + run_name);
    return {.run_name = run_name, .output_dir = output_dir};
}

} // namespace navkit::app_support
