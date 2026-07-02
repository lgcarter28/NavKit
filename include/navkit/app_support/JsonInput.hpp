// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/Types.hpp"

#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>

namespace navkit::app_support
{

inline nlohmann::json load_json_file(const std::filesystem::path& path)
{
    std::ifstream stream(path);
    if (!stream) {
        throw std::runtime_error("Failed to open JSON input: " + path.string());
    }

    return nlohmann::json::parse(stream);
}

template<typename Vec3>
Vec3 vec3_from_json(const nlohmann::json& value)
{
    Vec3 v;
    v << value.at(0).get<core::Scalar_t>(), value.at(1).get<core::Scalar_t>(),
        value.at(2).get<core::Scalar_t>();
    return v;
}

} // namespace navkit::app_support
