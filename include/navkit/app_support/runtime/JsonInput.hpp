// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/Types.hpp"

#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <numbers>
#include <stdexcept>
#include <string>

namespace navkit::app_support
{

namespace detail
{

inline nlohmann::json parse_json_file(const std::filesystem::path& path)
{
    std::ifstream stream(path);
    if (!stream) {
        throw std::runtime_error("Failed to open JSON input: " + path.string());
    }

    return nlohmann::json::parse(stream);
}

inline void merge_json_object(const nlohmann::json& source, nlohmann::json& target)
{
    if (!source.is_object()) {
        throw std::runtime_error("JSON component must be an object");
    }

    for (auto iter = source.begin(); iter != source.end(); ++iter) {
        if (iter.key() == "components") {
            continue;
        }
        if (target.contains(iter.key()) && target.at(iter.key()).is_object() &&
            iter.value().is_object()) {
            merge_json_object(iter.value(), target[iter.key()]);
        }
        else {
            target[iter.key()] = iter.value();
        }
    }
}

inline nlohmann::json load_json_file_impl(const std::filesystem::path& path)
{
    const std::filesystem::path normalized_path = std::filesystem::weakly_canonical(path);
    const nlohmann::json raw = parse_json_file(normalized_path);
    nlohmann::json merged = nlohmann::json::object();

    if (raw.contains("components")) {
        if (!raw.at("components").is_object()) {
            throw std::runtime_error("JSON 'components' must be an object of role-to-path entries");
        }
        const std::filesystem::path base_dir = normalized_path.parent_path();
        for (auto component_iter = raw.at("components").begin();
             component_iter != raw.at("components").end();
             ++component_iter) {
            if (!component_iter.value().is_string()) {
                throw std::runtime_error("JSON component '" + component_iter.key() +
                                         "' must be a relative path string");
            }
            const std::filesystem::path component_path =
                base_dir / component_iter.value().template get<std::string>();
            merge_json_object(load_json_file_impl(component_path), merged);
        }
    }

    merge_json_object(raw, merged);
    return merged;
}

} // namespace detail

inline nlohmann::json load_json_file(const std::filesystem::path& path)
{
    return detail::load_json_file_impl(path);
}

template<typename Vec3>
Vec3 vec3_from_json(const nlohmann::json& value)
{
    Vec3 v;
    v << value.at(0).get<core::Scalar_t>(), value.at(1).get<core::Scalar_t>(),
        value.at(2).get<core::Scalar_t>();
    return v;
}

[[nodiscard]] inline constexpr core::Scalar_t radians_from_degrees(const core::Scalar_t angle_deg)
{
    return angle_deg * std::numbers::pi_v<core::Scalar_t> / 180.0;
}

[[nodiscard]] inline constexpr core::Scalar_t degrees_from_radians(const core::Scalar_t angle_rad)
{
    return angle_rad * 180.0 / std::numbers::pi_v<core::Scalar_t>;
}

template<typename Vec3>
[[nodiscard]] Vec3 radians_from_degrees(const Vec3& angles_deg)
{
    return angles_deg * (std::numbers::pi_v<core::Scalar_t> / 180.0);
}

template<typename Vec3>
[[nodiscard]] Vec3 degrees_from_radians(const Vec3& angles_rad)
{
    return angles_rad * (180.0 / std::numbers::pi_v<core::Scalar_t>);
}

template<typename Vec3>
[[nodiscard]] Vec3 radians_from_degrees_json(const nlohmann::json& value)
{
    return radians_from_degrees(vec3_from_json<Vec3>(value));
}

} // namespace navkit::app_support
