// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/Types.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <numbers>
#include <stdexcept>
#include <string>
#include <vector>

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
        throw std::runtime_error("JSON configuration override must be an object");
    }

    for (nlohmann::json::const_iterator iter = source.begin(); iter != source.end(); ++iter) {
        if (target.contains(iter.key()) && target.at(iter.key()).is_object() &&
            iter.value().is_object()) {
            merge_json_object(iter.value(), target[iter.key()]);
        }
        else {
            target[iter.key()] = iter.value();
        }
    }
}

[[nodiscard]] inline std::string
config_reference_chain(const std::vector<std::filesystem::path>& stack,
                       const std::filesystem::path& next_path)
{
    std::string result{};
    for (const std::filesystem::path& path : stack) {
        if (!result.empty()) {
            result += " -> ";
        }
        result += path.string();
    }
    if (!result.empty()) {
        result += " -> ";
    }
    result += next_path.string();
    return result;
}

[[nodiscard]] inline nlohmann::json
load_json_file_impl(const std::filesystem::path& path,
                    std::vector<std::filesystem::path>& reference_stack);

[[nodiscard]] inline nlohmann::json
resolve_json_value(const nlohmann::json& value,
                   const std::filesystem::path& containing_file,
                   std::vector<std::filesystem::path>& reference_stack)
{
    if (value.is_array()) {
        nlohmann::json resolved = nlohmann::json::array();
        for (const nlohmann::json& element : value) {
            resolved.push_back(resolve_json_value(element, containing_file, reference_stack));
        }
        return resolved;
    }
    if (!value.is_object()) {
        return value;
    }
    if (value.contains("components")) {
        throw std::runtime_error(
            "Legacy JSON 'components' assembly is unsupported; declare each runtime field "
            "inline or with an explicit {'config': '<relative-path>'} reference");
    }
    if (value.contains("config")) {
        for (nlohmann::json::const_iterator iter = value.begin(); iter != value.end(); ++iter) {
            if (iter.key() != "config" && iter.key() != "overrides") {
                throw std::runtime_error(
                    "A JSON config reference may contain only 'config' and optional 'overrides'");
            }
        }
        if (!value.at("config").is_string()) {
            throw std::runtime_error("JSON config reference 'config' must be a path string");
        }
        if (value.contains("overrides") && !value.at("overrides").is_object()) {
            throw std::runtime_error("JSON config reference 'overrides' must be an object");
        }

        const std::filesystem::path referenced_path =
            containing_file.parent_path() / value.at("config").get<std::string>();
        nlohmann::json resolved = load_json_file_impl(referenced_path, reference_stack);
        if (value.contains("overrides")) {
            const nlohmann::json overrides =
                resolve_json_value(value.at("overrides"), containing_file, reference_stack);
            merge_json_object(overrides, resolved);
        }
        return resolved;
    }
    if (value.contains("overrides")) {
        throw std::runtime_error(
            "JSON 'overrides' is valid only beside an explicit 'config' reference");
    }

    nlohmann::json resolved = nlohmann::json::object();
    for (nlohmann::json::const_iterator iter = value.begin(); iter != value.end(); ++iter) {
        resolved[iter.key()] = resolve_json_value(iter.value(), containing_file, reference_stack);
    }
    return resolved;
}

[[nodiscard]] inline nlohmann::json
load_json_file_impl(const std::filesystem::path& path,
                    std::vector<std::filesystem::path>& reference_stack)
{
    const std::filesystem::path absolute_path = std::filesystem::absolute(path).lexically_normal();
    if (!std::filesystem::exists(absolute_path)) {
        throw std::runtime_error("JSON config reference does not exist; reference chain: " +
                                 config_reference_chain(reference_stack, absolute_path));
    }
    const std::filesystem::path normalized_path = std::filesystem::weakly_canonical(absolute_path);
    if (std::find(reference_stack.begin(), reference_stack.end(), normalized_path) !=
        reference_stack.end()) {
        throw std::runtime_error("JSON config reference cycle detected: " +
                                 config_reference_chain(reference_stack, normalized_path));
    }

    reference_stack.push_back(normalized_path);
    try {
        const nlohmann::json raw = parse_json_file(normalized_path);
        if (!raw.is_object()) {
            throw std::runtime_error("JSON config root must be an object: " +
                                     normalized_path.string());
        }
        nlohmann::json resolved = resolve_json_value(raw, normalized_path, reference_stack);
        reference_stack.pop_back();
        return resolved;
    }
    catch (const std::exception& error) {
        reference_stack.pop_back();
        throw std::runtime_error(std::string{error.what()} + " [while loading " +
                                 normalized_path.string() + "]");
    }
}

} // namespace detail

/**
 * \brief Loads and recursively resolves one explicit runtime configuration graph.
 *
 * \details Any object may either contain its typed fields inline or be the reference object
 * `{"config": "relative/path.json"}` with an optional `overrides` object. Referenced paths are
 * resolved relative to the file that contains the reference. Legacy root-level component
 * catalogs are intentionally rejected.
 */
inline nlohmann::json load_json_file(const std::filesystem::path& path)
{
    std::vector<std::filesystem::path> reference_stack{};
    return detail::load_json_file_impl(path, reference_stack);
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
