// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/runtime/RuntimeConfigError.hpp"

#include <cstddef>
#include <cstdint>
#include <nlohmann/json.hpp>
#include <string>
#include <string_view>
#include <vector>

namespace navkit::app_support::detail
{

[[nodiscard]] inline const nlohmann::json& require_object(const nlohmann::json& cfg,
                                                          std::string_view path)
{
    const auto iter = cfg.find(std::string(path));
    if (iter == cfg.end()) {
        throw_runtime_config_error("missing required object " + quoted_path(path));
    }
    if (!iter->is_object()) {
        throw_runtime_config_error("expected " + quoted_path(path) + " to be an object");
    }
    return *iter;
}

inline void
reject_object_if_present(const nlohmann::json& cfg, std::string_view path, std::string_view reason)
{
    if (cfg.contains(std::string(path))) {
        throw_runtime_config_error("unsupported runtime object " + quoted_path(path) + ": " +
                                   std::string(reason));
    }
}

inline void require_optional_string(const nlohmann::json& cfg, std::string_view path)
{
    const auto iter = cfg.find(std::string(path));
    if (iter != cfg.end() && !iter->is_string()) {
        throw_runtime_config_error("expected " + quoted_path(path) + " to be a string");
    }
}

inline void require_optional_bool(const nlohmann::json& cfg, std::string_view path)
{
    const auto iter = cfg.find(std::string(path));
    if (iter != cfg.end() && !iter->is_boolean()) {
        throw_runtime_config_error("expected " + quoted_path(path) + " to be a boolean");
    }
}

inline void require_optional_positive_number(const nlohmann::json& cfg, std::string_view path)
{
    const auto iter = cfg.find(std::string(path));
    if (iter == cfg.end()) {
        return;
    }
    if (!iter->is_number()) {
        throw_runtime_config_error("expected " + quoted_path(path) + " to be numeric");
    }
    if (iter->get<double>() <= 0.0) {
        throw_runtime_config_error("expected " + quoted_path(path) + " to be positive");
    }
}

inline void require_optional_nonnegative_number(const nlohmann::json& cfg, std::string_view path)
{
    const auto iter = cfg.find(std::string(path));
    if (iter == cfg.end()) {
        return;
    }
    if (!iter->is_number()) {
        throw_runtime_config_error("expected " + quoted_path(path) + " to be numeric");
    }
    if (iter->get<double>() < 0.0) {
        throw_runtime_config_error("expected " + quoted_path(path) + " to be nonnegative");
    }
}

inline void require_optional_unsigned_integer(const nlohmann::json& cfg, std::string_view path)
{
    const auto iter = cfg.find(std::string(path));
    if (iter == cfg.end()) {
        return;
    }
    if (!iter->is_number_integer() && !iter->is_number_unsigned()) {
        throw_runtime_config_error("expected " + quoted_path(path) + " to be an unsigned integer");
    }
    if (iter->is_number_integer() && iter->get<std::int64_t>() < 0) {
        throw_runtime_config_error("expected " + quoted_path(path) + " to be an unsigned integer");
    }
}

inline void require_optional_vec3(const nlohmann::json& cfg, std::string_view path)
{
    const auto iter = cfg.find(std::string(path));
    if (iter == cfg.end()) {
        return;
    }
    if (!iter->is_array() || iter->size() != 3U) {
        throw_runtime_config_error("expected " + quoted_path(path) +
                                   " to be an array with exactly three numeric entries");
    }
    for (const auto& value : *iter) {
        if (!value.is_number()) {
            throw_runtime_config_error("expected every entry in " + quoted_path(path) +
                                       " to be numeric");
        }
    }
}

inline void
require_optional_numeric_array(const nlohmann::json& cfg, std::string_view path, std::size_t count)
{
    const auto iter = cfg.find(std::string(path));
    if (iter == cfg.end()) {
        return;
    }
    if (!iter->is_array() || iter->size() != count) {
        throw_runtime_config_error("expected " + quoted_path(path) + " to be an array with " +
                                   std::to_string(count) + " numeric entries");
    }
    for (const auto& value : *iter) {
        if (!value.is_number()) {
            throw_runtime_config_error("expected every entry in " + quoted_path(path) +
                                       " to be numeric");
        }
    }
}

inline void
require_numeric_array(const nlohmann::json& cfg, std::string_view path, std::size_t count)
{
    if (!cfg.contains(std::string(path))) {
        throw_runtime_config_error("missing required numeric array " + quoted_path(path));
    }
    require_optional_numeric_array(cfg, path, count);
}

inline void require_vec3(const nlohmann::json& cfg, std::string_view path)
{
    if (!cfg.contains(std::string(path))) {
        throw_runtime_config_error("missing required vector " + quoted_path(path));
    }
    require_optional_vec3(cfg, path);
}

inline bool contains_key(const std::vector<std::string_view>& keys, const std::string& key)
{
    for (const auto allowed_key : keys) {
        if (key == allowed_key) {
            return true;
        }
    }
    return false;
}

inline void reject_unknown_top_level_keys(const nlohmann::json& cfg,
                                          const std::vector<std::string_view>& allowed_keys)
{
    for (const auto& [key, unused] : cfg.items()) {
        (void)unused;
        if (!contains_key(allowed_keys, key)) {
            throw_runtime_config_error("unknown top-level key '" + key + "'");
        }
    }
}

} // namespace navkit::app_support::detail
