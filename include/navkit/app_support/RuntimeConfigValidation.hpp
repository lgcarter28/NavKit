// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include <cstdint>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

namespace navkit::app_support
{

namespace detail
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

} // namespace detail

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

template<typename Config>
void validate_runtime_config(const nlohmann::json& cfg)
{
    using EmulatorBindings = typename Config::EmulatorBindings;

    if (!cfg.is_object()) {
        detail::throw_runtime_config_error("root input must be a JSON object");
    }

    auto allowed_keys = EmulatorRuntimeKeys<EmulatorBindings>::values();
    allowed_keys.push_back("run_name");
    allowed_keys.push_back("output_dir");
    allowed_keys.push_back("trajectory");
    allowed_keys.push_back("filter");
    detail::reject_unknown_top_level_keys(cfg, allowed_keys);

    detail::require_optional_string(cfg, "run_name");
    detail::require_optional_string(cfg, "output_dir");

    const auto& trajectory = detail::require_object(cfg, "trajectory");
    detail::require_optional_string(trajectory, "type");
    if (const auto type_iter = trajectory.find("type");
        type_iter != trajectory.end() && type_iter->get<std::string>() != "stationary") {
        detail::throw_runtime_config_error(
            "trajectory.type must be 'stationary' for the current navkit_sim trajectory provider");
    }
    detail::require_optional_positive_number(trajectory, "duration_s");
    detail::require_optional_positive_number(trajectory, "dt_s");
    detail::require_vec3(trajectory, "p_e_m");

    std::apply(
        [&cfg](auto... binding) {
            ((decltype(binding)::Emulator_t::validate_runtime_config(cfg)), ...);
        },
        EmulatorBindings{});

    if (const auto filter_iter = cfg.find("filter"); filter_iter != cfg.end()) {
        if (!filter_iter->is_object()) {
            detail::throw_runtime_config_error("expected 'filter' to be an object");
        }
        detail::require_optional_vec3(*filter_iter, "initial_position_offset_m");
        detail::require_optional_nonnegative_number(*filter_iter, "initial_position_sigma_m");
    }
}

} // namespace navkit::app_support
