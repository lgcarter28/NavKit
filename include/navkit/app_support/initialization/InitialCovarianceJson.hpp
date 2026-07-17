// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/runtime/RuntimeConfigJson.hpp"
#include "navkit/core/estimation/filter/InitialCovariance.hpp"
#include "navkit/core/estimation/state/StateDefPolicy.hpp"

#include <cstddef>
#include <nlohmann/json.hpp>

namespace navkit::app_support::detail
{

template<navkit::core::estimation::StateSpaceDefPolicy StateDef>
inline void validate_runtime_initial_covariance_shape(const nlohmann::json& cfg)
{
    const auto initialization_iter = cfg.find("initialization");
    if (initialization_iter == cfg.end() || !initialization_iter->is_object()) {
        return;
    }

    const auto covariance_iter = initialization_iter->find("initial_covariance");
    if (covariance_iter == initialization_iter->end()) {
        return;
    }
    if (!covariance_iter->is_object()) {
        throw_runtime_config_error("expected 'initialization.initial_covariance' to be an object");
    }

    const nlohmann::json& initial_covariance = *covariance_iter;
    const bool has_diag = initial_covariance.contains("diag");
    const bool has_full = initial_covariance.contains("full");
    if (has_diag == has_full) {
        throw_runtime_config_error(
            "initialization.initial_covariance must contain exactly one of 'diag' or 'full'");
    }

    if (has_diag) {
        require_numeric_array(
            initial_covariance, "diag", static_cast<std::size_t>(StateDef::Error::N));
    }
    else {
        require_numeric_array(initial_covariance,
                              "full",
                              static_cast<std::size_t>(StateDef::Error::N * StateDef::Error::N));
    }
}

template<navkit::core::estimation::StateSpaceDefPolicy StateDef>
[[nodiscard]] inline navkit::core::estimation::InitialCovariance<StateDef>
initial_covariance_from_json(
    const nlohmann::json& cfg,
    const navkit::core::estimation::InitialCovariance<StateDef>& compile_time_covariance)
{
    const auto initialization_iter = cfg.find("initialization");
    if (initialization_iter == cfg.end() || !initialization_iter->is_object()) {
        return compile_time_covariance;
    }

    const auto covariance_iter = initialization_iter->find("initial_covariance");
    if (covariance_iter == initialization_iter->end()) {
        return compile_time_covariance;
    }

    validate_runtime_initial_covariance_shape<StateDef>(cfg);
    const nlohmann::json& initial_covariance = *covariance_iter;

    navkit::core::estimation::InitialCovariance<StateDef> covariance =
        navkit::core::estimation::InitialCovariance<StateDef>::Zero();
    if (initial_covariance.contains("diag")) {
        const nlohmann::json& values = initial_covariance.at("diag");
        for (int i = 0; i < StateDef::Error::N; ++i) {
            covariance(i, i) =
                values.at(static_cast<std::size_t>(i)).template get<navkit::core::Scalar_t>();
        }
        return covariance;
    }

    const nlohmann::json& values = initial_covariance.at("full");
    for (int row = 0; row < StateDef::Error::N; ++row) {
        for (int col = 0; col < StateDef::Error::N; ++col) {
            const auto index = static_cast<std::size_t>((row * StateDef::Error::N) + col);
            covariance(row, col) = values.at(index).template get<navkit::core::Scalar_t>();
        }
    }
    return covariance;
}

} // namespace navkit::app_support::detail
