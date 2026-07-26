// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/initialization/InitialTruthReference.hpp"
#include "navkit/app_support/runtime/RuntimeConfigJson.hpp"
#include "navkit/core/estimation/filter/injection/InjectionPolicies.hpp"
#include "navkit/core/estimation/state/State.hpp"
#include "navkit/sim/RandomDraw.hpp"

#include <Eigen/Eigenvalues>
#include <cstddef>
#include <cstdint>
#include <nlohmann/json.hpp>
#include <random>
#include <string>
#include <string_view>
#include <vector>

namespace navkit::app_support::detail
{

inline void
reject_unknown_initial_estimate_error_keys(const nlohmann::json& initial_estimate_error,
                                           const std::vector<std::string_view>& allowed_keys)
{
    for (nlohmann::json::const_iterator iter = initial_estimate_error.begin();
         iter != initial_estimate_error.end();
         ++iter) {
        const std::string& key = iter.key();
        if (!contains_key(allowed_keys, key)) {
            throw_runtime_config_error("unknown key '" + key +
                                       "' in 'filter_initialization.initial_estimate_error'");
        }
    }
}

template<navkit::core::estimation::StateSpaceDefPolicy StateDef>
inline void validate_runtime_initial_estimate_error_shape(const nlohmann::json& cfg)
{
    const nlohmann::json::const_iterator filter_initialization_iter =
        cfg.find("filter_initialization");
    if (filter_initialization_iter == cfg.end() || !filter_initialization_iter->is_object()) {
        return;
    }

    const nlohmann::json::const_iterator estimate_error_iter =
        filter_initialization_iter->find("initial_estimate_error");
    if (estimate_error_iter == filter_initialization_iter->end()) {
        return;
    }
    if (!estimate_error_iter->is_object()) {
        throw_runtime_config_error(
            "expected 'filter_initialization.initial_estimate_error' to be an object");
    }
    if (filter_initialization_iter->contains("nominal_state")) {
        throw_runtime_config_error(
            "filter_initialization.initial_estimate_error cannot be combined with nominal_state");
    }

    const nlohmann::json& estimate_error = *estimate_error_iter;
    require_string(estimate_error, "type");
    const std::string type = estimate_error.at("type").get<std::string>();
    const std::size_t error_count = static_cast<std::size_t>(StateDef::Error::N);
    if (type == "explicit_error") {
        reject_unknown_initial_estimate_error_keys(estimate_error, {"type", "values"});
        require_numeric_array(estimate_error, "values", error_count);
        return;
    }
    if (type != "random_error") {
        throw_runtime_config_error(
            "filter_initialization.initial_estimate_error.type must be 'explicit_error' or "
            "'random_error'");
    }

    reject_unknown_initial_estimate_error_keys(estimate_error, {"type", "covariance", "seed"});
    const nlohmann::json& covariance = require_object(estimate_error, "covariance");
    const bool has_diag = covariance.contains("diag");
    const bool has_full = covariance.contains("full");
    if (has_diag == has_full) {
        throw_runtime_config_error(
            "filter_initialization.initial_estimate_error.covariance must contain exactly one "
            "of 'diag' or 'full'");
    }
    if (has_diag) {
        require_numeric_array(covariance, "diag", error_count);
    }
    else {
        require_numeric_array(covariance, "full", error_count * error_count);
    }
    require_unsigned_integer(estimate_error, "seed");
}

template<navkit::core::estimation::StateSpaceDefPolicy StateDef>
[[nodiscard]] inline navkit::core::estimation::ErrorState<StateDef>
initial_estimate_error_from_json(const nlohmann::json& cfg)
{
    using ErrorState = navkit::core::estimation::ErrorState<StateDef>;
    using ErrorCovariance = navkit::core::estimation::ErrorStateCov<StateDef>;

    validate_runtime_initial_estimate_error_shape<StateDef>(cfg);
    const nlohmann::json& estimate_error =
        cfg.at("filter_initialization").at("initial_estimate_error");
    if (estimate_error.at("type").get<std::string>() == "explicit_error") {
        ErrorState error = ErrorState::Zero();
        const nlohmann::json& values = estimate_error.at("values");
        for (int index = 0; index < StateDef::Error::N; ++index) {
            error(index) = values.at(static_cast<std::size_t>(index)).get<core::Scalar_t>();
        }
        return error;
    }

    ErrorCovariance covariance = ErrorCovariance::Zero();
    const nlohmann::json& covariance_json = estimate_error.at("covariance");
    if (covariance_json.contains("diag")) {
        const nlohmann::json& diagonal = covariance_json.at("diag");
        for (int index = 0; index < StateDef::Error::N; ++index) {
            covariance(index, index) =
                diagonal.at(static_cast<std::size_t>(index)).get<core::Scalar_t>();
        }
    }
    else {
        const nlohmann::json& full = covariance_json.at("full");
        for (int row = 0; row < StateDef::Error::N; ++row) {
            for (int column = 0; column < StateDef::Error::N; ++column) {
                const std::size_t value_index =
                    static_cast<std::size_t>((row * StateDef::Error::N) + column);
                covariance(row, column) = full.at(value_index).get<core::Scalar_t>();
            }
        }
    }

    if (!covariance.isApprox(covariance.transpose(), 1.0e-12)) {
        throw_runtime_config_error(
            "filter_initialization.initial_estimate_error.covariance must be symmetric");
    }
    const Eigen::SelfAdjointEigenSolver<ErrorCovariance> eigensolver(covariance);
    if (eigensolver.info() != Eigen::Success || eigensolver.eigenvalues().minCoeff() < -1.0e-12) {
        throw_runtime_config_error(
            "filter_initialization.initial_estimate_error.covariance must be positive "
            "semidefinite");
    }
    std::mt19937_64 rng(estimate_error.at("seed").get<std::uint64_t>());
    return navkit::sim::draw_normal_cov<StateDef::Error::N>(covariance, rng);
}

template<navkit::core::estimation::StateSpaceDefPolicy StateDef>
inline void
apply_runtime_initial_estimate_error(const nlohmann::json& cfg,
                                     const InitialTruthReference<StateDef>& reference,
                                     navkit::core::estimation::NominalState<StateDef>& state)
{
    const nlohmann::json::const_iterator filter_initialization_iter =
        cfg.find("filter_initialization");
    if (filter_initialization_iter == cfg.end() || !filter_initialization_iter->is_object() ||
        !filter_initialization_iter->contains("initial_estimate_error")) {
        return;
    }

    state = reference.truth_state;
    const navkit::core::estimation::ErrorState<StateDef> estimate_error =
        initial_estimate_error_from_json<StateDef>(cfg);
    navkit::core::estimation::DefaultInjectionPolicy<StateDef>::apply(state, estimate_error);
}

} // namespace navkit::app_support::detail
