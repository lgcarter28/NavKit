// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/initialization/NavInitialization.hpp"
#include "navkit/app_support/runtime/JsonInput.hpp"
#include "navkit/app_support/runtime/RuntimeConfigJson.hpp"
#include "navkit/app_support/trajectory/TrajectoryProvider.hpp"
#include "navkit/core/estimation/navigator/PvaStateDef.hpp"
#include "navkit/core/math/Quaternion.hpp"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <nlohmann/json.hpp>
#include <random>
#include <string>

namespace navkit::app_support::detail
{

static constexpr std::size_t pva_cov_diag_count = 9U;
static constexpr std::size_t pva_cov_full_count = 81U;

inline void require_initialization_type(const nlohmann::json& initialization,
                                        const std::string& expected_type)
{
    require_optional_string(initialization, "type");
    const auto type_iter = initialization.find("type");
    if (type_iter == initialization.end()) {
        throw_runtime_config_error("missing required initialization.type");
    }
    if (type_iter->get<std::string>() != expected_type) {
        throw_runtime_config_error("initialization.type must be '" + expected_type +
                                   "' for the selected compile-time app config");
    }
}

[[nodiscard]] inline NavInitialization base_nav_initialization(const TrajectoryRun& trajectory)
{
    NavInitialization nav_init;
    if (trajectory.truth_samples.empty()) {
        core::estimation::pos_e_m(nav_init.pva) = trajectory.initial_position_e_m;
        return nav_init;
    }

    const auto& truth = trajectory.truth_samples.front();
    nav_init.time_s = truth.time;
    core::estimation::pos_e_m(nav_init.pva) = truth.p_e;
    core::estimation::vel_e_mps(nav_init.pva) = truth.v_e;
    core::estimation::rpy_b2e_rad(nav_init.pva) = core::math::rpy_rad_from_quaternion(truth.q_b2e);
    return nav_init;
}

[[nodiscard]] inline core::Vec3 required_vec3_from_object(const nlohmann::json& cfg,
                                                          const char* key)
{
    require_vec3(cfg, key);
    return vec3_from_json<core::Vec3>(cfg.at(key));
}

[[nodiscard]] inline core::estimation::PvaState
pva_error_from_json(const nlohmann::json& initialization)
{
    const auto& pva_error = require_object(initialization, "pva_error");

    core::estimation::PvaState error = core::estimation::PvaState::Zero();
    core::estimation::pos_e_m(error) = required_vec3_from_object(pva_error, "pos_m");
    core::estimation::vel_e_mps(error) = required_vec3_from_object(pva_error, "vel_mps");
    core::estimation::rpy_b2e_rad(error) = required_vec3_from_object(pva_error, "rpy_b2e_rad");
    return error;
}

inline void validate_pva_covariance_shape(const nlohmann::json& initialization)
{
    const auto& pva_cov = require_object(initialization, "pva_cov");
    const bool has_diag = pva_cov.contains("diag");
    const bool has_full = pva_cov.contains("full");

    if (has_diag == has_full) {
        throw_runtime_config_error(
            "initialization.pva_cov must contain exactly one of 'diag' or 'full'");
    }

    if (has_diag) {
        require_numeric_array(pva_cov, "diag", pva_cov_diag_count);
    }
    else {
        require_numeric_array(pva_cov, "full", pva_cov_full_count);
    }
}

[[nodiscard]] inline core::estimation::PvaCovariance
pva_covariance_from_json(const nlohmann::json& initialization)
{
    validate_pva_covariance_shape(initialization);
    const auto& pva_cov = initialization.at("pva_cov");

    core::estimation::PvaCovariance covariance = core::estimation::PvaCovariance::Zero();
    if (pva_cov.contains("diag")) {
        const auto& diag = pva_cov.at("diag");
        for (int i = 0; i < core::estimation::PvaStateDef::N; ++i) {
            covariance(i, i) = diag.at(static_cast<std::size_t>(i)).get<core::Scalar_t>();
        }
        return covariance;
    }

    const auto& full = pva_cov.at("full");
    for (int row = 0; row < core::estimation::PvaStateDef::N; ++row) {
        for (int col = 0; col < core::estimation::PvaStateDef::N; ++col) {
            const auto index =
                static_cast<std::size_t>((row * core::estimation::PvaStateDef::N) + col);
            covariance(row, col) = full.at(index).get<core::Scalar_t>();
        }
    }
    return covariance;
}

inline void apply_pva_error(NavInitialization& nav_init, const core::estimation::PvaState& error)
{
    nav_init.pva += error;
}

[[nodiscard]] inline core::estimation::PvaState
sample_pva_error(const core::estimation::PvaCovariance& covariance, const std::uint64_t seed)
{
    Eigen::SelfAdjointEigenSolver<core::estimation::PvaCovariance> solver(covariance);
    if (solver.info() != Eigen::Success) {
        throw_runtime_config_error("initialization.pva_cov eigensolve failed");
    }

    constexpr core::Scalar_t tolerance = -1.0e-12;
    if (solver.eigenvalues().minCoeff() < tolerance) {
        throw_runtime_config_error("initialization.pva_cov must be positive semidefinite");
    }

    core::estimation::PvaState normal = core::estimation::PvaState::Zero();
    std::mt19937_64 rng(seed);
    std::normal_distribution<core::Scalar_t> distribution{0.0, 1.0};
    for (int i = 0; i < core::estimation::PvaStateDef::N; ++i) {
        normal(i) = distribution(rng);
    }

    core::estimation::PvaState sqrt_eigenvalues = core::estimation::PvaState::Zero();
    for (int i = 0; i < core::estimation::PvaStateDef::N; ++i) {
        sqrt_eigenvalues(i) = std::sqrt(std::max<core::Scalar_t>(0.0, solver.eigenvalues()(i)));
    }

    return solver.eigenvectors() * sqrt_eigenvalues.asDiagonal() * normal;
}

} // namespace navkit::app_support::detail
