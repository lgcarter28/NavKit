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
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <nlohmann/json.hpp>
#include <random>
#include <string>
#include <vector>

namespace navkit::app_support::detail
{

static constexpr std::size_t pva_error_cov_diag_count = 9U;
static constexpr std::size_t pva_error_cov_full_count = 81U;

inline void require_pva_initialization_type(const nlohmann::json& initialization,
                                            const std::string& expected_type)
{
    require_optional_string(initialization, "type");
    const auto type_iter = initialization.find("type");
    if (type_iter == initialization.end()) {
        throw_runtime_config_error("missing required pva_initialization.type");
    }
    if (type_iter->get<std::string>() != expected_type) {
        throw_runtime_config_error("pva_initialization.type must be '" + expected_type +
                                   "' for the selected compile-time app config");
    }
}

[[nodiscard]] inline std::string
pva_initialization_type_from_json(const nlohmann::json& initialization)
{
    require_optional_string(initialization, "type");
    const auto type_iter = initialization.find("type");
    if (type_iter == initialization.end()) {
        throw_runtime_config_error("missing required pva_initialization.type");
    }
    return type_iter->get<std::string>();
}

[[nodiscard]] inline PvaInitialization base_pva_initialization(const TrajectoryRun& trajectory)
{
    PvaInitialization pva_init;
    const navkit::sim::TruthSample& truth = trajectory.initial_truth;
    pva_init.t = truth.t;
    core::estimation::pos_e_m(pva_init.pva) = truth.p_e;
    core::estimation::vel_e_mps(pva_init.pva) = truth.v_e;
    core::estimation::rpy_b2e_rad(pva_init.pva) = core::math::rpy_rad_from_quaternion(truth.q_b2e);
    return pva_init;
}

[[nodiscard]] inline core::Vec3 required_vec3_from_object(const nlohmann::json& cfg,
                                                          const char* key)
{
    require_vec3(cfg, key);
    return vec3_from_json<core::Vec3>(cfg.at(key));
}

inline void require_exactly_one_pva_error_key(const nlohmann::json& pva_error,
                                              const std::vector<std::string>& keys,
                                              const std::string& group_name)
{
    const int count = count_present(pva_error, keys);
    if (count != 1) {
        throw_runtime_config_error("pva_initialization.pva_error must specify exactly one " +
                                   group_name + " convention");
    }
}

inline void validate_pva_error_shape(const nlohmann::json& initialization)
{
    const auto& pva_error = require_object(initialization, "pva_error");
    require_exactly_one_pva_error_key(pva_error, {"p_e_m", "p_n_m"}, "position-error");
    require_exactly_one_pva_error_key(pva_error, {"v_e_mps", "v_n_mps"}, "velocity-error");
    require_exactly_one_pva_error_key(
        pva_error, {"rotvec_b2e_deg", "rotvec_b2n_deg"}, "attitude-error");
    require_optional_vec3(pva_error, "p_e_m");
    require_optional_vec3(pva_error, "p_n_m");
    require_optional_vec3(pva_error, "v_e_mps");
    require_optional_vec3(pva_error, "v_n_mps");
    require_optional_vec3(pva_error, "rotvec_b2e_deg");
    require_optional_vec3(pva_error, "rotvec_b2n_deg");
}

inline void validate_pva_direct_shape(const nlohmann::json& initialization)
{
    const nlohmann::json& pva = require_object(initialization, "pva");
    require_vec3(pva, "p_e_m");
    require_vec3(pva, "v_e_mps");
    require_vec3(pva, "rpy_b2e_deg");
    require_optional_nonnegative_number(pva, "time_s");
}

[[nodiscard]] inline PvaInitialization pva_direct_from_json(const nlohmann::json& initialization)
{
    validate_pva_direct_shape(initialization);
    const nlohmann::json& pva = initialization.at("pva");

    PvaInitialization pva_init;
    if (pva.contains("time_s")) {
        if (!core::timestamp_from_seconds(
                pva.at("time_s").get<core::Time_t>(), core::TimeScale::Monotonic, pva_init.t)) {
            throw_runtime_config_error("pva_initialization.pva.time_s cannot form a timestamp");
        }
    }
    core::estimation::pos_e_m(pva_init.pva) = required_vec3_from_object(pva, "p_e_m");
    core::estimation::vel_e_mps(pva_init.pva) = required_vec3_from_object(pva, "v_e_mps");
    core::estimation::rpy_b2e_rad(pva_init.pva) =
        radians_from_degrees(required_vec3_from_object(pva, "rpy_b2e_deg"));
    return pva_init;
}

[[nodiscard]] inline Eigen::Matrix<core::Scalar_t, 3, 3>
n2e_matrix_at_position(const core::Vec3& p_e_m)
{
    return core::frames::ecef_to_ned_matrix(p_e_m).transpose();
}

[[nodiscard]] inline core::Vec3 pva_error_vec3_e_from_json(const nlohmann::json& pva_error,
                                                           const char* ecef_key,
                                                           const char* ned_key,
                                                           const core::Vec3& reference_p_e_m)
{
    if (pva_error.contains(ecef_key)) {
        return required_vec3_from_object(pva_error, ecef_key);
    }
    return n2e_matrix_at_position(reference_p_e_m) * required_vec3_from_object(pva_error, ned_key);
}

[[nodiscard]] inline core::estimation::PvaErrorState
pva_error_from_json(const nlohmann::json& initialization, const core::Vec3& reference_p_e_m)
{
    validate_pva_error_shape(initialization);
    const auto& pva_error = require_object(initialization, "pva_error");

    core::estimation::PvaErrorState error = core::estimation::PvaErrorState::Zero();
    core::estimation::pos_error_e_m(error) =
        pva_error_vec3_e_from_json(pva_error, "p_e_m", "p_n_m", reference_p_e_m);
    core::estimation::vel_error_e_mps(error) =
        pva_error_vec3_e_from_json(pva_error, "v_e_mps", "v_n_mps", reference_p_e_m);
    core::estimation::att_rotvec_e_rad(error) = radians_from_degrees(
        pva_error_vec3_e_from_json(pva_error, "rotvec_b2e_deg", "rotvec_b2n_deg", reference_p_e_m));
    return error;
}

[[nodiscard]] inline std::string pva_error_frame_from_json(const nlohmann::json& initialization)
{
    require_string(initialization, "pva_error_frame");
    const std::string frame = initialization.at("pva_error_frame").get<std::string>();
    if (frame != "ecef" && frame != "ned") {
        throw_runtime_config_error("pva_initialization.pva_error_frame must be 'ecef' or 'ned'");
    }
    return frame;
}

inline void validate_pva_error_covariance_shape(const nlohmann::json& initialization)
{
    const auto& pva_error_cov = require_object(initialization, "pva_error_cov");
    const bool has_diag = pva_error_cov.contains("diag");
    const bool has_full = pva_error_cov.contains("full");

    if (has_diag == has_full) {
        throw_runtime_config_error(
            "pva_initialization.pva_error_cov must contain exactly one of 'diag' or 'full'");
    }

    if (has_diag) {
        require_numeric_array(pva_error_cov, "diag", pva_error_cov_diag_count);
    }
    else {
        require_numeric_array(pva_error_cov, "full", pva_error_cov_full_count);
    }
}

[[nodiscard]] inline core::estimation::PvaCovariance
pva_error_covariance_from_json(const nlohmann::json& initialization,
                               const core::Vec3& reference_p_e_m)
{
    validate_pva_error_covariance_shape(initialization);
    const auto& pva_error_cov = initialization.at("pva_error_cov");

    core::estimation::PvaCovariance covariance = core::estimation::PvaCovariance::Zero();
    if (pva_error_cov.contains("diag")) {
        const auto& diag = pva_error_cov.at("diag");
        for (int i = 0; i < core::estimation::PvaErrorStateDef::N; ++i) {
            covariance(i, i) = diag.at(static_cast<std::size_t>(i)).get<core::Scalar_t>();
        }
    }
    else {
        const auto& full = pva_error_cov.at("full");
        for (int row = 0; row < core::estimation::PvaErrorStateDef::N; ++row) {
            for (int col = 0; col < core::estimation::PvaErrorStateDef::N; ++col) {
                const auto index =
                    static_cast<std::size_t>((row * core::estimation::PvaErrorStateDef::N) + col);
                covariance(row, col) = full.at(index).get<core::Scalar_t>();
            }
        }
    }

    if (!initialization.contains("pva_error")) {
        if (pva_error_frame_from_json(initialization) == "ecef") {
            return covariance;
        }

        core::estimation::PvaCovariance T = core::estimation::PvaCovariance::Identity();
        const Eigen::Matrix<core::Scalar_t, 3, 3> C_n2e = n2e_matrix_at_position(reference_p_e_m);
        T.template block<3, 3>(core::estimation::PvaErrorStateDef::Pos::i,
                               core::estimation::PvaErrorStateDef::Pos::i) = C_n2e;
        T.template block<3, 3>(core::estimation::PvaErrorStateDef::Vel::i,
                               core::estimation::PvaErrorStateDef::Vel::i) = C_n2e;
        T.template block<3, 3>(core::estimation::PvaErrorStateDef::AttRotVec::i,
                               core::estimation::PvaErrorStateDef::AttRotVec::i) = C_n2e;
        return T * covariance * T.transpose();
    }

    validate_pva_error_shape(initialization);
    const auto& pva_error = initialization.at("pva_error");
    core::estimation::PvaCovariance T = core::estimation::PvaCovariance::Identity();
    const Eigen::Matrix<core::Scalar_t, 3, 3> C_n2e = n2e_matrix_at_position(reference_p_e_m);
    if (pva_error.contains("p_n_m")) {
        T.template block<3, 3>(core::estimation::PvaErrorStateDef::Pos::i,
                               core::estimation::PvaErrorStateDef::Pos::i) = C_n2e;
    }
    if (pva_error.contains("v_n_mps")) {
        T.template block<3, 3>(core::estimation::PvaErrorStateDef::Vel::i,
                               core::estimation::PvaErrorStateDef::Vel::i) = C_n2e;
    }
    if (pva_error.contains("rotvec_b2n_deg")) {
        T.template block<3, 3>(core::estimation::PvaErrorStateDef::AttRotVec::i,
                               core::estimation::PvaErrorStateDef::AttRotVec::i) = C_n2e;
    }
    return T * covariance * T.transpose();
}

inline void apply_pva_error(PvaInitialization& pva_init,
                            const core::estimation::PvaErrorState& error)
{
    core::estimation::pos_e_m(pva_init.pva) += core::estimation::pos_error_e_m(error);
    core::estimation::vel_e_mps(pva_init.pva) += core::estimation::vel_error_e_mps(error);

    const Eigen::Quaternion<core::Scalar_t> q_b2e =
        core::math::quaternion_from_rpy_rad(core::estimation::rpy_b2e_rad(pva_init.pva));
    const Eigen::Quaternion<core::Scalar_t> delta_q =
        core::math::quaternion_from_rotvec_rad(core::estimation::att_rotvec_e_rad(error));
    core::estimation::rpy_b2e_rad(pva_init.pva) =
        core::math::rpy_rad_from_quaternion(delta_q * q_b2e);
}

[[nodiscard]] inline core::estimation::PvaErrorState
sample_pva_error(const core::estimation::PvaCovariance& covariance, const std::uint64_t seed)
{
    Eigen::SelfAdjointEigenSolver<core::estimation::PvaCovariance> solver(covariance);
    if (solver.info() != Eigen::Success) {
        throw_runtime_config_error("pva_initialization.pva_error_cov eigensolve failed");
    }

    constexpr core::Scalar_t tolerance = -1.0e-12;
    if (solver.eigenvalues().minCoeff() < tolerance) {
        throw_runtime_config_error(
            "pva_initialization.pva_error_cov must be positive semidefinite");
    }

    core::estimation::PvaErrorState normal = core::estimation::PvaErrorState::Zero();
    std::mt19937_64 rng(seed);
    std::normal_distribution<core::Scalar_t> distribution{0.0, 1.0};
    for (int i = 0; i < core::estimation::PvaErrorStateDef::N; ++i) {
        normal(i) = distribution(rng);
    }

    core::estimation::PvaErrorState sqrt_eigenvalues = core::estimation::PvaErrorState::Zero();
    for (int i = 0; i < core::estimation::PvaErrorStateDef::N; ++i) {
        sqrt_eigenvalues(i) = std::sqrt(std::max<core::Scalar_t>(0.0, solver.eigenvalues()(i)));
    }

    return solver.eigenvectors() * sqrt_eigenvalues.asDiagonal() * normal;
}

} // namespace navkit::app_support::detail
