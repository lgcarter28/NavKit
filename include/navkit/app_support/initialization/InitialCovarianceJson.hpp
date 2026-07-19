// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/runtime/RuntimeConfigJson.hpp"
#include "navkit/app_support/trajectory/TrajectoryProvider.hpp"
#include "navkit/core/estimation/filter/InitialCovariance.hpp"
#include "navkit/core/estimation/state/StateDefPolicy.hpp"

#include <Eigen/Core>
#include <cstddef>
#include <nlohmann/json.hpp>
#include <string>

namespace navkit::app_support::detail
{

template<typename StateDef>
concept InitialCovariancePvaErrorStateDef =
    requires {
        typename StateDef::Error::Pos;
        typename StateDef::Error::Vel;
        typename StateDef::Error::AttRotVec;
    } && (StateDef::Error::Pos::sz == 3) && (StateDef::Error::Vel::sz == 3) &&
    (StateDef::Error::AttRotVec::sz == 3);

template<typename Segment>
[[nodiscard]] constexpr bool index_in_segment(const int index)
{
    return index >= Segment::i && index < (Segment::i + Segment::sz);
}

template<typename StateDef>
[[nodiscard]] constexpr bool index_in_pva_error_segment(const int index)
{
    using Error = typename StateDef::Error;
    return index_in_segment<typename Error::Pos>(index) ||
           index_in_segment<typename Error::Vel>(index) ||
           index_in_segment<typename Error::AttRotVec>(index);
}

[[nodiscard]] inline std::string
initial_covariance_pva_frame_from_json(const nlohmann::json& initial_covariance)
{
    require_string(initial_covariance, "pva_frame");
    const std::string frame = initial_covariance.at("pva_frame").get<std::string>();
    if (frame != "ecef" && frame != "ned") {
        throw_runtime_config_error(
            "filter_initialization.initial_covariance.pva_frame must be 'ecef' or 'ned'");
    }
    return frame;
}

inline void
validate_runtime_initial_covariance_pva_diag_shape(const nlohmann::json& initial_covariance,
                                                   const std::size_t remaining_error_state_count)
{
    const std::string pva_frame = initial_covariance_pva_frame_from_json(initial_covariance);
    if (pva_frame.empty()) {
        throw_runtime_config_error(
            "filter_initialization.initial_covariance.pva_frame must not be empty");
    }
    const nlohmann::json& pva_diag = require_object(initial_covariance, "pva_diag");
    require_numeric_array(pva_diag, "pos_m2", 3U);
    require_numeric_array(pva_diag, "vel_m2ps2", 3U);
    require_numeric_array(pva_diag, "att_rotvec_rad2", 3U);
    require_numeric_array(
        initial_covariance, "remaining_error_state_diag", remaining_error_state_count);
}

template<navkit::core::estimation::StateSpaceDefPolicy StateDef>
inline void validate_runtime_initial_covariance_shape(const nlohmann::json& cfg)
{
    const auto filter_initialization_iter = cfg.find("filter_initialization");
    if (filter_initialization_iter == cfg.end() || !filter_initialization_iter->is_object()) {
        return;
    }

    const auto covariance_iter = filter_initialization_iter->find("initial_covariance");
    if (covariance_iter == filter_initialization_iter->end()) {
        return;
    }
    if (!covariance_iter->is_object()) {
        throw_runtime_config_error(
            "expected 'filter_initialization.initial_covariance' to be an object");
    }

    const nlohmann::json& initial_covariance = *covariance_iter;
    const bool has_diag = initial_covariance.contains("diag");
    const bool has_full = initial_covariance.contains("full");
    const bool has_pva_diag = initial_covariance.contains("pva_diag") ||
                              initial_covariance.contains("pva_frame") ||
                              initial_covariance.contains("remaining_error_state_diag");
    const int covariance_form_count =
        (has_diag ? 1 : 0) + (has_full ? 1 : 0) + (has_pva_diag ? 1 : 0);
    if (covariance_form_count != 1) {
        throw_runtime_config_error(
            "filter_initialization.initial_covariance must contain exactly one of 'diag', 'full', "
            "or 'pva_diag' with 'remaining_error_state_diag'");
    }

    if (has_diag) {
        require_numeric_array(
            initial_covariance, "diag", static_cast<std::size_t>(StateDef::Error::N));
        return;
    }

    if (has_full) {
        require_numeric_array(initial_covariance,
                              "full",
                              static_cast<std::size_t>(StateDef::Error::N * StateDef::Error::N));
        return;
    }

    if constexpr (InitialCovariancePvaErrorStateDef<StateDef>) {
        validate_runtime_initial_covariance_pva_diag_shape(
            initial_covariance, static_cast<std::size_t>(StateDef::Error::N - 9));
    }
    else {
        throw_runtime_config_error(
            "filter_initialization.initial_covariance pva_diag form requires Pos, Vel, and "
            "AttRotVec error-state segments");
    }
}

template<navkit::core::estimation::StateSpaceDefPolicy StateDef, typename Segment>
inline void
fill_covariance_diag_block(const nlohmann::json& values,
                           navkit::core::estimation::InitialCovariance<StateDef>& covariance)
{
    for (int i = 0; i < Segment::sz; ++i) {
        covariance(Segment::i + i, Segment::i + i) =
            values.at(static_cast<std::size_t>(i)).template get<navkit::core::Scalar_t>();
    }
}

template<navkit::core::estimation::StateSpaceDefPolicy StateDef>
[[nodiscard]] inline navkit::core::estimation::InitialCovariance<StateDef>
initial_covariance_pva_diag_from_json(const nlohmann::json& initial_covariance,
                                      const navkit::core::Vec3& reference_p_e_m)
{
    using Error = typename StateDef::Error;

    navkit::core::estimation::InitialCovariance<StateDef> covariance =
        navkit::core::estimation::InitialCovariance<StateDef>::Zero();
    const nlohmann::json& pva_diag = initial_covariance.at("pva_diag");
    fill_covariance_diag_block<StateDef, typename Error::Pos>(pva_diag.at("pos_m2"), covariance);
    fill_covariance_diag_block<StateDef, typename Error::Vel>(pva_diag.at("vel_m2ps2"), covariance);
    fill_covariance_diag_block<StateDef, typename Error::AttRotVec>(pva_diag.at("att_rotvec_rad2"),
                                                                    covariance);

    const nlohmann::json& remaining_values = initial_covariance.at("remaining_error_state_diag");
    std::size_t remaining_index = 0U;
    for (int i = 0; i < StateDef::Error::N; ++i) {
        if (!index_in_pva_error_segment<StateDef>(i)) {
            covariance(i, i) =
                remaining_values.at(remaining_index).template get<navkit::core::Scalar_t>();
            ++remaining_index;
        }
    }

    if (initial_covariance_pva_frame_from_json(initial_covariance) == "ecef") {
        return covariance;
    }

    navkit::core::estimation::InitialCovariance<StateDef> transform =
        navkit::core::estimation::InitialCovariance<StateDef>::Identity();
    const Eigen::Matrix<navkit::core::Scalar_t, 3, 3> C_n2e =
        ecef_to_ned_matrix(reference_p_e_m).transpose();
    transform.template block<3, 3>(Error::Pos::i, Error::Pos::i) = C_n2e;
    transform.template block<3, 3>(Error::Vel::i, Error::Vel::i) = C_n2e;
    transform.template block<3, 3>(Error::AttRotVec::i, Error::AttRotVec::i) = C_n2e;
    return transform * covariance * transform.transpose();
}

template<navkit::core::estimation::StateSpaceDefPolicy StateDef>
[[nodiscard]] inline navkit::core::estimation::InitialCovariance<StateDef>
initial_covariance_from_json(
    const nlohmann::json& cfg,
    const navkit::core::estimation::InitialCovariance<StateDef>& compile_time_covariance,
    const navkit::core::Vec3& reference_p_e_m)
{
    const auto filter_initialization_iter = cfg.find("filter_initialization");
    if (filter_initialization_iter == cfg.end() || !filter_initialization_iter->is_object()) {
        return compile_time_covariance;
    }

    const auto covariance_iter = filter_initialization_iter->find("initial_covariance");
    if (covariance_iter == filter_initialization_iter->end()) {
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

    if (initial_covariance.contains("pva_diag")) {
        return initial_covariance_pva_diag_from_json<StateDef>(initial_covariance, reference_p_e_m);
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
