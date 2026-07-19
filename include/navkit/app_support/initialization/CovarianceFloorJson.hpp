// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/runtime/RuntimeConfigJson.hpp"
#include "navkit/core/estimation/filter/CovarianceFloor.hpp"
#include "navkit/core/estimation/state/StateDefPolicy.hpp"
#include "navkit/core/frames/LocalLevel.hpp"

#include <Eigen/Core>
#include <cstddef>
#include <nlohmann/json.hpp>
#include <string>
#include <string_view>

namespace navkit::app_support::detail
{

template<typename StateDef>
concept CovarianceFloorPvaErrorStateDef =
    requires {
        typename StateDef::Error::Pos;
        typename StateDef::Error::Vel;
        typename StateDef::Error::AttRotVec;
    } && (StateDef::Error::Pos::sz == 3) && (StateDef::Error::Vel::sz == 3) &&
    (StateDef::Error::AttRotVec::sz == 3);

template<typename Segment>
[[nodiscard]] constexpr bool index_in_covariance_floor_segment(const int index)
{
    return index >= Segment::i && index < (Segment::i + Segment::sz);
}

template<typename StateDef>
[[nodiscard]] constexpr bool index_in_covariance_floor_pva_segment(const int index)
{
    using Error = typename StateDef::Error;
    return index_in_covariance_floor_segment<typename Error::Pos>(index) ||
           index_in_covariance_floor_segment<typename Error::Vel>(index) ||
           index_in_covariance_floor_segment<typename Error::AttRotVec>(index);
}

inline void require_nonnegative_numeric_array(const nlohmann::json& cfg,
                                              const std::string_view path,
                                              const std::size_t count)
{
    require_numeric_array(cfg, path, count);
    for (const nlohmann::json& value : cfg.at(std::string(path))) {
        if (value.get<navkit::core::Scalar_t>() < 0.0) {
            throw_runtime_config_error("expected every entry in " + quoted_path(path) +
                                       " to be nonnegative");
        }
    }
}

[[nodiscard]] inline std::string
covariance_floor_pva_frame_from_json(const nlohmann::json& covariance_floor)
{
    require_string(covariance_floor, "pva_frame");
    const std::string frame = covariance_floor.at("pva_frame").get<std::string>();
    if (frame != "ecef" && frame != "ned") {
        throw_runtime_config_error(
            "filter_initialization.covariance_floor.pva_frame must be 'ecef' or 'ned'");
    }
    return frame;
}

inline void
validate_runtime_covariance_floor_pva_diag_shape(const nlohmann::json& covariance_floor,
                                                 const std::size_t remaining_error_state_count)
{
    const std::string pva_frame = covariance_floor_pva_frame_from_json(covariance_floor);
    if (pva_frame.empty()) {
        throw_runtime_config_error(
            "filter_initialization.covariance_floor.pva_frame must not be empty");
    }
    const nlohmann::json& pva_diag = require_object(covariance_floor, "pva_diag");
    require_nonnegative_numeric_array(pva_diag, "pos_m2", 3U);
    require_nonnegative_numeric_array(pva_diag, "vel_m2ps2", 3U);
    require_nonnegative_numeric_array(pva_diag, "att_rotvec_rad2", 3U);
    require_nonnegative_numeric_array(
        covariance_floor, "remaining_error_state_diag", remaining_error_state_count);
}

template<navkit::core::estimation::StateSpaceDefPolicy StateDef>
inline void validate_runtime_covariance_floor_shape(const nlohmann::json& cfg)
{
    const auto filter_initialization_iter = cfg.find("filter_initialization");
    if (filter_initialization_iter == cfg.end() || !filter_initialization_iter->is_object()) {
        return;
    }

    const auto floor_iter = filter_initialization_iter->find("covariance_floor");
    if (floor_iter == filter_initialization_iter->end()) {
        return;
    }
    if (!floor_iter->is_object()) {
        throw_runtime_config_error(
            "expected 'filter_initialization.covariance_floor' to be an object");
    }

    const nlohmann::json& covariance_floor = *floor_iter;
    if (covariance_floor.contains("full")) {
        throw_runtime_config_error(
            "filter_initialization.covariance_floor supports diagonal floor forms only");
    }

    const bool has_diag = covariance_floor.contains("diag");
    const bool has_pva_diag = covariance_floor.contains("pva_diag") ||
                              covariance_floor.contains("pva_frame") ||
                              covariance_floor.contains("remaining_error_state_diag");
    const int floor_form_count = (has_diag ? 1 : 0) + (has_pva_diag ? 1 : 0);
    if (floor_form_count != 1) {
        throw_runtime_config_error(
            "filter_initialization.covariance_floor must contain exactly one of 'diag' or "
            "'pva_diag' with 'remaining_error_state_diag'");
    }

    if (has_diag) {
        require_nonnegative_numeric_array(
            covariance_floor, "diag", static_cast<std::size_t>(StateDef::Error::N));
        return;
    }

    if constexpr (CovarianceFloorPvaErrorStateDef<StateDef>) {
        validate_runtime_covariance_floor_pva_diag_shape(
            covariance_floor, static_cast<std::size_t>(StateDef::Error::N - 9));
    }
    else {
        throw_runtime_config_error(
            "filter_initialization.covariance_floor pva_diag form requires Pos, Vel, and "
            "AttRotVec error-state segments");
    }
}

template<navkit::core::estimation::StateSpaceDefPolicy StateDef, typename Segment>
inline void
fill_covariance_floor_diag_block(const nlohmann::json& values,
                                 navkit::core::estimation::CovarianceFloor<StateDef>& floor)
{
    for (int i = 0; i < Segment::sz; ++i) {
        floor(Segment::i + i, Segment::i + i) =
            values.at(static_cast<std::size_t>(i)).template get<navkit::core::Scalar_t>();
    }
}

template<navkit::core::estimation::StateSpaceDefPolicy StateDef>
[[nodiscard]] inline navkit::core::estimation::CovarianceFloor<StateDef>
covariance_floor_pva_diag_from_json(const nlohmann::json& covariance_floor,
                                    const navkit::core::Vec3& reference_p_e_m)
{
    using Error = typename StateDef::Error;

    navkit::core::estimation::CovarianceFloor<StateDef> floor =
        navkit::core::estimation::CovarianceFloor<StateDef>::Zero();
    const nlohmann::json& pva_diag = covariance_floor.at("pva_diag");
    fill_covariance_floor_diag_block<StateDef, typename Error::Pos>(pva_diag.at("pos_m2"), floor);
    fill_covariance_floor_diag_block<StateDef, typename Error::Vel>(pva_diag.at("vel_m2ps2"),
                                                                    floor);
    fill_covariance_floor_diag_block<StateDef, typename Error::AttRotVec>(
        pva_diag.at("att_rotvec_rad2"), floor);

    const nlohmann::json& remaining_values = covariance_floor.at("remaining_error_state_diag");
    std::size_t remaining_index = 0U;
    for (int i = 0; i < StateDef::Error::N; ++i) {
        if (!index_in_covariance_floor_pva_segment<StateDef>(i)) {
            floor(i, i) =
                remaining_values.at(remaining_index).template get<navkit::core::Scalar_t>();
            ++remaining_index;
        }
    }

    if (covariance_floor_pva_frame_from_json(covariance_floor) == "ecef") {
        return floor;
    }

    navkit::core::estimation::CovarianceFloor<StateDef> transform =
        navkit::core::estimation::CovarianceFloor<StateDef>::Identity();
    const Eigen::Matrix<navkit::core::Scalar_t, 3, 3> C_n2e =
        navkit::core::frames::ecef_to_ned_matrix(reference_p_e_m).transpose();
    transform.template block<3, 3>(Error::Pos::i, Error::Pos::i) = C_n2e;
    transform.template block<3, 3>(Error::Vel::i, Error::Vel::i) = C_n2e;
    transform.template block<3, 3>(Error::AttRotVec::i, Error::AttRotVec::i) = C_n2e;
    return transform * floor * transform.transpose();
}

template<navkit::core::estimation::StateSpaceDefPolicy StateDef>
[[nodiscard]] inline navkit::core::estimation::CovarianceFloor<StateDef> covariance_floor_from_json(
    const nlohmann::json& cfg,
    const navkit::core::estimation::CovarianceFloor<StateDef>& compile_time_floor,
    const navkit::core::Vec3& reference_p_e_m)
{
    const auto filter_initialization_iter = cfg.find("filter_initialization");
    if (filter_initialization_iter == cfg.end() || !filter_initialization_iter->is_object()) {
        return compile_time_floor;
    }

    const auto floor_iter = filter_initialization_iter->find("covariance_floor");
    if (floor_iter == filter_initialization_iter->end()) {
        return compile_time_floor;
    }

    validate_runtime_covariance_floor_shape<StateDef>(cfg);
    const nlohmann::json& covariance_floor = *floor_iter;

    navkit::core::estimation::CovarianceFloor<StateDef> floor =
        navkit::core::estimation::CovarianceFloor<StateDef>::Zero();
    if (covariance_floor.contains("diag")) {
        const nlohmann::json& values = covariance_floor.at("diag");
        for (int i = 0; i < StateDef::Error::N; ++i) {
            floor(i, i) =
                values.at(static_cast<std::size_t>(i)).template get<navkit::core::Scalar_t>();
        }
        return floor;
    }

    return covariance_floor_pva_diag_from_json<StateDef>(covariance_floor, reference_p_e_m);
}

} // namespace navkit::app_support::detail
