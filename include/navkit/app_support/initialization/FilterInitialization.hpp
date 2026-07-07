// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/runtime/JsonInput.hpp"
#include "navkit/core/config/Types.hpp"
#include "navkit/core/estimation/filter/FilterPolicy.hpp"
#include "navkit/core/estimation/state/StateDefPolicy.hpp"

#include <Eigen/Dense>
#include <nlohmann/json.hpp>

namespace navkit::app_support
{

template<navkit::core::estimation::StateDefPolicy StateDef,
         navkit::core::estimation::FilterPolicy Filter>
void configure_initial_filter_state(Filter& filter,
                                    const nlohmann::json& cfg,
                                    const Eigen::Matrix<core::Scalar_t, 3, 1>& p_e)
{
    typename Filter::State_t initial_state = Filter::State_t::Zero();
    initial_state.template segment<3>(StateDef::Pos::i) = p_e;

    const auto filter_config = cfg.find("filter");
    if (filter_config != cfg.end() && filter_config->contains("initial_position_offset_m")) {
        initial_state.template segment<3>(StateDef::Pos::i) +=
            vec3_from_json<Eigen::Matrix<core::Scalar_t, 3, 1>>(
                filter_config->at("initial_position_offset_m"));
    }

    filter.set_state(initial_state);

    typename Filter::P_t initial_covariance = Filter::P_t::Identity();
    const core::Scalar_t sigma_p0 =
        cfg.value("filter", nlohmann::json::object()).value("initial_position_sigma_m", 100.0);
    initial_covariance *= 1.0e-6;
    initial_covariance.template block<3, 3>(StateDef::Pos::i, StateDef::Pos::i) =
        (sigma_p0 * sigma_p0) * Eigen::Matrix<core::Scalar_t, 3, 3>::Identity();
    filter.set_covariance(initial_covariance);
}

} // namespace navkit::app_support
