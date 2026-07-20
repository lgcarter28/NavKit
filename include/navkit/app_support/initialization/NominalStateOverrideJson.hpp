// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/app_support/runtime/RuntimeConfigJson.hpp"
#include "navkit/core/estimation/state/State.hpp"
#include "navkit/core/estimation/state/StateDefPolicy.hpp"

#include <cstddef>
#include <nlohmann/json.hpp>
#include <string>
#include <string_view>
#include <vector>

namespace navkit::app_support::detail
{

template<typename Segment>
[[nodiscard]] consteval int segment_end()
{
    return Segment::i + Segment::sz;
}

[[nodiscard]] consteval int max_segment_end(const int current_end, const int candidate_end)
{
    return (candidate_end > current_end) ? candidate_end : current_end;
}

template<typename Nominal>
[[nodiscard]] consteval int nominal_pva_value_count()
{
    int pva_end = 0;
    if constexpr (requires { typename Nominal::Pos; }) {
        pva_end = max_segment_end(pva_end, segment_end<typename Nominal::Pos>());
    }
    if constexpr (requires { typename Nominal::Vel; }) {
        pva_end = max_segment_end(pva_end, segment_end<typename Nominal::Vel>());
    }
    if constexpr (requires { typename Nominal::AttQuat; }) {
        pva_end = max_segment_end(pva_end, segment_end<typename Nominal::AttQuat>());
    }
    if constexpr (requires { typename Nominal::AttRpy; }) {
        pva_end = max_segment_end(pva_end, segment_end<typename Nominal::AttRpy>());
    }
    if constexpr (requires { typename Nominal::AttDcm; }) {
        pva_end = max_segment_end(pva_end, segment_end<typename Nominal::AttDcm>());
    }
    if constexpr (requires { typename Nominal::AttRotVec; }) {
        pva_end = max_segment_end(pva_end, segment_end<typename Nominal::AttRotVec>());
    }
    return pva_end;
}

inline void
reject_unknown_filter_nominal_state_keys(const nlohmann::json& nominal_state,
                                         const std::vector<std::string_view>& allowed_keys)
{
    for (nlohmann::json::const_iterator iter = nominal_state.begin(); iter != nominal_state.end();
         ++iter) {
        const std::string& key = iter.key();
        if (!contains_key(allowed_keys, key)) {
            throw_runtime_config_error("unknown key '" + key +
                                       "' in 'filter_initialization.nominal_state'");
        }
    }
}

template<navkit::core::estimation::StateSpaceDefPolicy StateDef>
inline void validate_runtime_nominal_state_override_shape(const nlohmann::json& cfg)
{
    const nlohmann::json::const_iterator filter_initialization_iter =
        cfg.find("filter_initialization");
    if (filter_initialization_iter == cfg.end() || !filter_initialization_iter->is_object()) {
        return;
    }

    const nlohmann::json::const_iterator nominal_state_iter =
        filter_initialization_iter->find("nominal_state");
    if (nominal_state_iter == filter_initialization_iter->end()) {
        return;
    }
    if (!nominal_state_iter->is_object()) {
        throw_runtime_config_error(
            "expected 'filter_initialization.nominal_state' to be an object");
    }

    reject_unknown_filter_nominal_state_keys(*nominal_state_iter, {"non_pva_values"});

    if (nominal_state_iter->contains("non_pva_values")) {
        using Nominal = typename StateDef::Nominal;
        static constexpr int pva_count = nominal_pva_value_count<Nominal>();
        static_assert(pva_count <= Nominal::N);
        static constexpr std::size_t non_pva_count =
            static_cast<std::size_t>(Nominal::N - pva_count);
        require_numeric_array(*nominal_state_iter, "non_pva_values", non_pva_count);
    }
}

template<navkit::core::estimation::StateSpaceDefPolicy StateDef>
inline void
apply_runtime_nominal_state_override(const nlohmann::json& cfg,
                                     navkit::core::estimation::NominalState<StateDef>& state)
{
    const nlohmann::json::const_iterator filter_initialization_iter =
        cfg.find("filter_initialization");
    if (filter_initialization_iter == cfg.end() || !filter_initialization_iter->is_object()) {
        return;
    }

    const nlohmann::json::const_iterator nominal_state_iter =
        filter_initialization_iter->find("nominal_state");
    if (nominal_state_iter == filter_initialization_iter->end()) {
        return;
    }

    validate_runtime_nominal_state_override_shape<StateDef>(cfg);

    using Nominal = typename StateDef::Nominal;
    if (nominal_state_iter->contains("non_pva_values")) {
        static constexpr int pva_count = nominal_pva_value_count<Nominal>();
        const nlohmann::json& values = nominal_state_iter->at("non_pva_values");
        for (int state_index = pva_count; state_index < Nominal::N; ++state_index) {
            const std::size_t value_index = static_cast<std::size_t>(state_index - pva_count);
            state(state_index) = values.at(value_index).template get<navkit::core::Scalar_t>();
        }
    }
}

} // namespace navkit::app_support::detail
