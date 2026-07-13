// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/estimation/state/State.hpp"

#include <concepts>

namespace navkit::core::estimation
{

template<typename Candidate, typename StateDef>
concept MeasurementModelPolicy =
    StateDefPolicy<StateDef> &&
    requires(const NominalState<StateDef>& x, const typename Candidate::NoiseContext& ctx) {
        { Candidate::M } -> std::convertible_to<int>;
        typename Candidate::O_t;
        typename Candidate::R_t;
        typename Candidate::H_t;
        typename Candidate::K_t;
        typename Candidate::NoiseContext;

        { Candidate::obs(x) } -> std::same_as<typename Candidate::O_t>;
        { Candidate::compute_h(x) } -> std::same_as<typename Candidate::H_t>;
        { Candidate::compute_r(ctx) } -> std::same_as<typename Candidate::R_t>;
    } && requires {
        requires Candidate::M > 0;
        requires Candidate::O_t::RowsAtCompileTime == Candidate::M;
        requires Candidate::O_t::ColsAtCompileTime == 1;
        requires Candidate::R_t::RowsAtCompileTime == Candidate::M;
        requires Candidate::R_t::ColsAtCompileTime == Candidate::M;
        requires Candidate::H_t::RowsAtCompileTime == Candidate::M;
        requires Candidate::H_t::ColsAtCompileTime == StateDef::N;
        requires Candidate::K_t::RowsAtCompileTime == StateDef::N;
        requires Candidate::K_t::ColsAtCompileTime == Candidate::M;
    };

} // namespace navkit::core::estimation
