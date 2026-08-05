// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/api/config/ConfigApi.hpp"

namespace navkit::config::navkit::products::components
{

struct InsGyroAccelBiasInitialCovarianceDefault
{
    using StateDef = core::estimation::InsGyroAccelBiasStateDef;
    using InitialCovariance_t = core::estimation::InitialCovariance<StateDef>;

    inline static const InitialCovariance_t initial_covariance =
        core::estimation::diagonal_initial_covariance<StateDef>(
            core::estimation::InitialCovarianceDiagonal<StateDef>{
                .values =
                    {
                        // Pos
                        10000.0,
                        10000.0,
                        10000.0,
                        // Vel
                        100.0,
                        100.0,
                        100.0,
                        // AttRotVec
                        0.030461741978670857,
                        0.030461741978670857,
                        0.030461741978670857,
                        // GyroB
                        1.0e-9,
                        1.0e-9,
                        1.0e-9,
                        // AccB
                        1.0e-5,
                        1.0e-5,
                        1.0e-5,
                    },
            });
};

} // namespace navkit::config::navkit::products::components
