// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/api/config/ConfigApi.hpp"

namespace navkit::config::navkit::products::components
{

struct DefaultInsCovarianceFloor
{
    using StateDef = core::estimation::DefaultInsStateDef;
    using CovarianceFloor_t = core::estimation::CovarianceFloor<StateDef>;

    inline static const CovarianceFloor_t covariance_floor =
        core::estimation::diagonal_covariance_floor<StateDef>(
            core::estimation::CovarianceFloorDiagonal<StateDef>{
                .values =
                    {
                        // Pos
                        0.0,
                        0.0,
                        0.0,
                        // Vel
                        0.0,
                        0.0,
                        0.0,
                        // AttRotVec
                        0.0,
                        0.0,
                        0.0,
                        // GyroB
                        0.0,
                        0.0,
                        0.0,
                        // AccB
                        0.0,
                        0.0,
                        0.0,
                    },
            });
};

static_assert(core::estimation::CovarianceFloorConfigPolicy<DefaultInsCovarianceFloor,
                                                            core::estimation::DefaultInsStateDef>);

} // namespace navkit::config::navkit::products::components
