// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/math/Types.hpp"
#include "navkit/core/time/TimeTypes.hpp"

namespace navkit::sim
{

/** Exact zero-order-hold step for a scalar first-order response; zero tau is bypass. */
[[nodiscard]] core::Scalar_t exact_first_order_step(core::Scalar_t previous,
                                                    core::Scalar_t command,
                                                    core::Time_t time_constant_s,
                                                    core::Time_t dt_s);

/** Per-axis exact zero-order-hold step for a three-channel first-order response. */
[[nodiscard]] core::Vec3 exact_first_order_step(const core::Vec3& previous,
                                                const core::Vec3& command,
                                                const core::Vec3& time_constant_s,
                                                core::Time_t dt_s);

} // namespace navkit::sim
