// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/common/Config.hpp"

#include <cmath>

namespace navkit::core::environment
{

using navkit::core::Scalar_t;

template<typename Derived>
struct PlanetPolicyBase
{
    [[nodiscard]] static constexpr Scalar_t flattening()
    {
        if constexpr (requires { Derived::f; }) {
            return Derived::f;
        }
        else {
            return (Derived::a_m - Derived::b_m) / Derived::a_m;
        }
    }

    [[nodiscard]] static constexpr Scalar_t eccentricity_squared()
    {
        if constexpr (requires { Derived::e2; }) {
            return Derived::e2;
        }
        else {
            return 1.0 - (Derived::b_m * Derived::b_m) / (Derived::a_m * Derived::a_m);
        }
    }

    [[nodiscard]] static Scalar_t eccentricity()
    {
        return std::sqrt(eccentricity_squared());
    }

    [[nodiscard]] static constexpr Scalar_t semi_major_axis_m()
    {
        return Derived::a_m;
    }

    [[nodiscard]] static constexpr Scalar_t semi_minor_axis_m()
    {
        return Derived::b_m;
    }

    [[nodiscard]] static constexpr Scalar_t gravitational_parameter_m3_s2()
    {
        return Derived::mu_m3_s2;
    }

    [[nodiscard]] static constexpr Scalar_t rotation_rate_rad_s()
    {
        return Derived::omega_rad_s;
    }
};

} // namespace navkit::core::environment
