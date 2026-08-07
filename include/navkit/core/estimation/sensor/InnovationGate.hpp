// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/Types.hpp"
#include "navkit/core/math/ChiSquare.hpp"

#include <cmath>
#include <cstdint>
#include <limits>

namespace navkit::core::estimation
{

/** Runtime chi-square innovation gate with compile-time measurement dimension. */
template<int DegreesOfFreedom>
class InnovationGate
{
public:
    static_assert(DegreesOfFreedom > 0, "Innovation-gate DOF must be positive");

    static constexpr std::uint32_t dof = static_cast<std::uint32_t>(DegreesOfFreedom);

    /** Enables the gate and derives its threshold from the requested probability. */
    [[nodiscard]] bool configure_probability(const Scalar_t probability)
    {
        if (!std::isfinite(probability) || probability <= 0.0 || probability >= 1.0) {
            return false;
        }
        Scalar_t threshold = 0.0;
        if (!navkit::core::math::chi_square_quantile(probability, dof, threshold)) {
            return false;
        }
        m_probability = probability;
        m_threshold = threshold;
        m_enabled = true;
        return true;
    }

    void disable()
    {
        m_enabled = false;
        m_probability = 1.0;
        m_threshold = std::numeric_limits<Scalar_t>::infinity();
    }

    [[nodiscard]] bool accepts(const Scalar_t nis) const
    {
        return std::isfinite(nis) && nis >= 0.0 && (!m_enabled || nis <= m_threshold);
    }

    [[nodiscard]] bool enabled() const
    {
        return m_enabled;
    }

    [[nodiscard]] Scalar_t probability() const
    {
        return m_probability;
    }

    [[nodiscard]] Scalar_t threshold() const
    {
        return m_threshold;
    }

private:
    Scalar_t m_probability{1.0};
    Scalar_t m_threshold{std::numeric_limits<Scalar_t>::infinity()};
    bool m_enabled{false};
};

} // namespace navkit::core::estimation
