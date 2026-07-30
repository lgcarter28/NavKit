// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/Types.hpp"

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <random>

namespace navkit::sim
{

/**
 * Deterministically derives an independent random stream seed from a component
 * seed and a stable stream index.
 *
 * This keeps one user-facing/Monte-Carlo component seed while preventing
 * separately owned simulator instances from replaying the same random stream.
 */
[[nodiscard]] inline std::uint32_t derive_random_stream_seed(const std::uint32_t component_seed,
                                                             const std::uint32_t stream_index)
{
    std::seed_seq sequence{component_seed, stream_index};
    std::array<std::uint32_t, 1> derived_seed{};
    sequence.generate(derived_seed.begin(), derived_seed.end());
    return derived_seed.front();
}

template<int N, typename Rng>
[[nodiscard]] Eigen::Matrix<navkit::core::Scalar_t, N, 1>
draw_normal_diag_cov(const Eigen::Matrix<navkit::core::Scalar_t, N, 1>& covariance_diag, Rng& rng)
{
    std::normal_distribution<navkit::core::Scalar_t> unit_normal{0.0, 1.0};
    Eigen::Matrix<navkit::core::Scalar_t, N, 1> draw =
        Eigen::Matrix<navkit::core::Scalar_t, N, 1>::Zero();
    for (Eigen::Index axis = 0; axis < draw.size(); ++axis) {
        const auto variance = std::max<navkit::core::Scalar_t>(covariance_diag(axis), 0.0);
        if (variance > 0.0) {
            draw(axis) = std::sqrt(variance) * unit_normal(rng);
        }
    }
    return draw;
}

template<int N, typename Rng>
[[nodiscard]] Eigen::Matrix<navkit::core::Scalar_t, N, 1>
draw_normal_cov(const Eigen::Matrix<navkit::core::Scalar_t, N, N>& covariance, Rng& rng)
{
    using Scalar = navkit::core::Scalar_t;
    std::normal_distribution<Scalar> unit_normal{0.0, 1.0};
    Eigen::Matrix<Scalar, N, 1> unit_draw = Eigen::Matrix<Scalar, N, 1>::Zero();
    for (Eigen::Index axis = 0; axis < unit_draw.size(); ++axis) {
        unit_draw(axis) = unit_normal(rng);
    }

    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<Scalar, N, N>> eigensolver(covariance);
    if (eigensolver.info() != Eigen::Success) {
        return Eigen::Matrix<Scalar, N, 1>::Zero();
    }

    Eigen::Matrix<Scalar, N, 1> sqrt_eigenvalues = Eigen::Matrix<Scalar, N, 1>::Zero();
    for (Eigen::Index axis = 0; axis < sqrt_eigenvalues.size(); ++axis) {
        sqrt_eigenvalues(axis) = std::sqrt(std::max<Scalar>(eigensolver.eigenvalues()(axis), 0.0));
    }
    return eigensolver.eigenvectors() * sqrt_eigenvalues.asDiagonal() * unit_draw;
}

} // namespace navkit::sim
