// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <cmath>
#include <limits>

namespace navkit::core::estimation
{

template<typename Scalar>
struct CovarianceHealthTolerances
{
    Scalar symmetry_abs{Scalar{1.0e-10}};
    Scalar positive_semidefinite_abs{Scalar{1.0e-12}};
};

template<typename Scalar>
struct CovarianceHealthResult
{
    bool finite{false};
    bool symmetric{false};
    bool nonnegative_diagonal{false};
    bool positive_semidefinite{false};
    Scalar maximum_symmetry_error{std::numeric_limits<Scalar>::infinity()};
    Scalar minimum_diagonal{std::numeric_limits<Scalar>::quiet_NaN()};
    Scalar minimum_eigenvalue{std::numeric_limits<Scalar>::quiet_NaN()};

    [[nodiscard]] bool healthy() const
    {
        return finite && symmetric && nonnegative_diagonal && positive_semidefinite;
    }
};

template<typename Derived>
    requires(Derived::RowsAtCompileTime > 0 &&
             Derived::ColsAtCompileTime == Derived::RowsAtCompileTime)
[[nodiscard]] inline CovarianceHealthResult<typename Derived::Scalar> evaluate_covariance_health(
    const Eigen::MatrixBase<Derived>& covariance,
    const CovarianceHealthTolerances<typename Derived::Scalar>& tolerances = {})
{
    using Scalar = typename Derived::Scalar;
    constexpr int dimension = Derived::RowsAtCompileTime;
    using Covariance = Eigen::Matrix<Scalar, dimension, dimension>;

    CovarianceHealthResult<Scalar> result{};
    result.finite = covariance.allFinite();
    if (!result.finite) {
        return result;
    }

    if (!std::isfinite(tolerances.symmetry_abs) ||
        !std::isfinite(tolerances.positive_semidefinite_abs) ||
        tolerances.symmetry_abs < Scalar{0} || tolerances.positive_semidefinite_abs < Scalar{0}) {
        return result;
    }

    const Covariance evaluated_covariance = covariance;
    result.maximum_symmetry_error =
        (evaluated_covariance - evaluated_covariance.transpose()).cwiseAbs().maxCoeff();
    result.symmetric = result.maximum_symmetry_error <= tolerances.symmetry_abs;
    result.minimum_diagonal = evaluated_covariance.diagonal().minCoeff();
    result.nonnegative_diagonal = result.minimum_diagonal >= -tolerances.positive_semidefinite_abs;

    const Covariance symmetric_covariance =
        Scalar{0.5} * (evaluated_covariance + evaluated_covariance.transpose());
    const Eigen::SelfAdjointEigenSolver<Covariance> solver(symmetric_covariance,
                                                           Eigen::EigenvaluesOnly);
    if (solver.info() == Eigen::Success) {
        result.minimum_eigenvalue = solver.eigenvalues().minCoeff();
        result.positive_semidefinite =
            result.symmetric && result.minimum_eigenvalue >= -tolerances.positive_semidefinite_abs;
    }

    return result;
}

} // namespace navkit::core::estimation
