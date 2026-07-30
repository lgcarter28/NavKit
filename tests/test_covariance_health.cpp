// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/core/estimation/filter/CovarianceHealth.hpp"
#include "navkit/core/estimation/filter/KalmanFilter.hpp"
#include "navkit/core/estimation/state/Segment.hpp"
#include "navkit/core/estimation/state/StateDefs.hpp"

#include <Eigen/Dense>
#include <doctest/doctest.h>
#include <limits>

namespace navkit::core::estimation::test
{

TEST_CASE("Covariance health distinguishes finite symmetry diagonal and PSD failures")
{
    const Eigen::Matrix3d healthy_covariance =
        (Eigen::Vector3d{1.0, 2.0, 3.0} * Eigen::Vector3d{1.0, 2.0, 3.0}.transpose()) +
        (0.25 * Eigen::Matrix3d::Identity());
    const CovarianceHealthResult<double> healthy = evaluate_covariance_health(healthy_covariance);

    CHECK(healthy.healthy());
    CHECK(healthy.maximum_symmetry_error == doctest::Approx(0.0));
    CHECK(healthy.minimum_diagonal > 0.0);
    CHECK(healthy.minimum_eigenvalue > 0.0);

    Eigen::Matrix3d asymmetric = healthy_covariance;
    asymmetric(0, 1) += 1.0e-4;
    const CovarianceHealthResult<double> asymmetric_health = evaluate_covariance_health(asymmetric);
    CHECK(asymmetric_health.finite);
    CHECK_FALSE(asymmetric_health.symmetric);
    CHECK_FALSE(asymmetric_health.positive_semidefinite);
    CHECK_FALSE(asymmetric_health.healthy());

    Eigen::Matrix3d indefinite = Eigen::Matrix3d::Identity();
    indefinite(0, 1) = 2.0;
    indefinite(1, 0) = 2.0;
    const CovarianceHealthResult<double> indefinite_health = evaluate_covariance_health(indefinite);
    CHECK(indefinite_health.symmetric);
    CHECK(indefinite_health.nonnegative_diagonal);
    CHECK_FALSE(indefinite_health.positive_semidefinite);
    CHECK(indefinite_health.minimum_eigenvalue == doctest::Approx(-1.0));

    Eigen::Matrix3d nonfinite = Eigen::Matrix3d::Identity();
    nonfinite(2, 2) = std::numeric_limits<double>::quiet_NaN();
    const CovarianceHealthResult<double> nonfinite_health = evaluate_covariance_health(nonfinite);
    CHECK_FALSE(nonfinite_health.finite);
    CHECK_FALSE(nonfinite_health.healthy());
}

TEST_CASE("Covariance health applies explicit symmetry and PSD tolerances")
{
    Eigen::Matrix2d covariance = Eigen::Matrix2d::Identity();
    covariance(0, 1) = 5.0e-11;
    covariance(1, 0) = 0.0;
    covariance(1, 1) = -5.0e-13;

    const CovarianceHealthTolerances<double> permissive{
        .symmetry_abs = 1.0e-10,
        .positive_semidefinite_abs = 1.0e-12,
    };
    const CovarianceHealthResult<double> permissive_health =
        evaluate_covariance_health(covariance, permissive);
    CHECK(permissive_health.healthy());

    const CovarianceHealthTolerances<double> strict{
        .symmetry_abs = 1.0e-12,
        .positive_semidefinite_abs = 1.0e-14,
    };
    const CovarianceHealthResult<double> strict_health =
        evaluate_covariance_health(covariance, strict);
    CHECK_FALSE(strict_health.symmetric);
    CHECK_FALSE(strict_health.nonnegative_diagonal);
    CHECK_FALSE(strict_health.positive_semidefinite);
    CHECK_FALSE(strict_health.healthy());
}

TEST_CASE("Filter-owned covariance health remains valid after attitude covariance reset")
{
    using StateDef = InsGyroAccelBiasStateDef;
    using Error = StateDef::Error;
    using Filter = KalmanFilter<StateDef>;

    Filter filter{};
    ErrorStateCov<StateDef> factor = ErrorStateCov<StateDef>::Identity();
    factor.template block<3, 3>(Error::Pos::i, Error::AttRotVec::i) =
        0.1 * Eigen::Matrix3d::Identity();
    factor.template block<3, 3>(Error::Vel::i, Error::GyroB::i) =
        -0.05 * Eigen::Matrix3d::Identity();
    const ErrorStateCov<StateDef> covariance =
        (factor * factor.transpose()) + (0.25 * ErrorStateCov<StateDef>::Identity());
    filter.set_covariance(covariance);
    segment<Error::AttRotVec>(filter.error_state()) << 0.02, -0.03, 0.04;

    filter.reset();

    const CovarianceHealthResult<double> reset_health = filter.covariance_health();
    CHECK(reset_health.healthy());
    CHECK(reset_health.maximum_symmetry_error < 1.0e-14);
    CHECK(reset_health.minimum_eigenvalue > 0.0);
    CHECK(filter.error_state().isZero());
}

} // namespace navkit::core::estimation::test
