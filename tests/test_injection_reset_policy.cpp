// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/core/estimation/filter/KalmanFilter.hpp"
#include "navkit/core/estimation/filter/injection/InjectionPolicy.hpp"
#include "navkit/core/estimation/filter/reset/ResetPolicies.hpp"
#include "navkit/core/estimation/filter/reset/ResetPolicy.hpp"
#include "navkit/core/estimation/state/Segment.hpp"
#include "navkit/core/estimation/state/StateDefs.hpp"
#include "navkit/core/math/Quaternion.hpp"
#include "navkit/core/math/Skew.hpp"

#include <Eigen/Geometry>
#include <doctest/doctest.h>
#include <type_traits>

namespace navkit::core::estimation::test
{

struct PolicyTestNominalStateDef
{
    using Scalar_t = double;
    static constexpr int N = 3;
};

struct PolicyTestErrorStateDef
{
    using Scalar_t = double;
    static constexpr int N = 3;
};

struct PolicyTestStateDef
{
    using Nominal = PolicyTestNominalStateDef;
    using Error = PolicyTestErrorStateDef;
};

struct ValidInjection
{
    static void apply(NominalState<PolicyTestStateDef>& x, const ErrorState<PolicyTestStateDef>& dx)
    {
        x += dx;
    }
};

struct MissingInjectionApply
{};

struct MutableErrorInjection
{
    static void apply(NominalState<PolicyTestStateDef>& unused_state,
                      ErrorState<PolicyTestStateDef>& unused_dx)
    {
        static_cast<void>(unused_state);
        static_cast<void>(unused_dx);
    }
};

struct ValidReset
{
    static void reset_covariance(NominalState<PolicyTestStateDef>& unused_state,
                                 ErrorState<PolicyTestStateDef>& unused_dx,
                                 ErrorStateCov<PolicyTestStateDef>& unused_covariance)
    {
        static_cast<void>(unused_state);
        static_cast<void>(unused_dx);
        static_cast<void>(unused_covariance);
    }

    static void reset_dx(ErrorState<PolicyTestStateDef>& dx)
    {
        dx.setZero();
    }
};

struct MissingCovarianceReset
{
    static void reset_dx(ErrorState<PolicyTestStateDef>& dx)
    {
        dx.setZero();
    }
};

struct MissingErrorStateReset
{
    static void reset_covariance(NominalState<PolicyTestStateDef>& unused_state,
                                 ErrorState<PolicyTestStateDef>& unused_dx,
                                 ErrorStateCov<PolicyTestStateDef>& unused_covariance)
    {
        static_cast<void>(unused_state);
        static_cast<void>(unused_dx);
        static_cast<void>(unused_covariance);
    }
};

TEST_CASE("InjectionPolicy accepts compatible policies")
{
    static_assert(InjectionPolicy<ValidInjection, PolicyTestStateDef>);
    static_assert(InjectionPolicy<DefaultInjectionPolicy<InsGyroAccelBiasStateDef>,
                                  InsGyroAccelBiasStateDef>);

    CHECK(true);
}

TEST_CASE("InjectionPolicy rejects incompatible policies")
{
    static_assert(!InjectionPolicy<MissingInjectionApply, PolicyTestStateDef>);
    static_assert(!InjectionPolicy<MutableErrorInjection, PolicyTestStateDef>);

    CHECK(true);
}

TEST_CASE("ResetPolicy accepts compatible policies")
{
    static_assert(ResetPolicy<ValidReset, PolicyTestStateDef>);
    static_assert(
        ResetPolicy<DefaultResetPolicy<InsGyroAccelBiasStateDef>, InsGyroAccelBiasStateDef>);

    CHECK(true);
}

TEST_CASE("ResetPolicy rejects incomplete policies")
{
    static_assert(!ResetPolicy<MissingCovarianceReset, PolicyTestStateDef>);
    static_assert(!ResetPolicy<MissingErrorStateReset, PolicyTestStateDef>);

    CHECK(true);
}

TEST_CASE("KalmanFilter accepts constrained injection and reset policies")
{
    using Filter = KalmanFilter<PolicyTestStateDef, ValidInjection, ValidReset>;
    static_assert(std::is_default_constructible_v<Filter>);

    Filter filter;
    CHECK(filter.state().isZero());
}

TEST_CASE("Default INS injection follows truth-minus-estimate additive convention")
{
    using StateDef = InsGyroAccelBiasStateDef;
    using Nominal = StateDef::Nominal;
    using Error = StateDef::Error;

    NominalState<StateDef> x = NominalState<StateDef>::Zero();
    ErrorState<StateDef> dx = ErrorState<StateDef>::Zero();

    segment<Nominal::Pos>(x) << 10.0, -5.0, 2.0;
    segment<Nominal::Vel>(x) << 1.0, 2.0, 3.0;
    segment<Nominal::GyroB>(x) << 0.1, 0.2, 0.3;
    segment<Nominal::AccB>(x) << -0.1, -0.2, -0.3;
    segment<Nominal::AttQuat>(x) << 1.0, 0.0, 0.0, 0.0;

    segment<Error::Pos>(dx) << -1.0, 2.0, -3.0;
    segment<Error::Vel>(dx) << 0.5, -0.25, 0.125;
    segment<Error::GyroB>(dx) << 0.01, 0.02, 0.03;
    segment<Error::AccB>(dx) << -0.01, -0.02, -0.03;

    DefaultInjectionPolicy<StateDef>::apply(x, dx);

    CHECK(segment<Nominal::Pos>(x).isApprox(Eigen::Vector3d{9.0, -3.0, -1.0}));
    CHECK(segment<Nominal::Vel>(x).isApprox(Eigen::Vector3d{1.5, 1.75, 3.125}));
    CHECK(segment<Nominal::GyroB>(x).isApprox(Eigen::Vector3d{0.11, 0.22, 0.33}));
    CHECK(segment<Nominal::AccB>(x).isApprox(Eigen::Vector3d{-0.11, -0.22, -0.33}));
}

TEST_CASE("Default INS injection maps ECEF attitude-error correction to stored q_b2e")
{
    using StateDef = InsGyroAccelBiasStateDef;
    using Nominal = StateDef::Nominal;
    using Error = StateDef::Error;

    NominalState<StateDef> x = NominalState<StateDef>::Zero();
    ErrorState<StateDef> dx = ErrorState<StateDef>::Zero();

    const Eigen::Quaterniond q_b2e =
        navkit::core::math::quaternion_from_rpy_rad(Eigen::Vector3d{0.1, -0.2, 0.3});
    segment<Nominal::AttQuat>(x) << q_b2e.w(), q_b2e.x(), q_b2e.y(), q_b2e.z();

    const Eigen::Vector3d correction{1.0e-3, -2.0e-3, 3.0e-3};
    segment<Error::AttRotVec>(dx) = correction;

    DefaultInjectionPolicy<StateDef>::apply(x, dx);

    const Eigen::Quaterniond delta_q = navkit::core::math::quaternion_from_rotvec_rad(correction);
    const Eigen::Quaterniond expected_q_b2e =
        navkit::core::math::normalized_with_positive_scalar(delta_q * q_b2e);

    const Eigen::Matrix<double, 4, 1> q_segment = segment<Nominal::AttQuat>(x);
    const Eigen::Quaterniond actual_q_b2e{q_segment(0), q_segment(1), q_segment(2), q_segment(3)};

    CHECK(actual_q_b2e.coeffs().isApprox(expected_q_b2e.coeffs(), 1.0e-12));
}

TEST_CASE("Default reset maps covariance through the left-global attitude reset Jacobian")
{
    using StateDef = InsGyroAccelBiasStateDef;
    using Error = StateDef::Error;

    NominalState<StateDef> x = NominalState<StateDef>::Zero();
    ErrorState<StateDef> dx = ErrorState<StateDef>::Zero();
    ErrorStateCov<StateDef> covariance = ErrorStateCov<StateDef>::Identity();
    covariance.template block<3, 3>(Error::Pos::i, Error::AttRotVec::i) =
        0.25 * Eigen::Matrix3d::Identity();
    covariance.template block<3, 3>(Error::AttRotVec::i, Error::Pos::i) =
        0.25 * Eigen::Matrix3d::Identity();
    covariance.template block<3, 3>(Error::AttRotVec::i, Error::AttRotVec::i) << 2.0, 0.1, -0.2,
        0.1, 3.0, 0.3, -0.2, 0.3, 4.0;

    const Eigen::Vector3d correction_rad{0.02, -0.03, 0.04};
    segment<Error::AttRotVec>(dx) = correction_rad;

    ErrorStateCov<StateDef> expected_reset = ErrorStateCov<StateDef>::Identity();
    expected_reset.template block<3, 3>(Error::AttRotVec::i, Error::AttRotVec::i) =
        Eigen::Matrix3d::Identity() + (0.5 * navkit::core::math::skew_symmetric(correction_rad));
    const ErrorStateCov<StateDef> expected_covariance =
        expected_reset * covariance * expected_reset.transpose();

    DefaultResetPolicy<StateDef>::reset_covariance(x, dx, covariance);

    CHECK(covariance.isApprox(expected_covariance, 1.0e-14));
    CHECK(covariance.isApprox(covariance.transpose(), 1.0e-14));
}

TEST_CASE("Left-global attitude reset Jacobian sign matches quaternion composition")
{
    const Eigen::Vector3d correction_rad{1.0e-4, -2.0e-4, 3.0e-4};
    const Eigen::Quaterniond correction_q =
        navkit::core::math::quaternion_from_rotvec_rad(correction_rad);
    constexpr double perturbation_rad = 1.0e-7;

    Eigen::Matrix3d numerical_jacobian{};
    for (Eigen::Index axis = 0; axis < 3; ++axis) {
        Eigen::Vector3d perturbation = Eigen::Vector3d::Zero();
        perturbation(axis) = perturbation_rad;

        const Eigen::Quaterniond true_plus_q =
            navkit::core::math::quaternion_from_rotvec_rad(correction_rad + perturbation);
        const Eigen::Quaterniond true_minus_q =
            navkit::core::math::quaternion_from_rotvec_rad(correction_rad - perturbation);
        const Eigen::Vector3d reset_plus_rad =
            navkit::core::math::rotvec_rad_from_quaternion(true_plus_q * correction_q.conjugate());
        const Eigen::Vector3d reset_minus_rad =
            navkit::core::math::rotvec_rad_from_quaternion(true_minus_q * correction_q.conjugate());
        numerical_jacobian.col(axis) =
            (reset_plus_rad - reset_minus_rad) / (2.0 * perturbation_rad);
    }

    const Eigen::Matrix3d expected_first_order =
        Eigen::Matrix3d::Identity() + (0.5 * navkit::core::math::skew_symmetric(correction_rad));
    CHECK(numerical_jacobian.isApprox(expected_first_order, 1.0e-7));
    CHECK(numerical_jacobian(0, 1) == doctest::Approx(-0.5 * correction_rad.z()).epsilon(1.0e-3));
}

TEST_CASE("Default reset leaves covariance unchanged when no attitude error segment exists")
{
    NominalState<PolicyTestStateDef> x = NominalState<PolicyTestStateDef>::Zero();
    ErrorState<PolicyTestStateDef> dx = ErrorState<PolicyTestStateDef>::Ones();
    ErrorStateCov<PolicyTestStateDef> covariance{};
    covariance << 2.0, 0.1, 0.2, 0.1, 3.0, 0.3, 0.2, 0.3, 4.0;
    const ErrorStateCov<PolicyTestStateDef> expected = covariance;

    DefaultResetPolicy<PolicyTestStateDef>::reset_covariance(x, dx, covariance);

    CHECK(covariance.isApprox(expected));
}

} // namespace navkit::core::estimation::test
