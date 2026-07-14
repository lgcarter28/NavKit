// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/Types.hpp"
#include "navkit/core/environment/RotatingPlanetKinematics.hpp"
#include "navkit/core/environment/gravity/GravityGradient.hpp"
#include "navkit/core/environment/gravity/GravityPolicy.hpp"
#include "navkit/core/environment/planet/PlanetPolicy.hpp"
#include "navkit/core/estimation/navigator/ImuConingSculling.hpp"
#include "navkit/core/estimation/navigator/ImuIncrement.hpp"
#include "navkit/core/estimation/navigator/propagation/EcefInsProcessNoise.hpp"
#include "navkit/core/estimation/state/Segment.hpp"
#include "navkit/core/estimation/state/State.hpp"
#include "navkit/core/estimation/state/StateDefPolicy.hpp"
#include "navkit/core/math/Quaternion.hpp"
#include "navkit/core/math/Skew.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <cstddef>
#include <type_traits>

namespace navkit::core::estimation
{

struct MechanizedImuInterval
{
    Time_t time_s{0.0};
    Time_t dt_s{0.0};
    Vec3 delta_theta_ib_b_rad{Vec3::Zero()};
    Vec3 delta_v_ib_b_mps{Vec3::Zero()};
    Vec3 delta_theta_eb_b_rad{Vec3::Zero()};
    Vec3 specific_force_ib_b_mps2{Vec3::Zero()};
};

template<environment::RotatingPlanetPolicy Planet,
         environment::GravityPolicy Gravity,
         typename ProcessNoise = EcefInsZeroProcessNoise,
         std::size_t ImuBufferCapacity = 512U,
         std::size_t CovarianceHistoryCapacity = 256U,
         Time_t CovarianceUpdateRateHz = 100.0>
struct EcefInsPropagation
{
    static_assert(std::is_same_v<typename Gravity::Planet_t, Planet>,
                  "EcefInsPropagation Gravity must use the selected Planet.");

    static constexpr std::size_t imu_buffer_capacity =
        ImuBufferCapacity; // NOLINT(readability-identifier-naming)
    static constexpr std::size_t covariance_history_capacity =
        CovarianceHistoryCapacity; // NOLINT(readability-identifier-naming)
    static constexpr Time_t covariance_update_rate_hz = CovarianceUpdateRateHz;

    template<StateSpaceDefPolicy StateDef>
    static bool process_imu_increment(const ImuIncrement& increment, NominalState<StateDef>& state)
    {
        const auto interval = corrected_interval_from_single<StateDef>(state, increment);
        return propagate_nominal_state<StateDef>(interval, state);
    }

    template<StateSpaceDefPolicy StateDef>
    static bool process_imu_increment_pair(const ImuIncrement& first,
                                           const ImuIncrement& second,
                                           NominalState<StateDef>& state)
    {
        const auto interval = corrected_interval_from_pair<StateDef>(state, first, second);
        return propagate_nominal_state<StateDef>(interval, state);
    }

    template<StateSpaceDefPolicy StateDef>
    static bool covariance_step_from_increment(const NominalState<StateDef>& state,
                                               const ImuIncrement& increment,
                                               ErrorStateCov<StateDef>& phi,
                                               ErrorStateCov<StateDef>& qd)
    {
        const auto interval = corrected_interval_from_single<StateDef>(state, increment);
        return covariance_step_from_interval<StateDef>(state, interval, phi, qd);
    }

    template<StateSpaceDefPolicy StateDef>
    static bool covariance_step_from_increment_pair(const NominalState<StateDef>& state,
                                                    const ImuIncrement& first,
                                                    const ImuIncrement& second,
                                                    ErrorStateCov<StateDef>& phi,
                                                    ErrorStateCov<StateDef>& qd)
    {
        const auto interval = corrected_interval_from_pair<StateDef>(state, first, second);
        return covariance_step_from_interval<StateDef>(state, interval, phi, qd);
    }

    template<typename StateDef, typename State_t>
    [[nodiscard]] static Vec3 position_e_m(const State_t& state)
    {
        using Nominal = typename StateDef::Nominal;
        return segment<typename Nominal::Pos>(state);
    }

    template<typename StateDef, typename State_t>
    [[nodiscard]] static Vec3 velocity_e_mps(const State_t& state)
    {
        using Nominal = typename StateDef::Nominal;
        return segment<typename Nominal::Vel>(state);
    }

    template<typename StateDef, typename State_t>
    [[nodiscard]] static Eigen::Quaternion<Scalar_t> quaternion_e2b(const State_t& state)
    {
        using Nominal = typename StateDef::Nominal;
        const auto q_segment = segment<typename Nominal::AttQuat>(state);
        return navkit::core::math::normalized_with_positive_scalar(
            Eigen::Quaternion<Scalar_t>{q_segment(0), q_segment(1), q_segment(2), q_segment(3)});
    }

    template<typename StateDef, typename State_t>
    [[nodiscard]] static Vec3 gyro_bias_radps(const State_t& state)
    {
        using Nominal = typename StateDef::Nominal;
        if constexpr (requires { typename Nominal::GyroB; }) {
            if constexpr (requires { typename Nominal::GyroBias; }) {
                return segment<typename Nominal::GyroBias>(state);
            }
            else {
                return segment<typename Nominal::GyroB>(state);
            }
        }
        else {
            return Vec3::Zero();
        }
    }

    template<typename StateDef, typename State_t>
    [[nodiscard]] static Vec3 accel_bias_mps2(const State_t& state)
    {
        using Nominal = typename StateDef::Nominal;
        if constexpr (requires { typename Nominal::AccB; }) {
            if constexpr (requires { typename Nominal::AccelBias; }) {
                return segment<typename Nominal::AccelBias>(state);
            }
            else {
                return segment<typename Nominal::AccB>(state);
            }
        }
        else {
            return Vec3::Zero();
        }
    }

    template<typename StateDef, typename State_t>
    static void set_position_e_m(State_t& state, const Vec3& value)
    {
        using Nominal = typename StateDef::Nominal;
        segment<typename Nominal::Pos>(state) = value;
    }

    template<typename StateDef, typename State_t>
    static void set_velocity_e_mps(State_t& state, const Vec3& value)
    {
        using Nominal = typename StateDef::Nominal;
        segment<typename Nominal::Vel>(state) = value;
    }

    template<typename StateDef, typename State_t>
    static void set_quaternion_e2b(State_t& state, const Eigen::Quaternion<Scalar_t>& value)
    {
        const auto q = navkit::core::math::normalized_with_positive_scalar(value);
        using Nominal = typename StateDef::Nominal;
        segment<typename Nominal::AttQuat>(state) << q.w(), q.x(), q.y(), q.z();
    }

    template<typename StateDef>
    [[nodiscard]] static ErrorStateCov<StateDef>
    build_f_matrix(const NominalState<StateDef>& state, const MechanizedImuInterval& interval)
    {
        using Error = typename StateDef::Error;
        ErrorStateCov<StateDef> F = ErrorStateCov<StateDef>::Zero();

        const auto q_e2b = quaternion_e2b<StateDef>(state);
        const auto C_b_e = q_e2b.conjugate().toRotationMatrix();
        const auto omega_ie_e = environment::planet_rate_fixed_radps<Planet>();
        const auto Omega_ie_e = navkit::core::math::skew_symmetric(omega_ie_e);
        const auto f_ib_e = C_b_e * interval.specific_force_ib_b_mps2;

        F.template block<3, 3>(Error::Pos::i, Error::Vel::i).setIdentity();
        F.template block<3, 3>(Error::Vel::i, Error::Pos::i) =
            environment::gravity_gradient_fixed_mps2_per_m<Gravity>(position_e_m<StateDef>(state));
        F.template block<3, 3>(Error::Vel::i, Error::Vel::i) = -2.0 * Omega_ie_e;
        F.template block<3, 3>(Error::Vel::i, Error::AttRotVec::i) =
            navkit::core::math::skew_symmetric(f_ib_e);
        if constexpr (requires { typename Error::AccB; }) {
            F.template block<3, 3>(Error::Vel::i, Error::AccB::i) = -C_b_e;
        }
        F.template block<3, 3>(Error::AttRotVec::i, Error::AttRotVec::i) = -Omega_ie_e;
        if constexpr (requires { typename Error::GyroB; }) {
            F.template block<3, 3>(Error::AttRotVec::i, Error::GyroB::i) = C_b_e;
        }

        return F;
    }

    template<typename StateDef>
    [[nodiscard]] static Eigen::Matrix<Scalar_t, StateDef::Error::N, 12>
    build_g_matrix(const NominalState<StateDef>& state)
    {
        using Error = typename StateDef::Error;
        Eigen::Matrix<Scalar_t, Error::N, 12> G = Eigen::Matrix<Scalar_t, Error::N, 12>::Zero();
        const auto q_e2b = quaternion_e2b<StateDef>(state);
        const auto C_b_e = q_e2b.conjugate().toRotationMatrix();

        G.template block<3, 3>(Error::Vel::i, 3) = C_b_e;
        G.template block<3, 3>(Error::AttRotVec::i, 0) = -C_b_e;
        if constexpr (requires { typename Error::GyroB; }) {
            G.template block<3, 3>(Error::GyroB::i, 6).setIdentity();
        }
        if constexpr (requires { typename Error::AccB; }) {
            G.template block<3, 3>(Error::AccB::i, 9).setIdentity();
        }
        return G;
    }

    [[nodiscard]] static Eigen::Matrix<Scalar_t, 12, 12> build_qc_matrix()
    {
        Eigen::Matrix<Scalar_t, 12, 12> Qc = Eigen::Matrix<Scalar_t, 12, 12>::Zero();
        Qc.template block<3, 3>(0, 0) = ProcessNoise::gyro_white_noise_psd_rad2ps().asDiagonal();
        Qc.template block<3, 3>(3, 3) = ProcessNoise::accel_white_noise_psd_m2ps3().asDiagonal();
        Qc.template block<3, 3>(6, 6) = ProcessNoise::gyro_bias_rw_psd_rad2ps3().asDiagonal();
        Qc.template block<3, 3>(9, 9) = ProcessNoise::accel_bias_rw_psd_m2ps5().asDiagonal();
        return Qc;
    }

    template<typename StateDef>
    [[nodiscard]] static ErrorStateCov<StateDef> first_order_phi(const ErrorStateCov<StateDef>& F,
                                                                 const Time_t dt_s)
    {
        return ErrorStateCov<StateDef>::Identity() + (F * dt_s);
    }

    template<typename StateDef>
    [[nodiscard]] static ErrorStateCov<StateDef>
    first_order_qd(const Eigen::Matrix<Scalar_t, StateDef::Error::N, 12>& G, const Time_t dt_s)
    {
        return G * build_qc_matrix() * G.transpose() * dt_s;
    }

private:
    template<StateSpaceDefPolicy StateDef>
    [[nodiscard]] static MechanizedImuInterval
    corrected_interval_from_single(const NominalState<StateDef>& state,
                                   const ImuIncrement& increment)
    {
        const auto delta_theta =
            increment.delta_theta_ib_b_rad - (gyro_bias_radps<StateDef>(state) * increment.dt_s);
        const auto delta_v =
            increment.delta_v_ib_b_mps - (accel_bias_mps2<StateDef>(state) * increment.dt_s);
        const auto corrected = coning_sculling_single(delta_theta, delta_v);
        return mechanized_interval_from_corrected<StateDef>(
            state, increment.time_s, increment.dt_s, corrected);
    }

    template<StateSpaceDefPolicy StateDef>
    [[nodiscard]] static MechanizedImuInterval corrected_interval_from_pair(
        const NominalState<StateDef>& state, const ImuIncrement& first, const ImuIncrement& second)
    {
        if (first.dt_s <= 0.0 || second.dt_s <= 0.0 || second.time_s <= first.time_s) {
            return {};
        }

        const auto gyro_bias = gyro_bias_radps<StateDef>(state);
        const auto accel_bias = accel_bias_mps2<StateDef>(state);
        const auto delta_theta_1 = first.delta_theta_ib_b_rad - (gyro_bias * first.dt_s);
        const auto delta_theta_2 = second.delta_theta_ib_b_rad - (gyro_bias * second.dt_s);
        const auto delta_v_1 = first.delta_v_ib_b_mps - (accel_bias * first.dt_s);
        const auto delta_v_2 = second.delta_v_ib_b_mps - (accel_bias * second.dt_s);
        const auto corrected =
            coning_sculling_two_sample(delta_theta_1, delta_v_1, delta_theta_2, delta_v_2);
        return mechanized_interval_from_corrected<StateDef>(
            state, second.time_s, first.dt_s + second.dt_s, corrected);
    }

    template<StateSpaceDefPolicy StateDef>
    [[nodiscard]] static bool covariance_step_from_interval(const NominalState<StateDef>& state,
                                                            const MechanizedImuInterval& interval,
                                                            ErrorStateCov<StateDef>& phi,
                                                            ErrorStateCov<StateDef>& qd)
    {
        if (interval.dt_s <= 0.0) {
            phi.setIdentity();
            qd.setZero();
            return false;
        }

        const auto F = build_f_matrix<StateDef>(state, interval);
        const auto G = build_g_matrix<StateDef>(state);
        phi = first_order_phi<StateDef>(F, interval.dt_s);
        qd = first_order_qd<StateDef>(G, interval.dt_s);
        return true;
    }

    template<StateSpaceDefPolicy StateDef>
    [[nodiscard]] static bool propagate_nominal_state(const MechanizedImuInterval& interval,
                                                      NominalState<StateDef>& state)
    {
        if (interval.dt_s <= 0.0) {
            return false;
        }

        auto state_after = state;
        propagate_nominal<StateDef>(state, interval, state_after);
        state = state_after;
        return true;
    }

    template<StateSpaceDefPolicy StateDef>
    static void propagate_nominal(const NominalState<StateDef>& state_before,
                                  const MechanizedImuInterval& interval,
                                  NominalState<StateDef>& state_after)
    {
        const auto p_k = position_e_m<StateDef>(state_before);
        const auto v_k = velocity_e_mps<StateDef>(state_before);
        const auto q_e2b_k = quaternion_e2b<StateDef>(state_before);
        const auto q_b2e_k = q_e2b_k.conjugate();
        const auto C_b2e_k = q_b2e_k.toRotationMatrix();

        const auto gravity_e = Gravity::acceleration(p_k);
        const auto omega_ie_e = environment::planet_rate_fixed_radps<Planet>();
        const auto v_next = v_k + (C_b2e_k * interval.delta_v_ib_b_mps) +
                            ((gravity_e - (2.0 * omega_ie_e.cross(v_k))) * interval.dt_s);
        const auto p_next = p_k + (0.5 * (v_k + v_next) * interval.dt_s);

        const auto delta_q_e2b =
            navkit::core::math::quaternion_from_rotvec_rad(interval.delta_theta_eb_b_rad);
        const auto q_e2b_next =
            navkit::core::math::normalized_with_positive_scalar(delta_q_e2b * q_e2b_k);

        set_position_e_m<StateDef>(state_after, p_next);
        set_velocity_e_mps<StateDef>(state_after, v_next);
        set_quaternion_e2b<StateDef>(state_after, q_e2b_next);
    }

    template<StateSpaceDefPolicy StateDef>
    [[nodiscard]] static MechanizedImuInterval
    mechanized_interval_from_corrected(const NominalState<StateDef>& state,
                                       const Time_t time_s,
                                       const Time_t dt_s,
                                       const ConingSculling& corrected)
    {
        if (dt_s <= 0.0) {
            return {};
        }

        const auto q_e2b = quaternion_e2b<StateDef>(state);
        const auto delta_theta_eb_b =
            corrected.delta_theta_ib_b_rad -
            ((q_e2b * environment::planet_rate_fixed_radps<Planet>()) * dt_s);
        return {.time_s = time_s,
                .dt_s = dt_s,
                .delta_theta_ib_b_rad = corrected.delta_theta_ib_b_rad,
                .delta_v_ib_b_mps = corrected.delta_v_ib_b_mps,
                .delta_theta_eb_b_rad = delta_theta_eb_b,
                .specific_force_ib_b_mps2 = corrected.delta_v_ib_b_mps / dt_s};
    }
};

} // namespace navkit::core::estimation
