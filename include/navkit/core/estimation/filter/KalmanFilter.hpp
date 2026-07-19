// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/estimation/filter/CovarianceFloor.hpp"
#include "navkit/core/estimation/filter/MeasurementStatistics.hpp"
#include "navkit/core/estimation/filter/MeasurementStatisticsStorage.hpp"
#include "navkit/core/estimation/filter/injection/InjectionPolicies.hpp"
#include "navkit/core/estimation/filter/injection/InjectionPolicy.hpp"
#include "navkit/core/estimation/filter/reset/ResetPolicies.hpp"
#include "navkit/core/estimation/filter/reset/ResetPolicy.hpp"
#include "navkit/core/estimation/measurement/MeasurementModelPolicy.hpp"
#include "navkit/core/estimation/sensor/SensorTuplePolicy.hpp"
#include "navkit/core/estimation/sensor/SensorTupleTraits.hpp"
#include "navkit/core/estimation/state/Segment.hpp"
#include "navkit/core/estimation/state/State.hpp"
#include "navkit/core/math/Quaternion.hpp"
#include "navkit/core/profiling/NullProfiler.hpp"
#include "navkit/core/profiling/ProfilePoint.hpp"
#include "navkit/core/profiling/ProfilerPolicy.hpp"

#include <Eigen/Dense>
#include <tuple>
#include <type_traits>

namespace navkit::core::estimation
{

template<StateSpaceDefPolicy StateDef,
         InjectionPolicy<StateDef> Injection = DefaultInjectionPolicy<StateDef>,
         ResetPolicy<StateDef> Reset = DefaultResetPolicy<StateDef>,
         SensorTuplePolicy Sensors = std::tuple<>,
         navkit::core::profiling::ProfilerPolicy Profiler = navkit::core::profiling::NullProfiler>
class KalmanFilter
{
public:
    using StateDef_t = StateDef;
    using Nominal = typename StateDef::Nominal;
    using Error = typename StateDef::Error;
    using Sensors_t = Sensors;
    using State_t = NominalState<StateDef>;
    using ErrorState_t = ErrorState<StateDef>;
    using P_t = ErrorStateCov<StateDef>;
    using MeasurementStatisticsTuple_t = MeasurementStatisticsStorage_t<Sensors>;
    using Profiler_t = Profiler;

    KalmanFilter()
    {
        m_x.setZero();
        m_dx.setZero();
        m_P.setIdentity();
        normalize_nominal_attitude(m_x);
    }

    static const P_t& I()
    {
        static const P_t identity = P_t::Identity();
        return identity;
    }

    State_t& state()
    {
        return m_x;
    }

    [[nodiscard]] const State_t& state() const
    {
        return m_x;
    }

    ErrorState_t& error_state()
    {
        return m_dx;
    }

    [[nodiscard]] const ErrorState_t& error_state() const
    {
        return m_dx;
    }

    [[nodiscard]] const ErrorState_t& last_correction() const
    {
        return m_last_correction;
    }

    [[nodiscard]] bool last_correction_valid() const
    {
        return m_last_correction_valid;
    }

    P_t& covariance()
    {
        return m_P;
    }

    [[nodiscard]] const P_t& covariance() const
    {
        return m_P;
    }

    void set_state(const State_t& x)
    {
        m_x = x;
        normalize_nominal_attitude(m_x);
    }

    void set_covariance(const P_t& P)
    {
        m_P = P;
        apply_covariance_floor();
    }

    void set_covariance_floor(const P_t& floor)
    {
        m_covariance_floor = floor;
        apply_covariance_floor();
    }

    [[nodiscard]] const P_t& covariance_floor() const
    {
        return m_covariance_floor;
    }

    void propagate_covariance(const P_t& phi, const P_t& qd)
    {
        m_P = (phi * m_P * phi.transpose()) + qd;
        m_P = 0.5 * (m_P + m_P.transpose());
        apply_covariance_floor();
    }

    void apply_covariance_floor()
    {
        apply_diagonal_covariance_floor<StateDef>(m_P, m_covariance_floor);
    }

    template<SensorPolicy Sensor>
        requires MeasurementModelPolicy<typename Sensor::MeasurementModel_t, StateDef>
    [[nodiscard]] bool measurement_statistics_available() const
    {
        if constexpr (sensor_id_exists_v<Sensor::Id, Sensors_t> &&
                      Sensor::Diagnostics_t::enable_measurement_statistics) {
            return measurement_statistics<Sensor>().valid;
        }
        else {
            return false;
        }
    }

    template<SensorPolicy Sensor>
        requires MeasurementModelPolicy<typename Sensor::MeasurementModel_t, StateDef>
    MeasurementStatistics<Sensor>& measurement_statistics()
    {
        static_assert(
            sensor_id_exists_v<Sensor::Id, Sensors_t>,
            "Requested Sensor does not have a MeasurementStatistics entry in this KalmanFilter.");
        return std::get<SensorIndexFromId_v<Sensor::Id, Sensors_t>>(m_measurement_stats);
    }

    template<SensorPolicy Sensor>
        requires MeasurementModelPolicy<typename Sensor::MeasurementModel_t, StateDef>
    [[nodiscard]] const MeasurementStatistics<Sensor>& measurement_statistics() const
    {
        static_assert(
            sensor_id_exists_v<Sensor::Id, Sensors_t>,
            "Requested Sensor does not have a MeasurementStatistics entry in this KalmanFilter.");
        return std::get<SensorIndexFromId_v<Sensor::Id, Sensors_t>>(m_measurement_stats);
    }

    template<MeasurementModelPolicy<StateDef> MeasurementModel>
    void covariance_update(const P_t& P_i,
                           const typename MeasurementModel::H_t& H,
                           const typename MeasurementModel::R_t& R,
                           const typename MeasurementModel::O_t& innovation,
                           typename MeasurementModel::R_t& S,
                           typename MeasurementModel::K_t& K,
                           ErrorState_t& dx,
                           P_t& P_f)
    {
        S = H * P_i * H.transpose() + R;
        K = P_i * H.transpose() * S.ldlt().solve(MeasurementModel::R_t::Identity());
        dx = K * innovation;
        const P_t IKH = I() - K * H;
        P_f = IKH * P_i * IKH.transpose() + K * R * K.transpose();
    }

    template<MeasurementModelPolicy<StateDef> MeasurementModel>
    void observation_update(const typename MeasurementModel::O_t& z,
                            Time_t time,
                            const typename MeasurementModel::ObservationContext& ctx,
                            bool accepted = true)
    {
        observation_update_impl<MeasurementModel, void>(z, time, ctx, accepted);
    }

    template<MeasurementModelPolicy<StateDef> MeasurementModel>
    void observation_update(const typename MeasurementModel::O_t& z,
                            const typename MeasurementModel::ObservationContext& ctx)
    {
        observation_update<MeasurementModel>(z, 0.0, ctx, true);
    }

    template<SensorPolicy Sensor>
        requires MeasurementModelPolicy<typename Sensor::MeasurementModel_t, StateDef>
    void process_sensor(Sensor& sensor)
    {
        using MeasurementModel = typename Sensor::MeasurementModel_t;
        typename Sensor::Measurement_t meas{};
        while (sensor.has_measurement()) {
            if (!sensor.pop(meas)) {
                break;
            }
            sensor.update_observation_context(meas);
            observation_update_impl<MeasurementModel, Sensor>(
                meas.z, meas.time, sensor.observation_context(), true);
        }
    }

    void inject()
    {
        m_last_correction = m_dx;
        m_last_correction_valid = m_pending_correction_valid;
        Injection::apply(m_x, m_dx);
    }

    void reset()
    {
        Reset::reset_covariance(m_x, m_dx, m_P);
        apply_covariance_floor();
        Reset::reset_dx(m_dx);
        m_pending_correction_valid = false;
    }

private:
    static void normalize_nominal_attitude(State_t& x)
    {
        if constexpr (requires { typename Nominal::AttQuat; }) {
            auto q_segment = segment<typename Nominal::AttQuat>(x);
            Eigen::Quaternion<Scalar_t> q{q_segment(0), q_segment(1), q_segment(2), q_segment(3)};
            if (q.norm() <= 0.0) {
                q.setIdentity();
            }
            else {
                q = navkit::core::math::normalized_with_positive_scalar(q);
            }
            q_segment << q.w(), q.x(), q.y(), q.z();
        }
    }

    template<MeasurementModelPolicy<StateDef> MeasurementModel, typename Sensor>
    void observation_update_impl(const typename MeasurementModel::O_t& z,
                                 Time_t time,
                                 const typename MeasurementModel::ObservationContext& ctx,
                                 bool accepted)
    {
        auto profile_scope =
            Profiler::profile(navkit::core::profiling::ProfilePoint::KalmanObservationUpdate);
        static_cast<void>(profile_scope);

        const typename MeasurementModel::O_t innov = z - MeasurementModel::obs(m_x, ctx);
        const typename MeasurementModel::H_t H = MeasurementModel::compute_h(m_x, ctx);
        const typename MeasurementModel::R_t R = MeasurementModel::compute_r(ctx);

        typename MeasurementModel::R_t S{};
        typename MeasurementModel::K_t K{};
        ErrorState_t dx{};
        P_t P_f{};

        covariance_update<MeasurementModel>(m_P, H, R, innov, S, K, dx, P_f);

        const Scalar_t nis = innov.dot(S.ldlt().solve(innov));

        if constexpr (!std::is_void_v<Sensor>) {
            record_measurement_statistics<Sensor>(time, accepted, innov, S, R, H, K, nis);
        }

        if (accepted) {
            m_dx += dx;
            m_P = P_f;
            apply_covariance_floor();
            m_pending_correction_valid = true;
        }
    }

    template<SensorPolicy Sensor>
        requires MeasurementModelPolicy<typename Sensor::MeasurementModel_t, StateDef>
    void record_measurement_statistics(const Time_t time,
                                       const bool accepted,
                                       const typename Sensor::MeasurementModel_t::O_t& innovation,
                                       const typename Sensor::MeasurementModel_t::R_t& S,
                                       const typename Sensor::MeasurementModel_t::R_t& R,
                                       const typename Sensor::MeasurementModel_t::H_t& H,
                                       const typename Sensor::MeasurementModel_t::K_t& K,
                                       const Scalar_t nis)
    {
        if constexpr (sensor_id_exists_v<Sensor::Id, Sensors_t> &&
                      Sensor::Diagnostics_t::enable_measurement_statistics) {
            auto& stats = measurement_statistics<Sensor>();
            stats.valid = true;
            stats.accepted = accepted;
            stats.time = time;
            stats.innovation = innovation;
            stats.innovation_covariance = S;
            stats.measurement_covariance = R;
            stats.jacobian_h = H;
            stats.kalman_gain = K;
            stats.nis = nis;
        }
    }

    State_t m_x{};
    ErrorState_t m_dx{};
    ErrorState_t m_last_correction{};
    P_t m_P{};
    P_t m_covariance_floor{P_t::Zero()};
    MeasurementStatisticsTuple_t m_measurement_stats{};
    bool m_pending_correction_valid{false};
    bool m_last_correction_valid{false};
};

} // namespace navkit::core::estimation
