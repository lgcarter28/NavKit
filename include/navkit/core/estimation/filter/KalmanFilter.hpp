// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/estimation/filter/MeasurementStatistics.hpp"
#include "navkit/core/estimation/filter/injection/InjectionPolicies.hpp"
#include "navkit/core/estimation/filter/injection/InjectionPolicy.hpp"
#include "navkit/core/estimation/filter/reset/ResetPolicies.hpp"
#include "navkit/core/estimation/filter/reset/ResetPolicy.hpp"
#include "navkit/core/estimation/measurement/MeasurementPolicy.hpp"
#include "navkit/core/estimation/state/State.hpp"
#include "navkit/core/profiling/NullProfiler.hpp"
#include "navkit/core/profiling/ProfilePoint.hpp"
#include "navkit/core/profiling/ProfilePolicy.hpp"

#include <Eigen/Dense>
#include <tuple>

namespace navkit::core::estimation
{

template<StateDefPolicy StateDef,
         InjectionPolicy<StateDef> Injection = DefaultInjectionPolicy<StateDef>,
         ResetPolicy<StateDef> Reset = DefaultResetPolicy<StateDef>,
         typename MeasurementModels = std::tuple<>,
         navkit::core::profiling::ProfilerPolicy Profiler = navkit::core::profiling::NullProfiler>
class KalmanFilter
{
public:
    using StateDef_t = StateDef;
    using State_t = State<StateDef>;
    using P_t = StateCov<StateDef>;
    using MeasurementModels_t = MeasurementModels;
    using MeasurementStatistics_t = MeasurementStatisticsTuple_t<MeasurementModels_t>;
    using Profiler_t = Profiler;

    KalmanFilter()
    {
        m_x.setZero();
        m_dx.setZero();
        m_P.setIdentity();
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

    State_t& error_state()
    {
        return m_dx;
    }

    [[nodiscard]] const State_t& error_state() const
    {
        return m_dx;
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
    }

    void set_covariance(const P_t& P)
    {
        m_P = P;
    }

    template<MeasurementPolicy<StateDef> Model>
    [[nodiscard]] bool has_measurement_statistics() const
    {
        if constexpr (tuple_contains_v<Model, MeasurementModels_t>) {
            return measurement_statistics<Model>().valid;
        }
        else {
            return false;
        }
    }

    template<MeasurementPolicy<StateDef> Model>
    MeasurementStatistics<Model>& measurement_statistics()
    {
        static_assert(tuple_contains_v<Model, MeasurementModels_t>,
                      "Requested Model is not listed in the KalmanFilter MeasurementModels tuple.");
        return std::get<tuple_index_v<Model, MeasurementModels_t>>(m_measurement_stats);
    }

    template<MeasurementPolicy<StateDef> Model>
    [[nodiscard]] const MeasurementStatistics<Model>& measurement_statistics() const
    {
        static_assert(tuple_contains_v<Model, MeasurementModels_t>,
                      "Requested Model is not listed in the KalmanFilter MeasurementModels tuple.");
        return std::get<tuple_index_v<Model, MeasurementModels_t>>(m_measurement_stats);
    }

    template<MeasurementPolicy<StateDef> Model>
    void covariance_update(const P_t& P_i,
                           const typename Model::H_t& H,
                           const typename Model::R_t& R,
                           const typename Model::O_t& innovation,
                           typename Model::R_t& S,
                           typename Model::K_t& K,
                           State_t& dx,
                           P_t& P_f)
    {
        S = H * P_i * H.transpose() + R;
        K = P_i * H.transpose() * S.ldlt().solve(Model::R_t::Identity());
        dx = K * innovation;
        const P_t IKH = I() - K * H;
        P_f = IKH * P_i * IKH.transpose() + K * R * K.transpose();
    }

    template<MeasurementPolicy<StateDef> Model>
    void observation_update(const typename Model::O_t& z,
                            Time_t time,
                            const typename Model::NoiseContext& ctx,
                            bool accepted = true)
    {
        auto profile_scope =
            Profiler::profile(navkit::core::profiling::ProfilePoint::KalmanObservationUpdate);
        static_cast<void>(profile_scope);

        const typename Model::O_t innov = z - Model::obs(m_x);
        const typename Model::H_t H = Model::compute_h(m_x);
        const typename Model::R_t R = Model::compute_r(ctx);

        typename Model::R_t S{};
        typename Model::K_t K{};
        State_t dx{};
        P_t P_f{};

        covariance_update<Model>(m_P, H, R, innov, S, K, dx, P_f);

        const Scalar_t nis = innov.dot(S.ldlt().solve(innov));
        record_measurement_statistics<Model>(time, accepted, innov, S, R, H, K, nis);

        if (accepted) {
            m_dx += dx;
            m_P = P_f;
        }
    }

    template<MeasurementPolicy<StateDef> Model>
    void observation_update(const typename Model::O_t& z, const typename Model::NoiseContext& ctx)
    {
        observation_update<Model>(z, 0.0, ctx, true);
    }

    template<typename Sensor>
        requires MeasurementPolicy<typename Sensor::Model_t, StateDef>
    void process_sensor(Sensor& sensor)
    {
        using Model = typename Sensor::Model_t;
        typename Sensor::Measurement_t meas{};
        while (sensor.has_measurement()) {
            if (!sensor.pop(meas)) {
                break;
            }
            sensor.update_noise_context(meas);
            observation_update<Model>(meas.z, meas.time, sensor.noise_context(), true);
        }
    }

    void inject()
    {
        Injection::apply(m_x, m_dx);
    }

    void reset()
    {
        Reset::reset_covariance(m_x, m_dx, m_P);
        Reset::reset_dx(m_dx);
    }

private:
    template<MeasurementPolicy<StateDef> Model>
    void record_measurement_statistics(const Time_t time,
                                       const bool accepted,
                                       const typename Model::O_t& innovation,
                                       const typename Model::R_t& S,
                                       const typename Model::R_t& R,
                                       const typename Model::H_t& H,
                                       const typename Model::K_t& K,
                                       const Scalar_t nis)
    {
        if constexpr (tuple_contains_v<Model, MeasurementModels_t>) {
            auto& stats = measurement_statistics<Model>();
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
    State_t m_dx{};
    P_t m_P{};
    MeasurementStatistics_t m_measurement_stats{};
};

} // namespace navkit::core::estimation
