// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/estimation/filter/AppliedCorrection.hpp"
#include "navkit/core/estimation/filter/CovarianceFloor.hpp"
#include "navkit/core/estimation/filter/CovarianceHealth.hpp"
#include "navkit/core/estimation/filter/MeasurementStatistics.hpp"
#include "navkit/core/estimation/filter/MeasurementStatisticsStorage.hpp"
#include "navkit/core/estimation/filter/injection/InjectionPolicies.hpp"
#include "navkit/core/estimation/filter/injection/InjectionPolicy.hpp"
#include "navkit/core/estimation/filter/reset/ResetPolicies.hpp"
#include "navkit/core/estimation/filter/reset/ResetPolicy.hpp"
#include "navkit/core/estimation/measurement/MeasurementModelPolicy.hpp"
#include "navkit/core/estimation/sensor/InnovationGate.hpp"
#include "navkit/core/estimation/sensor/SensorTuplePolicy.hpp"
#include "navkit/core/estimation/sensor/SensorTupleTraits.hpp"
#include "navkit/core/estimation/state/Segment.hpp"
#include "navkit/core/estimation/state/State.hpp"
#include "navkit/core/math/Quaternion.hpp"
#include "navkit/core/profiling/NullProfiler.hpp"
#include "navkit/core/profiling/ProfilePoint.hpp"
#include "navkit/core/profiling/ProfilerPolicy.hpp"
#include "navkit/core/time/Timestamp.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <limits>
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
    using AppliedCorrection_t = AppliedCorrection<StateDef>;
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

    [[nodiscard]] bool pending_correction_valid() const
    {
        return m_pending_correction_valid;
    }

    /** Clears the per-sensor measurement diagnostics for a new processing cycle. */
    void clear_measurement_statistics()
    {
        std::apply([](auto&... statistics) { ((statistics.valid = false), ...); },
                   m_measurement_stats);
    }

    P_t& covariance()
    {
        return m_P;
    }

    [[nodiscard]] const P_t& covariance() const
    {
        return m_P;
    }

    [[nodiscard]] CovarianceHealthResult<typename P_t::Scalar>
    covariance_health(const CovarianceHealthTolerances<typename P_t::Scalar>& tolerances = {}) const
    {
        return evaluate_covariance_health(m_P, tolerances);
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
    [[nodiscard]] bool covariance_update(const P_t& P_i,
                                         const typename MeasurementModel::H_t& H,
                                         const typename MeasurementModel::R_t& R,
                                         const typename MeasurementModel::O_t& innovation,
                                         typename MeasurementModel::R_t& S,
                                         typename MeasurementModel::K_t& K,
                                         ErrorState_t& dx,
                                         P_t& P_f,
                                         Scalar_t& nis)
    {
        K.setZero();
        dx.setZero();
        P_f = P_i;
        nis = std::numeric_limits<Scalar_t>::quiet_NaN();

        S = H * P_i * H.transpose() + R;
        const Eigen::LDLT<typename MeasurementModel::R_t> decomposition{S};
        if (decomposition.info() != Eigen::Success || !decomposition.isPositive()) {
            return false;
        }
        const typename MeasurementModel::O_t whitened_innovation = decomposition.solve(innovation);
        const typename MeasurementModel::R_t inverse_s =
            decomposition.solve(MeasurementModel::R_t::Identity());
        if (decomposition.info() != Eigen::Success || !whitened_innovation.allFinite() ||
            !inverse_s.allFinite()) {
            return false;
        }
        nis = innovation.dot(whitened_innovation);
        if (!std::isfinite(nis) || nis < 0.0) {
            return false;
        }
        K = P_i * H.transpose() * inverse_s;
        dx = K * innovation;
        const P_t IKH = I() - K * H;
        P_f = IKH * P_i * IKH.transpose() + K * R * K.transpose();
        if (!K.allFinite() || !dx.allFinite() || !P_f.allFinite()) {
            K.setZero();
            dx.setZero();
            P_f = P_i;
            return false;
        }
        return true;
    }

    template<MeasurementModelPolicy<StateDef> MeasurementModel>
    void observation_update(const typename MeasurementModel::O_t& z,
                            const Timestamp& t,
                            const typename MeasurementModel::ObservationContext& ctx)
    {
        const InnovationGate<MeasurementModel::M> disabled_gate{};
        observation_update_impl<MeasurementModel, void>(z, t, ctx, disabled_gate);
    }

    template<MeasurementModelPolicy<StateDef> MeasurementModel>
    void observation_update(const typename MeasurementModel::O_t& z,
                            const typename MeasurementModel::ObservationContext& ctx)
    {
        observation_update<MeasurementModel>(z, Timestamp{}, ctx);
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
                meas.z, meas.t, sensor.observation_context(), sensor.innovation_gate());
        }
    }

    [[nodiscard]] AppliedCorrection_t inject()
    {
        const AppliedCorrection_t applied{.value = m_dx, .valid = m_pending_correction_valid};
        Injection::apply(m_x, m_dx);
        return applied;
    }

    [[nodiscard]] static AppliedCorrection_t
    compose_applied_corrections(const AppliedCorrection_t& first, const AppliedCorrection_t& second)
    {
        if (!first.valid) {
            return second;
        }
        if (!second.valid) {
            return first;
        }

        AppliedCorrection_t composed{.valid = true};
        Injection::compose(first.value, second.value, composed.value);
        return composed;
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
                                 const Timestamp& t,
                                 const typename MeasurementModel::ObservationContext& ctx,
                                 const InnovationGate<MeasurementModel::M>& gate)
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
        Scalar_t nis = std::numeric_limits<Scalar_t>::quiet_NaN();

        const bool innovation_covariance_valid =
            covariance_update<MeasurementModel>(m_P, H, R, innov, S, K, dx, P_f, nis);
        const bool accepted = innovation_covariance_valid && gate.accepts(nis);

        if constexpr (!std::is_void_v<Sensor>) {
            record_measurement_statistics<Sensor>(
                t, accepted, innovation_covariance_valid, gate, innov, S, R, H, K, nis);
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
    void record_measurement_statistics(const Timestamp& t,
                                       const bool accepted,
                                       const bool innovation_covariance_valid,
                                       const InnovationGate<Sensor::MeasurementModel_t::M>& gate,
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
            stats.innovation_covariance_valid = innovation_covariance_valid;
            stats.gate_enabled = gate.enabled();
            stats.t = t;
            stats.innovation = innovation;
            stats.innovation_covariance = S;
            stats.measurement_covariance = R;
            stats.jacobian_h = H;
            stats.kalman_gain = K;
            stats.nis = nis;
            stats.gate_probability = gate.probability();
            stats.gate_threshold = gate.threshold();
            stats.gate_dof = gate.dof;
        }
    }

    /**
     * \brief Current nominal state estimate.
     *
     * \details Contains the nominal state after all completed error-state
     * injections.
     */
    State_t m_x{};

    /**
     * \brief Pending error-state correction accumulated since the most recent
     * reset.
     *
     * \details Accepted observations add their corrections to this value.
     * Injection applies it to `m_x`, after which reset returns it to zero.
     * Under the current `UpdateAfterEachSensor` policy, it normally contains
     * the correction from one sensor's processed measurement buffer.
     */
    ErrorState_t m_dx{};

    /**
     * \brief Current error-state covariance.
     *
     * \details Contains the covariance associated with the filter error state.
     */
    P_t m_P{};

    /**
     * \brief Configured elementwise diagonal covariance floor.
     *
     * \details Applied to the diagonal of `m_P` to enforce the configured
     * minimum variances.
     */
    P_t m_covariance_floor{P_t::Zero()};

    /**
     * \brief Latest measurement-update diagnostics for each sensor.
     *
     * \details Stores measurement statistics independently for every sensor in
     * the configured sensor tuple.
     */
    MeasurementStatisticsTuple_t m_measurement_stats{};

    /**
     * \brief Indicates whether an accepted correction is awaiting injection.
     *
     * \details Set when at least one accepted measurement update has contributed
     * to `m_dx`; cleared after injection and reset.
     */
    bool m_pending_correction_valid{false};
};

} // namespace navkit::core::estimation
