#pragma once

#include <Eigen/Dense>
#include "navkit/core/State.hpp"
#include "navkit/core/policies/InjectionPolicies.hpp"
#include "navkit/core/policies/ResetPolicies.hpp"

namespace navkit
{

    template <typename StateDef,
              typename InjectionPolicy = DefaultInjectionPolicy<StateDef>,
              typename ResetPolicy = DefaultResetPolicy<StateDef>>
    class KalmanFilter
    {
    public:
        using StateDef_t = StateDef;
        using State_t = State<StateDef>;
        using P_t = StateCov<StateDef>;

        KalmanFilter()
        {
            m_x.setZero();
            m_dx.setZero();
            m_P.setIdentity();
        }

        static const P_t &I()
        {
            static const P_t identity = P_t::Identity();
            return identity;
        }

        State_t &state() { return m_x; }
        [[nodiscard]] const State_t &state() const { return m_x; }

        State_t &error_state() { return m_dx; }
        [[nodiscard]] const State_t &error_state() const { return m_dx; }

        P_t &covariance() { return m_P; }
        [[nodiscard]] const P_t &covariance() const { return m_P; }

        void set_state(const State_t &x) { m_x = x; }
        void set_covariance(const P_t &P) { m_P = P; }

        template <typename Model>
        void covariance_update(const P_t &P_i,
                               const typename Model::H_t &H,
                               const typename Model::R_t &R,
                               const typename Model::O_t &innovation,
                               typename Model::R_t &S,
                               typename Model::K_t &K,
                               State_t &dx,
                               P_t &P_f)
        {
            S = H * P_i * H.transpose() + R;
            K = P_i * H.transpose() * S.ldlt().solve(Model::R_t::Identity());
            dx = K * innovation;
            const P_t IKH = I() - K * H;
            P_f = IKH * P_i * IKH.transpose() + K * R * K.transpose();
        }

        template <typename Model>
        void observation_update(const typename Model::O_t &z,
                                const typename Model::NoiseContext &ctx)
        {
            const typename Model::O_t innov = z - Model::obs(m_x);
            const typename Model::H_t H = Model::compute_h(m_x);
            const typename Model::R_t R = Model::compute_r(ctx);

            typename Model::R_t S{};
            typename Model::K_t K{};
            State_t dx{};
            P_t P_f{};

            covariance_update<Model>(m_P, H, R, innov, S, K, dx, P_f);
            m_dx += dx;
            m_P = P_f;
        }

        template <typename Sensor>
        void process_sensor(Sensor &sensor)
        {
            using Model = typename Sensor::Model_t;
            typename Sensor::Measurement_t meas{};
            while (sensor.has_measurement())
            {
                if (!sensor.pop(meas))
                {
                    break;
                }
                sensor.update_noise_context(meas);
                observation_update<Model>(meas.z, sensor.noise_context());
            }
        }

        void inject()
        {
            InjectionPolicy::apply(m_x, m_dx);
        }

        void reset()
        {
            ResetPolicy::reset_covariance(m_x, m_dx, m_P);
            ResetPolicy::reset_dx(m_dx);
        }

    private:
        State_t m_x{};
        State_t m_dx{};
        P_t m_P{};
    };

} // namespace navkit
