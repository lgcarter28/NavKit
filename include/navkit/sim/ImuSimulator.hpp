// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/config/Types.hpp"
#include "navkit/core/estimation/navigator/ImuIncrement.hpp"
#include "navkit/core/math/Types.hpp"
#include "navkit/sim/RandomDraw.hpp"
#include "navkit/sim/TruthSample.hpp"

#include <random>

namespace navkit::sim
{

using navkit::core::Scalar_t;
using navkit::core::Time_t;
using navkit::core::Vec3;
using navkit::core::estimation::ImuIncrement;

struct ImuTriadErrorConfig
{
    Vec3 bias{Vec3::Zero()};
    Vec3 bias_random_walk_psd{Vec3::Zero()};
    Vec3 white_noise_psd{Vec3::Zero()};
    Vec3 scale_factor{Vec3::Zero()};
    // [alpha_x, alpha_y, alpha_z], radians.
    Vec3 misalignment_rad{Vec3::Zero()};
    // [n_yx, n_zx, n_zy].
    Vec3 nonorthogonality{Vec3::Zero()};
    // Zero disables quantization on that axis.
    Vec3 quantization{Vec3::Zero()};
};

struct ImuSimulatorConfig
{
    unsigned int seed{42U};
    ImuTriadErrorConfig gyro{};
    ImuTriadErrorConfig accel{};
};

struct ImuInterval
{
    Time_t time_s{0.0};
    Time_t dt_s{0.0};
    Vec3 omega_ib_b_radps{Vec3::Zero()};
    Vec3 specific_force_ib_b_mps2{Vec3::Zero()};
};

struct ImuIntervalDebug
{
    ImuInterval interval{};
    Vec3 p_bar_e_m{Vec3::Zero()};
    Vec3 v_bar_e_mps{Vec3::Zero()};
    Vec3 a_bar_e_mps2{Vec3::Zero()};
    Vec3 gravity_e_mps2{Vec3::Zero()};
    Vec3 specific_force_e_mps2{Vec3::Zero()};
    Vec3 delta_theta_eb_b_rad{Vec3::Zero()};
};

template<bool OutputConingScullingCompensated = false>
class ImuSimulator
{
public:
    static constexpr bool output_coning_sculling_compensated_v = OutputConingScullingCompensated;

    explicit ImuSimulator(const ImuSimulatorConfig& cfg = {});

    [[nodiscard]] static ImuIncrement increment_from_interval(const ImuInterval& interval);

    [[nodiscard]] static bool interval_from_truth_ecef(const TruthSample& previous,
                                                       const TruthSample& current,
                                                       ImuInterval& interval);

    [[nodiscard]] static bool interval_from_truth_ecef(const TruthSample& previous,
                                                       const TruthSample& current,
                                                       ImuInterval& interval,
                                                       ImuIntervalDebug& debug);

    [[nodiscard]] static Vec3 calibration_matrix_apply(const Vec3& input,
                                                       const ImuTriadErrorConfig& config);

    [[nodiscard]] static Vec3 quantize(const Vec3& value, const Vec3& quantum);

    [[nodiscard]] bool
    generate(const TruthSample& previous, const TruthSample& current, ImuIncrement& increment);

    [[nodiscard]] bool generate(const TruthSample& previous,
                                const TruthSample& current,
                                ImuIncrement& increment,
                                ImuInterval& interval);

    [[nodiscard]] bool generate(const TruthSample& previous,
                                const TruthSample& current,
                                ImuIncrement& increment,
                                ImuInterval& interval,
                                ImuIntervalDebug& debug);

    void initialize(const TruthSample& initial);

    [[nodiscard]] bool generate(const TruthSample& current, ImuIncrement& increment);

    [[nodiscard]] bool
    generate(const TruthSample& current, ImuIncrement& increment, ImuInterval& interval);

    [[nodiscard]] bool generate(const TruthSample& current,
                                ImuIncrement& increment,
                                ImuInterval& interval,
                                ImuIntervalDebug& debug);

    [[nodiscard]] const Vec3& gyro_bias_radps() const
    {
        return m_gyro_bias_radps;
    }

    [[nodiscard]] const Vec3& accel_bias_mps2() const
    {
        return m_accel_bias_mps2;
    }

    [[nodiscard]] bool output_coning_sculling_compensated() const
    {
        return output_coning_sculling_compensated_v;
    }

private:
    [[nodiscard]] Vec3 draw_normal_vector(const Vec3& covariance_diag);
    void update_biases(Time_t dt_s);

    ImuSimulatorConfig m_cfg;
    Vec3 m_gyro_bias_radps{Vec3::Zero()};
    Vec3 m_accel_bias_mps2{Vec3::Zero()};
    std::mt19937 m_rng;
    TruthSample m_previous{};
    bool m_initialized{false};
};

} // namespace navkit::sim
