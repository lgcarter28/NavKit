// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/environment/RotatingPlanetKinematics.hpp"
#include "navkit/core/environment/planet/PlanetPolicy.hpp"
#include "navkit/core/math/Types.hpp"
#include "navkit/core/time/Duration.hpp"

#include <Eigen/Geometry>
#include <cmath>

namespace navkit::core::frames
{

template<environment::RotatingPlanetPolicy Planet>
[[nodiscard]] inline bool fixed_to_inertial_matrix(const Time_t elapsed_s, Mat3& C_f2i)
{
    if (!std::isfinite(elapsed_s)) {
        return false;
    }
    const Eigen::AngleAxis<Scalar_t> rotation{Planet::omega_rad_s * elapsed_s, Vec3::UnitZ()};
    C_f2i = rotation.toRotationMatrix();
    return true;
}

/**
 * Constructs the fixed-to-inertial transform at `t` under a uniform-rate
 * planet orientation model whose fixed and inertial axes are aligned at
 * `t_epoch`.
 */
template<environment::RotatingPlanetPolicy Planet>
[[nodiscard]] inline bool
fixed_to_inertial_matrix(const Timestamp& t, const Timestamp& t_epoch, Mat3& C_f2i)
{
    Duration elapsed{};
    if (!elapsed_time(t, t_epoch, elapsed)) {
        return false;
    }
    return fixed_to_inertial_matrix<Planet>(duration_seconds(elapsed), C_f2i);
}

template<environment::RotatingPlanetPolicy Planet>
[[nodiscard]] inline bool inertial_to_fixed_matrix(const Time_t elapsed_s, Mat3& C_i2f)
{
    Mat3 C_f2i{};
    if (!fixed_to_inertial_matrix<Planet>(elapsed_s, C_f2i)) {
        return false;
    }
    C_i2f = C_f2i.transpose();
    return true;
}

template<environment::RotatingPlanetPolicy Planet>
[[nodiscard]] inline bool
inertial_to_fixed_matrix(const Timestamp& t, const Timestamp& t_epoch, Mat3& C_i2f)
{
    Mat3 C_f2i{};
    if (!fixed_to_inertial_matrix<Planet>(t, t_epoch, C_f2i)) {
        return false;
    }
    C_i2f = C_f2i.transpose();
    return true;
}

template<environment::RotatingPlanetPolicy Planet>
[[nodiscard]] inline bool
fixed_to_inertial_position(const Vec3& p_f_m, const Time_t elapsed_s, Vec3& p_i_m)
{
    Mat3 C_f2i{};
    if (!p_f_m.allFinite() || !fixed_to_inertial_matrix<Planet>(elapsed_s, C_f2i)) {
        return false;
    }
    p_i_m = C_f2i * p_f_m;
    return true;
}

template<environment::RotatingPlanetPolicy Planet>
[[nodiscard]] inline bool fixed_to_inertial_velocity(const Vec3& p_f_m,
                                                     const Vec3& v_f_mps,
                                                     const Time_t elapsed_s,
                                                     Vec3& v_i_mps)
{
    Mat3 C_f2i{};
    if (!p_f_m.allFinite() || !v_f_mps.allFinite() ||
        !fixed_to_inertial_matrix<Planet>(elapsed_s, C_f2i)) {
        return false;
    }
    const Vec3 w_if_f_radps = environment::planet_rate_fixed_radps<Planet>();
    v_i_mps = C_f2i * (v_f_mps + w_if_f_radps.cross(p_f_m));
    return true;
}

template<environment::RotatingPlanetPolicy Planet>
[[nodiscard]] inline bool fixed_to_inertial_acceleration(const Vec3& p_f_m,
                                                         const Vec3& v_f_mps,
                                                         const Vec3& a_f_mps2,
                                                         const Time_t elapsed_s,
                                                         Vec3& a_i_mps2)
{
    Mat3 C_f2i{};
    if (!p_f_m.allFinite() || !v_f_mps.allFinite() || !a_f_mps2.allFinite() ||
        !fixed_to_inertial_matrix<Planet>(elapsed_s, C_f2i)) {
        return false;
    }
    const Vec3 w_if_f_radps = environment::planet_rate_fixed_radps<Planet>();
    a_i_mps2 = C_f2i * (a_f_mps2 + (2.0 * w_if_f_radps.cross(v_f_mps)) +
                        w_if_f_radps.cross(w_if_f_radps.cross(p_f_m)));
    return true;
}

template<environment::RotatingPlanetPolicy Planet>
[[nodiscard]] inline bool
inertial_to_fixed_position(const Vec3& p_i_m, const Time_t elapsed_s, Vec3& p_f_m)
{
    Mat3 C_f2i{};
    if (!p_i_m.allFinite() || !fixed_to_inertial_matrix<Planet>(elapsed_s, C_f2i)) {
        return false;
    }
    p_f_m = C_f2i.transpose() * p_i_m;
    return true;
}

template<environment::RotatingPlanetPolicy Planet>
[[nodiscard]] inline bool inertial_to_fixed_velocity(const Vec3& p_i_m,
                                                     const Vec3& v_i_mps,
                                                     const Time_t elapsed_s,
                                                     Vec3& v_f_mps)
{
    Mat3 C_f2i{};
    if (!p_i_m.allFinite() || !v_i_mps.allFinite() ||
        !fixed_to_inertial_matrix<Planet>(elapsed_s, C_f2i)) {
        return false;
    }
    const Vec3 p_f_m = C_f2i.transpose() * p_i_m;
    const Vec3 w_if_f_radps = environment::planet_rate_fixed_radps<Planet>();
    v_f_mps = (C_f2i.transpose() * v_i_mps) - w_if_f_radps.cross(p_f_m);
    return true;
}

template<environment::RotatingPlanetPolicy Planet>
[[nodiscard]] inline bool inertial_to_fixed_acceleration(const Vec3& p_i_m,
                                                         const Vec3& v_i_mps,
                                                         const Vec3& a_i_mps2,
                                                         const Time_t elapsed_s,
                                                         Vec3& a_f_mps2)
{
    Mat3 C_f2i{};
    Vec3 p_f_m{};
    Vec3 v_f_mps{};
    if (!a_i_mps2.allFinite() || !fixed_to_inertial_matrix<Planet>(elapsed_s, C_f2i) ||
        !inertial_to_fixed_position<Planet>(p_i_m, elapsed_s, p_f_m) ||
        !inertial_to_fixed_velocity<Planet>(p_i_m, v_i_mps, elapsed_s, v_f_mps)) {
        return false;
    }
    const Vec3 w_if_f_radps = environment::planet_rate_fixed_radps<Planet>();
    a_f_mps2 = (C_f2i.transpose() * a_i_mps2) - (2.0 * w_if_f_radps.cross(v_f_mps)) -
               w_if_f_radps.cross(w_if_f_radps.cross(p_f_m));
    return true;
}

} // namespace navkit::core::frames
