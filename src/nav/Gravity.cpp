#include "navkit/nav/Gravity.hpp"

namespace navkit::gravity {

Eigen::Matrix<Scalar_t, 3, 1> simple_gravity_ecef(const Eigen::Matrix<Scalar_t, 3, 1>& p_e) {
    constexpr Scalar_t mu = 3.986004418e14;
    const Scalar_t r = p_e.norm();
    if (r <= 0.0) {
        return Eigen::Matrix<Scalar_t, 3, 1>::Zero();
    }
    return -mu / (r * r * r) * p_e;
}

} // namespace navkit::gravity
