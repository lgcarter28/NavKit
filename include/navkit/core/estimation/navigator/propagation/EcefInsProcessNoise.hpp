// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/math/Types.hpp"

namespace navkit::core::estimation
{

struct EcefInsZeroProcessNoise
{
    [[nodiscard]] static Vec3 gyro_white_noise_psd_rad2ps()
    {
        return Vec3::Zero();
    }

    [[nodiscard]] static Vec3 accel_white_noise_psd_m2ps3()
    {
        return Vec3::Zero();
    }

    [[nodiscard]] static Vec3 gyro_bias_rw_psd_rad2ps3()
    {
        return Vec3::Zero();
    }

    [[nodiscard]] static Vec3 accel_bias_rw_psd_m2ps5()
    {
        return Vec3::Zero();
    }
};

} // namespace navkit::core::estimation
