// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/estimation/navigator/propagation/ImuProcessNoise.hpp"

namespace navkit::config::navkit::products::components
{

struct ImuProcessNoiseDefault
{
    using ProcessNoise_t = core::estimation::ImuProcessNoise;

    inline static const ProcessNoise_t process_noise{
        .gyro_white_noise_psd_rad2ps = core::Vec3::Constant(1.0e-11),
        .accel_white_noise_psd_m2ps3 = core::Vec3::Constant(1.0e-7),
        .gyro_bias_drive_psd_rad2ps3 = core::Vec3::Constant(1.0e-14),
        .accel_bias_drive_psd_m2ps5 = core::Vec3::Constant(1.0e-10),
    };
};

static_assert(core::estimation::ImuProcessNoiseConfigPolicy<ImuProcessNoiseDefault>);

} // namespace navkit::config::navkit::products::components
