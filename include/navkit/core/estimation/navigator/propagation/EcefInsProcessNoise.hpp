// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/math/Types.hpp"

#include <concepts>
#include <type_traits>

namespace navkit::core::estimation
{

struct EcefInsProcessNoise
{
    Vec3 gyro_white_noise_psd_rad2ps{Vec3::Zero()};
    Vec3 accel_white_noise_psd_m2ps3{Vec3::Zero()};
    Vec3 gyro_bias_drive_psd_rad2ps3{Vec3::Zero()};
    Vec3 accel_bias_drive_psd_m2ps5{Vec3::Zero()};
};

template<typename Candidate>
concept EcefInsProcessNoiseConfigPolicy = requires {
    typename Candidate::ProcessNoise_t;
    requires std::same_as<typename Candidate::ProcessNoise_t, EcefInsProcessNoise>;
    requires std::same_as<std::remove_cvref_t<decltype(Candidate::process_noise)>,
                          EcefInsProcessNoise>;
};

struct EcefInsZeroProcessNoise
{
    using ProcessNoise_t = EcefInsProcessNoise;

    inline static const ProcessNoise_t process_noise{};
};

} // namespace navkit::core::estimation
