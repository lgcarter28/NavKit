// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

namespace navkit::planet
{

template<typename T>
concept PlanetPolicy = requires {
    T::mu_m3_s2;
    typename T::InertialFrame;
    typename T::FixedFrame;
};

template<typename T>
concept RotatingPlanetPolicy = PlanetPolicy<T> && requires { T::omega_rad_s; };

template<typename T>
concept SphericalPlanetPolicy = PlanetPolicy<T> && requires { T::radius_m; };

template<typename T>
concept EllipsoidPlanetPolicy = PlanetPolicy<T> && requires {
    T::a_m;
    T::b_m;
};

template<typename T>
concept J2PlanetPolicy = PlanetPolicy<T> && requires {
    T::J2;
    T::a_m;
};

} // namespace navkit::planet
