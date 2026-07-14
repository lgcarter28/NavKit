// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include <concepts>

namespace navkit::core::estimation
{

// A SegmentPolicy describes a fixed compile-time slice of a state vector.
// It intentionally checks only the generic Segment interface used by helpers
// such as segment<TSeg>(...) and block<TSeg>(...).
template<typename T>
concept SegmentPolicy = requires {
    { T::i } -> std::convertible_to<int>;
    { T::sz } -> std::convertible_to<int>;
    requires T::i >= 0;
    requires T::sz > 0;
};

// A StateDefPolicy is the minimum compile-time interface required to allocate
// a fixed-size NavKit state vector/covariance. More specialized algorithms and
// measurement policies may impose additional requirements, such as Pos, Vel,
// Att, clock states, or other named segments.
template<typename T>
concept StateDefPolicy = requires {
    typename T::Scalar_t;
    { T::N } -> std::convertible_to<int>;
    requires T::N > 0;
};

// A StateSpaceDefPolicy names the complete filter state-space layout selected
// by a product config: one concrete nominal state vector definition and one
// concrete error/covariance state vector definition.
template<typename T>
concept StateSpaceDefPolicy =
    requires {
        typename T::Nominal;
        typename T::Error;
    } && StateDefPolicy<typename T::Nominal> && StateDefPolicy<typename T::Error> &&
    std::same_as<typename T::Nominal::Scalar_t, typename T::Error::Scalar_t>;

} // namespace navkit::core::estimation
