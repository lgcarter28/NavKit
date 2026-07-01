// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include <concepts>

namespace navkit::core::config
{

template<typename Candidate>
concept NumericConfigPolicy = requires {
    typename Candidate::Scalar_t;
    typename Candidate::Time_t;
};

template<typename Candidate>
concept ConfigPolicy = requires {
    typename Candidate::Numeric;

    requires NumericConfigPolicy<typename Candidate::Numeric>;
};

} // namespace navkit::core::config
