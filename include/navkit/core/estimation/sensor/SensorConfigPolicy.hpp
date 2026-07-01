// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include <concepts>
#include <cstddef>

namespace navkit::core::estimation
{

template<typename Candidate>
concept BufferConfigPolicy = requires {
    { Candidate::BufferSize } -> std::convertible_to<std::size_t>;

    requires Candidate::BufferSize > 0;
};

} // namespace navkit::core::estimation
