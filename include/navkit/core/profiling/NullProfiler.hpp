// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/profiling/ProfilePoint.hpp"

namespace navkit::core::profiling
{

struct NullProfileScope
{};

struct NullProfiler
{
    using Scope = NullProfileScope;

    [[nodiscard]] static constexpr Scope profile(ProfilePoint) noexcept
    {
        return {};
    }
};

} // namespace navkit::core::profiling
