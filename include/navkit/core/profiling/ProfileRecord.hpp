// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/profiling/ProfilePoint.hpp"

#include <cstdint>

namespace navkit::core::profiling
{

enum class ProfileRecordFlags : std::uint16_t
{
    None = 0U,
    DroppedBefore = 1U << 0U,
    Incomplete = 1U << 1U
};

template<typename Tick>
struct ProfileRecord
{
    ProfilePoint point{};
    Tick start_tick{};
    Tick elapsed_ticks{};
    std::uint32_t sequence{};
    std::uint32_t parent_sequence{};
    std::uint16_t depth{};
    ProfileRecordFlags flags{ProfileRecordFlags::None};
};

} // namespace navkit::core::profiling
