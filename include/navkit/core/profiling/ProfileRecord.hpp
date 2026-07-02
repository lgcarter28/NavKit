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

[[nodiscard]] constexpr ProfileRecordFlags operator|(ProfileRecordFlags lhs,
                                                     ProfileRecordFlags rhs) noexcept
{
    return static_cast<ProfileRecordFlags>(static_cast<std::uint16_t>(lhs) |
                                           static_cast<std::uint16_t>(rhs));
}

[[nodiscard]] constexpr ProfileRecordFlags operator&(ProfileRecordFlags lhs,
                                                     ProfileRecordFlags rhs) noexcept
{
    return static_cast<ProfileRecordFlags>(static_cast<std::uint16_t>(lhs) &
                                           static_cast<std::uint16_t>(rhs));
}

[[nodiscard]] constexpr bool has_profile_record_flag(ProfileRecordFlags flags,
                                                     ProfileRecordFlags flag) noexcept
{
    return (flags & flag) != ProfileRecordFlags::None;
}

template<typename Tick, typename Sequence = std::uint32_t, typename Depth = std::uint16_t>
struct ProfileRecord
{
    using Tick_t = Tick;
    using Sequence_t = Sequence;
    using Depth_t = Depth;

    ProfilePoint point{};
    Tick start_tick{};
    Tick elapsed_ticks{};
    Sequence sequence{};
    Sequence parent_sequence{};
    Depth depth{};
    ProfileRecordFlags flags{ProfileRecordFlags::None};
};

} // namespace navkit::core::profiling
