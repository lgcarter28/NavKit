// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include "navkit/core/profiling/ProfileRecord.hpp"

#include <type_traits>

namespace navkit::core::profiling
{

template<typename Sink, typename Clock, typename = void>
struct ProfileSinkRecord
{
    using type = ProfileRecord<typename Clock::Tick>;
};

template<typename Sink, typename Clock>
struct ProfileSinkRecord<Sink, Clock, std::void_t<typename Sink::Record>>
{
    using type = typename Sink::Record;
};

template<typename Sink, typename Clock>
using ProfileSinkRecord_t = typename ProfileSinkRecord<Sink, Clock>::type;

} // namespace navkit::core::profiling
