// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

#include <concepts>

namespace navkit::core::profiling
{

template<typename Candidate>
concept ProfileScopePolicy = std::destructible<Candidate>;

} // namespace navkit::core::profiling
