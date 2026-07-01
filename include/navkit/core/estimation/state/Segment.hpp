// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#pragma once

namespace navkit::core::estimation
{

template<int I, int SZ>
struct Segment
{
    static constexpr int i = I;
    static constexpr int sz = SZ;
};

template<typename TSeg, typename TMat>
auto block(TMat& m)
{
    return m.template block<TSeg::sz, TSeg::sz>(TSeg::i, TSeg::i);
}

template<typename TSeg, typename TMat>
auto block(const TMat& m)
{
    return m.template block<TSeg::sz, TSeg::sz>(TSeg::i, TSeg::i);
}

template<typename TSeg, typename TVec>
auto segment(TVec& v)
{
    return v.template segment<TSeg::sz>(TSeg::i);
}

template<typename TSeg, typename TVec>
auto segment(const TVec& v)
{
    return v.template segment<TSeg::sz>(TSeg::i);
}

} // namespace navkit::core::estimation
