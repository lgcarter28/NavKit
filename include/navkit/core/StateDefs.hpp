#pragma once

#include "navkit/core/Segment.hpp"

namespace navkit
{

struct InsStateDef
{
    using Pos = Segment<0, 3>;
    using Vel = Segment<3, 3>;
    using Att = Segment<6, 3>;
    using GyroB = Segment<9, 3>;
    using GyroSf = Segment<12, 3>;
    using AccB = Segment<15, 3>;
    using AccSf = Segment<18, 3>;

    static constexpr int N = 21;
};

struct GnssTcStateDef
{
    using Pos = Segment<0, 3>;
    using Vel = Segment<3, 3>;
    using Att = Segment<6, 3>;
    using GyroB = Segment<9, 3>;
    using GyroSf = Segment<12, 3>;
    using AccB = Segment<15, 3>;
    using AccSf = Segment<18, 3>;
    using ClkB = Segment<21, 1>;
    using ClkD = Segment<22, 1>;

    static constexpr int N = 23;
};

} // namespace navkit
