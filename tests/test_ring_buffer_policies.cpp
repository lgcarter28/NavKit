// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/core/containers/RingBuffer.hpp"
#include "test_main.hpp"

namespace navkit::core::containers::test
{

TEST_CASE("RingBuffer overwrite policy keeps the newest fixed-capacity window")
{
    RingBuffer<int, 3, OverflowPolicy::OverwriteOldest> buffer;

    CHECK(buffer.push(1));
    CHECK(buffer.push(2));
    CHECK(buffer.push(3));
    CHECK(buffer.full());

    CHECK(buffer.push(4));
    CHECK(buffer.size() == 3U);

    int out = 0;
    CHECK(buffer.pop(out));
    CHECK(out == 2);
    CHECK(buffer.pop(out));
    CHECK(out == 3);
    CHECK(buffer.pop(out));
    CHECK(out == 4);
    CHECK(buffer.empty());
}

TEST_CASE("RingBuffer clear resets occupancy and preserves capacity")
{
    RingBuffer<int, 2> buffer;
    CHECK(buffer.push(7));
    CHECK(buffer.push(8));
    CHECK(buffer.full());

    buffer.clear();

    CHECK(buffer.empty());
    CHECK(buffer.size() == 0U);
    CHECK(buffer.capacity() == 2U);
    CHECK(buffer.push(9));

    int out = 0;
    CHECK(buffer.pop(out));
    CHECK(out == 9);
}

} // namespace navkit::core::containers::test
