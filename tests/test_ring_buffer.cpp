// Copyright (c) 2026 William Gordon Carter.
// All Rights Reserved.

#include "navkit/containers/RingBuffer.hpp"
#include "test_main.hpp"

TEST_CASE("RingBuffer push/pop order")
{
    navkit::RingBuffer<int, 3> rb;
    CHECK(rb.empty());
    CHECK(rb.push(1));
    CHECK(rb.push(2));
    int out = 0;
    CHECK(rb.pop(out));
    CHECK(out == 1);
    CHECK(rb.pop(out));
    CHECK(out == 2);
    CHECK(rb.empty());
}

TEST_CASE("RingBuffer rejects overflow by default")
{
    navkit::RingBuffer<int, 2> rb;
    CHECK(rb.push(1));
    CHECK(rb.push(2));
    CHECK_FALSE(rb.push(3));
}
