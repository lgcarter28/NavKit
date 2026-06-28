#include <Eigen/Dense>
#include "test_main.hpp"
#include "navkit/core/Segment.hpp"

TEST_CASE("Segment helper selects vector block") {
    using Pos = navkit::Segment<0, 3>;
    Eigen::Matrix<double, 6, 1> x;
    x << 1, 2, 3, 4, 5, 6;
    const auto p = navkit::segment<Pos>(x);
    CHECK(p(0) == doctest::Approx(1.0));
    CHECK(p(1) == doctest::Approx(2.0));
    CHECK(p(2) == doctest::Approx(3.0));
}
