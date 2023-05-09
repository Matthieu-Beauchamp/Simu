#include "catch2/catch_test_macros.hpp"
#include "Simu/math/Interval.hpp"

TEST_CASE("Interval")
{
    SECTION("Closed Intervals")
    {
        auto interval = simu::makeClosedInterval(0.f, 1.f);

        REQUIRE_FALSE(interval.contains(-1.f));
        REQUIRE_FALSE(interval.contains(2.f));

        REQUIRE(interval.contains(0.f));
        REQUIRE(interval.contains(0.5f));
        REQUIRE(interval.contains(1.f));
    }

}
