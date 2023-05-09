#include "catch2/catch_test_macros.hpp"
#include "Simu/math/Interval.hpp"

TEST_CASE("Interval")
{
    SECTION("Basic interval")
    {
        simu::Interval<float> interval{0.f, 1.f};

        REQUIRE_FALSE(interval.contains(-0.1f));
        REQUIRE_FALSE(interval.contains(1.1f));

        REQUIRE(interval.contains(0.f));
        REQUIRE(interval.contains(0.5f));
        REQUIRE(interval.contains(1.f));
    }

    SECTION("Approximation")
    {
        auto interval = simu::approx(0.f, 0.1f);

        REQUIRE_FALSE(interval.contains(-0.5f));
        REQUIRE_FALSE(interval.contains(0.5f));

        REQUIRE(interval.contains(-0.1f));
        REQUIRE(interval.contains(0.f));
        REQUIRE(interval.contains(0.1f));
    }
}
