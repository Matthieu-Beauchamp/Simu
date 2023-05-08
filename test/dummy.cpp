#include "catch2/catch_test_macros.hpp"
#include "Simu/math/math.hpp"

TEST_CASE("dummy test")
{
    REQUIRE(simu::testCoverage(1) == 2);
}
