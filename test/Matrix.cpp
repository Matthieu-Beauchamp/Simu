#include "catch2/catch_test_macros.hpp"
#include "Simu/math/Matrix.hpp"

TEST_CASE("Matrix")
{
    SECTION("Matrix Data")
    {
        simu::Mat22 null{};
        REQUIRE(all(null == simu::Mat22{0, 0, 0, 0}));
    }
}