#include "catch2/catch_test_macros.hpp"

#include "Simu/config.hpp"

TEST_CASE("Assertion")
{
    struct OkAssert
    {
        OkAssert() { SIMU_ASSERT(true, "Not a error"); }
    };

    REQUIRE_NOTHROW(OkAssert{});

    struct FailAssert
    {
        FailAssert() { SIMU_ASSERT(false, "Error"); }
    };

    REQUIRE_THROWS(FailAssert{});
    REQUIRE_THROWS_AS(FailAssert{}, std::exception);
    REQUIRE_THROWS_AS(FailAssert{}, simu::Exception);
}