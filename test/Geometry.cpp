#include "catch2/catch_test_macros.hpp"

#include "Simu/math/Geometry.hpp"
#include "Simu/math/Interval.hpp"

using namespace simu;

TEST_CASE("Geometry")
{
    SECTION("Convex geometry")
    {
        SECTION("Invalid Geometry")
        {
            // not enough vertices
            REQUIRE_THROWS(ConvexGeometry{});
            REQUIRE_THROWS(ConvexGeometry{
                Vertex{0, 0}
            });
            REQUIRE_THROWS(ConvexGeometry{
                Vertex{0, 0},
                Vertex{1, 0}
            });

            // null area
            REQUIRE_THROWS(ConvexGeometry{
                Vertex{0, 0},
                Vertex{1, 1},
                Vertex{2, 2}
            });

            // negative winding order
            REQUIRE_THROWS(ConvexGeometry{
                Vertex{0, 0},
                Vertex{0, 1},
                Vertex{1, 0}
            });
        }


        SECTION("Geometric properties") {
            ConvexGeometry square{
                Vertex{0, 0},
                Vertex{1, 0},
                Vertex{1, 1},
                Vertex{0, 1},
            };

            REQUIRE(square.properties().area == 1.f);
            REQUIRE(all(square.properties().centroid == Vec2{0.f, 0.f})); // recenters automatically
            REQUIRE(approx(1.f/6, 1e-6f).contains(square.properties().momentOfArea));
        }
    }
}