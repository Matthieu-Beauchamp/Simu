#include "catch2/catch_test_macros.hpp"

#include "Simu/math/Polygon.hpp"
#include "Simu/math/GeometricProperties.hpp"
#include "Simu/math/Interval.hpp"

using namespace simu;

TEST_CASE("Geometry")
{
    SECTION("Convex polygon")
    {
        SECTION("Invalid polygon")
        {
            // not enough vertices
            REQUIRE_THROWS(ConvexPolygon{});
            REQUIRE_THROWS(ConvexPolygon{
                Vertex{0, 0}
            });
            REQUIRE_THROWS(ConvexPolygon{
                Vertex{0, 0},
                Vertex{1, 0}
            });

            // null area
            REQUIRE_THROWS(ConvexPolygon{
                Vertex{0, 0},
                Vertex{1, 1},
                Vertex{2, 2}
            });

            // negative winding order
            REQUIRE_THROWS(ConvexPolygon{
                Vertex{0, 0},
                Vertex{0, 1},
                Vertex{1, 0}
            });
        }


        SECTION("Geometric properties") {
            ConvexPolygon square{
                Vertex{0, 0},
                Vertex{1, 0},
                Vertex{1, 1},
                Vertex{0, 1},
            };

            GeometricProperties properties{square};

            REQUIRE(properties.area == 1.f);
            REQUIRE(all(properties.centroid == Vec2{0.5f, 0.5f}));
            REQUIRE(approx(1.f/6, 1e-6f).contains(properties.momentOfArea));
        
            REQUIRE(all(square.furthestVertexInDirection(Vec2{1, 1}) == Vertex{1.f, 1.f}));
        }
    }
}