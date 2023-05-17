#include "catch2/catch_test_macros.hpp"

#include "Simu/math/Polygon.hpp"
#include "Simu/math/Interval.hpp"

using namespace simu;

TEST_CASE("Geometry")
{
    SECTION("Convex polygon")
    {
        SECTION("Invalid polygon")
        {
            // not enough vertices
            REQUIRE_THROWS(Polygon{});
            REQUIRE_THROWS(Polygon{
                Vertex{0, 0}
            });
            REQUIRE_THROWS(Polygon{
                Vertex{0, 0},
                Vertex{1, 0}
            });
        }


        SECTION("Geometric properties")
        {
            Polygon square{
                Vertex{0, 0},
                Vertex{1, 0},
                Vertex{1, 1},
                Vertex{0, 1},
            };

            GeometricProperties properties{square};

            REQUIRE(properties.area == 1.f);
            REQUIRE(all(properties.centroid == Vec2{0.5f, 0.5f}));
            REQUIRE(approx(1.f / 6, 1e-6f).contains(properties.momentOfArea));

            REQUIRE(all(
                square.furthestVertexInDirection(Vec2{1, 1}) == Vertex{1.f, 1.f}
            ));
        }

        SECTION("Polygons are always positively oriented")
        {
            Polygon square{
                Vertex{0, 0},
                Vertex{0, 1},
                Vertex{1, 1},
                Vertex{1, 0},
            };

            GeometricProperties properties = square.properties();

            // vertex order is reversed to be positive
            REQUIRE(properties.area == 1.f);
            REQUIRE(all(properties.centroid == Vec2{0.5f, 0.5f}));
            REQUIRE(approx(1.f / 6, 1e-6f).contains(properties.momentOfArea));

            REQUIRE(all(
                square.furthestVertexInDirection(Vec2{1, 1}) == Vertex{1.f, 1.f}
            ));
        }

        SECTION("Degenerate polygon")
        {
            // null area
            Polygon line{
                Vertex{0, 0},
                Vertex{1, 1},
                Vertex{2, 2}
            };

            GeometricProperties properties{line};

            REQUIRE(properties.area == 0.f);

            REQUIRE(properties.isDegenerate);

            REQUIRE(all(
                line.furthestVertexInDirection(Vec2{1, 1}) == Vertex{2.f, 2.f}
            ));
        }

        SECTION("With holes")
        {
            Polygon box{
                Vertex{0, 0},

                // hole is negatively oriented
                Vertex{1, 1},
                Vertex{1, 3},
                Vertex{3, 3},
                Vertex{3, 1},
                Vertex{1, 1},

                Vertex{0, 0},
                Vertex{4, 0},
                Vertex{4, 4},
                Vertex{0, 4}
            };

            Polygon outer{
                Vertex{0, 0},
                Vertex{4, 0},
                Vertex{4, 4},
                Vertex{0, 4}
            };

            // here positively oriented
            Polygon hole{
                Vertex{1, 1},
                Vertex{3, 1},
                Vertex{3, 3},
                Vertex{1, 3},
            };

            GeometricProperties properties{box};

            REQUIRE_FALSE(properties.isDegenerate);

            REQUIRE(properties.area == outer.properties().area - hole.properties().area);

            REQUIRE(all(properties.centroid == Vec2{2.f, 2.f}));

            // this form is only true since they share the same centroid
            REQUIRE(approx(properties.momentOfArea, 1e-5f)
                    .contains(outer.properties().momentOfArea - hole.properties().momentOfArea));

            REQUIRE(all(
                box.furthestVertexInDirection(Vec2{1, 1}) == Vertex{4.f, 4.f}
            ));
        }
    }
}