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
                Vec2{0, 0}
            });
            REQUIRE_THROWS(Polygon{
                Vec2{0, 0},
                Vec2{1, 0}
            });
        }


        SECTION("Geometric properties")
        {
            Polygon square{
                Vec2{0, 0},
                Vec2{1, 0},
                Vec2{1, 1},
                Vec2{0, 1},
            };

            Shape::Properties properties = square.properties();


            REQUIRE(properties.area == 1.f);
            REQUIRE(all(properties.centroid == Vec2{0.5f, 0.5f}));
            REQUIRE(approx(1.f / 6, 1e-6f).contains(properties.momentOfArea));

            REQUIRE(all(
                *furthestVertexInDirection(square, Vec2{1, 1}) == Vec2{1.f, 1.f}
            ));
        }

        SECTION("Polygons are always positively oriented")
        {
            Polygon square{
                Vec2{0, 0},
                Vec2{0, 1},
                Vec2{1, 1},
                Vec2{1, 0},
            };

            Shape::Properties properties = square.properties();

            // vertex order is reversed to be positive
            REQUIRE(properties.area == 1.f);
            REQUIRE(all(properties.centroid == Vec2{0.5f, 0.5f}));
            REQUIRE(approx(1.f / 6, 1e-6f).contains(properties.momentOfArea));

            REQUIRE(all(
                *furthestVertexInDirection(square, Vec2{1, 1}) == Vec2{1.f, 1.f}
            ));
        }

        SECTION("Degenerate polygon")
        {
            // null area
            Polygon line{
                Vec2{0, 0},
                Vec2{1, 1},
                Vec2{2, 2}
            };

            Shape::Properties properties = line.properties();

            REQUIRE(properties.area == 0.f);

            REQUIRE(properties.isDegenerate);

            REQUIRE(
                all(*furthestVertexInDirection(line, Vec2{1, 1}) == Vec2{2.f, 2.f})
            );
        }

        SECTION("With holes")
        {
            Polygon box{
                Vec2{0, 0},

 // hole is negatively oriented
                Vec2{1, 1},
                Vec2{1, 3},
                Vec2{3, 3},
                Vec2{3, 1},
                Vec2{1, 1},

                Vec2{0, 0},
                Vec2{4, 0},
                Vec2{4, 4},
                Vec2{0, 4}
            };

            Polygon outer{
                Vec2{0, 0},
                Vec2{4, 0},
                Vec2{4, 4},
                Vec2{0, 4}
            };

            // here positively oriented
            Polygon hole{
                Vec2{1, 1},
                Vec2{3, 1},
                Vec2{3, 3},
                Vec2{1, 3},
            };

            Shape::Properties properties = box.properties();

            REQUIRE_FALSE(properties.isDegenerate);

            REQUIRE(
                properties.area == outer.properties().area - hole.properties().area
            );

            REQUIRE(all(properties.centroid == Vec2{2.f, 2.f}));

            // this form is only true since they share the same centroid
            REQUIRE(approx(properties.momentOfArea, 1e-5f)
                        .contains(
                            outer.properties().momentOfArea - hole.properties().momentOfArea
                        ));

            REQUIRE(
                all(*furthestVertexInDirection(box, Vec2{1, 1}) == Vec2{4.f, 4.f})
            );
        }
    }
}