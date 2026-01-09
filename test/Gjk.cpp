#include "catch2/catch_test_macros.hpp"

#include "Simu/math/Gjk.hpp"
#include "Simu/math/Polygon.hpp"

using namespace simu;

PolygonOld translated(const PolygonOld& polygon, Vec2 translation)
{
    std::vector<Vertex> vertices{polygon.begin(), polygon.end()};
    for (Vertex& v : vertices)
        v += translation;

    return PolygonOld{vertices.begin(), vertices.end()};
}

void testPenetration(const PolygonOld& first, const PolygonOld& second, Vec2 penetration)
{
    Gjk gjk{first, second};
    REQUIRE(gjk.areColliding());
    Vec2 pen = gjk.penetration();
    REQUIRE(all(pen == penetration));

    Gjk gjkInv{second, first};
    REQUIRE(gjkInv.areColliding());
    pen = gjkInv.penetration();
    REQUIRE(all(pen == -penetration));

    if (any(penetration != Vec2{0, 0}))
    {
        testPenetration(translated(first, -penetration), second, Vec2{0, 0});
        testPenetration(first, translated(second, penetration), Vec2{0, 0});
    }
}

void testSeparation(const PolygonOld& first, const PolygonOld& second, Vec2 separation)
{
    Gjk gjk{first, second};
    REQUIRE_FALSE(gjk.areColliding());
    Vec2 sep = gjk.separation();
    REQUIRE(all(sep == separation));

    Gjk gjkInv{second, first};
    REQUIRE_FALSE(gjkInv.areColliding());
    sep = gjkInv.separation();
    REQUIRE(all(sep == -separation));

    testPenetration(translated(first, separation), second, Vec2{0, 0});
    testPenetration(first, translated(second, -separation), Vec2{0, 0});
}

TEST_CASE("Gjk")
{
    SECTION("Polygon collides with itself")
    {
        PolygonOld square{
            Vertex{0, 0},
            Vertex{1, 0},
            Vertex{1, 1},
            Vertex{0, 1},
        };

        Gjk gjk{square, square};
        REQUIRE(gjk.areColliding());
    }

    SECTION("Touching is colliding")
    {
        PolygonOld lower{
            Vertex{0, 0},
            Vertex{1, 0},
            Vertex{0, 1},
        };

        PolygonOld upper{
            Vertex{1, 0},
            Vertex{1, 1},
            Vertex{0, 1},
        };

        Gjk gjk{lower, upper};
        REQUIRE(gjk.areColliding());

        Gjk gjkInv{upper, lower};
        REQUIRE(gjkInv.areColliding());
    }

    SECTION("No Collision")
    {
        PolygonOld lower{
            Vertex{0, 0},
            Vertex{1, 0},
            Vertex{0, 1},
        };

        PolygonOld upper{
            Vertex{2, 0},
            Vertex{2, 1},
            Vertex{1, 1},
        };

        Gjk gjk{lower, upper};
        REQUIRE_FALSE(gjk.areColliding());

        Gjk gjkInv{upper, lower};
        REQUIRE_FALSE(gjkInv.areColliding());
    }

    SECTION("Separation")
    {
        SECTION("Edge to Edge")
        {
            PolygonOld left{
                Vertex{0, 0},
                Vertex{1, 0},
                Vertex{1, 1},
                Vertex{0, 1}
            };

            PolygonOld right{
                Vertex{2, 0},
                Vertex{3, 0},
                Vertex{3, 1},
                Vertex{2, 1}
            };

            testSeparation(left, right, Vec2{1, 0});
        }

        SECTION("Edge to Edge (diagonals)")
        {
            PolygonOld lower{
                Vertex{0, 0},
                Vertex{1, 0},
                Vertex{0, 1},
            };

            PolygonOld upper{
                Vertex{2, 0},
                Vertex{2, 1},
                Vertex{1, 1},
            };

            testSeparation(lower, upper, Vec2{0.5, 0.5});
        }

        SECTION("Point to edge")
        {
            PolygonOld left{
                Vertex{0, 0},
                Vertex{1, 1},
                Vertex{0, 2},
            };

            PolygonOld right{
                Vertex{2, 0},
                Vertex{3, 0},
                Vertex{3, 2},
                Vertex{2, 2}
            };

            testSeparation(left, right, Vec2{1, 0});
        }

        SECTION("Point to point")
        {
            PolygonOld left{
                Vertex{0, 0},
                Vertex{1, 1},
                Vertex{0, 2},
            };

            PolygonOld right{
                Vertex{3, 0},
                Vertex{3, 2},
                Vertex{2, 1}
            };

            testSeparation(left, right, Vec2{1, 0});
        }
    }

    SECTION("Penetration")
    {
        SECTION("Edge to Edge")
        {
            PolygonOld left{
                Vertex{0, 0},
                Vertex{1, 0},
                Vertex{1, 1},
                Vertex{0, 1}
            };

            PolygonOld right{
                Vertex{0.5, 0},
                Vertex{1.5, 0},
                Vertex{1.5, 1},
                Vertex{0.5, 1}
            };

            testPenetration(left, right, Vec2{0.5, 0});
        }

        SECTION("Edge to Edge (diagonals)")
        {
            PolygonOld lower{
                Vertex{0, 0},
                Vertex{1, 0},
                Vertex{0, 1},
            };

            PolygonOld upper{
                Vertex{0.5,  0},
                Vertex{0.5,  1},
                Vertex{-0.5, 1},
            };

            testPenetration(lower, upper, Vec2{0.25, 0.25});
        }

        SECTION("Point to edge")
        {
            PolygonOld left{
                Vertex{0, 0},
                Vertex{1, 1},
                Vertex{0, 2},
            };

            PolygonOld right{
                Vertex{0.5, 0},
                Vertex{1.5, 0},
                Vertex{1.5, 2},
                Vertex{0.5, 2}
            };

            testPenetration(left, right, Vec2{0.5, 0});
        }

        SECTION("Point to point")
        {
            PolygonOld left{
                Vertex{0, 0},
                Vertex{2, 0},
                Vertex{2, 2},
                Vertex{0, 2},
            };

            PolygonOld right{
                Vertex{1, 1},
                Vertex{3, 1},
                Vertex{3, 3},
                Vertex{1, 3}
            };

            Vec2 upwards{0, 1};
            Vec2 sideways{1, 0};

            Gjk gjk{left, right};
            REQUIRE(gjk.areColliding());
            REQUIRE(
                (all(gjk.penetration() == upwards)
                 || all(gjk.penetration() == sideways))
            );

            Gjk gjkInv{right, left};
            REQUIRE(gjkInv.areColliding());
            REQUIRE(
                (all(gjkInv.penetration() == -upwards)
                 || all(gjkInv.penetration() == -sideways))
            );

            testPenetration(translated(left, -sideways), right, Vec2{0, 0});
            testPenetration(translated(left, -upwards), right, Vec2{0, 0});

            testPenetration(left, translated(right, sideways), Vec2{0, 0});
            testPenetration(left, translated(right, upwards), Vec2{0, 0});
        }
    }

    SECTION("Degenerate")
    {
        // In practice, colliders must have a surface, ie points and lines are not
        //  allowed. We can remove the handling for these special cases
        //  for better performance.

        // PolygonOld line{
        //     Vertex{1, 0},
        //     Vertex{1, 1},
        //     Vertex{1, 2},
        // };

        // testSeparation(line, translated(line, Vec2{1, 0}), Vec2{1, 0});

        // PolygonOld hline{
        //     Vertex{0, 0.5},
        //     Vertex{1, 0.5},
        //     Vertex{2, 0.5},
        // };

        // testPenetration(line, hline, Vec2{0, -0.5});

        // PolygonOld point{
        //     Vertex{0, 0},
        //     Vertex{0, 0},
        //     Vertex{0, 0}
        // };

        // testSeparation(point, translated(point, Vec2{1, 0}), Vec2{1, 0});
        // testPenetration(point, point, Vec2{0, 0});
    }
}