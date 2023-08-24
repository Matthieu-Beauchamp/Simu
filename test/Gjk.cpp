#include "catch2/catch_test_macros.hpp"

#include "Simu/math/Gjk.hpp"
#include "Simu/math/Polygon.hpp"

using namespace simu;

Polygon translated(const Polygon& polygon, Vec2 translation)
{
    std::vector<Vec2> vertices{polygon.begin(), polygon.end()};
    for (Vec2& v : vertices)
        v += translation;

    return Polygon{vertices.begin(), vertices.end()};
}

void testPenetration(const Polygon& first, const Polygon& second, Vec2 penetration)
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

void testSeparation(const Polygon& first, const Polygon& second, Vec2 separation)
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
        Polygon square{
            Vec2{0, 0},
            Vec2{1, 0},
            Vec2{1, 1},
            Vec2{0, 1},
        };

        Gjk gjk{square, square};
        REQUIRE(gjk.areColliding());
    }

    SECTION("Touching is colliding")
    {
        Polygon lower{
            Vec2{0, 0},
            Vec2{1, 0},
            Vec2{0, 1},
        };

        Polygon upper{
            Vec2{1, 0},
            Vec2{1, 1},
            Vec2{0, 1},
        };

        Gjk gjk{lower, upper};
        REQUIRE(gjk.areColliding());

        Gjk gjkInv{upper, lower};
        REQUIRE(gjkInv.areColliding());
    }

    SECTION("No Collision")
    {
        Polygon lower{
            Vec2{0, 0},
            Vec2{1, 0},
            Vec2{0, 1},
        };

        Polygon upper{
            Vec2{2, 0},
            Vec2{2, 1},
            Vec2{1, 1},
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
            Polygon left{
                Vec2{0, 0},
                Vec2{1, 0},
                Vec2{1, 1},
                Vec2{0, 1}
            };

            Polygon right{
                Vec2{2, 0},
                Vec2{3, 0},
                Vec2{3, 1},
                Vec2{2, 1}
            };

            testSeparation(left, right, Vec2{1, 0});
        }

        SECTION("Edge to Edge (diagonals)")
        {
            Polygon lower{
                Vec2{0, 0},
                Vec2{1, 0},
                Vec2{0, 1},
            };

            Polygon upper{
                Vec2{2, 0},
                Vec2{2, 1},
                Vec2{1, 1},
            };

            testSeparation(lower, upper, Vec2{0.5, 0.5});
        }

        SECTION("Point to edge")
        {
            Polygon left{
                Vec2{0, 0},
                Vec2{1, 1},
                Vec2{0, 2},
            };

            Polygon right{
                Vec2{2, 0},
                Vec2{3, 0},
                Vec2{3, 2},
                Vec2{2, 2}
            };

            testSeparation(left, right, Vec2{1, 0});
        }

        SECTION("Point to point")
        {
            Polygon left{
                Vec2{0, 0},
                Vec2{1, 1},
                Vec2{0, 2},
            };

            Polygon right{
                Vec2{3, 0},
                Vec2{3, 2},
                Vec2{2, 1}
            };

            testSeparation(left, right, Vec2{1, 0});
        }
    }

    SECTION("Penetration")
    {
        SECTION("Edge to Edge")
        {
            Polygon left{
                Vec2{0, 0},
                Vec2{1, 0},
                Vec2{1, 1},
                Vec2{0, 1}
            };

            Polygon right{
                Vec2{0.5, 0},
                Vec2{1.5, 0},
                Vec2{1.5, 1},
                Vec2{0.5, 1}
            };

            testPenetration(left, right, Vec2{0.5, 0});
        }

        SECTION("Edge to Edge (diagonals)")
        {
            Polygon lower{
                Vec2{0, 0},
                Vec2{1, 0},
                Vec2{0, 1},
            };

            Polygon upper{
                Vec2{0.5,  0},
                Vec2{0.5,  1},
                Vec2{-0.5, 1},
            };

            testPenetration(lower, upper, Vec2{0.25, 0.25});
        }

        SECTION("Point to edge")
        {
            Polygon left{
                Vec2{0, 0},
                Vec2{1, 1},
                Vec2{0, 2},
            };

            Polygon right{
                Vec2{0.5, 0},
                Vec2{1.5, 0},
                Vec2{1.5, 2},
                Vec2{0.5, 2}
            };

            testPenetration(left, right, Vec2{0.5, 0});
        }

        SECTION("Point to point")
        {
            Polygon left{
                Vec2{0, 0},
                Vec2{2, 0},
                Vec2{2, 2},
                Vec2{0, 2},
            };

            Polygon right{
                Vec2{1, 1},
                Vec2{3, 1},
                Vec2{3, 3},
                Vec2{1, 3}
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

        // Polygon line{
        //     Vec2{1, 0},
        //     Vec2{1, 1},
        //     Vec2{1, 2},
        // };

        // testSeparation(line, translated(line, Vec2{1, 0}), Vec2{1, 0});

        // Polygon hline{
        //     Vec2{0, 0.5},
        //     Vec2{1, 0.5},
        //     Vec2{2, 0.5},
        // };

        // testPenetration(line, hline, Vec2{0, -0.5});

        // Polygon point{
        //     Vec2{0, 0},
        //     Vec2{0, 0},
        //     Vec2{0, 0}
        // };

        // testSeparation(point, translated(point, Vec2{1, 0}), Vec2{1, 0});
        // testPenetration(point, point, Vec2{0, 0});
    }
}