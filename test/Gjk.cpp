#include "catch2/catch_test_macros.hpp"

#include "Simu/math/Gjk.hpp"
#include "Simu/math/Polygon.hpp"

using namespace simu;

Polygon translated(const Polygon& polygon, Vec2 translation)
{
    std::vector<Vertex> vertices{polygon.begin(), polygon.end()};
    for (Vertex& v : vertices)
        v += translation;

    return Polygon{vertices.begin(), vertices.end()};
}

void testPenetration(const Polygon& first, const Polygon& second, Vec2 penetration)
{
    Gjk gjk{first, second};
    REQUIRE(gjk.areColliding());
    REQUIRE(all(gjk.penetration() == penetration));

    Gjk gjkInv{second, first};
    REQUIRE(gjkInv.areColliding());
    REQUIRE(all(gjkInv.penetration() == -penetration));

    if (all(penetration != Vec2{0, 0}))
    {
        testPenetration(translated(first, penetration), second, Vec2{0, 0});
        testPenetration(first, translated(second, -penetration), Vec2{0, 0});
    }
}

void testSeparation(const Polygon& first, const Polygon& second, Vec2 separation)
{
    Gjk gjk{first, second};
    REQUIRE_FALSE(gjk.areColliding());
    REQUIRE(all(gjk.separation() == separation));

    Gjk gjkInv{second, first};
    REQUIRE_FALSE(gjkInv.areColliding());
    REQUIRE(all(gjkInv.separation() == -separation));

    testPenetration(translated(first, separation), second, Vec2{0, 0});
    testPenetration(first, translated(second, -separation), Vec2{0, 0});
}

TEST_CASE("Gjk")
{
    SECTION("Polygon collides with itself")
    {
        Polygon square{
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
        Polygon lower{
            Vertex{0, 0},
            Vertex{1, 0},
            Vertex{0, 1},
        };

        Polygon upper{
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
        Polygon lower{
            Vertex{0, 0},
            Vertex{1, 0},
            Vertex{0, 1},
        };

        Polygon upper{
            Vertex{2, 0},
            Vertex{2, 1},
            Vertex{1, 1},
        };

        Gjk gjk{lower, upper};
        REQUIRE_FALSE(gjk.areColliding());

        Gjk gjkInv{upper, lower};
        REQUIRE_FALSE(gjkInv.areColliding());
    }

    SECTION("Distance")
    {
        SECTION("Edge to Edge")
        {
            Polygon left{
                Vertex{0, 0},
                Vertex{1, 0},
                Vertex{1, 1},
                Vertex{0, 1}
            };

            Polygon right{
                Vertex{2, 0},
                Vertex{3, 0},
                Vertex{3, 1},
                Vertex{2, 1}
            };

            testSeparation(left, right, Vec2{1, 0});
        }

        SECTION("Edge to Edge (diagonals)")
        {
            Polygon lower{
                Vertex{0, 0},
                Vertex{1, 0},
                Vertex{0, 1},
            };

            Polygon upper{
                Vertex{2, 0},
                Vertex{2, 1},
                Vertex{1, 1},
            };

            testSeparation(lower, upper, Vec2{0.5, 0.5});
        }

        SECTION("Point to edge")
        {
            Polygon left{
                Vertex{0, 0},
                Vertex{1, 1},
                Vertex{0, 2},
            };

            Polygon right{
                Vertex{2, 0},
                Vertex{3, 0},
                Vertex{3, 2},
                Vertex{2, 2}
            };

            testSeparation(left, right, Vec2{1, 0});
        }

        SECTION("Point to point")
        {
            Polygon left{
                Vertex{0, 0},
                Vertex{1, 1},
                Vertex{0, 2},
            };

            Polygon right{
                Vertex{3, 0},
                Vertex{3, 2},
                Vertex{2, 1}
            };

            testSeparation(left, right, Vec2{1, 0});
        }
    }
}