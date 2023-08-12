#include "catch2/catch_test_macros.hpp"

#include "Simu/physics/BoundingBox.hpp"

using namespace simu;

BoundingBox translated(const BoundingBox& box, Vec2 dir)
{
    return BoundingBox{box.min() + dir, box.max() + dir};
}

void testOverlap(const BoundingBox& lhs, const BoundingBox& rhs)
{
    REQUIRE(lhs.overlaps(rhs));
    REQUIRE(rhs.overlaps(lhs));
}

void testNoOverlap(const BoundingBox& lhs, const BoundingBox& rhs)
{
    REQUIRE_FALSE(lhs.overlaps(rhs));
    REQUIRE_FALSE(rhs.overlaps(lhs));
}

TEST_CASE("Bounding Box")
{
    SECTION("BoundingBox validity")
    {
        // empty bounds
        REQUIRE_FALSE(BoundingBox{}.isValid());

        // single point
        REQUIRE(BoundingBox{Vec2{}, Vec2{}}.isValid());

        // normal bounds
        REQUIRE(BoundingBox{
            Vec2{0, 0},
            Vec2{1, 1}
        }
                    .isValid());
    }

    SECTION("Basic")
    {
        BoundingBox a{
            Vec2{0, 0},
            Vec2{1, 1}
        };

        REQUIRE(a.overlaps(a));

        testOverlap(a, translated(a, Vec2{0.5, 0}));
        testOverlap(a, translated(a, Vec2{-0.5, 0}));
        testOverlap(a, translated(a, Vec2{0, 0.5}));
        testOverlap(a, translated(a, Vec2{0, -0.5}));

        testOverlap(a, translated(a, Vec2{0.5, 0.5}));
        testOverlap(a, translated(a, Vec2{-0.5, 0.5}));
        testOverlap(a, translated(a, Vec2{-0.5, -0.5}));
        testOverlap(a, translated(a, Vec2{0.5, -0.5}));

        // touching is overlapping
        testOverlap(a, translated(a, Vec2{1, 0}));
        testOverlap(a, translated(a, Vec2{-1, 0}));
        testOverlap(a, translated(a, Vec2{0, 1}));
        testOverlap(a, translated(a, Vec2{0, -1}));

        testOverlap(a, translated(a, Vec2{1, 1}));
        testOverlap(a, translated(a, Vec2{-1, 1}));
        testOverlap(a, translated(a, Vec2{-1, -1}));
        testOverlap(a, translated(a, Vec2{1, -1}));
    }

    SECTION("Fully contained")
    {
        BoundingBox a{
            Vec2{0, 0},
            Vec2{1, 1}
        };

        BoundingBox b{
            Vec2{0.25, 0.25},
            Vec2{0.75, 0.75}
        };

        testOverlap(a, b);
    }

    SECTION("No contact")
    {
        BoundingBox a{
            Vec2{0, 0},
            Vec2{1, 1}
        };

        testNoOverlap(a, translated(a, Vec2{1.1f, 0}));
        testNoOverlap(a, translated(a, Vec2{-1.1f, 0}));
        testNoOverlap(a, translated(a, Vec2{0, 1.1f}));
        testNoOverlap(a, translated(a, Vec2{0, -1.1f}));

        testNoOverlap(a, translated(a, Vec2{1.1f, 1.1f}));
        testNoOverlap(a, translated(a, Vec2{-1.1f, 1.1f}));
        testNoOverlap(a, translated(a, Vec2{-1.1f, -1.1f}));
        testNoOverlap(a, translated(a, Vec2{1.1f, -1.1f}));
    }

    SECTION("Overlap edge cases"){
        BoundingBox empty{};
        BoundingBox origin{Vec2{}, Vec2{}};
        BoundingBox box{
            Vec2{0, 0},
            Vec2{1, 1}
        };

        testNoOverlap(empty, origin);
        testNoOverlap(empty, box);

        testOverlap(origin, box);
    }

    SECTION("From geometry and point boxes")
    {
        Vertices vertices{
            Vertex{0,   0.5},
            Vertex{0.5, 0  },
            Vertex{1,   0.5},
            Vertex{0.5, 1  },
        };

        BoundingBox box{vertices};

        REQUIRE(all(box.min() == Vec2{0, 0}));
        REQUIRE(all(box.max() == Vec2{1, 1}));

        for (const Vertex& v : vertices)
        {
            testOverlap(box, BoundingBox{v, v});
        }
    }

    SECTION("Combinations")
    {
        BoundingBox empty{Vec2{1, 0}, Vec2{}};
        BoundingBox emptyDifferent{Vec2{1, 1}, Vec2{}};
        BoundingBox origin{Vec2{}, Vec2{}};
        BoundingBox box{
            Vec2{1, 1},
            Vec2{2, 2}
        };

        REQUIRE(empty.combined(emptyDifferent) == empty);
        REQUIRE(empty.combined(emptyDifferent) == emptyDifferent);
        REQUIRE(empty.combined(origin) == origin);
        REQUIRE(empty.combined(box) == box);

        REQUIRE(origin.combined(box) == BoundingBox{Vec2{}, Vec2{2, 2}});
    }
}