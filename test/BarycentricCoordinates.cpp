#include "catch2/catch_test_macros.hpp"

#include "Simu/math/BarycentricCoordinates.hpp"

using namespace simu;


Vec2 A{0, 0};
Vec2 B{1, 0};
Vec2 C{1, 1};

Vec2 closestPoint(Vec2 Q)
{
    Vec2 posWinding = TriangleBarycentric{A, B, C, Q}.closestPoint;
    Vec2 negWinding = TriangleBarycentric{C, B, A, Q}.closestPoint;
    REQUIRE(all(posWinding == negWinding));
    return posWinding;
}

TEST_CASE("Barycentric Coordinates")
{
    SECTION("Vertex")
    {
        REQUIRE(all(closestPoint(Vec2{-1, 0}) == A));
        REQUIRE(all(closestPoint(Vec2{1, -1}) == B));
        REQUIRE(all(closestPoint(Vec2{2, 2}) == C));
    }

    SECTION("Edge")
    {
        REQUIRE(all(closestPoint(Vec2{0.5, -1}) == Vec2{0.5, 0}));
        REQUIRE(all(closestPoint(Vec2{2, 0.5}) == Vec2{1, 0.5}));
        REQUIRE(all(closestPoint(Vec2{0, 1}) == Vec2{0.5, 0.5}));
    }

    SECTION("Interior")
    {
        Vec2 inside{0.75, 0.25};
        REQUIRE(all(closestPoint(inside) == inside));
    }

    SECTION("Degenerate")
    {
        Vec2 L0{0, 0};
        Vec2 L1{1, 0};
        Vec2 L2{2, 0};

        SECTION("Degenerate line")
        {
            REQUIRE(all(LineBarycentric(L0, L0, Vec2{1, 0}).closestPoint == L0));
            REQUIRE(all(LineBarycentric(L0, L0, Vec2{1, 1}).closestPoint == L0));
            REQUIRE(all(LineBarycentric(L0, L0, Vec2{0, 1}).closestPoint == L0));
            REQUIRE(all(LineBarycentric(L0, L0, Vec2{-1, 1}).closestPoint == L0));
            REQUIRE(all(LineBarycentric(L0, L0, Vec2{-1, 0}).closestPoint == L0));
            REQUIRE(all(LineBarycentric(L0, L0, Vec2{-1, -1}).closestPoint == L0)
            );
            REQUIRE(all(LineBarycentric(L0, L0, Vec2{0, -1}).closestPoint == L0));
            REQUIRE(all(LineBarycentric(L0, L0, Vec2{1, -1}).closestPoint == L0));
        }


        auto closestPoint = [=](Vec2 Q) -> Vec2 {
            Vec2 normal  = TriangleBarycentric{L0, L1, L2, Q}.closestPoint;
            Vec2 reverse = TriangleBarycentric{L2, L1, L0, Q}.closestPoint;

            Vec2 middleOut = TriangleBarycentric{L1, L0, L2, Q}.closestPoint;
            Vec2 middleOutReverse
                = TriangleBarycentric{L0, L2, L1, Q}.closestPoint;

            Vec2 repeated = TriangleBarycentric{L0, L0, L2, Q}.closestPoint;

            REQUIRE(all(normal == reverse));
            REQUIRE(all(normal == middleOut));
            REQUIRE(all(normal == middleOutReverse));
            REQUIRE(all(normal == repeated));

            return normal;
        };

        SECTION("Degenerate triangle")
        {
            REQUIRE(all(closestPoint(Vec2{-1, 0}) == L0));
            REQUIRE(all(closestPoint(Vec2{0, 0}) == L0));
            REQUIRE(all(closestPoint(Vec2{1, 0}) == L1));
            REQUIRE(all(closestPoint(Vec2{2, 0}) == L2));
            REQUIRE(all(closestPoint(Vec2{3, 0}) == L2));

            REQUIRE(all(closestPoint(Vec2{-1, 1}) == L0));
            REQUIRE(all(closestPoint(Vec2{0, 1}) == L0));
            REQUIRE(all(closestPoint(Vec2{1, 1}) == L1));
            REQUIRE(all(closestPoint(Vec2{2, 1}) == L2));
            REQUIRE(all(closestPoint(Vec2{3, 1}) == L2));

            REQUIRE(all(closestPoint(Vec2{-1, -1}) == L0));
            REQUIRE(all(closestPoint(Vec2{0, -1}) == L0));
            REQUIRE(all(closestPoint(Vec2{1, -1}) == L1));
            REQUIRE(all(closestPoint(Vec2{2, -1}) == L2));
            REQUIRE(all(closestPoint(Vec2{3, -1}) == L2));

            REQUIRE(all(closestPoint(Vec2{0.25, 1}) == Vec2{0.25, 0}));
            REQUIRE(all(closestPoint(Vec2{0.50, 1}) == Vec2{0.50, 0}));
            REQUIRE(all(closestPoint(Vec2{0.75, 1}) == Vec2{0.75, 0}));

            REQUIRE(all(closestPoint(Vec2{1.25, -1}) == Vec2{1.25, 0}));
            REQUIRE(all(closestPoint(Vec2{1.50, -1}) == Vec2{1.50, 0}));
            REQUIRE(all(closestPoint(Vec2{1.75, -1}) == Vec2{1.75, 0}));
        }
    }

    SECTION("Precision issue")
    {
        Vec2 A{-6.00139189f, -8.94069672e-8f};
        Vec2 B{3.99860811f, -8.94069672e-8f};
        Vec2 Q{0.f, 0.f};
        REQUIRE(all(
            LineBarycentric{A, B, Q}.closestPoint == Vec2{0.f, -8.94069672e-8f}
        ));
    }
    SECTION("Precision issue")
    {
        // debugger does not show the exact value that were seen in practice...
        Vec2 A{-6.0012547f, -2.98023224e-8f};
        Vec2 B{3.99874353f, -2.98023224e-8f};
        Vec2 Q{0.f, 0.f};
        REQUIRE(all(
            LineBarycentric{A, B, Q}.closestPoint == Vec2{0.f, -2.98023224e-8f}
        ));
    }
}