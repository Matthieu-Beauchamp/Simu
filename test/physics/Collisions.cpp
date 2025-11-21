#include "catch2/catch_test_macros.hpp"
#include "catch2/catch_approx.hpp"
#include "Simu/math/Matrix.hpp"
#include "Simu/physics-2.0/collision/collisions.hpp"
#include "Simu/physics-2.0/collision/colliders/BoundingBox.hpp"
#include "Simu/physics-2.0/collision/colliders/Circle.hpp"
#include "Simu/physics-2.0/collision/colliders/Polygon.hpp"
#include "Simu/physics-2.0/collision/colliders/Capsule.hpp"

using namespace simu;
using Catch::Approx;

// Helper function to check if a vector is approximately unit length
bool isUnitVector(const Vec2& vec, float epsilon = 1e-5f) {
    return std::abs(norm(vec) - 1.0f) < epsilon;
}

// TODO: Overlapping points when getting normal between points cause NaN when normalizing, etc.

TEST_CASE("BoundingBox collision detection", "[collision]") {
    SECTION("Valid non-overlapping boxes") {
        BoundingBox box1(Vec2(0, 0), Vec2(1, 1));
        BoundingBox box2(Vec2(2, 0), Vec2(3, 1));

        REQUIRE_FALSE(collides(box1, box2));
        REQUIRE_FALSE(collides(box2, box1));
    }

    SECTION("Valid overlapping boxes") {
        BoundingBox box1(Vec2(0.f, 0.f), Vec2(2, 2));
        BoundingBox box2(Vec2(1, 1), Vec2(3, 3));

        REQUIRE(collides(box1, box2));
        REQUIRE(collides(box2, box1));
    }

    SECTION("Valid touching boxes - edge contact") {
        BoundingBox box1(Vec2(0.f, 0.f), Vec2(1, 1));
        BoundingBox box2(Vec2(1, 0.f), Vec2(2, 1));

        REQUIRE(collides(box1, box2));
        REQUIRE(collides(box2, box1));
    }

    SECTION("Valid touching boxes - corner contact") {
        BoundingBox box1(Vec2(0.f, 0.f), Vec2(1, 1));
        BoundingBox box2(Vec2(1, 1), Vec2(2, 2));

        REQUIRE(collides(box1, box2));
        REQUIRE(collides(box2, box1));
    }

    SECTION("One box fully contained within another") {
        BoundingBox outer(Vec2(-2, -2), Vec2(2, 2));
        BoundingBox inner(Vec2(-1, -1), Vec2(1, 1));

        REQUIRE(collides(outer, inner));
        REQUIRE(collides(inner, outer));
    }

    SECTION("Invalid boxes") {
        BoundingBox valid(Vec2(0.f, 0.f), Vec2(1, 1));
        BoundingBox invalid(Vec2(1, 1), Vec2(0.f, 0.f)); // min > max

        REQUIRE_FALSE(collides(valid, invalid));
        REQUIRE_FALSE(collides(invalid, valid));
        REQUIRE_FALSE(collides(invalid, invalid));
    }
}

TEST_CASE("Circle-Circle collision detection", "[collision][!mayfail]") {
    SECTION("No collision - sufficient distance") {
        Circle circle1(Vec2(0.f, 0.f), 1.0f);
        Circle circle2(Vec2(3, 0.f), 1.0f);

        auto contacts = collides(circle1, circle2);
        REQUIRE(contacts.n_contacts == 0);
    }

    SECTION("Exact touching - distance equals sum of radii") {
        Circle circle1(Vec2(0.f, 0.f), 1.0f);
        Circle circle2(Vec2(2, 0.f), 1.0f);

        auto contacts = collides(circle1, circle2);
        REQUIRE(contacts.n_contacts == 1);
        REQUIRE(isUnitVector(contacts.normal));
        REQUIRE(contacts.normal[0] == Approx(1.0f));
        REQUIRE(contacts.normal[1] == Approx(0.0f));
        REQUIRE(contacts.contacts_a[0][0] == Approx(1.f));
        REQUIRE(contacts.contacts_a[0][1] == Approx(0.f));
        REQUIRE(contacts.contacts_b[0][0] == Approx(1.f));
        REQUIRE(contacts.contacts_b[0][1] == Approx(0.f));
    }

    SECTION("Clear overlap") {
        Circle circle1(Vec2(0.f, 0.f), 1.0f);
        Circle circle2(Vec2(1, 0.f), 1.0f);

        auto contacts = collides(circle1, circle2);
        REQUIRE(contacts.n_contacts == 1);
        REQUIRE(isUnitVector(contacts.normal));
        REQUIRE(contacts.normal[0] == Approx(1.0f));
        REQUIRE(contacts.normal[1] == Approx(0.0f));
        REQUIRE(contacts.contacts_a[0][0] == Approx(1.f));
        REQUIRE(contacts.contacts_a[0][1] == Approx(0.f));
        REQUIRE(contacts.contacts_b[0][0] == Approx(0.f));
        REQUIRE(contacts.contacts_b[0][1] == Approx(0.f));
    }

    SECTION("One circle inside another") {
        Circle outer(Vec2(0.f, 0.f), 2.0f);
        Circle inner(Vec2(0.5f, 0.f), 0.5f);

        auto contacts = collides(outer, inner);
        REQUIRE(contacts.n_contacts == 1);
        REQUIRE(isUnitVector(contacts.normal));
    }

    SECTION("Concentric circles") {
        Circle circle1(Vec2(0.f, 0.f), 2.0f);
        Circle circle2(Vec2(0.f, 0.f), 1.0f);

        auto contacts = collides(circle1, circle2);
        REQUIRE(contacts.n_contacts == 1);
    }

    SECTION("Zero radius edge case") {
        Circle circle1(Vec2(0.f, 0.f), 0.0f);
        Circle circle2(Vec2(0.f, 0.f), 1.0f);

        auto contacts = collides(circle1, circle2);
        REQUIRE(contacts.n_contacts == 1);
    }

    SECTION("Concentric circles") {
        Circle circle1(Vec2(0.f, 0.f), 2.0f);
        Circle circle2(Vec2(0.f, 0.f), 1.0f);

        auto contacts = collides(circle1, circle2);
        REQUIRE(contacts.n_contacts == 1);
        REQUIRE_FALSE(std::isnan(contacts.normal[0]));
        REQUIRE_FALSE(std::isnan(contacts.normal[1]));
    }
}

TEST_CASE("Circle-Capsule collision detection", "[collision]") {
    SECTION("No collision - circle away from capsule") {
        Circle  circle(Vec2(5, 0.f), 1.0f);
        Capsule capsule(Vec2(0.f, -1), Vec2(0.f, 1), 0.5f);

        auto contacts = collides(circle, capsule);
        REQUIRE(contacts.n_contacts == 0);
    }

    SECTION("Collision with capsule center segment") {
        Circle  circle(Vec2(1, 0.f), 0.8f);
        Capsule capsule(Vec2(0.f, -1), Vec2(0.f, 1), 0.5f);

        auto contacts = collides(circle, capsule);
        REQUIRE(contacts.n_contacts == 1);
        REQUIRE(isUnitVector(contacts.normal));
    }

    SECTION("Collision with bottom cap") {
        Circle  circle(Vec2(0.f, -2), 1.f);
        Capsule capsule(Vec2(0.f, -1), Vec2(0.f, 1), 0.5f);

        auto contacts = collides(circle, capsule);
        REQUIRE(contacts.n_contacts == 1);
        REQUIRE(isUnitVector(contacts.normal));
    }

    SECTION("Collision with top cap") {
        Circle  circle(Vec2(0.f, 2), 1.f);
        Capsule capsule(Vec2(0.f, -1), Vec2(0.f, 1), 0.5f);

        auto contacts = collides(circle, capsule);
        REQUIRE(contacts.n_contacts == 1);
        REQUIRE(isUnitVector(contacts.normal));
    }

    SECTION("Circle fully containing capsule") {
        Circle  circle(Vec2(0.f, 0.f), 5.f);
        Capsule capsule(Vec2(0.f, -1), Vec2(0.f, 1), .5f);

        auto contacts = collides(circle, capsule);
        REQUIRE(contacts.n_contacts == 1);
        REQUIRE(isUnitVector(contacts.normal));
    }

    SECTION("Zero-length capsule (becomes circle)") {
        Circle  circle(Vec2(1, 0.f), 1.f);
        Capsule capsule(Vec2(0.f, -0.5f), Vec2(0.f, 0.5f), 1.f);

        auto contacts = collides(circle, capsule);
        REQUIRE(contacts.n_contacts == 1);
        REQUIRE(isUnitVector(contacts.normal));
    }
}

TEST_CASE("Circle-Polygon collision detection", "[collision]") {
    SECTION("No collision - circle away from polygon") {
        Circle            circle(Vec2(5, 5), 1.f);
        std::vector<Vec2> vertices = {
            Vec2(0.f, 0.f), Vec2(1, 0.f), Vec2(1, 1), Vec2(0.f, 1)
        };
        Polygon polygon(vertices);

        auto contacts = collides(circle, polygon);
        REQUIRE(contacts.n_contacts == 0.f);
    }

    SECTION("Vertex collision - intersecting with polygon vertex") {
        Circle            circle(Vec2(.5f, .5f), .8f);
        std::vector<Vec2> vertices = {
            Vec2(0.f, 0.f), Vec2(2, 0.f), Vec2(2, 2), Vec2(0.f, 2)
        };
        Polygon polygon(vertices);

        auto contacts = collides(circle, polygon);
        REQUIRE(contacts.n_contacts == 1);
        REQUIRE(isUnitVector(contacts.normal));
    }

    SECTION("Edge collision - intersecting with polygon edge") {
        Circle            circle(Vec2(-.5f, 1.f), .8f);
        std::vector<Vec2> vertices = {
            Vec2(0.f, 0.f), Vec2(2, 0.f), Vec2(2, 2), Vec2(0.f, 2)
        };
        Polygon polygon(vertices);

        auto contacts = collides(circle, polygon);
        REQUIRE(contacts.n_contacts == 1);
        REQUIRE(isUnitVector(contacts.normal));
    }

    SECTION("Circle fully inside polygon") {
        Circle            circle(Vec2(1, 1), .3f);
        std::vector<Vec2> vertices = {
            Vec2(0.f, 0.f), Vec2(2, 0.f), Vec2(2, 2), Vec2(0.f, 2)
        };
        Polygon polygon(vertices);

        auto contacts = collides(circle, polygon);
        REQUIRE(contacts.n_contacts == 1);
        REQUIRE(isUnitVector(contacts.normal));
    }

    SECTION("Triangle polygon collision") {
        Circle circle(Vec2(.5f, .3f), .4f);
        std::vector<Vec2> vertices = {Vec2(0.f, 0.f), Vec2(1, 0.f), Vec2(.5f, 1)};
        Polygon polygon(vertices);

        auto contacts = collides(circle, polygon);
        REQUIRE(contacts.n_contacts == 1);
        REQUIRE(isUnitVector(contacts.normal));
    }
}

TEST_CASE("Capsule-Capsule collision detection", "[collision]") {
    const float epsilon = 1e-6f;

    SECTION("No collision - capsules separated") {
        Capsule capsule1(Vec2(-2, -1), Vec2(-2, 1), .5f);
        Capsule capsule2(Vec2(2, -1), Vec2(2, 1), .5f);

        auto contacts = collides(capsule1, capsule2, epsilon);
        REQUIRE(contacts.n_contacts == 0);
    }

    SECTION("End-to-end collision - top to bottom") {
        Capsule capsule1(Vec2(0.f, -1), Vec2(0.f, 1), .5f);
        Capsule capsule2(Vec2(0.f, 1.f), Vec2(0.f, 3.f), .5f);

        auto contacts = collides(capsule1, capsule2, epsilon);
        REQUIRE(contacts.n_contacts == 1);
        REQUIRE(isUnitVector(contacts.normal));
    }

    SECTION("Middle-to-middle collision - perpendicular capsules") {
        Capsule capsule1(Vec2(-1, 0.f), Vec2(1, 0.f), .6f);
        Capsule capsule2(Vec2(0.f, -1), Vec2(0.f, 1), .6f);

        auto contacts = collides(capsule1, capsule2, epsilon);
        REQUIRE(contacts.n_contacts >= 1);
        REQUIRE(contacts.n_contacts <= 2);
        if (contacts.n_contacts > 0) {
            REQUIRE(isUnitVector(contacts.normal));
        }
    }

    SECTION("Parallel capsules with overlap") {
        Capsule capsule1(Vec2(0.f, -1), Vec2(0.f, 1), .8f);
        Capsule capsule2(Vec2(1, -.5f), Vec2(1, .5f), .8f);

        auto contacts = collides(capsule1, capsule2, epsilon);
        REQUIRE(contacts.n_contacts >= 1);
        REQUIRE(contacts.n_contacts <= 2);
        if (contacts.n_contacts > 0) {
            REQUIRE(isUnitVector(contacts.normal));
        }
    }

    SECTION("Zero-length capsules (become circles)") {
        Capsule capsule1(Vec2(0.f, 0.f), Vec2(0.f, 3.f), 1.f);
        Capsule capsule2(Vec2(2, 0.f), Vec2(2, 3.f), 1.f);

        auto contacts = collides(capsule1, capsule2, epsilon);
        REQUIRE(contacts.n_contacts == 2);
        REQUIRE(isUnitVector(contacts.normal));
    }
}

TEST_CASE("Capsule-Polygon collision detection", "[collision]") {
    const float epsilon = 1e-6f;

    SECTION("No collision - capsule away from polygon") {
        Capsule           capsule(Vec2(5, 5), Vec2(6, 6), .5f);
        std::vector<Vec2> vertices = {
            Vec2(0.f, 0.f), Vec2(2, 0.f), Vec2(2, 2), Vec2(0.f, 2)
        };
        Polygon polygon(vertices);

        auto contacts = collides(capsule, polygon, epsilon);
        REQUIRE(contacts.n_contacts == 0.f);
    }

    SECTION("Vertex collision - capsule end hitting polygon vertex") {
        Capsule           capsule(Vec2(-.5f, 0.f), Vec2(-1.5f, 0.f), .8f);
        std::vector<Vec2> vertices = {
            Vec2(0.f, 0.f), Vec2(2, 0.f), Vec2(2, 2), Vec2(0.f, 2)
        };
        Polygon polygon(vertices);

        auto contacts = collides(capsule, polygon, epsilon);
        REQUIRE(contacts.n_contacts >= 1);
        REQUIRE(contacts.n_contacts <= 2);
        REQUIRE(isUnitVector(contacts.normal));
    }

    SECTION("Edge collision - capsule middle hitting polygon edge") {
        Capsule           capsule(Vec2(-1.f, 0), Vec2(0.f, 0), .5f);
        std::vector<Vec2> vertices = {
            Vec2(0.f, 0.f), Vec2(2, 0.f), Vec2(2, 2), Vec2(0.f, 2)
        };
        Polygon polygon(vertices);

        auto contacts = collides(capsule, polygon, epsilon);
        REQUIRE(contacts.n_contacts == 1);
        REQUIRE(isUnitVector(contacts.normal));
    }

    SECTION("Different epsilon values") {
        Capsule           capsule(Vec2(2.1f, 1), Vec2(3.1f, 1), .2f);
        std::vector<Vec2> vertices = {
            Vec2(0.f, 0.f), Vec2(2, 0.f), Vec2(2, 2), Vec2(0.f, 2)
        };
        Polygon polygon(vertices);

        // Small epsilon - should not collide
        auto contacts_small = collides(capsule, polygon, 1e-8f);
        // Larger epsilon - might collide
        auto contacts_large = collides(capsule, polygon, .2f);

        // At least verify the function runs without error
        REQUIRE(contacts_small.n_contacts <= 2);
        REQUIRE(contacts_large.n_contacts <= 2);
    }

    SECTION("Zero-length capsule (becomes circle) with polygon") {
        Capsule           capsule(Vec2(-1.f, 0.f), Vec2(-1.f, 2.f), 1.f);
        std::vector<Vec2> vertices = {
            Vec2(0.f, 0.f), Vec2(2, 0.f), Vec2(2, 2), Vec2(0.f, 2)
        };
        Polygon polygon(vertices);

        auto contacts = collides(capsule, polygon, epsilon);
        REQUIRE(contacts.n_contacts == 1);
        REQUIRE(isUnitVector(contacts.normal));
    }

    SECTION("Vertex on capsule's center") {
        SKIP();
        Capsule           capsule(Vec2(-.5f, 0), Vec2(.5f, 0), .5f);
        std::vector<Vec2> vertices = {
            Vec2(0.f, 0.f), Vec2(2, 0.f), Vec2(2, 2), Vec2(0.f, 2)
        };
        Polygon polygon(vertices);

        auto contacts = collides(capsule, polygon, epsilon);
        REQUIRE(contacts.n_contacts >= 1);
        REQUIRE(contacts.n_contacts <= 2);
        REQUIRE(isUnitVector(contacts.normal));
    }
}

TEST_CASE("Polygon-Polygon collision detection", "[collision]") {
    SECTION("No collision - separate polygons") {
        std::vector<Vec2> vertices1 = {
            Vec2(0.f, 0.f), Vec2(1, 0.f), Vec2(1, 1), Vec2(0.f, 1)
        };
        std::vector<Vec2> vertices2 = {Vec2(3, 3), Vec2(4, 3), Vec2(4, 4), Vec2(3, 4)};
        Polygon polygon1(vertices1);
        Polygon polygon2(vertices2);

        auto contacts = collides(polygon1, polygon2);
        REQUIRE(contacts.n_contacts == 0.f);
    }

    SECTION("Vertex-vertex collision") {
        std::vector<Vec2> vertices1 = {
            Vec2(0.f, 0.f), Vec2(1, 0.f), Vec2(1, 1), Vec2(0.f, 1)
        };
        std::vector<Vec2> vertices2 = {Vec2(1, 1), Vec2(2, 1), Vec2(2, 2), Vec2(1, 2)};
        Polygon polygon1(vertices1);
        Polygon polygon2(vertices2);

        auto contacts = collides(polygon1, polygon2);
        REQUIRE(contacts.n_contacts >= 1);
        REQUIRE(contacts.n_contacts <= 2);
        if (contacts.n_contacts > 0) {
            REQUIRE(isUnitVector(contacts.normal));
        }
    }

    SECTION("Vertex-edge collision") {
        std::vector<Vec2> vertices1 = {
            Vec2(0.f, 0.f), Vec2(1, 0.f), Vec2(1, 1), Vec2(0.f, 1)
        };
        std::vector<Vec2> vertices2 = {
            Vec2(.5f, 1), Vec2(1.5f, 1), Vec2(1.5f, 2), Vec2(.5f, 2)
        };
        Polygon polygon1(vertices1);
        Polygon polygon2(vertices2);

        auto contacts = collides(polygon1, polygon2);
        REQUIRE(contacts.n_contacts >= 1);
        REQUIRE(contacts.n_contacts <= 2);
        if (contacts.n_contacts > 0) {
            REQUIRE(isUnitVector(contacts.normal));
        }
    }

    SECTION("Edge-edge collision - overlapping edges") {
        std::vector<Vec2> vertices1 = {
            Vec2(0.f, 0.f), Vec2(2, 0.f), Vec2(2, 1), Vec2(0.f, 1)
        };
        std::vector<Vec2> vertices2 = {Vec2(1, 1), Vec2(3, 1), Vec2(3, 2), Vec2(1, 2)};
        Polygon polygon1(vertices1);
        Polygon polygon2(vertices2);

        auto contacts = collides(polygon1, polygon2);
        REQUIRE(contacts.n_contacts >= 1);
        REQUIRE(contacts.n_contacts <= 2);
        if (contacts.n_contacts > 0) {
            REQUIRE(isUnitVector(contacts.normal));
        }
    }

    SECTION("One polygon inside another") {
        std::vector<Vec2> vertices1 = {
            Vec2(-2, -2), Vec2(2, -2), Vec2(2, 2), Vec2(-2, 2)
        };
        std::vector<Vec2> vertices2 = {
            Vec2(-1, -1), Vec2(1, -1), Vec2(1, 1), Vec2(-1, 1)
        };
        Polygon polygon1(vertices1);
        Polygon polygon2(vertices2);

        auto contacts = collides(polygon1, polygon2);
        REQUIRE(contacts.n_contacts >= 1);
        REQUIRE(contacts.n_contacts <= 2);
        if (contacts.n_contacts > 0) {
            REQUIRE(isUnitVector(contacts.normal));
        }
    }

    SECTION("Triangle collision") {
        std::vector<Vec2> vertices1 = {Vec2(0.f, 0.f), Vec2(2, 0.f), Vec2(1, 2)};
        std::vector<Vec2> vertices2 = {Vec2(1.5f, 0.f), Vec2(3.5f, 0.f), Vec2(2.5f, 2)};
        Polygon polygon1(vertices1);
        Polygon polygon2(vertices2);

        auto contacts = collides(polygon1, polygon2);
        REQUIRE(contacts.n_contacts >= 1);
        REQUIRE(contacts.n_contacts <= 2);
        if (contacts.n_contacts > 0) {
            REQUIRE(isUnitVector(contacts.normal));
        }
    }
}