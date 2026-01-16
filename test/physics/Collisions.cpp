#include "catch2/catch_test_macros.hpp"
#include "catch2/catch_approx.hpp"
#include "Simu/math/Matrix.hpp"
#include "Simu/physics-2.0/collision.hpp"
#include "../matchers/IsUnitVector.hpp"
#include "../matchers/IsApprox.h"

using namespace simu;
using Catch::Approx;

// TODO: Overlapping points when getting normal between points cause NaN when normalizing, etc.

TEST_CASE("BoundingBox collision detection", "[collision]") {
    SECTION("Valid non-overlapping boxes") {
        BoundingBox box1(Vec2(0, 0), Vec2(1, 1));
        BoundingBox box2(Vec2(2, 0), Vec2(3, 1));

        REQUIRE_FALSE(collide(box1, box2));
        REQUIRE_FALSE(collide(box2, box1));
    }

    SECTION("Valid overlapping boxes") {
        BoundingBox box1(Vec2(0.f, 0.f), Vec2(2, 2));
        BoundingBox box2(Vec2(1, 1), Vec2(3, 3));

        REQUIRE(collide(box1, box2));
        REQUIRE(collide(box2, box1));
    }

    SECTION("Valid touching boxes - edge contact") {
        BoundingBox box1(Vec2(0.f, 0.f), Vec2(1, 1));
        BoundingBox box2(Vec2(1, 0.f), Vec2(2, 1));

        REQUIRE(collide(box1, box2));
        REQUIRE(collide(box2, box1));
    }

    SECTION("Valid touching boxes - corner contact") {
        BoundingBox box1(Vec2(0.f, 0.f), Vec2(1, 1));
        BoundingBox box2(Vec2(1, 1), Vec2(2, 2));

        REQUIRE(collide(box1, box2));
        REQUIRE(collide(box2, box1));
    }

    SECTION("One box fully contained within another") {
        BoundingBox outer(Vec2(-2, -2), Vec2(2, 2));
        BoundingBox inner(Vec2(-1, -1), Vec2(1, 1));

        REQUIRE(collide(outer, inner));
        REQUIRE(collide(inner, outer));
    }

    SECTION("Invalid boxes") {
        BoundingBox valid(Vec2(0.f, 0.f), Vec2(1, 1));
        BoundingBox invalid(Vec2(1, 1), Vec2(0.f, 0.f)); // min > max

        REQUIRE_FALSE(collide(valid, invalid));
        REQUIRE_FALSE(collide(invalid, valid));
        REQUIRE_FALSE(collide(invalid, invalid));
    }
}

TEST_CASE("Circle-Circle collision detection", "[collision][!mayfail]") {
    SECTION("No collision - sufficient distance") {
        Circle circle1(Vec2(0.f, 0.f), 1.0f);
        Circle circle2(Vec2(3, 0.f), 1.0f);

        auto contacts = collide(circle1, circle2);
        REQUIRE(contacts.n_contacts == 0);
    }

    SECTION("Exact touching - distance equals sum of radii") {
        Circle circle1(Vec2(0.f, 0.f), 1.0f);
        Circle circle2(Vec2(2, 0.f), 1.0f);

        auto contacts = collide(circle1, circle2);
        REQUIRE(contacts.n_contacts == 1);

        REQUIRE_THAT(contacts.normal, isUnitVector());
        REQUIRE_THAT(contacts.normal, isApprox(Vec2(1.f, 0.f)));

        REQUIRE_THAT(contacts.contacts_a[0], isApprox(Vec2(1.f, 0.f)));
        REQUIRE_THAT(contacts.contacts_b[0], isApprox(Vec2(1.f, 0.f)));
    }

    SECTION("Clear overlap") {
        Circle circle1(Vec2(0.f, 0.f), 1.0f);
        Circle circle2(Vec2(1, 0.f), 1.0f);

        auto contacts = collide(circle1, circle2);
        REQUIRE(contacts.n_contacts == 1);

        REQUIRE_THAT(contacts.normal, isUnitVector());
        REQUIRE_THAT(contacts.normal, isApprox(Vec2(1.f, 0.f)));
        REQUIRE_THAT(contacts.contacts_a[0], isApprox(Vec2(1.f, 0.f)));
        REQUIRE_THAT(contacts.contacts_b[0], isApprox(Vec2(0.f, 0.f)));
    }

    SECTION("One circle inside another") {
        Circle outer(Vec2(0.f, 0.f), 2.0f);
        Circle inner(Vec2(0.5f, 0.f), 0.5f);

        auto contacts = collide(outer, inner);
        REQUIRE(contacts.n_contacts == 1);
        REQUIRE_THAT(contacts.normal, isUnitVector());
    }

    SECTION("Concentric circles") {
        Circle circle1(Vec2(0.f, 0.f), 2.0f);
        Circle circle2(Vec2(0.f, 0.f), 1.0f);

        auto contacts = collide(circle1, circle2);
        REQUIRE(contacts.n_contacts == 1);

        REQUIRE_FALSE(std::isnan(contacts.normal[0]));
        REQUIRE_FALSE(std::isnan(contacts.normal[1]));
        REQUIRE_THAT(contacts.normal, isUnitVector());
    }

    SECTION("Zero radius edge case") {
        Circle circle1(Vec2(0.f, 0.f), 0.0f);
        Circle circle2(Vec2(0.f, 0.f), 1.0f);

        auto contacts = collide(circle1, circle2);
        REQUIRE(contacts.n_contacts == 1);

        REQUIRE_FALSE(std::isnan(contacts.normal[0]));
        REQUIRE_FALSE(std::isnan(contacts.normal[1]));
    }
}

// TODO: Constructor updated...
TEST_CASE("Circle-Capsule collision detection", "[collision][!mayfail]") {
    SECTION("No collision - circle away from capsule") {
        Circle  circle(Vec2(5, 0.f), 1.0f);
        Capsule capsule(Vec2(0.f, -1), Vec2(0.f, 1), 0.5f);

        auto contacts = collide(circle, capsule);
        REQUIRE(contacts.n_contacts == 0);
    }

    SECTION("Collision with capsule center segment") {
        // Circle overlaps capsule central segment to the right
        Circle  circle(Vec2(1, 0.f), 0.8f);
        Capsule capsule(Vec2(0.f, -1), Vec2(0.f, 1), 0.5f);

        auto contacts = collide(circle, capsule);
        REQUIRE(contacts.n_contacts == 1);
        REQUIRE_THAT(contacts.normal, isUnitVector());
        REQUIRE_THAT(contacts.normal, isApprox(Vec2(-1.f, 0.f)));
    }

    SECTION("Collision with bottom cap") {
        Circle  circle(Vec2(0.f, -2), 1.f);
        Capsule capsule(Vec2(0.f, -1), Vec2(0.f, 1), 0.5f);

        auto contacts = collide(circle, capsule);
        REQUIRE(contacts.n_contacts == 1);
        REQUIRE_THAT(contacts.normal, isUnitVector());
        REQUIRE_THAT(contacts.normal, isApprox(Vec2(0.f, 1.f)));
    }

    SECTION("Collision with top cap") {
        Circle  circle(Vec2(0.f, 2), 1.f);
        Capsule capsule(Vec2(0.f, -1), Vec2(0.f, 1), 0.5f);

        auto contacts = collide(circle, capsule);
        REQUIRE(contacts.n_contacts == 1);
        REQUIRE_THAT(contacts.normal, isUnitVector());
        REQUIRE_THAT(contacts.normal, isApprox(Vec2(0.f, -1.f)));
    }

    SECTION("Circle in capsule") {
        Circle  circle(Vec2(0.05f, 0.f), 5.f);
        Capsule capsule(Vec2(0.f, -1), Vec2(0.f, 1), .5f);

        auto contacts = collide(circle, capsule);
        REQUIRE(contacts.n_contacts == 1);
        REQUIRE_THAT(contacts.normal, isUnitVector());
    }

    SECTION("Zero-length capsule (becomes circle)") {
        Circle  circle(Vec2(1, 0.f), 1.f);
        Capsule capsule(Vec2(0.f, -0.5f), Vec2(0.f, 0.5f), 1.f);

        auto contacts = collide(circle, capsule);
        REQUIRE(contacts.n_contacts == 1);
        REQUIRE_THAT(contacts.normal, isUnitVector());
        REQUIRE_THAT(contacts.normal, isApprox(Vec2(-1.f, 0.f)));
    }

    SECTION("Circle center on capsule's axis") {
        Circle  circle(Vec2(0.f, 0.f), 5.f);
        Capsule capsule(Vec2(0.f, -1), Vec2(0.f, 1), .5f);

        auto contacts = collide(circle, capsule);
        REQUIRE(contacts.n_contacts == 1);
        REQUIRE_THAT(contacts.normal, isUnitVector());
    }
}

TEST_CASE("Circle-Polygon collision detection", "[collision]") {
    SECTION("No collision - circle away from polygon") {
        Circle            circle(Vec2(5, 5), 1.f);
        std::vector<Vec2> vertices = {
            Vec2(0.f, 0.f), Vec2(1, 0.f), Vec2(1, 1), Vec2(0.f, 1)
        };
        Polygon polygon(vertices);

        auto contacts = collide(circle, polygon);
        REQUIRE(contacts.n_contacts == 0);
    }

    SECTION("Vertex collision - intersecting with polygon vertex") {
        Circle            circle(Vec2(.5f, .5f), .8f);
        std::vector<Vec2> vertices = {
            Vec2(0.f, 0.f), Vec2(2, 0.f), Vec2(2, 2), Vec2(0.f, 2)
        };
        Polygon polygon(vertices);

        auto contacts = collide(circle, polygon);
        REQUIRE(contacts.n_contacts == 1);
        REQUIRE_THAT(contacts.normal, isUnitVector());
        REQUIRE_THAT(contacts.normal, isApprox(normalized(Vec2(-0.5f, -0.5f))));
    }

    SECTION("Edge collision - intersecting with polygon edge") {
        Circle            circle(Vec2(-.5f, 1.f), .8f);
        std::vector<Vec2> vertices = {
            Vec2(0.f, 0.f), Vec2(2, 0.f), Vec2(2, 2), Vec2(0.f, 2)
        };
        Polygon polygon(vertices);

        auto contacts = collide(circle, polygon);
        REQUIRE(contacts.n_contacts == 1);
        REQUIRE_THAT(contacts.normal, isUnitVector());
        REQUIRE_THAT(contacts.normal, isApprox(Vec2(1.f, 0.f)));
    }

    SECTION("Circle fully inside polygon") {
        Circle            circle(Vec2(1, 1), .3f);
        std::vector<Vec2> vertices = {
            Vec2(0.f, 0.f), Vec2(2, 0.f), Vec2(2, 2), Vec2(0.f, 2)
        };
        Polygon polygon(vertices);

        auto contacts = collide(circle, polygon);
        REQUIRE(contacts.n_contacts == 1);
        REQUIRE_THAT(contacts.normal, isUnitVector());
    }

    SECTION("Triangle polygon collision") {
        Circle circle(Vec2(.5f, .3f), .2f);
        std::vector<Vec2> vertices = {Vec2(0.f, 0.f), Vec2(1, 0.f), Vec2(.5f, 1)};
        Polygon polygon(vertices);

        auto contacts = collide(circle, polygon);
        REQUIRE(contacts.n_contacts == 1);
        REQUIRE_THAT(contacts.normal, isUnitVector());
        REQUIRE_THAT(contacts.normal, isApprox(Vec2(0, 1)));
        REQUIRE_THAT(contacts.contacts_a[0], isApprox(Vec2(.5f, .5f)));
        REQUIRE_THAT(contacts.contacts_b[0], isApprox(Vec2(.5f, 0.f)));
    }
}

// TODO: constructor update
TEST_CASE("Capsule-Capsule collision detection", "[collision]") {
    const float epsilon = 1e-6f;

    SECTION("No collision - capsules separated") {
        Capsule capsule1(Vec2(-2, -1), Vec2(-2, 1), .5f);
        Capsule capsule2(Vec2(2, -1), Vec2(2, 1), .5f);

        auto contacts = collide(capsule1, capsule2, epsilon);
        REQUIRE(contacts.n_contacts == 0);
    }

    SECTION("End-to-end collision - top to bottom") {
        Capsule capsule1(Vec2(0.f, -1), Vec2(0.f, 1), .5f);
        Capsule capsule2(Vec2(0.f, 1.f), Vec2(0.f, 3.f), .5f);

        auto contacts = collide(capsule1, capsule2, epsilon);
        REQUIRE(contacts.n_contacts == 1);
        REQUIRE_THAT(contacts.normal, isUnitVector());
        REQUIRE_THAT(contacts.normal, isApprox(Vec2(0.f, 1.f)));
    }

    SECTION("Middle-to-middle collision - perpendicular capsules") {
        Capsule capsule1(Vec2(-1, 0.f), Vec2(1, 0.f), .6f);
        Capsule capsule2(Vec2(0.f, -1), Vec2(0.f, 1), .6f);

        auto contacts = collide(capsule1, capsule2, epsilon);
        REQUIRE(contacts.n_contacts == 1);
        REQUIRE_THAT(contacts.normal, isUnitVector());
    }

    SECTION("Parallel capsules with overlap") {
        Capsule capsule1(Vec2(0.f, -1), Vec2(0.f, 1), .8f);
        Capsule capsule2(Vec2(1, -.5f), Vec2(1, .5f), .8f);

        auto contacts = collide(capsule1, capsule2, epsilon);
        REQUIRE(contacts.n_contacts == 2);
        REQUIRE_THAT(contacts.normal, isUnitVector());
        REQUIRE_THAT(contacts.normal, isApprox(Vec2(1.f, 0.f)));
        REQUIRE_THAT(contacts.contacts_a[0], isApprox(Vec2(0.8f, .2f)));
        REQUIRE_THAT(contacts.contacts_a[1], isApprox(Vec2(0.8f, -.2f)));
        REQUIRE_THAT(contacts.contacts_b[0], isApprox(Vec2(0.2f, .2f)));
        REQUIRE_THAT(contacts.contacts_b[1], isApprox(Vec2(0.2f, -.2f)));
    }

    SECTION("Zero-length capsules (become circles)") {
        Capsule capsule1(Vec2(0.f, 0.f), Vec2(0.f, 3.f), 1.f);
        Capsule capsule2(Vec2(2, 0.f), Vec2(2, 3.f), 1.f);

        auto contacts = collide(capsule1, capsule2, epsilon);
        REQUIRE(contacts.n_contacts == 2);
        REQUIRE_THAT(contacts.normal, isUnitVector());
    }
}

// TODO: Constructor update
TEST_CASE("Capsule-Polygon collision detection", "[collision][!mayfail]") {
    const float epsilon = 1e-6f;

    SECTION("No collision - capsule away from polygon") {
        Capsule           capsule(Vec2(5, 5), Vec2(6, 6), .5f);
        std::vector<Vec2> vertices = {
            Vec2(0.f, 0.f), Vec2(2, 0.f), Vec2(2, 2), Vec2(0.f, 2)
        };
        Polygon polygon(vertices);

        auto contacts = collide(capsule, polygon, epsilon);
        REQUIRE(contacts.n_contacts == 0);
    }

    SECTION("Vertex collision - capsule end hitting polygon vertex") {
        Capsule           capsule(Vec2(-.5f, 0.f), Vec2(-1.5f, 0.f), .8f);
        std::vector<Vec2> vertices = {
            Vec2(0.f, 0.f), Vec2(2, 0.f), Vec2(2, 2), Vec2(0.f, 2)
        };
        Polygon polygon(vertices);

        auto contacts = collide(capsule, polygon, epsilon);
        REQUIRE((contacts.n_contacts == 1));
        REQUIRE_THAT(contacts.normal, isUnitVector());
        REQUIRE_THAT(contacts.normal, isApprox(Vec2(1.f, 0.f)));
        REQUIRE_THAT(contacts.contacts_a[0], isApprox(Vec2(-.5f, 0.f)));
        REQUIRE_THAT(contacts.contacts_b[0], isApprox(Vec2(0.f, 0.f)));
    }

    SECTION("Edge collision - capsule middle hitting polygon edge") {
        Capsule           capsule(Vec2(-1.f, 0), Vec2(0.f, 0), .5f);
        std::vector<Vec2> vertices = {
            Vec2(0.f, 0.f), Vec2(2, 0.f), Vec2(2, 2), Vec2(0.f, 2)
        };
        Polygon polygon(vertices);

        auto contacts = collide(capsule, polygon, epsilon);
        REQUIRE(contacts.n_contacts == 1);
        REQUIRE_THAT(contacts.normal, isUnitVector());
        REQUIRE_THAT(contacts.normal, isApprox(Vec2(1.f, 0.f)));
    }

    SECTION("Zero-length capsule (becomes circle) with polygon") {
        Capsule           capsule(Vec2(-1.f, 0.f), Vec2(-1.f, 2.f), 1.f);
        std::vector<Vec2> vertices = {
            Vec2(0.f, 0.f), Vec2(2, 0.f), Vec2(2, 2), Vec2(0.f, 2)
        };
        Polygon polygon(vertices);

        auto contacts = collide(capsule, polygon, epsilon);
        REQUIRE(contacts.n_contacts == 1);
        REQUIRE_THAT(contacts.normal, isUnitVector());
    }

    SECTION("Vertex on capsule's center") {
        Capsule           capsule(Vec2(-.5f, 0), Vec2(.5f, 0), .5f);
        std::vector<Vec2> vertices = {
            Vec2(0.f, 0.f), Vec2(2, 0.f), Vec2(2, 2), Vec2(0.f, 2)
        };
        Polygon polygon(vertices);

        auto contacts = collide(capsule, polygon, epsilon);
        REQUIRE((contacts.n_contacts == 1));
        REQUIRE_THAT(contacts.normal, isUnitVector());
    }
}

TEST_CASE("Polygon-Polygon collision detection", "[collision]") {
    const float epsilon = 1e-6f;

    SECTION("No collision - separate polygons") {
        std::vector<Vec2> vertices1 = {
            Vec2(0.f, 0.f), Vec2(1, 0.f), Vec2(1, 1), Vec2(0.f, 1)
        };
        std::vector<Vec2> vertices2 = {Vec2(3, 3), Vec2(4, 3), Vec2(4, 4), Vec2(3, 4)};
        Polygon polygon1(vertices1);
        Polygon polygon2(vertices2);

        auto contacts = collide(polygon1, polygon2, epsilon);
        REQUIRE(contacts.n_contacts == 0);
    }

    SECTION("Vertex-vertex collision") {
        std::vector<Vec2> vertices1 = {
            Vec2(0.f, 0.f), Vec2(1, 0.f), Vec2(1, 1), Vec2(0.f, 1)
        };
        std::vector<Vec2> vertices2 = {Vec2(1, 1), Vec2(2, 1), Vec2(1.5, 2)};
        Polygon polygon1(vertices1);
        Polygon polygon2(vertices2);

        auto contacts = collide(polygon1, polygon2, epsilon);
        REQUIRE(contacts.n_contacts == 1);
        REQUIRE_THAT(contacts.normal, isUnitVector());
        REQUIRE_THAT(contacts.normal, isApprox(normalized(Vec2(1.f, 0.f))));
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

        auto contacts = collide(polygon1, polygon2, epsilon);
        REQUIRE(contacts.n_contacts == 2);
        REQUIRE_THAT(contacts.normal, isUnitVector());
        REQUIRE_THAT(contacts.normal, isApprox(Vec2(0.f, 1.f)));
    }

    SECTION("Edge-edge collision - overlapping edges") {
        std::vector<Vec2> vertices1 = {
            Vec2(0.f, 0.f), Vec2(2, 0.f), Vec2(2, 1), Vec2(0.f, 1)
        };
        std::vector<Vec2> vertices2 = {Vec2(1, 1), Vec2(3, 1), Vec2(3, 2), Vec2(1, 2)};
        Polygon polygon1(vertices1);
        Polygon polygon2(vertices2);

        auto contacts = collide(polygon1, polygon2, epsilon);
        REQUIRE(contacts.n_contacts == 2);
        REQUIRE_THAT(contacts.contacts_a[0], isApprox(Vec2(2.f, 1.f)));
        REQUIRE_THAT(contacts.contacts_a[1], isApprox(Vec2(1.f, 1.f)));
        REQUIRE_THAT(contacts.contacts_b[0], isApprox(Vec2(2.f, 1.f)));
        REQUIRE_THAT(contacts.contacts_b[1], isApprox(Vec2(1.f, 1.f)));
        REQUIRE_THAT(contacts.normal, isUnitVector());
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

        auto contacts = collide(polygon1, polygon2, epsilon);
        REQUIRE(contacts.n_contacts >= 1);
        REQUIRE_THAT(contacts.normal, isUnitVector());
    }

    SECTION("Triangle collision") {
        std::vector<Vec2> vertices1 = {Vec2(0.f, 0.f), Vec2(2, 0.f), Vec2(1, 2)};
        std::vector<Vec2> vertices2 = {Vec2(1.5f, 0.f), Vec2(3.5f, 0.f), Vec2(2.5f, 2)};
        Polygon polygon1(vertices1);
        Polygon polygon2(vertices2);

        auto contacts = collide(polygon1, polygon2, epsilon);
        REQUIRE(contacts.n_contacts == 1);
        REQUIRE_THAT(contacts.normal, isUnitVector());
        REQUIRE_THAT(contacts.normal, isApprox(normalized(Vec2(2.f, 1.f))));
        REQUIRE_THAT(contacts.contacts_b[0], isApprox(Vec2(1.5f, 0.f)));
    }
}
