#include "Simu/physics-2.0/collision/broadphase/BoundingVolumeHierarchy.hpp"


#include "catch2/catch_test_macros.hpp"
#include "catch2/catch_approx.hpp"
#include "Simu/physics-2.0/collision/colliders/BoundingBox.hpp"
#include "Simu/physics-2.0/collision/colliders/Polygon.hpp"
#include "../matchers/IsApprox.h"

#include <catch2/generators/catch_generators.hpp>

using namespace simu;
using Catch::Approx;

struct TestCase
{
    std::vector<ObjectId>                    entities;
    std::vector<BoundingBox>               bounds;
    std::vector<std::pair<ObjectId, ObjectId>> expected_collisions;
};

constexpr TestCase simple_case() {
    TestCase                  test_case{};
    internal::EntityGenerator generator;

    test_case.entities.push_back(generator.create());
    test_case.bounds.emplace_back(Vec2(0, 0), Vec2(1, 1));

    test_case.entities.push_back(generator.create());
    test_case.bounds.emplace_back(Vec2(0.5, 0.5), Vec2(1.5, 1.5));

    test_case.entities.push_back(generator.create());
    test_case.bounds.emplace_back(Vec2(2, 2), Vec2(3, 3));

    test_case.expected_collisions = {
        {test_case.entities[0], test_case.entities[1]},
    };

    return test_case;
}

constexpr TestCase repeated_object() {
    TestCase                  test_case{};
    internal::EntityGenerator generator;

    for (int i = 0; i < 5; ++i) {
        test_case.entities.push_back(generator.create());
        test_case.bounds.emplace_back(Vec2(0, 0), Vec2(1, 1));
    }

    for (int i = 0; i < 5; ++i) {
        for (int j = i + 1; j < 5; ++j) {
            test_case.expected_collisions.emplace_back(
                test_case.entities[i], test_case.entities[j]
            );
        }
    }

    return test_case;
}

TEST_CASE("Bounding Volume Hierarchy") {
    auto partition_method = GENERATE(
        BoundingVolumeHierarchy::mean_centroid_split
        // ...
    );


    auto test_case = GENERATE(simple_case(), repeated_object());

    BoundingVolumeHierarchy bvh;
    REQUIRE_NOTHROW(bvh = partition_method(test_case.entities, test_case.bounds));

    bvh.collide(bvh, [&test_case](ObjectId a, ObjectId b) {
        // collisions to self are always reported when testing a tree against itself
        // collisions between pairs are reported twice, order them to keep only one.
        if (a == b || a.id() > b.id()) {
            return;
        }

        auto it = std::find(
            test_case.expected_collisions.begin(),
            test_case.expected_collisions.end(),
            std::make_pair(a, b)
        );

        REQUIRE(it != test_case.expected_collisions.end());

        test_case.expected_collisions.erase(it);
    });

    REQUIRE(test_case.expected_collisions.empty());
}