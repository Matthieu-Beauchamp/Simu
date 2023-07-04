#include <numbers>

#include "catch2/catch_test_macros.hpp"

#include "Simu/physics/PhysicsWorld.hpp"
#include "Simu/physics/PhysicsBody.hpp"
#include "Simu/physics/Constraint.hpp"
#include "Simu/physics/Island.hpp"

using namespace simu;

const std::initializer_list<Vec2> square{
    Vec2{0, 0},
    Vec2{1, 0},
    Vec2{1, 1},
    Vec2{0, 1},
};

const BodyDescriptor squareDescriptor{Polygon{square}};

const float pi = std::numbers::pi_v<float>;

template <class ConstraintRange>
void requireCoversAllConstraintsOnce(ConstraintRange allConstraints, Islands& islands)
{
    for (Constraint& constraint : allConstraints)
    {
        Uint32 count = 0;
        for (Island& island : islands.islands())
            for (Constraint* c : island.constraints())
                if (c == &constraint)
                    ++count;

        REQUIRE(count == 1);
    }
}

TEST_CASE("Island")
{
    SECTION("Islands creation")
    {
        SECTION("Single island")
        {
            PhysicsWorld world;

            PhysicsBody* previous = world.makeBody(squareDescriptor);
            for (Uint32 i = 1; i < 5; ++i)
            {
                BodyDescriptor descr{squareDescriptor};
                descr.position[0]    = 2 * i;
                PhysicsBody* current = world.makeBody(descr);
                world.makeConstraint<WeldConstraint>(Bodies<2>{previous, current}
                );

                previous = current;
            }

            Islands islands{world.bodies()};
            REQUIRE(islands.islands().size() == 1);
            requireCoversAllConstraintsOnce(world.constraints(), islands);
        }

        SECTION("Multiple islands") {}

        SECTION("Structural seperates islands")
        {
            // many edge cases to check here
        }
    }

    SECTION("Sleeping")
    {
        SECTION("Awake object -> awake island") {}
        SECTION("New force awakes islands") {}
        SECTION("New constraint awakes islands") {}
        SECTION("Killing a force awakes islands") {}
        SECTION("Killing a constraint awakes islands") {}
        SECTION("Killing a body awakes islands") {}
    }
}