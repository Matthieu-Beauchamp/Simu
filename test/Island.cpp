#include <numbers>

#include "catch2/catch_test_macros.hpp"

#include "Simu/physics/World.hpp"
#include "Simu/physics/Body.hpp"
#include "Simu/physics/Constraint.hpp"
#include "Simu/physics/Island.hpp"

using namespace simu;

BodyDescriptor getSquareDescriptor()
{
    return BodyDescriptor{
        Polygon{
                Vec2{0, 0},
                Vec2{1, 0},
                Vec2{1, 1},
                Vec2{0, 1},
                }
    };
}

constexpr float pi = std::numbers::pi_v<float>;

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
        SECTION("No island")
        {
            World world;

            for (Uint32 i = 1; i < 5; ++i)
            {
                BodyDescriptor descr{getSquareDescriptor()};
                descr.position[0] = 2 * i;
                world.makeBody(descr);
            }

            Islands islands{world.bodies()};
            REQUIRE(islands.islands().size() == 0);
        }

        SECTION("Single island")
        {
            World world;

            Body* previous = world.makeBody(getSquareDescriptor());
            for (Uint32 i = 1; i < 5; ++i)
            {
                BodyDescriptor descr{getSquareDescriptor()};
                descr.position[0]    = 2 * i;
                Body* current = world.makeBody(descr);
                world.makeConstraint<WeldConstraint>(Bodies<2>{previous, current}
                );

                previous = current;
            }

            Islands islands{world.bodies()};
            REQUIRE(islands.islands().size() == 1);
            requireCoversAllConstraintsOnce(world.constraints(), islands);
        }

        SECTION("Multiple islands")
        {
            World world;

            auto makeBody = [&](Uint32 index) -> Body* {
                BodyDescriptor descr{getSquareDescriptor()};
                descr.position[0] = 2 * index;
                return world.makeBody(descr);
            };

            world.makeConstraint<WeldConstraint>(
                Bodies<2>{makeBody(0), makeBody(1)}
            );

            world.makeConstraint<WeldConstraint>(
                Bodies<2>{makeBody(2), makeBody(3)}
            );

            Islands islands{world.bodies()};
            REQUIRE(islands.islands().size() == 2);
            requireCoversAllConstraintsOnce(world.constraints(), islands);
        }

        SECTION("Structural seperates islands")
        {
            SECTION("stacks on floor")
            {
                World world;

                auto makeStack = [&](Uint32 index, Uint32 height) {
                    BodyDescriptor descr{getSquareDescriptor()};
                    for (Uint32 i = 0; i < height; ++i)
                    {
                        descr.position[0] = 2 * index;
                        descr.position[1] = i;
                        world.makeBody(descr);
                    }
                };

                Uint32         nStacks     = 5;
                Uint32         stackHeight = 5;
                BodyDescriptor floorDescr{getSquareDescriptor()};
                floorDescr.dominance = 0.f;
                floorDescr.polygon   = Polygon{
                    Vec2{0.f,                 -1.f},
                    Vec2{2.f * nStacks + 1.f, -1.f},
                    Vec2{2.f * nStacks + 1.f, 0.f },
                    Vec2{0.f,                 0.f }
                };

                world.makeBody(floorDescr);

                for (Uint32 i = 0; i < nStacks; ++i)
                    makeStack(i, stackHeight);

                world.step(1.f); // makes contact constraints

                Islands islands{world.bodies()};
                REQUIRE(islands.islands().size() == nStacks);

                auto c = world.constraints();
                REQUIRE(
                    std::distance(c.begin(), c.end()) == nStacks * stackHeight
                );
                requireCoversAllConstraintsOnce(world.constraints(), islands);
            }

            SECTION("Per constraint structural")
            {
                // A      D
                //  \ C /
                //  /   \
                // B     E
                //
                // If C is structural in all constraints, we have 4 islands
                // if it is not in at least one, then we have a single island
                // no matter the structural property of A, B, D, E

                World   world{};
                BodyDescriptor descr{getSquareDescriptor()};

                Body* A = world.makeBody(descr);
                descr.position[0] += 2;
                Body* B = world.makeBody(descr);
                descr.position[0] += 2;
                Body* C = world.makeBody(descr);
                descr.position[0] += 2;
                Body* D = world.makeBody(descr);
                descr.position[0] += 2;
                Body* E = world.makeBody(descr);

                ////////////////////////////////////////////////////////////
                // C is never structural

                // clang-format off
                auto AC = world.makeConstraint<WeldConstraint>(Bodies<2>{{A, C}, Vec2{0.f, 1.f}}, true);
                auto BC = world.makeConstraint<WeldConstraint>(Bodies<2>{{B, C}, Vec2{0.f, 1.f}}, true);
                auto DC = world.makeConstraint<WeldConstraint>(Bodies<2>{{D, C}, Vec2{0.f, 1.f}}, true);
                auto EC = world.makeConstraint<WeldConstraint>(Bodies<2>{{E, C}, Vec2{0.f, 1.f}}, true);
                // clang-format on

                Islands islands{world.bodies()};
                REQUIRE(islands.islands().size() == 1);
                requireCoversAllConstraintsOnce(world.constraints(), islands);

                AC->kill();
                BC->kill();
                DC->kill();
                EC->kill();
                world.step(1.f); // cleanup


                ////////////////////////////////////////////////////////////
                // C sometimes interacts as structural

                // clang-format off
                AC = world.makeConstraint<WeldConstraint>(Bodies<2>{{A, C}, Vec2{0.f, 1.f}}, true);
                BC = world.makeConstraint<WeldConstraint>(Bodies<2>{{B, C}, Vec2{1.f, 0.f}}, true);
                DC = world.makeConstraint<WeldConstraint>(Bodies<2>{{D, C}, Vec2{0.f, 1.f}}, true);
                EC = world.makeConstraint<WeldConstraint>(Bodies<2>{{E, C}, Vec2{1.f, 0.f}}, true);
                // clang-format on

                islands = Islands{world.bodies()};
                REQUIRE(islands.islands().size() == 1);
                requireCoversAllConstraintsOnce(world.constraints(), islands);

                AC->kill();
                BC->kill();
                DC->kill();
                EC->kill();
                world.step(1.f); // cleanup


                ////////////////////////////////////////////////////////////
                // C always interacts as structural

                // clang-format off
                AC = world.makeConstraint<WeldConstraint>(Bodies<2>{{A, C}, Vec2{1.f, 0.f}}, true);
                BC = world.makeConstraint<WeldConstraint>(Bodies<2>{{B, C}, Vec2{1.f, 0.f}}, true);
                DC = world.makeConstraint<WeldConstraint>(Bodies<2>{{D, C}, Vec2{1.f, 0.f}}, true);
                EC = world.makeConstraint<WeldConstraint>(Bodies<2>{{E, C}, Vec2{1.f, 0.f}}, true);
                // clang-format on

                islands = Islands{world.bodies()};
                REQUIRE(islands.islands().size() == 4);
                requireCoversAllConstraintsOnce(world.constraints(), islands);
            }
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
