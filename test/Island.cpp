#include <numbers>

#include "catch2/catch_test_macros.hpp"

#include "Simu/physics/World.hpp"
#include "Simu/physics/Body.hpp"
#include "Simu/physics/Constraint.hpp"
#include "Simu/physics/Island.hpp"

using namespace simu;


constexpr float pi = std::numbers::pi_v<float>;


struct Islands : public std::list<Island>
{
    Islands(auto bodies)
    {
        std::vector<Body*> toProcess{};
        for (Body& b : bodies)
            toProcess.emplace_back(&b);

        auto removeBody = [&](Body* body) { std::erase(toProcess, body); };

        FreeListAllocator<int> alloc;
        while (!toProcess.empty())
        {
            emplace_back(alloc);
            back().init(toProcess.back());
            for (Body* body : back().bodies())
                removeBody(body);
        }
    }

    void done()
    {
        for (auto& isl : *this)
            isl.endSolve();

        clear();
    }
};

template <class ContactRange>
void requireCoversAllConstraintsOnce(ContactRange allConstraints, Islands& islands)
{
    for (Constraint& constraint : allConstraints)
    {
        Uint32 count = 0;
        for (Island& island : islands)
        {
            for (Constraint* c : island.constraints())
                if (c == &constraint)
                    ++count;
        }
        REQUIRE(count == 1);
    }
}

template <class ContactRange>
void requireCoversAllContactsOnce(ContactRange allContacts, Islands& islands)
{
    for (ContactConstraint& contact : allContacts)
    {
        Uint32 count = 0;
        for (Island& island : islands)
        {
            for (ContactConstraint* c : island.contacts())
                if (c == &contact)
                    ++count;
        }
        REQUIRE(count == 1);
    }
}

Body* makeBox(World& w, Vec2 pos = Vec2{0, 0}, Vec2 dim = Vec2{2, 2})
{
    BodyDescriptor descr{};
    descr.position = pos;
    auto b         = w.makeBody(descr);

    b->addCollider(ColliderDescriptor{Polygon::box(dim)});
    return b;
}

TEST_CASE("Island")
{
    SECTION("Islands creation")
    {
        SECTION("Disconnected")
        {
            World world;

            for (Uint32 i = 0; i < 5; ++i)
            {
                makeBox(world, Vec2{2.f * i, 0.f});
            }

            Islands islands{world.bodies()};
            REQUIRE(islands.size() == 5);
        }

        SECTION("Single island")
        {
            World world;

            Body* previous = makeBox(world);
            for (Uint32 i = 1; i < 5; ++i)
            {
                Body* current = makeBox(world, Vec2{2.f * i, 0.f});
                world.makeConstraint<WeldConstraint>(Bodies{previous, current});

                previous = current;
            }

            Islands islands{world.bodies()};
            REQUIRE(islands.size() == 1);
            requireCoversAllConstraintsOnce(world.constraints(), islands);
        }

        SECTION("Multiple islands")
        {
            World world;

            auto makeBody = [&](Uint32 index) -> Body* {
                return makeBox(world, Vec2{2.f * index, 0.f});
            };

            world.makeConstraint<WeldConstraint>(Bodies{makeBody(0), makeBody(1)});

            world.makeConstraint<WeldConstraint>(Bodies{makeBody(2), makeBody(3)});

            Islands islands{world.bodies()};
            REQUIRE(islands.size() == 2);
            requireCoversAllConstraintsOnce(world.constraints(), islands);
        }

        SECTION("Structural seperates islands")
        {
            SECTION("stacks on floor")
            {
                World world;

                auto makeStack = [&](Uint32 index, Uint32 height) {
                    for (Uint32 i = 0; i < height; ++i)
                    {
                        makeBox(world, Vec2{2.f * index, 0.99f * i}, Vec2{1.f, 1.f});
                    }
                };

                Uint32         nStacks     = 5;
                Uint32         stackHeight = 5;
                BodyDescriptor floorDescr{};
                floorDescr.dominance = 0.f;

                float w = 2.f * nStacks + 1.f;
                float h = 1.f;
                world.makeBody(floorDescr)
                    ->addCollider(ColliderDescriptor{
                        Polygon::box(Vec2{w, h}, Vec2{w / 2, -h / 2})});

                for (Uint32 i = 0; i < nStacks; ++i)
                    makeStack(i, stackHeight);

                world.step(1.f); // makes contact constraints

                Islands islands{world.bodies()};
                REQUIRE(islands.size() == nStacks);

                auto c = world.contacts();
                REQUIRE(std::distance(c.begin(), c.end()) == nStacks * stackHeight);

                requireCoversAllContactsOnce(c, islands);
            }

            SECTION("Per constraint structural")
            {
                // A      D
                //  \ C /
                //  /   \ 
                // B     E
                //
                // If C is structural in all constraints, we have 4 islands
                // if it is not in at least one, then we have a single
                // island no matter the structural property of A, B, D, E

                World world{};

                float x = 0.f;
                Body* A = makeBox(world, Vec2{x, 0.f}, Vec2{1, 1});
                Body* B = makeBox(world, Vec2{x += 2.f, 0.f}, Vec2{1, 1});
                Body* C = makeBox(world, Vec2{x += 2.f, 0.f}, Vec2{1, 1});
                Body* D = makeBox(world, Vec2{x += 2.f, 0.f}, Vec2{1, 1});
                Body* E = makeBox(world, Vec2{x += 2.f, 0.f}, Vec2{1, 1});

                ////////////////////////////////////////////////////////////
                // C is never structural

                // clang-format off
                auto AC = world.makeConstraint<WeldConstraint>(Bodies{{A, C}, Vec2{0.f, 1.f}}, true);
                auto BC = world.makeConstraint<WeldConstraint>(Bodies{{B, C}, Vec2{0.f, 1.f}}, true);
                auto DC = world.makeConstraint<WeldConstraint>(Bodies{{D, C}, Vec2{0.f, 1.f}}, true);
                auto EC = world.makeConstraint<WeldConstraint>(Bodies{{E, C}, Vec2{0.f, 1.f}}, true);
                // clang-format on

                Islands islands{world.bodies()};

                // depends on processing order...
                //  processing C will include all other bodies.
                //  all others interact as structural and produce a single island per body
                // REQUIRE(islands.size() == 1);
                requireCoversAllConstraintsOnce(world.constraints(), islands);
                islands.done();

                AC->kill();
                BC->kill();
                DC->kill();
                EC->kill();
                world.step(1.f); // cleanup


                ////////////////////////////////////////////////////////////
                // C sometimes interacts as structural

                // clang-format off
                AC = world.makeConstraint<WeldConstraint>(Bodies{{A, C}, Vec2{0.f, 1.f}}, true);
                BC = world.makeConstraint<WeldConstraint>(Bodies{{B, C}, Vec2{1.f, 0.f}}, true);
                DC = world.makeConstraint<WeldConstraint>(Bodies{{D, C}, Vec2{0.f, 1.f}}, true);
                EC = world.makeConstraint<WeldConstraint>(Bodies{{E, C}, Vec2{1.f, 0.f}}, true);
                // clang-format on

                islands = Islands{world.bodies()};
                REQUIRE(islands.size() == 1);
                requireCoversAllConstraintsOnce(world.constraints(), islands);
                islands.done();

                AC->kill();
                BC->kill();
                DC->kill();
                EC->kill();
                world.step(1.f); // cleanup


                ////////////////////////////////////////////////////////////
                // C always interacts as structural

                // clang-format off
                AC = world.makeConstraint<WeldConstraint>(Bodies{{A, C}, Vec2{1.f, 0.f}}, true);
                BC = world.makeConstraint<WeldConstraint>(Bodies{{B, C}, Vec2{1.f, 0.f}}, true);
                DC = world.makeConstraint<WeldConstraint>(Bodies{{D, C}, Vec2{1.f, 0.f}}, true);
                EC = world.makeConstraint<WeldConstraint>(Bodies{{E, C}, Vec2{1.f, 0.f}}, true);
                // clang-format on

                islands = Islands{world.bodies()};
                REQUIRE(islands.size() == 4); // C in every Island, one per body
                requireCoversAllConstraintsOnce(world.constraints(), islands);
                islands.done();
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
