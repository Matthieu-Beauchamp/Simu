#include <numbers>

#include "catch2/catch_test_macros.hpp"

#include "Simu/utility/FloatingPointExceptions.hpp"

#include "Simu/physics/World.hpp"
#include "Simu/physics/Body.hpp"
#include "Simu/physics/Constraint.hpp"
#include "Simu/physics/Motors.hpp"

using namespace simu;

Body* makeBox(World& w, Vec2 pos = Vec2{0, 0}, float theta = 0.f, Vec2 dim = Vec2{1, 1})
{
    BodyDescriptor descr{};
    descr.position    = pos;
    descr.orientation = theta;
    auto b            = w.makeBody(descr);

    b->addCollider(ColliderDescriptor{Polygon::box(dim)});
    return b;
}

constexpr float pi = std::numbers::pi_v<float>;

TEST_CASE("Physics")
{
    SECTION("Single body stepping")
    {
        const Vec2  p{1, 1};
        const Vec2  v{1, 1};
        const float theta = pi / 4;
        const float w     = pi / 2;

        World world{};

        auto body            = makeBox(world, p, theta);
        Vec2 initialCentroid = body->localCentroid();

        body->setVelocity(v);
        body->setAngularVelocity(w);

        REQUIRE(all(body->position() == p));
        REQUIRE(all(body->velocity() == v));
        REQUIRE(body->orientation() == theta);
        REQUIRE(body->angularVelocity() == w);

        float dt = 3.f;
        world.step(dt);
        REQUIRE(all(body->velocity() == v));
        REQUIRE(all(body->position() == p + dt * v));
        REQUIRE(body->angularVelocity() == w);
        REQUIRE(body->orientation() == theta + dt * w);

        // rotation is done about the polygon's centroid.
        // body->position is the position of the origin of the body's local space, rotation not considered.
        // The centroid is not necessarilly centered at {0, 0} in local space.
        REQUIRE(all(body->centroid() == initialCentroid + body->position()));
    }

    SECTION("Global force fields (gravity)")
    {
        World world{};

        float initialHeight = 100.f;


        std::vector<Body*> bodies;
        constexpr Uint32   nBodies = 5;
        for (Uint32 i = 0; i < nBodies; ++i)
        {
            bodies.emplace_back(makeBox(world, Vec2{(float)i, initialHeight}));
        }

        Vec2 gravity = Vec2{0.f, -10.f};
        world.makeForceField<Gravity>(gravity);

        float dt = 1.f;
        world.step(dt);
        for (auto body : bodies)
        {
            // implicitely requires use of symplectic euler integrator
            REQUIRE(all(body->velocity() == dt * gravity));
            REQUIRE(body->position()[1] == initialHeight + dt * body->velocity()[1]);
        }
    }

    SECTION("Multiple force fields")
    {
        World world{};

        Vec2 leftPos{-10.f, 0.f};
        auto left = makeBox(world, leftPos);

        Vec2 middlePos{0.f, 0.f};
        auto middle = makeBox(world, middlePos);

        Vec2 rightPos{10.f, 0.f};
        auto right = makeBox(world, rightPos);

        Vec2 leftForce{1.f, 0};
        world.makeForceField<LinearField>(
            leftForce, ForceField::region(Vec2{-10.f, 0.f}, Vec2{0.f, 10.f})
        );

        Vec2 middleForce{0.f, -1.f};
        world.makeForceField<LinearField>(
            middleForce, ForceField::region(Vec2{-2.f, 0.f}, Vec2{2.f, 10.f})
        );

        Vec2 rightForce{-1.f, 0};
        world.makeForceField<LinearField>(
            rightForce, ForceField::region(Vec2{0.f, 0.f}, Vec2{10.f, 10.f})
        );

        float dt = 1.f;
        world.step(dt);

        REQUIRE(all(left->velocity() == leftForce * dt / left->mass()));
        REQUIRE(all(left->position() == leftPos + left->velocity() * dt));

        // overlaps with all forces, left and right cancel out
        REQUIRE(all(middle->velocity() == middleForce * dt / middle->mass()));
        REQUIRE(all(middle->position() == middlePos + middle->velocity() * dt));

        REQUIRE(all(right->velocity() == rightForce * dt / right->mass()));
        REQUIRE(all(right->position() == rightPos + right->velocity() * dt));
    }

    SECTION("Constraints")
    {
        SECTION("Rotation constraint")
        {
            World world{};

            auto left  = makeBox(world, Vec2{-10.f, 0});
            auto right = makeBox(world, Vec2{10.f, 0});

            auto c = world.makeConstraint<RotationConstraint>(Bodies{left, right});

            left->applyImpulse(Vec2{0, 1}, Vec2{1, 0});
            left->applyImpulse(Vec2{0, -1}, Vec2{-1, 0});
            float w = left->angularVelocity();
            REQUIRE(w > 0.f);

            world.step(1.f);

            REQUIRE(left->angularVelocity() == w / 2);
            REQUIRE(right->angularVelocity() == w / 2);

            right->setAngularVelocity(right->angularVelocity() - w);
            world.step(1.f);
            REQUIRE(approx(left->angularVelocity(), simu::EPSILON).contains(0.f));
            REQUIRE(approx(right->angularVelocity(), simu::EPSILON).contains(0.f));
        }

        SECTION("Hinge constraint")
        {
            World world{};

            auto left  = makeBox(world, Vec2{-1.f, 0.f});
            auto right = makeBox(world, Vec2{0.f, 0.f});

            auto c = world.makeConstraint<HingeConstraint>(
                Bodies{left, right}, Vec2{-0.5f, 0.f}
            );

            // translation /////////////////////////////////////
            left->applyImpulse(Vec2{-1.f, 0.f});
            world.step(1.f);

            REQUIRE(all(left->velocity() == Vec2{-0.5f, 0}));
            REQUIRE(all(left->velocity() == right->velocity()));

            REQUIRE(left->angularVelocity() == 0.f);
            REQUIRE(left->angularVelocity() == right->angularVelocity());

            right->applyImpulse(Vec2{1.f, 0});
            world.step(1.f);

            REQUIRE(all(left->velocity() == Vec2{0.f, 0}));
            REQUIRE(all(left->velocity() == right->velocity()));

            REQUIRE(left->angularVelocity() == 0.f);
            REQUIRE(left->angularVelocity() == right->angularVelocity());

            // rotation ///////////////////////////////////////
            left->applyImpulse(Vec2{0, -1.f});
            right->applyImpulse(Vec2{0, 1.f});

            world.step(0.1f); // dx == 0 here
            world.step(0.1f);

            float dx = right->velocity()[0];
            float dy = right->velocity()[1];
            float w  = right->angularVelocity();

            REQUIRE(dx < 0);
            REQUIRE(dy > 0);
            REQUIRE(w > 0);

            REQUIRE(all(left->velocity() == -right->velocity()));
            REQUIRE(approx(left->angularVelocity(), simu::EPSILON)
                        .contains(right->angularVelocity()));
        }

        SECTION("Hinge Constraint is free to rotate, also tests disabling "
                "collisison")
        {
            World world{};

            auto left  = makeBox(world);
            auto right = makeBox(world);

            auto c = world.makeConstraint<HingeConstraint>(
                Bodies{left, right}, left->centroid(), true
            );

            left->setAngularVelocity(1.f);
            world.step(1.f);

            REQUIRE(left->angularVelocity() == 1.f);
            REQUIRE(right->angularVelocity() == 0.f);
            REQUIRE(all(left->velocity() == Vec2{}));
            REQUIRE(all(right->velocity() == Vec2{}));

            // remove constraint, bodies are now colliding
            c->kill();
            world.step(1.f); // contact created after solving
            world.step(1.f); // contact is solved.
            REQUIRE(any(left->velocity() != Vec2{}));
            REQUIRE(any(right->velocity() != Vec2{}));
        }

        SECTION("Weld constraint")
        {
            World world{};

            auto left  = makeBox(world, Vec2{-1.f, 0.f});
            auto right = makeBox(world, Vec2{0.f, 0.f});

            auto c = world.makeConstraint<WeldConstraint>(Bodies{left, right});

            // applied at the centroid of the combined bodies
            right->applyImpulse(Vec2{0.f, 1.f}, Vec2{-0.5f, 0});
            REQUIRE(right->angularVelocity() != 0.f);
            REQUIRE(right->velocity()[1] != 0.f);

            world.step(1.f);

            REQUIRE(approx(left->angularVelocity(), simu::EPSILON).contains(0.f));
            REQUIRE(approx(right->angularVelocity(), simu::EPSILON).contains(0.f));

            REQUIRE(all(left->velocity() == Vec2{0.f, 0.5f}));
            REQUIRE(all(right->velocity() == Vec2{0.f, 0.5f}));

            left->applyImpulse(Vec2{0.f, -1.f}, Vec2{0.5f, 0});
            world.step(1.f);

            REQUIRE(approx(left->angularVelocity(), simu::EPSILON).contains(0.f));
            REQUIRE(approx(right->angularVelocity(), simu::EPSILON).contains(0.f));

            REQUIRE(all(left->velocity() == Vec2{}));
            REQUIRE(all(right->velocity() == Vec2{}));
        }

        SECTION("Motors")
        {
            World world{};

            auto body = makeBox(world, Vec2{0.f, 0.f});

            constexpr float pi = std::numbers::pi_v<float>;

            auto rot = world.makeConstraint<RotationMotor>(
                Bodies::singleBody(body),
                RotationMotor::Specs::fromAccel(
                    pi * 2, // maximum speed of 1 turn per second
                    pi / 2  // maximum acceleration of 1/4 turn per second
                )
            );

            auto trans = world.makeConstraint<TranslationMotor>(
                Bodies::singleBody(body),
                TranslationMotor::Specs::fromAccel(
                    1.f,  // maximum speed of 1 meter per second
                    0.25f // maximum acceleration of 1/4 meter per second
                )
            );

            // motors do nothing until given a direction
            world.step(1.f);
            REQUIRE(all(body->velocity() == Vec2{}));
            REQUIRE(body->angularVelocity() == 0.f);

            rot->direction(Vector<float, 1>{1.f});
            trans->direction(Vec2{1.f, 0.f});
            for (Uint32 dt = 1; dt < 5; ++dt)
            {
                world.step(1.f);
                REQUIRE(all(approx(body->velocity(), Vec2::filled(simu::EPSILON))
                                .contains(dt * 0.25f * trans->direction())));
                REQUIRE(approx(body->angularVelocity(), simu::EPSILON)
                            .contains(dt * pi / 2 * rot->direction()[0]));
            }

            rot->direction(-rot->direction());
            trans->direction(-trans->direction());
            for (Uint32 dt = 0; dt < 4; ++dt)
                world.step(1.f);

            REQUIRE(all(approx(body->velocity(), Vec2::filled(simu::EPSILON))
                            .contains(Vec2{})));
            REQUIRE(approx(body->angularVelocity(), simu::EPSILON).contains(0.f));
        }

        SECTION("Motor throttling")
        {
            World world{};

            auto body = makeBox(world);

            auto trans = world.makeConstraint<TranslationMotor>(
                Bodies::singleBody(body),
                TranslationMotor::Specs::fromAccel(
                    1.f, // maximum speed of 1 meter per second
                    2.f  // maximum acceleration of 2 meters per second
                )
            );

            trans->throttle(2.f);
            REQUIRE(trans->throttle() == 1.f);

            trans->throttle(-1.f);
            REQUIRE(trans->throttle() == 0.f);

            trans->throttle(0.25f);
            REQUIRE(trans->throttle() == 0.25f);

            trans->direction(Vec2{1, 0});
            world.step(1.f);
            REQUIRE(all(body->velocity() == Vec2{0.5f, 0}));

            trans->throttle(1.f);
            world.step(1.f);
            REQUIRE(all(body->velocity() == Vec2{1.f, 0}));


            trans->direction(Vec2{-1, 0});
            trans->throttle(0.25f);
            world.step(1.f);
            REQUIRE(all(body->velocity() == Vec2{0.5f, 0}));

            trans->throttle(1.f);
            world.step(1.f);
            REQUIRE(all(body->velocity() == Vec2{-1.f, 0}));
        }

        SECTION("Motor moving multiple bodies")
        {
            World world{};

            auto body1 = makeBox(world, Vec2{0.f, 0.f});
            auto body2 = makeBox(world, Vec2{1.f, 0.f});

            world.makeConstraint<WeldConstraint>(Bodies{body1, body2});

            auto trans = world.makeConstraint<TranslationMotor>(
                Bodies::singleBody(body2),
                TranslationMotor::Specs::fromAccel(
                    2.f,  // maximum speed of 2 meter per second
                    0.5f, // maximum acceleration of 0.5 meters per second
                    body1->mass() + body2->mass() // total mass
                )
            );

            trans->direction(Vec2{1, 0});
            world.step(1.f);
            REQUIRE(all(body1->velocity() == Vec2{0.5f, 0}));
            REQUIRE(all(body2->velocity() == Vec2{0.5f, 0}));

            world.step(1.f);
            REQUIRE(all(body1->velocity() == Vec2{1.f, 0}));
            REQUIRE(all(body2->velocity() == Vec2{1.f, 0}));

            world.step(3.f);
            REQUIRE(all(
                approx(body1->velocity(), Vec2::filled(0.01f)).contains(Vec2{2.f, 0})
            ));
            REQUIRE(all(
                approx(body2->velocity(), Vec2::filled(0.01f)).contains(Vec2{2.f, 0})
            ));

            trans->direction(Vec2{-1, 0});
            world.step(1.f);
            REQUIRE(all(approx(body1->velocity(), Vec2::filled(0.01f))
                            .contains(Vec2{1.5f, 0})));
            REQUIRE(all(approx(body2->velocity(), Vec2::filled(0.01f))
                            .contains(Vec2{1.5f, 0})));

            world.step(10.f);
            REQUIRE(all(approx(body1->velocity(), Vec2::filled(0.05f))
                            .contains(Vec2{-2.f, 0})));
            REQUIRE(all(approx(body2->velocity(), Vec2::filled(0.05f))
                            .contains(Vec2{-2.f, 0})));
        }
    }

    SECTION("Collisions")
    {
        SECTION("No collision restitution")
        {
            World world{};

            auto body1 = makeBox(world, Vec2{-2.f, 0.f});
            auto body2 = makeBox(world, Vec2{1.f, 0.f});

            body1->setVelocity(Vec2{0.5f, 0});
            body2->setVelocity(Vec2{-0.5f, 0});

            world.step(1.f);
            world.step(1.f);   // touching at x=0
            world.step(0.01f); // collision
            world.step(0.01f);

            REQUIRE(all(approx(body1->velocity(), Vec2::filled(simu::EPSILON))
                            .contains(Vec2{})));
            REQUIRE(all(approx(body2->velocity(), Vec2::filled(simu::EPSILON))
                            .contains(Vec2{})));
        }

        SECTION("Full collision restitution")
        {
            World world{};

            BodyDescriptor     descr{};
            ColliderDescriptor cDescr{Polygon::box(Vec2::filled(1.f))};
            cDescr.material.bounciness.value = 1.f;

            descr.position = Vec2{-2, 0};
            auto body1     = world.makeBody(descr);
            body1->addCollider(cDescr);
            body1->setVelocity(Vec2{0.5f, 0});

            descr.position = Vec2{1, 0};
            auto body2     = world.makeBody(descr);
            body2->addCollider(cDescr);
            body2->setVelocity(Vec2{-0.5f, 0});

            world.step(1.f);
            world.step(1.f);   // touching at x=0
            world.step(0.01f); // collision
            world.step(0.01f);

            REQUIRE(all(approx(body1->velocity(), Vec2::filled(simu::EPSILON))
                            .contains(Vec2{-0.5f, 0})));
            REQUIRE(all(approx(body2->velocity(), Vec2::filled(simu::EPSILON))
                            .contains(Vec2{0.5f, 0})));
        }
    }

    SECTION("Body dominance")
    {
        SECTION("Collision with structural")
        {
            World world{};

            BodyDescriptor     descr{};
            ColliderDescriptor cDescr{Polygon::box(Vec2::filled(1.f))};
            cDescr.material.bounciness.value = 1.f;

            descr.position  = Vec2{-2, 0};
            auto projectile = world.makeBody(descr);
            projectile->addCollider(cDescr);
            projectile->setVelocity(Vec2{1.f, 0.f});
            REQUIRE(!projectile->isStructural());

            descr.position  = Vec2{0, 0};
            descr.dominance = 0.f;
            auto wall       = world.makeBody(descr);
            wall->addCollider(cDescr);
            REQUIRE(wall->isStructural());

            world.step(1.1f); // collision
            world.step(0.1f);

            REQUIRE(all(wall->velocity() == Vec2{}));
            REQUIRE(all(approx(projectile->velocity(), Vec2::filled(simu::EPSILON))
                            .contains(Vec2{-1.f, 0.f})));
        }

        SECTION("Dominance for movement propagation")
        {
            World world{};

            float x        = 0.f;
            auto  driver   = makeBox(world, Vec2{x, 0.f});
            auto  middle   = makeBox(world, Vec2{x += 2, 0.f});
            auto  follower = makeBox(world, Vec2{x += 2, 0.f});

            world.makeConstraint<RotationConstraint>(
                Bodies{
                    {driver, middle},
                    Vec2{0.f,    1.f   }
            },
                true
            );

            world.makeConstraint<RotationConstraint>(
                Bodies{
                    {middle, follower},
                    Vec2{0.f,    1.f     }
            },
                true
            );

            driver->setAngularVelocity(1.f);
            world.step(1.f);

            REQUIRE(driver->angularVelocity() == 1.f);
            REQUIRE(middle->angularVelocity() == 1.f);
            REQUIRE(follower->angularVelocity() == 1.f);
        }
    }

    SECTION("Motorcycle")
    {
        constexpr float wheelRadius     = 0.5f;
        constexpr float fromGroundToPos = 2 * wheelRadius + 0.5f;

        struct MotorCycle : public Body
        {
            MotorCycle(Vec2 pos) : Body{BodyDescriptor{pos}} {}

            void onConstruction(World& world) override
            {
                ColliderDescriptor cDescr{Polygon::box(Vec2{2.f, 1.f})};
                addCollider(cDescr);

                BodyDescriptor     wheelDescr{};
                ColliderDescriptor wheelColliderDescr = wheelDescriptor();

                wheelDescr.position = toWorldSpace() * Vec2{-1.f, -1.f};
                rearWheel           = world.makeBody(wheelDescr);
                rearWheel->addCollider(wheelColliderDescr);

                wheelDescr.position = toWorldSpace() * Vec2{1.f, -1.f};
                frontWheel          = world.makeBody(wheelDescr);
                frontWheel->addCollider(wheelColliderDescr);

                rearHinge = world.makeConstraint<HingeConstraint>(
                    Bodies{this, rearWheel}, rearWheel->centroid()
                );

                frontHinge = world.makeConstraint<HingeConstraint>(
                    Bodies{this, frontWheel}, frontWheel->centroid()
                );

                float totalMass = mass() + 2 * rearWheel->mass();

                motor = world.makeConstraint<RotationMotor>(
                    Bodies::singleBody(rearWheel),
                    RotationMotor::Specs::fromTargetForce(
                        2 * pi, wheelRadius, 10.f * totalMass
                    )
                );
            }

            void onKill() override
            {
                // since this->shouldDie()
                REQUIRE(rearHinge->isDead());
                REQUIRE(frontHinge->isDead());

                rearWheel->kill();
                REQUIRE(motor->isDead());

                frontWheel->kill();
            }

            static ColliderDescriptor wheelDescriptor()
            {
                std::vector<Vec2> points;
                Uint32            nPoints = 24;
                for (Uint32 i = 0; i < nPoints; ++i)
                {
                    float theta = i * 2 * pi / nPoints;
                    points.emplace_back(wheelRadius* Vec2{
                        std::cos(theta), std::sin(theta)});
                }

                Material material{};
                material.friction.value = 1.f;

                return ColliderDescriptor{
                    Polygon{points.begin(), points.end()},
                    material
                };
            }

            Body*            rearWheel = nullptr;
            HingeConstraint* rearHinge = nullptr;

            Body*            frontWheel = nullptr;
            HingeConstraint* frontHinge = nullptr;

            RotationMotor* motor = nullptr;
        };

        World world{};
        auto  motorcycle = world.makeBody<MotorCycle>(Vec2{0.f, 2.5f});

        REQUIRE(motorcycle->rearWheel != nullptr);
        REQUIRE(motorcycle->rearHinge != nullptr);
        REQUIRE(motorcycle->frontWheel != nullptr);
        REQUIRE(motorcycle->frontHinge != nullptr);
        REQUIRE(motorcycle->motor != nullptr);

        BodyDescriptor groundDescr{};
        groundDescr.dominance = 0.f;

        ColliderDescriptor groundColliderDescr{
            Polygon::box(Vec2{10.f, 1.f}, Vec2{0.f, -0.5f})};

        auto ground = world.makeBody(groundDescr);
        ground->addCollider(groundColliderDescr);

        auto gravity = world.makeForceField<Gravity>(Vec2{0, -10.f});

        world.step(0.2f);
        for (Uint32 i = 0; i < 25; ++i)
            world.step(0.01f);

        float           pen = groundColliderDescr.material.penetration.value;
        Interval<float> restingheight{
            fromGroundToPos - ContactConstraint::sinkTolerance * pen,
            fromGroundToPos - pen};

        REQUIRE(restingheight.contains(motorcycle->position()[1]));

        motorcycle->motor->direction(Vector<float, 1>{-1});
        motorcycle->motor->throttle(1.f);

        for (Uint32 i = 0; i < 25; ++i)
            world.step(0.01f);

        REQUIRE(motorcycle->rearWheel->velocity()[0] > 0);
        REQUIRE(motorcycle->frontWheel->velocity()[0] > 0);
        REQUIRE(motorcycle->velocity()[0] > 0);

        REQUIRE(motorcycle->rearWheel->angularVelocity() < 0);
        REQUIRE(motorcycle->frontWheel->angularVelocity() < 0);

        motorcycle->kill();
        ground->kill();
        gravity->kill();
        world.step(0.1f);
        REQUIRE(world.bodies().empty());
        REQUIRE(world.constraints().empty());
        REQUIRE(world.forceFields().empty());
    }

    SECTION("Contact constraint divergence issue")
    {
        World world{};

        BodyDescriptor groundDescr{};
        groundDescr.dominance = 0.f;

        ColliderDescriptor groundColliderDescr{
            Polygon::box(Vec2{10.f, 1.f}, Vec2{0.f, -0.5f})};

        auto ground = world.makeBody(groundDescr);
        ground->addCollider(groundColliderDescr);

        auto gravity = world.makeForceField<Gravity>(Vec2{0, -10.f});


        BodyDescriptor wheelDescr{
            Vec2{1.06283140f, 0.494556785f},
            -0.132951573f
        };

        std::vector<Vec2> points;
        Uint32            nPoints = 24;
        for (Uint32 i = 0; i < nPoints; ++i)
        {
            float theta = i * 2 * pi / nPoints;
            points.emplace_back(0.5f * Vec2{std::cos(theta), std::sin(theta)});
        }

        Material material{};
        material.friction.value = 1.f;

        ColliderDescriptor wheelColliderDescr{
            Polygon{points.begin(), points.end()},
            material
        };

        auto wheel = world.makeBody(wheelDescr);
        wheel->addCollider(wheelColliderDescr);

        Vec2 initialVel{0.563797712f, -0.153355882f};
        wheel->setVelocity(initialVel);
        wheel->setAngularVelocity(-1.12384140f);

        world.step(0.01f);
        world.step(0.01f);
        world.step(0.01f);

        // exact values are not important here, currently the velocity diverges to ~{0, 3}
        REQUIRE(approx(wheel->velocity()[1], 0.1f).contains(0.f));
    }


    SECTION("Box stack")
    {
        // This test is very slow, but necessary.
        // Even if the stack seems stable after a few steps, we must ensure that
        //  the stack can hold for very long
        //  (no fatal accumulation of errors, no awkward contact manifold problems, etc.)

        World world{};
        world.makeForceField<Gravity>(Vec2{0.f, -10.f});

        auto makeBox = [&](Int32 index) -> Body* {
            BodyDescriptor descr{};
            descr.position[1] = static_cast<float>(index);

            ColliderDescriptor cDescr{
                Polygon::box(Vec2{1.f, 1.f}, Vec2{0.5f, 0.5f})};
            cDescr.material.friction.value = 0.5f;

            auto b = world.makeBody(descr);
            b->addCollider(cDescr);
            return b;
        };

        BodyDescriptor floorDescr{};
        floorDescr.dominance = 0.f;

        // TODO: Using an offset center {0, -0.5f} causes issues
        //  where the boxes have a big orientation, yet they are stable
        CHECK(false);
        ColliderDescriptor floorColliderDescr{
            Polygon::box(Vec2{4.f, 1.f})};
        floorColliderDescr.material.friction.value = 0.5f;

        Body* floor = world.makeBody(floorDescr);
        floor->addCollider(floorColliderDescr);

        constexpr Int32           nBoxes = 10;
        std::array<Body*, nBoxes> boxes{};

        Int32 i = 0;
        for (auto& box : boxes)
            box = makeBox(i++);

        // TODO: For stability, use at least as many velocity iterations as the
        // height of the tower.
        //  (Needs world.settings)

        for (Uint32 steps = 0; steps < 6000; ++steps)
        {
            world.step(0.01f);

            for (auto& box : boxes)
            {
                if (!approx(box->orientation(), 1 * simu::EPSILON).contains(0.f))
                {
                    box->setAngularVelocity(box->angularVelocity());
                    break;
                }
            }
        }

        float floorHeight = 0.5f;
        float pen         = floorColliderDescr.material.penetration.value;
        i                 = 0;
        const float err   = 0.001f;
        for (const Body* box : boxes)
        {
            Interval<float> restingheight{
                floorHeight - ContactConstraint::sinkTolerance * pen,
                floorHeight};

            REQUIRE(approx(box->orientation(), 0.2f).contains(0.f));
            REQUIRE(approx(box->angularVelocity(), err).contains(0.f));

            REQUIRE(approx(box->position()[0], 0.1f).contains(0.f));
            REQUIRE(restingheight.contains(box->position()[1]));

            REQUIRE(approx(box->velocity()[0], err).contains(0.f));
            REQUIRE(approx(box->velocity()[1], 0.005f).contains(0.f));

            floorHeight = box->position()[1] + 1.f;
        }
    }

    SECTION("Ranges types")
    {
        World        world{};
        const World& cWorld = world;

        // clang-format off
        STATIC_REQUIRE(std::is_same_v<decltype(*world.bodies().begin()), Body&>);
        STATIC_REQUIRE(std::is_same_v<decltype(*cWorld.bodies().begin()), const Body&>);

        STATIC_REQUIRE(std::is_same_v<decltype(*world.constraints().begin()), Constraint&>);
        STATIC_REQUIRE(std::is_same_v<decltype(*cWorld.constraints().begin()), const Constraint&>);

        STATIC_REQUIRE(std::is_same_v<decltype(*world.forceFields().begin()), ForceField&>);
        STATIC_REQUIRE(std::is_same_v<decltype(*cWorld.forceFields().begin()), const ForceField&>);
        // clang-format on
    }
}
