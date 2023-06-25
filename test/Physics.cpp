#include <numbers>

#include "catch2/catch_test_macros.hpp"

#include "Simu/utility/FloatingPointExceptions.hpp"

#include "Simu/physics/PhysicsWorld.hpp"
#include "Simu/physics/PhysicsBody.hpp"
#include "Simu/physics/Constraint.hpp"
#include "Simu/physics/Motors.hpp"

using namespace simu;

const std::initializer_list<Vec2> square{
    Vec2{0, 0},
    Vec2{1, 0},
    Vec2{1, 1},
    Vec2{0, 1},
};

const BodyDescriptor squareDescriptor{Polygon{square}};

const float pi = std::numbers::pi_v<float>;

TEST_CASE("Physics")
{
    SECTION("Single body stepping")
    {
        const Vec2  p{1, 1};
        const Vec2  v{1, 1};
        const float theta = pi / 4;
        const float w     = pi / 2;

        PhysicsWorld world{};

        BodyDescriptor descr{squareDescriptor};
        descr.position    = p;
        descr.orientation = theta;
        auto body         = world.makeBody(descr);

        body->velocity()        = v;
        body->angularVelocity() = w;

        REQUIRE(all(body->position() == p));
        REQUIRE(all(body->velocity() == v));
        REQUIRE(body->orientation() == theta);
        REQUIRE(body->angularVelocity() == w);

        float dt = 3.f;
        world.step(dt);
        REQUIRE(all(body->position() == p + dt * v));
        REQUIRE(all(body->velocity() == v));
        REQUIRE(body->orientation() == theta + dt * w);
        REQUIRE(body->angularVelocity() == w);

        // rotation is done about the polygon's centroid.
        // body->position is the position of the origin of the body's local space, rotation not considered.
        // The centroid is not necessarilly centered at {0, 0} in local space.
        REQUIRE(all(
            body->properties().centroid
            == squareDescriptor.polygon.properties().centroid + body->position()
        ));
    }

    SECTION("Global force fields (gravity)")
    {
        PhysicsWorld world{};

        float initialHeight = 100.f;


        std::vector<PhysicsWorld::BodyPtr> bodies;
        constexpr Uint32                   nBodies = 5;
        for (Uint32 i = 0; i < nBodies; ++i)
        {
            BodyDescriptor descr{squareDescriptor};
            descr.position = Vec2{static_cast<float>(i), initialHeight};
            bodies.emplace_back(world.makeBody(descr));
        }

        Vec2 gravity = Vec2{0.f, -10.f};
        world.makeForceField<Gravity>(gravity);

        float dt = 1.f;
        world.step(dt);
        for (auto body : bodies)
        {
            // implicitely requires use of symplectic euler integrator
            REQUIRE(all(body->velocity() == dt * gravity));
            REQUIRE(
                body->position()[1] == initialHeight + dt * body->velocity()[1]
            );
        }
    }

    SECTION("Multiple force fields")
    {
        PhysicsWorld world{};

        BodyDescriptor descr{squareDescriptor};

        Vec2 leftPos{-10.f, 0.f};
        descr.position = leftPos;
        auto left      = world.makeBody(descr);

        Vec2 middlePos{0.f, 0.f};
        descr.position = middlePos;
        auto middle    = world.makeBody(descr);

        Vec2 rightPos{10.f, 0.f};
        descr.position = rightPos;
        auto right     = world.makeBody(descr);

        Vec2 leftForce{1.f, 0};
        world.makeForceField<LinearField>(
            leftForce,
            ForceField::region(Vec2{-10.f, 0.f}, Vec2{0.f, 10.f})
        );

        Vec2 middleForce{0.f, -1.f};
        world.makeForceField<LinearField>(
            middleForce,
            ForceField::region(Vec2{-2.f, 0.f}, Vec2{2.f, 10.f})
        );

        Vec2 rightForce{-1.f, 0};
        world.makeForceField<LinearField>(
            rightForce,
            ForceField::region(Vec2{0.f, 0.f}, Vec2{10.f, 10.f})
        );

        float dt = 1.f;
        world.step(dt);

        REQUIRE(all(left->velocity() == leftForce * dt / left->properties().mass)
        );
        REQUIRE(all(left->position() == leftPos + left->velocity() * dt));

        // overlaps with all forces, left and right cancel out
        REQUIRE(all(
            middle->velocity() == middleForce * dt / middle->properties().mass
        ));
        REQUIRE(all(middle->position() == middlePos + middle->velocity() * dt));

        REQUIRE(
            all(right->velocity() == rightForce * dt / right->properties().mass)
        );
        REQUIRE(all(right->position() == rightPos + right->velocity() * dt));
    }

    SECTION("Constraints")
    {
        SECTION("Rotation constraint")
        {
            PhysicsWorld world{};

            BodyDescriptor descr{squareDescriptor};
            descr.position = Vec2{-10.f, 0};
            auto left      = world.makeBody(descr);

            descr.position = Vec2{10.f, 0};
            auto right     = world.makeBody(descr);


            auto c
                = world.makeConstraint<RotationConstraint>(Bodies<2>{left, right}
                );

            left->applyImpulse(Vec2{0, 1}, Vec2{1, 0});
            left->applyImpulse(Vec2{0, -1}, Vec2{-1, 0});
            float w = left->angularVelocity();
            REQUIRE(w > 0.f);

            world.step(1.f);

            REQUIRE(left->angularVelocity() == w / 2);
            REQUIRE(right->angularVelocity() == w / 2);

            right->angularVelocity() -= w;
            world.step(1.f);
            REQUIRE(left->angularVelocity() == 0.f);
            REQUIRE(right->angularVelocity() == 0.f);
        }

        SECTION("Hinge constraint")
        {
            PhysicsWorld world{};

            BodyDescriptor descr{squareDescriptor};
            descr.position = Vec2{-1.f, 0};
            auto left      = world.makeBody(descr);

            descr.position = Vec2{0.f, 0};
            auto right     = world.makeBody(descr);

            auto c = world.makeConstraint<HingeConstraint>(
                Bodies<2>{left, right},
                Vec2{0.f, 0.5f}
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

            // TODO: Why is there an error in C when this is the only constraint?

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
            PhysicsWorld world{};

            BodyDescriptor descr{squareDescriptor};
            descr.position = Vec2{0, 0};
            auto left      = world.makeBody(descr);

            descr.position = left->properties().centroid;
            auto right     = world.makeBody(descr);

            auto c = world.makeConstraint<HingeConstraint>(
                Bodies<2>{left, right},
                left->properties().centroid,
                true
            );

            left->angularVelocity() = 1.f;
            world.step(1.f);

            REQUIRE(left->angularVelocity() == 1.f);
            REQUIRE(right->angularVelocity() == 0.f);
            REQUIRE(all(left->velocity() == Vec2{}));
            REQUIRE(all(right->velocity() == Vec2{}));

            // remove constraint, bodies are now colliding
            c->kill();
            world.step(1.f);
            REQUIRE(all(left->velocity() != Vec2{}));
            REQUIRE(all(right->velocity() != Vec2{}));
        }

        SECTION("Weld constraint")
        {
            PhysicsWorld world{};

            BodyDescriptor descr{squareDescriptor};
            descr.position = Vec2{-1.f, 0};
            auto left      = world.makeBody(descr);

            descr.position = Vec2{0.f, 0};
            auto right     = world.makeBody(descr);

            auto c
                = world.makeConstraint<WeldConstraint>(Bodies<2>{left, right});

            // applied at the centroid of the combined bodies
            right->applyImpulse(Vec2{0.f, 1.f}, Vec2{-0.5f, 0});
            REQUIRE(right->angularVelocity() != 0.f);
            REQUIRE(right->velocity()[1] != 0.f);

            world.step(1.f);

            REQUIRE(left->angularVelocity() == 0.f);
            REQUIRE(right->angularVelocity() == 0.f);

            REQUIRE(all(left->velocity() == Vec2{0.f, 0.5f}));
            REQUIRE(all(right->velocity() == Vec2{0.f, 0.5f}));

            left->applyImpulse(Vec2{0.f, -1.f}, Vec2{0.5f, 0});
            world.step(1.f);

            REQUIRE(left->angularVelocity() == 0.f);
            REQUIRE(right->angularVelocity() == 0.f);

            REQUIRE(all(left->velocity() == Vec2{}));
            REQUIRE(all(right->velocity() == Vec2{}));
        }

        SECTION("Motors")
        {
            PhysicsWorld world{};

            BodyDescriptor descr{squareDescriptor};

            descr.position = Vec2{0, 0};
            auto body      = world.makeBody(descr);

            constexpr float pi = std::numbers::pi_v<float>;

            auto rot = world.makeConstraint<RotationMotor>(
                Bodies<1>{body},
                RotationMotor::Specs::fromAccel(
                    pi * 2, // maximum speed of 1 turn per second
                    pi / 2  // maximum acceleration of 1/4 turn per second
                )
            );

            auto trans = world.makeConstraint<TranslationMotor>(
                Bodies<1>{body},
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
            PhysicsWorld world{};

            BodyDescriptor descr{squareDescriptor};

            descr.position = Vec2{0, 0};
            auto body      = world.makeBody(descr);

            auto trans = world.makeConstraint<TranslationMotor>(
                Bodies<1>{body},
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
            PhysicsWorld world{};

            BodyDescriptor descr{squareDescriptor};

            descr.position = Vec2{0, 0};
            auto body1     = world.makeBody(descr);
            descr.position = Vec2{1, 0};
            auto body2     = world.makeBody(descr);

            world.makeConstraint<WeldConstraint>(Bodies<2>{body1, body2});

            auto trans = world.makeConstraint<TranslationMotor>(
                Bodies<1>{body2},
                TranslationMotor::Specs::fromAccel(
                    2.f,  // maximum speed of 2 meter per second
                    0.5f, // maximum acceleration of 0.5 meters per second
                    body1->properties().mass
                        + body2->properties().mass // total mass
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
            REQUIRE(all(approx(body1->velocity(), Vec2::filled(0.01f))
                            .contains(Vec2{2.f, 0})));
            REQUIRE(all(approx(body2->velocity(), Vec2::filled(0.01f))
                            .contains(Vec2{2.f, 0})));

            trans->direction(Vec2{-1, 0});
            world.step(1.f);
            REQUIRE(all(approx(body1->velocity(), Vec2::filled(0.01f))
                            .contains(Vec2{1.5f, 0})));
            REQUIRE(all(approx(body2->velocity(), Vec2::filled(0.01f))
                            .contains(Vec2{1.5f, 0})));

            world.step(10.f);
            REQUIRE(all(approx(body1->velocity(), Vec2::filled(0.01f))
                            .contains(Vec2{-2.f, 0})));
            REQUIRE(all(approx(body2->velocity(), Vec2::filled(0.01f))
                            .contains(Vec2{-2.f, 0})));
        }
    }

    SECTION("Collisions")
    {
        SECTION("No collision restitution")
        {
            PhysicsWorld world{};

            BodyDescriptor descr{squareDescriptor};
            descr.material.bounciness.value = 0.f;

            descr.position    = Vec2{-2, 0};
            auto body1        = world.makeBody(descr);
            body1->velocity() = Vec2{0.5f, 0};

            descr.position    = Vec2{1, 0};
            auto body2        = world.makeBody(descr);
            body2->velocity() = Vec2{-0.5f, 0};

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
            // PhysicsWorld world{};

            // BodyDescriptor descr{squareDescriptor};
            // descr.material.bounciness.value = 1.f;
            // descr.polygon = Polygon{
            //     Vec2{0.f, 0.f},
            //     Vec2{1.f, 0.5f},
            //     Vec2{0.f, 1.f},
            // };

            // descr.position    = Vec2{-2, 0};
            // auto body1        = world.makeBody(descr);
            // body1->velocity() = Vec2{0.5f, 0};

            // descr.position    = Vec2{1, 0};
            // auto body2        = world.makeBody(descr);
            // body2->velocity() = Vec2{-0.5f, 0};

            // world.step(1.f);
            // world.step(1.f);   // touching at x=0
            // world.step(0.01f); // collision
            // world.step(0.01f);

            PhysicsWorld world{};

            BodyDescriptor descr{squareDescriptor};
            descr.material.bounciness.value = 1.f;

            descr.position    = Vec2{-2, 0};
            auto body1        = world.makeBody(descr);
            body1->velocity() = Vec2{0.5f, 0};

            descr.position    = Vec2{1, 0};
            auto body2        = world.makeBody(descr);
            body2->velocity() = Vec2{-0.5f, 0};

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
            PhysicsWorld world{};

            BodyDescriptor descr{squareDescriptor};
            descr.material.bounciness.value = 1.f;

            descr.position         = Vec2{-2, 0};
            auto projectile        = world.makeBody(descr);
            projectile->velocity() = Vec2{1.f, 0.f};
            REQUIRE(!projectile->isStructural());

            descr.position  = Vec2{0, 0};
            descr.dominance = 0.f;
            auto wall       = world.makeBody(descr);
            REQUIRE(wall->isStructural());

            world.step(1.1f); // collision
            world.step(0.1f);

            REQUIRE(all(wall->velocity() == Vec2{}));
            REQUIRE(all(approx(projectile->velocity(), Vec2::filled(simu::EPSILON))
                            .contains(Vec2{-1.f, 0.f})));
        }

        SECTION("Dominance for movement propagation")
        {
            PhysicsWorld world{};

            BodyDescriptor descr{squareDescriptor};

            auto driver   = world.makeBody(descr);
            auto middle   = world.makeBody(descr);
            auto follower = world.makeBody(descr);

            world.makeConstraint<RotationConstraint>(
                Bodies<2>{driver, middle},
                true,
                Vec2{0.f, 1.f}
            );

            world.makeConstraint<RotationConstraint>(
                Bodies<2>{middle, follower},
                true,
                Vec2{0.f, 1.f}
            );

            driver->angularVelocity() = 1.f;
            world.step(1.f);

            REQUIRE(driver->angularVelocity() == 1.f);
            REQUIRE(middle->angularVelocity() == 1.f);
            REQUIRE(follower->angularVelocity() == 1.f);
        }
    }

    SECTION("Motorcycle")
    {
        constexpr float wheelRadius = 0.5f;

        struct MotorCycle : public PhysicsBody
        {
            MotorCycle(Vec2 pos)
                : PhysicsBody{
                    BodyDescriptor{
                                   Polygon{
                            Vec2{-1.f, -0.5f},
                            Vec2{1.f, -0.5f},
                            Vec2{1.f, 0.5f},
                            Vec2{-1.f, 0.5f}},
                                   Material{},
                                   pos}
            }
            {
            }

            void onConstruction(PhysicsWorld& world) override
            {
                BodyDescriptor wheelDescr = wheelDescriptor();
                wheelDescr.position       = toWorldSpace() * Vec2{-1.f, -1.5f};
                rearWheel                 = world.makeBody(wheelDescr);
                wheelDescr.position       = toWorldSpace() * Vec2{1.f, -1.5f};
                frontWheel                = world.makeBody(wheelDescr);

                rearHinge = world.makeConstraint<HingeConstraint>(
                    Bodies<2>{this, rearWheel},
                    rearWheel->properties().centroid
                );

                frontHinge = world.makeConstraint<HingeConstraint>(
                    Bodies<2>{this, frontWheel},
                    frontWheel->properties().centroid
                );

                float totalMass
                    = properties().mass + 2 * rearWheel->properties().mass;

                motor = world.makeConstraint<RotationMotor>(
                    Bodies<1>{rearWheel},
                    RotationMotor::Specs::fromTargetForce(
                        2 * pi,
                        wheelRadius,
                        10.f * totalMass
                    )
                );
            }

            void onKill() override
            {
                // since this->shouldDie()
                REQUIRE(rearHinge->shouldDie());
                REQUIRE(frontHinge->shouldDie());

                rearWheel->kill();
                REQUIRE(motor->shouldDie());

                frontWheel->kill();
            }

            static BodyDescriptor wheelDescriptor()
            {
                std::vector<Vec2> points;
                Uint32            nPoints = 24;
                for (Uint32 i = 0; i < nPoints; ++i)
                {
                    float theta = i * 2 * pi / nPoints;
                    points.emplace_back(
                        wheelRadius* Vec2{std::cos(theta), std::sin(theta)}
                    );
                }

                Material material{};
                material.friction.value = 1.f;

                return BodyDescriptor{
                    Polygon{points.begin(), points.end()},
                    material
                };
            }

            PhysicsBody*     rearWheel = nullptr;
            HingeConstraint* rearHinge = nullptr;

            PhysicsBody*     frontWheel = nullptr;
            HingeConstraint* frontHinge = nullptr;

            RotationMotor* motor = nullptr;
        };

        PhysicsWorld world{};
        auto         motorcycle = world.makeBody<MotorCycle>(Vec2{0.f, 2.5f});

        REQUIRE(motorcycle->rearWheel != nullptr);
        REQUIRE(motorcycle->rearHinge != nullptr);
        REQUIRE(motorcycle->frontWheel != nullptr);
        REQUIRE(motorcycle->frontHinge != nullptr);
        REQUIRE(motorcycle->motor != nullptr);

        BodyDescriptor groundDescr{
            Polygon{
                    Vec2{-5.f, -1.f},
                    Vec2{5.f, -1.f},
                    Vec2{5.f, 0.f},
                    Vec2{-5.f, 0.f}}
        };

        groundDescr.dominance = 0.f;

        auto ground  = world.makeBody(groundDescr);
        auto gravity = world.makeForceField<Gravity>(Vec2{0, -10.f});

        world.step(0.2f);
        for (Uint32 i = 0; i < 25; ++i)
            world.step(0.01f);

        REQUIRE(all(approx(motorcycle->position(), Vec2::filled(0.005f))
                        .contains(Vec2{0.f, 2.f})));

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

    SECTION("Box stack")
    {
        PhysicsWorld world{};
        world.makeForceField<Gravity>(Vec2{0.f, -10.f});

        auto makeBox = [&](Int32 index, float dominance = 1.f) -> PhysicsBody* {
            BodyDescriptor descr{squareDescriptor};
            descr.position[1]             = index;
            descr.dominance               = dominance;
            descr.material.friction.value = 0.5f;
            return world.makeBody(descr);
        };

        PhysicsBody* floor = makeBox(-1, 0.f);

        constexpr Int32                  nBoxes = 10;
        std::array<PhysicsBody*, nBoxes> boxes{};

        Int32 i = 0;
        for (auto& box : boxes)
            box = makeBox(i++);

        for (Uint32 steps = 0; steps < 100; ++steps)
        {
            world.step(0.01f);
        }

        i = 0;
        for (const PhysicsBody* box : boxes)
        {
            REQUIRE(all(approx(box->properties().centroid, Vec2::filled(0.01f))
                            .contains(Vec2{0.5f, 0.5f + i++})));
        }
    }

    SECTION("Ranges types")
    {
        PhysicsWorld        world{};
        const PhysicsWorld& cWorld = world;

        // clang-format off
        STATIC_REQUIRE(std::is_same_v<decltype(*world.bodies().begin()), PhysicsBody&>);
        STATIC_REQUIRE(std::is_same_v<decltype(*cWorld.bodies().begin()), const PhysicsBody&>);

        STATIC_REQUIRE(std::is_same_v<decltype(*world.constraints().begin()), Constraint&>);
        STATIC_REQUIRE(std::is_same_v<decltype(*cWorld.constraints().begin()), const Constraint&>);

        STATIC_REQUIRE(std::is_same_v<decltype(*world.forceFields().begin()), ForceField&>);
        STATIC_REQUIRE(std::is_same_v<decltype(*cWorld.forceFields().begin()), const ForceField&>);
        // clang-format on
    }
}