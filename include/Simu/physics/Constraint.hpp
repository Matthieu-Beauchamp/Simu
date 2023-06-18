////////////////////////////////////////////////////////////
//
// Simu
// Copyright (C) 2023 Matthieu Beauchamp-Boulay
//
// This software is provided 'as-is', without any express or implied warranty.
// In no event will the authors be held liable for any damages arising from the use of this software.
//
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it freely,
// subject to the following restrictions:
//
// 1. The origin of this software must not be misrepresented;
//    you must not claim that you wrote the original software.
//    If you use this software in a product, an acknowledgment
//    in the product documentation would be appreciated but is not required.
//
// 2. Altered source versions must be plainly marked as such,
//    and must not be misrepresented as being the original software.
//
// 3. This notice may not be removed or altered from any source distribution.
//
////////////////////////////////////////////////////////////

#pragma once

#include <optional>

#include "Simu/config.hpp"
#include "Simu/math/BarycentricCoordinates.hpp"
#include "Simu/math/Gjk.hpp"

#include "Simu/physics/PhysicsBody.hpp"
#include "Simu/physics/ContactManifold.hpp"
#include "Simu/physics/ConstraintFunction.hpp"
#include "Simu/physics/ConstraintSolver.hpp"


namespace simu
{

class Constraint : public PhysicsObject
{
public:

    ~Constraint() override = default;

    virtual bool isActive()          = 0;
    virtual void initSolve(float dt) = 0;
    virtual void solve(float dt)     = 0;
    virtual void commit()            = 0;
};

template <ConstraintFunction F>
class ConstraintImplementation : public Constraint
{
public:

    typedef F::Value Value;

    ConstraintImplementation(
        const Bodies<F::nBodies>&                bodies,
        const F&                                 f,
        bool                                     disableContacts,
        std::optional<Vector<float, F::nBodies>> dominanceRatios = std::nullopt
    )
        : f{f}, bodies_{bodies}, disableContacts_{disableContacts}
    {
        if (dominanceRatios.has_value())
        {
            dominances_ = dominanceRatios.value();
        }
        else
        {
            Uint32 i = 0;
            for (auto body : bodies)
                dominances_[i++] = body->dominance();
        }
    }

    void onConstruction(PhysicsWorld& world) override;
    void onDestruction(PhysicsWorld& world) override;

    bool shouldDie() const override
    {
        for (const PhysicsBody* body : bodies_)
            if (body->isDead())
                return true;

        return false;
    }

    bool isActive() override
    {
        eval_ = f.eval(bodies_);
        return f.isActive(eval_);
    }

    void initSolve(float dt) override
    {
        // TODO: Apply guess based on previous lambda
        // if (warmStarting)
        //      solver.applyImpulse(oldJacobian, oldLambda*dt/oldDt)
        //
        // ConstraintSolver::apply(
        //     bodies_,
        //     ConstraintSolver::impulse<F::dimension, F::nBodies>(jacobian_, lambda_)
        // );
        //
        // Careful! motor constraints should not apply twice!

        jacobian_ = f.jacobian(bodies_);
        bias_     = f.bias(bodies_);
        lambda_   = Value{};
    }

    void solve(float dt) override
    {
        Value dLambda = ConstraintSolver::solveLambda<F::dimension, F::nBodies>(
            bodies_,
            dominances_,
            eval_,
            jacobian_,
            dt,
            bias_, // TODO: This may need recomputing (depends on velocity?)
            f.restitution(),
            f.damping(),
            lambda_
        );

        Value oldLambda = lambda_;
        lambda_ += dLambda;
        lambda_ = f.clampLambda(lambda_, dt);

        VelocityVector<F::nBodies> impulse
            = ConstraintSolver::impulse<F::dimension, F::nBodies>(
                jacobian_,
                lambda_ - oldLambda
            );

        ConstraintSolver::apply(bodies_, dominances_, impulse);
    }

    void commit() override
    {
        // TODO: Do position correction with split impulses here?
    }

    F f;

private:

    Bodies<F::nBodies> bodies_;

    F::Jacobian jacobian_{};
    F::Value    eval_{};
    F::Value    bias_{};
    F::Value    lambda_{};

    Vector<float, F::nBodies> dominances_;
    bool                      disableContacts_;
};


////////////////////////////////////////////////////////////
// Predefined constraints
////////////////////////////////////////////////////////////

class RotationConstraint
    : public ConstraintImplementation<RotationConstraintFunction>
{
public:

    typedef ConstraintImplementation<RotationConstraintFunction> Base;
    typedef RotationConstraintFunction                           F;

    struct Descriptor : public F::Descriptor
    {
        Descriptor(float restitution = 0.2f, float damping = 0.f)
            : F::Descriptor{
                Vector<float, 1>{restitution},
                Vector<float, 1>{damping}}
        {
        }
    };

    RotationConstraint(const Bodies<2>& bodies, 
        Descriptor descr = Descriptor{}, 
        bool disableContacts = true, 
        std::optional<Vec2> dominanceRatios = std::nullopt) 
        : Base{bodies, F{descr, bodies}, disableContacts, dominanceRatios}
    {
    }
};

class HingeConstraint : public ConstraintImplementation<HingeConstraintFunction>
{
public:

    typedef ConstraintImplementation<HingeConstraintFunction> Base;
    typedef HingeConstraintFunction                           F;

    struct Descriptor : public F::Descriptor
    {
        Descriptor(float restitution = 0.2f, float damping = 0.f)
            : F::Descriptor{Vec2::filled(restitution), Vec2::filled(damping)}
        {
        }
    };

    HingeConstraint(
        const Bodies<2>& bodies,
        Vec2             worldSpaceSharedPoint,
        Descriptor    descr = Descriptor{}, bool disableContacts = true, std::optional<Vec2> dominanceRatios = std::nullopt
    ) : Base{bodies, F{descr, bodies, worldSpaceSharedPoint}, disableContacts, dominanceRatios}
    {
    }
};


typedef ConstraintFunctions<HingeConstraintFunction, RotationConstraintFunction>
    WeldConstraintFunction;

class WeldConstraint : public ConstraintImplementation<WeldConstraintFunction>
{
public:

    typedef HingeConstraintFunction    HingeF;
    typedef RotationConstraintFunction RotF;
    typedef WeldConstraintFunction     F;

    typedef ConstraintImplementation<F> Base;

    struct Descriptor : public HingeF::Descriptor, RotF::Descriptor
    {
        Descriptor(
            float restitution        = 0.2f,
            float angularRestitution = 0.2f,
            float damping            = 0.f,
            float angularDamping     = 0.f
        )
            : HingeF::Descriptor{Vec2::filled(restitution), Vec2::filled(damping)},
              RotF::Descriptor{
                  Vector<float, 1>{angularRestitution},
                  Vector<float, 1>{angularDamping}}
        {
        }
    };

    WeldConstraint(
        const Bodies<2>&    bodies,
        Descriptor          descr           = Descriptor{},
        bool                disableContacts = true,
        std::optional<Vec2> dominanceRatios = std::nullopt
    )
        : Base{
            bodies,
            makeWeldFunction(bodies, descr),
            disableContacts,
            dominanceRatios}
    {
    }

private:

    static Vec2 centroidMidPoint(const Bodies<2>& bodies)
    {
        return (bodies[0]->properties().centroid
                + bodies[1]->properties().centroid)
               / 2;
    }

    static HingeF makeHingeFunction(const Bodies<2>& bodies, Descriptor descr)
    {
        return HingeF{
            static_cast<HingeF::Descriptor>(descr),
            bodies,
            centroidMidPoint(bodies)};
    }

    static RotF makeRotFunction(const Bodies<2>& bodies, Descriptor descr)
    {
        return RotF{static_cast<RotF::Descriptor>(descr), bodies};
    }

    static F makeWeldFunction(const Bodies<2>& bodies, Descriptor descr)
    {
        return F{makeHingeFunction(bodies, descr), makeRotFunction(bodies, descr)};
    }
};


class RotationMotorConstraint
    : public ConstraintImplementation<RotationMotorConstraintFunction>
{
public:

    typedef RotationMotorConstraintFunction F;
    typedef ConstraintImplementation<F>     Base;

    RotationMotorConstraint(
        const Bodies<1>& bodies,
        float            maxAngularVelocity,
        float            maxTorque
    )
        : Base{
            bodies,
            F{bodies, maxAngularVelocity, maxTorque},
            false,
            Vector<float, 1>{1.f}
    }
    {
    }

    float throttle() const { return f.throttle(); }
    void  throttle(float throttle) { f.throttle(throttle); }

    Value direction() const { return f.direction(); }
    void  direction(Value direction) { f.direction(direction); }
};


class TranslationMotorConstraint
    : public ConstraintImplementation<TranslationMotorConstraintFunction>
{
public:

    typedef TranslationMotorConstraintFunction F;
    typedef ConstraintImplementation<F>        Base;

    TranslationMotorConstraint(const Bodies<1>& bodies, float maxVelocity, float maxForce)
        : Base{
            bodies,
            F{bodies, maxVelocity, maxForce},
            false,
            Vector<float, 1>{1.f}

    }
    {
    }

    float throttle() const { return f.throttle(); }
    void  throttle(float throttle) { f.throttle(throttle); }

    Value direction() const { return f.direction(); }
    void  direction(Value direction) { f.direction(direction); }
};


class SingleContactConstraint
    : public ConstraintImplementation<SingleContactFunction>
{
public:

    typedef ConstraintImplementation<SingleContactFunction> Base;

    typedef ContactManifold<Collider> Manifold;

    SingleContactConstraint(const Bodies<2>& bodies, const Manifold& manifold)
        : Base{bodies, makeFunction(bodies, manifold), false, std::nullopt}
    {
    }

private:

    static SingleContactFunction
    makeFunction(const ConstBodies<2>& bodies, const Manifold& manifold)
    {
        return SingleContactFunction{
            NonPenetrationConstraintFunction{bodies, manifold, 0},
            FrictionConstraintFunction{bodies, manifold}
        };
    }
};

class DoubleContactConstraint
    : public ConstraintImplementation<DoubleContactFunction>
{
public:

    typedef ConstraintImplementation<DoubleContactFunction> Base;

    typedef ContactManifold<Collider> Manifold;

    DoubleContactConstraint(const Bodies<2>& bodies, const Manifold& manifold)
        : Base{bodies, makeFunction(bodies, manifold), false, std::nullopt}
    {
    }

private:

    static DoubleContactFunction
    makeFunction(const ConstBodies<2>& bodies, const Manifold& manifold)
    {
        return DoubleContactFunction{
            NonPenetrationConstraintFunction{bodies, manifold, 0},
            NonPenetrationConstraintFunction{bodies, manifold, 1},
            FrictionConstraintFunction{bodies, manifold}
        };
    }
};

struct Contact
{
    Contact(const Bodies<2>& bodies) : bodies{bodies} {}

    Gjk<Collider> makeGjk() const
    {
        return Gjk<Collider>{bodies[0]->collider(), bodies[1]->collider()};
    }

    // gjk must return a non null penetration to make a manifold
    ContactManifold<Collider> makeManifold() const
    {
        return makeManifold(makeGjk());
    }
    ContactManifold<Collider> makeManifold(const Gjk<Collider>& gjk) const
    {
        return ContactManifold<Collider>{
            PointerArray<Collider, 2, true>{
                                            &bodies[0]->collider(),
                                            &bodies[1]->collider()},
            gjk.penetration()
        };
    }

    Bodies<2> bodies;
};

class ContactConstraint : public Constraint
{
public:

    ContactConstraint(const Bodies<2>& bodies) : contact_{bodies} {}

    bool isActive() override
    {
        auto gjk = contact_.makeGjk();
        if (all(gjk.penetration() == Vec2{}))
            return false;

        auto manifold = contact_.makeManifold(gjk);
        makeContactConstraint(manifold);

        return contactConstraint_->isActive();
    }

    void initSolve(float dt) override { contactConstraint_->initSolve(dt); }
    void solve(float dt) override { contactConstraint_->solve(dt); }
    void commit() override { contactConstraint_->commit(); }

private:


    Contact                     contact_;
    std::unique_ptr<Constraint> contactConstraint_ = nullptr;

    void makeContactConstraint(const ContactManifold<Collider>& manifold)
    {
        if (manifold.nContacts == 1)
            contactConstraint_ = std::make_unique<SingleContactConstraint>(
                contact_.bodies,
                manifold
            );
        else
            contactConstraint_ = std::make_unique<DoubleContactConstraint>(
                contact_.bodies,
                manifold
            );
    }
};


} // namespace simu

#include "Simu/physics/Constraint.inl.hpp"
