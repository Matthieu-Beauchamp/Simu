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

#include <memory>

#include "Simu/math/Gjk.hpp"
#include "Simu/physics/ContactManifold.hpp"

#include "Simu/physics/Body.hpp"

#include "Simu/physics/ConstraintFunction.hpp"
#include "Simu/physics/ConstraintSolver.hpp"

namespace simu
{

template <
    ConstraintFunction            F,
    ConstraintSolver              S    = EqualitySolver<F>,
    std::derived_from<Constraint> Base = Constraint>
class ConstraintImplementation : public Base
{
public:

    typedef typename F::Value Value;
    typedef S                 Solver;

    ConstraintImplementation(
        const Bodies<F::nBodies>& bodies,
        const F&                  f,
        bool                      disableContacts
    )
        : Base{},
          f{f},
          solver{bodies, f},
          bodies_{bodies},
          disableContacts_{disableContacts}
    {
        Uint32 howManyStructural = 0;
        for (Body* body : this->bodies())
            if (isBodyStructural(body))
                ++howManyStructural;

        SIMU_ASSERT(
            howManyStructural <= 1,
            "Cannot solve a constraint with multiple structural bodies. (also "
            "breaks island creation..?)"
        );
    }

    void onConstruction(World& world) override;
    void onDestruction(World& world) override;

    bool shouldDie() override
    {
        for (Body* body : bodies_)
            if (body->isDead())
                return true;

        return false;
    }

    bool isActive() override
    {
        // TODO: Shouldn't something similar be done for inequality constraints?
        if constexpr (std::is_same_v<S, LimitsSolver<F>>)
            return solver.isActive(bodies_, f);
        else
            return true;
    }

    void initSolve() override { solver.initSolve(bodies_, f); }
    void warmstart(float dt) override { solver.warmstart(bodies_, f, dt); }

    void solveVelocities(float dt) override
    {
        solver.solveVelocity(bodies_, f, dt);
    }

    void solvePositions() override { solver.solvePosition(bodies_, f); }

    BodiesView      bodies() override { return bodies_.view(); }
    ConstBodiesView bodies() const override { return bodies_.view(); }

    bool isBodyStructural(const Body* body) const override
    {
        return bodies_.isBodyStructural(body);
    }

protected:

    F f;
    S solver;

    Bodies<F::nBodies>&       getBodies() { return bodies_; }
    const Bodies<F::nBodies>& getBodies() const { return bodies_; }

private:

    Bodies<F::nBodies> bodies_;

    bool disableContacts_;
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

    RotationConstraint(const Bodies<2>& bodies, bool disableContacts = true)
        : Base{bodies, F{bodies}, disableContacts}
    {
    }
};

class HingeConstraint : public ConstraintImplementation<HingeConstraintFunction>
{
public:

    typedef ConstraintImplementation<HingeConstraintFunction> Base;
    typedef HingeConstraintFunction                           F;

    HingeConstraint(
        const Bodies<2>& bodies,
        Vec2             worldSpaceSharedPoint,
        bool             disableContacts = true
    )
        : Base{
            bodies,
            F{bodies, worldSpaceSharedPoint},
            disableContacts
    }
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


    WeldConstraint(const Bodies<2>& bodies, bool disableContacts = true)
        : Base{bodies, makeWeldFunction(bodies), disableContacts}
    {
    }

private:

    static Vec2 centroidMidPoint(const Bodies<2>& bodies)
    {
        return (bodies[0]->properties().centroid
                + bodies[1]->properties().centroid)
               / 2;
    }

    static F makeWeldFunction(const Bodies<2>& bodies)
    {
        return F{
            HingeF{bodies, centroidMidPoint(bodies)},
            RotF{bodies}
        };
    }
};


struct ContactInfo
{
    std::array<Vec2, 2> refContacts;
    std::array<Vec2, 2> incContacts;
    Uint32              nContacts;

    Vec2 normal;
};


class ContactConstraintI : public Constraint
{
public:

    typedef ContactManifold<Collider> Manifold;

    virtual bool isContactValid(const Bodies<2>& bodies, float maxPen) const = 0;

    virtual void update(const Bodies<2>& bodies, const Manifold& manifold) = 0;

    virtual float lambdaHint() const = 0;

    typedef Vector<float, 6> Impulse;
    virtual Impulse          impulseHint() const = 0;

    virtual Uint32 nContacts() const = 0;

    virtual ContactInfo contactInfo(Bodies<2> bodies) const = 0;

    virtual void setRestitution(float beta) = 0;
};

class SingleContactConstraint : public ConstraintImplementation<
                                    SingleContactFunction,
                                    priv::ContactSolver<SingleContactFunction>,
                                    ContactConstraintI>
{
public:

    typedef ConstraintImplementation<
        SingleContactFunction,
        priv::ContactSolver<SingleContactFunction>,
        ContactConstraintI>
        Base;

    typedef ContactManifold<Collider> Manifold;

    SingleContactConstraint(
        const Bodies<2>&        bodies,
        const Manifold&         manifold,
        const Vector<float, 6>& impulseHint
    )
        : Base{bodies, makeFunction(bodies, manifold), false}
    {
        auto J = f.jacobian(bodies);
        solver.setLambdaHint(solve(J * transpose(J), J * impulseHint));
    }

    bool isContactValid(const Bodies<2>& bodies, float maxPen) const override
    {
        return normSquared(f.contactDistance(bodies)) < maxPen * maxPen;
    }

    void update(const Bodies<2>& bodies, const Manifold& manifold) override
    {
        f = makeFunction(bodies, manifold);
    }

    float lambdaHint() const override
    {
        return solver.getAccumulatedLambda()[0];
    }

    Vector<float, 6> impulseHint() const override
    {
        return transpose(solver.getJacobian()) * solver.getAccumulatedLambda();
    }

    Uint32 nContacts() const override { return 1; }


    ContactInfo contactInfo(Bodies<2> bodies) const override
    {
        ContactInfo info;
        info.nContacts = 1;

        auto worldContacts  = f.worldSpaceContacts(bodies);
        info.refContacts[0] = worldContacts[f.reference()];
        info.incContacts[0] = worldContacts[f.incident()];

        info.normal = f.normal(bodies);
        return info;
    }

    void setRestitution(float beta) override
    {
        solver.restitution() = Value::filled(beta);
    }

private:

    static SingleContactFunction
    makeFunction(const Bodies<2>& bodies, const Manifold& manifold)
    {
        return SingleContactFunction{bodies, manifold, 0};
    }
};

class DoubleContactConstraint : public ConstraintImplementation<
                                    DoubleContactFunction,
                                    priv::ContactSolver<DoubleContactFunction>,
                                    ContactConstraintI>
{
public:

    typedef ConstraintImplementation<
        DoubleContactFunction,
        priv::ContactSolver<DoubleContactFunction>,
        ContactConstraintI>
        Base;

    typedef ContactManifold<Collider> Manifold;

    DoubleContactConstraint(
        const Bodies<2>&       bodies,
        const Manifold&        manifold,
        const Vector<float, 6> impulseHint
    )
        : Base{bodies, makeFunction(bodies, manifold), false}
    {
        auto J = f.jacobian(bodies);
        solver.setLambdaHint(solve(J * transpose(J), J * impulseHint));
    }

    bool isContactValid(const Bodies<2>& bodies, float maxPen) const override
    {
        bool firstIsValid
            = normSquared(std::get<0>(f.constraints).contactDistance(bodies))
              < maxPen * maxPen;
        bool secondIsValid
            = normSquared(std::get<1>(f.constraints).contactDistance(bodies))
              < maxPen * maxPen;
        return firstIsValid && secondIsValid;
    }


    void update(const Bodies<2>& bodies, const Manifold& manifold) override
    {
        f = makeFunction(bodies, manifold);
    }

    float lambdaHint() const override
    {
        Vec2 lambda = solver.getAccumulatedLambda();
        return lambda[0] + lambda[1];
    }

    Vector<float, 6> impulseHint() const override
    {
        return transpose(solver.getJacobian()) * solver.getAccumulatedLambda();
    }

    Uint32 nContacts() const override { return 2; }

    ContactInfo contactInfo(Bodies<2> bodies) const override
    {
        ContactInfo info;
        info.nContacts = 2;

        auto c0             = std::get<0>(f.constraints);
        auto worldContacts  = c0.worldSpaceContacts(bodies);
        info.refContacts[0] = worldContacts[c0.reference()];
        info.incContacts[0] = worldContacts[c0.incident()];

        auto c1             = std::get<1>(f.constraints);
        worldContacts       = c1.worldSpaceContacts(bodies);
        info.refContacts[1] = worldContacts[c1.reference()];
        info.incContacts[1] = worldContacts[c1.incident()];

        info.normal = c0.normal(bodies);

        return info;
    }

    void setRestitution(float beta) override
    {
        solver.restitution() = Value::filled(beta);
    }

private:

    static DoubleContactFunction
    makeFunction(const Bodies<2>& bodies, const Manifold& manifold)
    {
        return DoubleContactFunction{
            NonPenetrationConstraintFunction{bodies, manifold, 0},
            NonPenetrationConstraintFunction{bodies, manifold, 1}
        };
    }
};


class FrictionConstraint
    : public ConstraintImplementation<FrictionConstraintFunction>
{
public:

    typedef ConstraintImplementation<FrictionConstraintFunction> Base;

    typedef ContactManifold<Collider> Manifold;

    FrictionConstraint(const Bodies<2>& bodies, const Manifold& manifold)
        : Base{bodies, makeFunction(bodies, manifold), false}
    {
    }

    void update(const Bodies<2>& bodies, const Manifold& manifold)
    {
        float lambda = f.normalLambda;
        f            = makeFunction(bodies, manifold);
        setNormalLambda(lambda);
    }

    void setNormalLambda(float lambda) { f.normalLambda = lambda; }

private:

    static FrictionConstraintFunction
    makeFunction(const Bodies<2>& bodies, const Manifold& manifold)
    {
        return FrictionConstraintFunction{bodies, manifold};
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
            &bodies[0]->collider(),
            &bodies[1]->collider(),
            gjk.penetration()};
    }

    Bodies<2> bodies;
};

class ContactConstraint : public Constraint
{
public:

    ContactConstraint(const Bodies<2>& bodies) : contact_{bodies}
    {
        maxPen_ = CombinableProperty{
            contact_.bodies[0]->material().penetration,
            contact_.bodies[1]->material().penetration
        }.value;
    }

    bool shouldDie() override
    {
        for (Body* body : contact_.bodies)
            if (body->isDead())
                return true;

        return false;
    }

    bool isActive() override
    {
        updateContact();

        return contactConstraint_ != nullptr && contactConstraint_->isActive();
    }

    void initSolve() override
    {
        appliedVelocityConstraint_ = true;

        // TODO: Use baumgarte?
        // if (normSquared(penetration_) >= maxPen_ * maxPen_)
        //     contactConstraint_->setRestitution(0.05f);
        // else
        //     contactConstraint_->setRestitution(0.f);

        frictionConstraint_->initSolve();
        contactConstraint_->initSolve();
    }

    void warmstart(float dt) override
    {
        frictionConstraint_->warmstart(dt);
        contactConstraint_->warmstart(dt);
    }

    void solveVelocities(float dt) override
    {
        frictionConstraint_->solveVelocities(dt);
        contactConstraint_->solveVelocities(dt);
        frictionConstraint_->setNormalLambda(contactConstraint_->lambdaHint());
    }

    void solvePositions() override
    {
        // TODO: Use NGS?
        if (appliedVelocityConstraint_
            && normSquared(penetration_) >= maxPen_ * maxPen_)
            contactConstraint_->solvePositions();
    }

    BodiesView      bodies() override { return contact_.bodies.view(); }
    ConstBodiesView bodies() const override { return contact_.bodies.view(); }

    bool isBodyStructural(const Body* body) const override
    {
        return body->isStructural();
    };


    ContactInfo contactInfo() const
    {
        if (contactConstraint_ != nullptr)
            return contactConstraint_->contactInfo(contact_.bodies);

        return ContactInfo{};
    }

protected:

    void preStep() override { appliedVelocityConstraint_ = false; }

private:

    Contact                             contact_;
    std::unique_ptr<ContactConstraintI> contactConstraint_  = nullptr;
    std::unique_ptr<FrictionConstraint> frictionConstraint_ = nullptr;

    Vec2  penetration_;
    float maxPen_;
    bool  appliedVelocityConstraint_ = false;

    void updateContact()
    {
        auto gjk     = contact_.makeGjk();
        penetration_ = gjk.penetration();

        bool hasNoConstraint = (contactConstraint_ == nullptr);

        bool canComputeNewManifold
            = normSquared(penetration_) >= maxPen_ * maxPen_;
        if (!canComputeNewManifold)
        {
            if (!hasNoConstraint
                && !contactConstraint_->isContactValid(contact_.bodies, maxPen_))
            {
                contactConstraint_ = nullptr;
            }

            return;
        }

        auto manifold = contact_.makeManifold(gjk);

        if (hasNoConstraint
            || (manifold.nContacts != contactConstraint_->nContacts()))
        {
            contactConstraint_ = makeContactConstraint(
                manifold,
                hasNoConstraint ? Vector<float, 6>{}
                                : contactConstraint_->impulseHint()
            );
        }
        else
        {
            contactConstraint_->update(contact_.bodies, manifold);
        }

        frictionConstraint_->update(contact_.bodies, manifold);
    }

    std::unique_ptr<ContactConstraintI> makeContactConstraint(
        const ContactManifold<Collider>& manifold,
        const Vector<float, 6>&          impulseHint
    )
    {
        if (frictionConstraint_ == nullptr)
            frictionConstraint_
                = std::make_unique<FrictionConstraint>(contact_.bodies, manifold);

        switch (manifold.nContacts)
        {
            case 1:
                return std::make_unique<SingleContactConstraint>(
                    contact_.bodies,
                    manifold,
                    impulseHint
                );
            case 2:
                return std::make_unique<DoubleContactConstraint>(
                    contact_.bodies,
                    manifold,
                    impulseHint
                );
            default: return nullptr;
        }
    }
};


} // namespace simu

#include "Simu/physics/Constraint.inl.hpp"
