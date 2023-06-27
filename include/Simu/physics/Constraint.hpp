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

    virtual bool isActive()                = 0;
    virtual void initSolve(float dt)       = 0;
    virtual void solveVelocities(float dt) = 0;
    virtual void solvePositions()          = 0;
};

// TODO: The caching is not very useful, storing the K^-1 matrix is more beneficial,
//  make the solver an owned object.
// Add support for NGS stabilization (while keeping Baumgarde for springs?)
//
template <ConstraintFunction F, ConstraintSolver S = EqualitySolver<F>, class Base = Constraint>
class ConstraintImplementation : public Base
{
public:

    typedef F::Value                   Value;
    typedef S                          Solver;
    typedef typename Solver::Dominance Dominance;

    ConstraintImplementation(
        const Bodies<F::nBodies>& bodies,
        const F&                  f,
        bool                      disableContacts,
        std::optional<Dominance>  dominanceRatios = std::nullopt
    )
        : f{f},
          bodies_{bodies},
          solver{bodies, f, dominance(bodies, dominanceRatios)},
          disableContacts_{disableContacts}
    {
    }

    void onConstruction(PhysicsWorld& world) override;
    void onDestruction(PhysicsWorld& world) override;

    bool shouldDie() override
    {
        for (PhysicsBody* body : bodies_)
            if (body->isDead())
                return true;

        return false;
    }

    bool isActive() override { return true; }

    void initSolve(float dt) override { solver.initSolve(bodies_, f, dt); }

    void solveVelocities(float dt) override
    {
        solver.solveVelocity(bodies_, f, dt);
    }

    void solvePositions() override { solver.solvePosition(bodies_, f); }

    // TODO: Setters for restitution/damping, stored in solver.

protected:

    F f;
    S solver;

private:

    static Dominance dominance(
        const Bodies<F::nBodies>& bodies,
        std::optional<Dominance>  dominanceRatios
    )
    {
        if (dominanceRatios.has_value())
            return dominanceRatios.value();

        Dominance d{};
        Uint32    i = 0;
        for (auto body : bodies)
            d[i++] = body->dominance();

        return d;
    }

    Bodies<F::nBodies> bodies_;
    Dominance          dominances_;

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

    RotationConstraint(
        const Bodies<2>&    bodies,
        bool                disableContacts = true,
        std::optional<Vec2> dominanceRatios = std::nullopt
    )
        : Base{bodies, F{bodies}, disableContacts, dominanceRatios}
    {
    }
};

class HingeConstraint : public ConstraintImplementation<HingeConstraintFunction>
{
public:

    typedef ConstraintImplementation<HingeConstraintFunction> Base;
    typedef HingeConstraintFunction                           F;

    HingeConstraint(
        const Bodies<2>&    bodies,
        Vec2                worldSpaceSharedPoint,
        bool                disableContacts = true,
        std::optional<Vec2> dominanceRatios = std::nullopt
    )
        : Base{
            bodies,
            F{bodies, worldSpaceSharedPoint},
            disableContacts,
            dominanceRatios
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


    WeldConstraint(
        const Bodies<2>&    bodies,
        bool                disableContacts = true,
        std::optional<Vec2> dominanceRatios = std::nullopt
    )
        : Base{bodies, makeWeldFunction(bodies), disableContacts, dominanceRatios}
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


class ContactConstraintI : public Constraint
{
public:

    typedef ContactManifold<Collider> Manifold;
    typedef const ConstBodies<2>&     CBodies;

    virtual bool isContactValid(CBodies bodies, float maxPen) const = 0;

    virtual void update(CBodies bodies, const Manifold& manifold) = 0;
    virtual Vec2 lambdaHint() const                               = 0;

    virtual Uint32 nContacts() const = 0;
};

class SingleContactConstraint
    : public ConstraintImplementation<SingleContactFunction, InequalitySolver<SingleContactFunction>, ContactConstraintI>
{
public:

    typedef ConstraintImplementation<SingleContactFunction, InequalitySolver<SingleContactFunction>, ContactConstraintI> Base;

    typedef ContactManifold<Collider> Manifold;

    SingleContactConstraint(
        const Bodies<2>& bodies,
        const Manifold&  manifold,
        Vec2             lambdaHint
    )
        : Base{bodies, makeFunction(bodies, manifold), false, std::nullopt}
    {
        solver.setLambdaHint(lambdaHint);
    }

    void solveVelocities(float dt) override
    {
        Base::solveVelocities(dt);
        auto J = solver.getJacobian().asRows()[0];
        std::get<0>(f.constraints).accumulatedRelVel
            = transpose(J) * solver.getInverseMass() * J
              * solver.getAccumulatedLambda()[0];
    }

    bool isContactValid(CBodies bodies, float maxPen) const override
    {
        return normSquared(std::get<0>(f.constraints).contactDistance(bodies))
               < maxPen * maxPen;
    }

    void update(CBodies bodies, const Manifold& manifold) override
    {
        f = makeFunction(bodies, manifold);
    }

    Vec2 lambdaHint() const override { return solver.getAccumulatedLambda(); }

    Uint32 nContacts() const override { return 1; }

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
    : public ConstraintImplementation<DoubleContactFunction,InequalitySolver<DoubleContactFunction>,  ContactConstraintI>
{
public:

    typedef ConstraintImplementation<DoubleContactFunction,InequalitySolver<DoubleContactFunction>,  ContactConstraintI> Base;

    typedef ContactManifold<Collider> Manifold;

    DoubleContactConstraint(
        const Bodies<2>& bodies,
        const Manifold&  manifold,
        Vec2             lambdaHint
    )
        : Base{bodies, makeFunction(bodies, manifold), false, std::nullopt}
    {
        solver.setLambdaHint(
            Vec3{lambdaHint[0] / 2, lambdaHint[0] / 2, lambdaHint[1]}
        );
    }

    void solveVelocities(float dt) override
    {
        Base::solveVelocities(dt);
        auto J = Matrix<float, 6, 2>::fromCols(
            {solver.getJacobian().asRows()[0], solver.getJacobian().asRows()[1]}
        );
        auto accumulatedRelVel = transpose(J) * solver.getInverseMass()
                                 * J* Vec2{
                                     solver.getAccumulatedLambda()[0],
                                     solver.getAccumulatedLambda()[1]};

        std::get<0>(f.constraints).accumulatedRelVel[0] = accumulatedRelVel[0];
        std::get<1>(f.constraints).accumulatedRelVel[0] = accumulatedRelVel[1];
    }

    bool isContactValid(CBodies bodies, float maxPen) const override
    {
        bool firstIsValid
            = normSquared(std::get<0>(f.constraints).contactDistance(bodies))
              < maxPen * maxPen;
        bool secondIsValid
            = normSquared(std::get<1>(f.constraints).contactDistance(bodies))
              < maxPen * maxPen;
        return firstIsValid && secondIsValid;
    }


    void update(CBodies bodies, const Manifold& manifold) override
    {
        f = makeFunction(bodies, manifold);
    }

    Vec2 lambdaHint() const override
    {
        Vec3 lambda = solver.getAccumulatedLambda();
        return Vec2{lambda[0] + lambda[1], lambda[2]};
    }

    Uint32 nContacts() const override { return 2; }

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

    bool shouldDie() override
    {
        for (PhysicsBody* body : contact_.bodies)
            if (body->isDead())
                return true;

        return false;
    }

    bool isActive() override
    {
        updateContact();

        return contactConstraint_ != nullptr && contactConstraint_->isActive();
    }

    void initSolve(float dt) override { contactConstraint_->initSolve(dt); }

    void solveVelocities(float dt) override
    {
        contactConstraint_->solveVelocities(dt);
    }

    void solvePositions() override
    {
        if (isActive())
            contactConstraint_->solvePositions();
    }

private:

    Contact                             contact_;
    std::unique_ptr<ContactConstraintI> contactConstraint_ = nullptr;

    void updateContact()
    {
        auto gjk         = contact_.makeGjk();
        Vec2 penetration = gjk.penetration();

        float maxPen = CombinableProperty{
            contact_.bodies[0]->material().penetration,
            contact_.bodies[1]->material().penetration
        }.value;

        bool hasNoConstraint = (contactConstraint_ == nullptr);

        bool canComputeNewManifold = normSquared(penetration) >= maxPen * maxPen;
        if (!canComputeNewManifold)
        {
            if (!hasNoConstraint
                && !contactConstraint_->isContactValid(contact_.bodies, maxPen))
                contactConstraint_ = nullptr;

            return;
        }

        auto manifold = contact_.makeManifold(gjk);

        if (hasNoConstraint
            || (manifold.nContacts != contactConstraint_->nContacts()))
        {
            contactConstraint_ = makeContactConstraint(
                manifold,
                hasNoConstraint ? Vec2{} : contactConstraint_->lambdaHint()
            );
        }
        else
        {
            contactConstraint_->update(contact_.bodies, manifold);
        }
    }

    std::unique_ptr<ContactConstraintI> makeContactConstraint(
        const ContactManifold<Collider>& manifold,
        Vec2                             lambdaHint = Vec2{}
    )
    {
        switch (manifold.nContacts)
        {
            case 1:
                return std::make_unique<SingleContactConstraint>(
                    contact_.bodies,
                    manifold,
                    lambdaHint
                );
            case 2:
                return std::make_unique<DoubleContactConstraint>(
                    contact_.bodies,
                    manifold,
                    lambdaHint
                );
            default: return nullptr;
        }
    }
};


} // namespace simu

#include "Simu/physics/Constraint.inl.hpp"
