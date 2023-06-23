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
template <ConstraintFunction F>
class ConstraintImplementation : public Constraint
{
public:

    typedef F::Value                   Value;
    typedef ConstraintSolver<F>        Solver;
    typedef typename Solver::Dominance Dominance;

    ConstraintImplementation(
        const Bodies<F::nBodies>& bodies,
        const F&                  f,
        bool                      disableContacts,
        std::optional<Dominance>  dominanceRatios = std::nullopt
    )
        : f{f},
          bodies_{bodies},
          dominances_{dominance(bodies_, dominanceRatios)},
          solver{bodies_, f, dominances_},
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

    bool isActive() override { return f.isActive(f.eval(bodies_)); }

    void initSolve(float /* dt */) override { solver.initSolve(bodies_, f); }

    void solveVelocities(float dt) override
    {
        solver.solveVelocity(bodies_, f, dt);
    }

    void solvePositions() override { solver.solvePosition(bodies_, f); }

    F f;

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

public:

    // Care with initialization order
    Solver solver;
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

    bool shouldDie() override
    {
        for (PhysicsBody* body : contact_.bodies)
            if (body->isDead())
                return true;

        return false;
    }

    bool isActive() override
    {
        auto gjk = contact_.makeGjk();
        if (all(gjk.penetration() == Vec2{}))
            return false;

        auto manifold      = contact_.makeManifold(gjk);
        contactConstraint_ = makeContactConstraint(manifold);

        return contactConstraint_ != nullptr && contactConstraint_->isActive();
    }

    void initSolve(float dt) override { contactConstraint_->initSolve(dt); }
    void solveVelocities(float dt) override
    {
        contactConstraint_->solveVelocities(dt);
        switch (nContacts)
        {
            case 1:
            {
                auto c = static_cast<SingleContactConstraint*>(
                    contactConstraint_.get()
                );
                std::get<0>(c->f.constraints).accumulatedDv
                    = c->solver.getInverseMass()
                      * c->solver.getJacobian().asRows()[0]
                      * c->solver.getLambda()[0];
                break;
            }
            case 2:
            {
                auto c = static_cast<DoubleContactConstraint*>(
                    contactConstraint_.get()
                );
                auto accumulatedDv
                    = c->solver.getInverseMass()
                      * Matrix<float, 6, 2>::fromCols(
                          {c->solver.getJacobian().asRows()[0],
                           c->solver.getJacobian().asRows()[1]}
                      )
                      * Vec2{c->solver.getLambda()[0], c->solver.getLambda()[1]};
                std::get<0>(c->f.constraints).accumulatedDv = accumulatedDv;
                std::get<1>(c->f.constraints).accumulatedDv = accumulatedDv;
                break;
            }
        }
    }
    void solvePositions() override { contactConstraint_->solvePositions(); }

private:

    Contact                     contact_;
    std::unique_ptr<Constraint> contactConstraint_ = nullptr;
    Uint32                      nContacts          = 0;

    std::unique_ptr<Constraint>
    makeContactConstraint(const ContactManifold<Collider>& manifold)
    {
        nContacts = manifold.nContacts;
        switch (manifold.nContacts)
        {
            case 1:
                return std::make_unique<SingleContactConstraint>(
                    contact_.bodies,
                    manifold
                );
            case 2:
                return std::make_unique<DoubleContactConstraint>(
                    contact_.bodies,
                    manifold
                );
            default: return nullptr;
        }
    }
};


} // namespace simu

#include "Simu/physics/Constraint.inl.hpp"
