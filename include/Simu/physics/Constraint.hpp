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
        return (bodies[0]->centroid() + bodies[1]->centroid()) / 2;
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


class ContactConstraint : public Constraint
{
public:

    ContactConstraint(const Bodies<2>& bodies)
        : bodies_{bodies}, 
          manifold_{bodies[0], bodies[1]}, 
          restitutionCoeff_{CombinableProperty{bodies[0]->material().bounciness, 
                                               bodies[1]->material().bounciness}.value},
          frictionCoeff_{CombinableProperty{bodies[0]->material().friction, 
                                            bodies[1]->material().friction}.value}
    {
    }

    bool shouldDie() override
    {
        for (Body* body : bodies_)
            if (body->isDead())
                return true;

        return false;
    }

    bool isActive() override
    {
        updateContacts();
        return manifold_.nContacts() != 0;
    }

    void initSolve() override
    {
        // TODO: Use baumgarte?
        // if (normSquared(penetration_) >= maxPen_ * maxPen_)
        //     contactConstraint_->setRestitution(0.05f);
        // else
        //     contactConstraint_->setRestitution(0.f);

        computeKs(true);
        computeBounce();
    }

    void warmstart(float dt) override
    {
        Impulse P = tangentImpulse(tangentLambda_);
        P += normalImpulse(normalLambda_);
        bodies_.applyImpulse(P);
    }

    void solveVelocities(float dt) override
    {
        solveFrictionVelocity();
        solveContactVelocity();
    }

    void solvePositions() override
    {
        computeKs(false);

        Vec2 C{};
        auto relPos = relativePosition();
        for (Uint32 c = 0; c < frame_.nContacts; ++c)
            C[c] = dot(frame_.normal, relPos[c]);


        Vec2 acceptablePen
            = -sinkTolerance * Vec2::filled(manifold_.minimumPenetration());
        if (all(C > acceptablePen))
            return;

        Vec2 error = clamp(
            posCorrectionFactor
                * (C + Vec2::filled(manifold_.minimumPenetration())),
            -Vec2::filled(maxCorrection),
            Vec2::filled(0.f)
        );

        if (frame_.nContacts == 1)
        {
            float posLambda = -error[0] / normalK_(0, 0);
            posLambda       = std::max(posLambda, 0.f);

            bodies_.applyPositionCorrection(
                elementWiseMul(bodies_.inverseMassVec(), normalImpulse(posLambda))
            );
        }
        else if (frame_.nContacts == 2)
        {
            Vec2 posLambda = solveInequalities(
                normalK_,
                -error,
                [](const Vec2& lambda, Uint32 i) {
                    return std::max(0.f, lambda[i]);
                }
            );

            bodies_.applyPositionCorrection(
                elementWiseMul(bodies_.inverseMassVec(), normalImpulse(posLambda))
            );
        }
    }

    BodiesView      bodies() override { return bodies_.view(); }
    ConstBodiesView bodies() const override { return bodies_.view(); }

    bool isBodyStructural(const Body* body) const override
    {
        return body->isStructural();
    };


    ContactInfo contactInfo() const
    {
        ContactInfo info;
        info.nContacts = manifold_.nContacts();

        const Uint32 ref    = manifold_.referenceIndex();
        const Uint32 inc    = manifold_.incidentIndex();
        info.refContacts[0] = frame_.worldContacts[ref][0];
        info.incContacts[0] = frame_.worldContacts[inc][0];
        info.refContacts[1] = frame_.worldContacts[ref][1];
        info.incContacts[1] = frame_.worldContacts[inc][1];

        info.normal = frame_.normal;

        return info;
    }

private:

    // frame and manifold will be updated after this call,
    //  until the positions of the bodies change.
    void updateContacts()
    {
        frame_ = manifold_.frameManifold();
        // TODO: Use vertex matching in contact manifold
        bool needsNewManifold = frame_.nContacts == 0;

        if (!needsNewManifold)
        {
            auto  relPos        = relativePosition();
            float distTolerance = manifold_.minimumPenetration();

            bool tangentDistanceExceeded = false;
            bool roseTooHigh             = false;
            for (Uint32 c = 0; c < frame_.nContacts; ++c)
            {
                float tangentDist = dot(relPos[c], frame_.tangent);
                if (squared(tangentDist) > squared(distTolerance))
                    tangentDistanceExceeded = true;

                float separation = dot(relPos[c], frame_.normal);
                if (separation > distTolerance)
                    roseTooHigh = true;
            }

            const Uint32 inc = manifold_.incidentIndex();

            Vec2 nearestIncident = furthestVertexInDirection(
                bodies_[inc]->collider(),
                -frame_.normal
            );

            bool hasNewCandidate
                = any(frame_.worldContacts[inc][0] != nearestIncident);
            if (frame_.nContacts == 2)
                hasNewCandidate
                    = hasNewCandidate
                      && any(frame_.worldContacts[inc][1] != nearestIncident);

            needsNewManifold
                = tangentDistanceExceeded || roseTooHigh || hasNewCandidate;
        }

        if (needsNewManifold)
        {
            Uint32 nPreviousContacts = frame_.nContacts;

            manifold_.update();
            frame_ = manifold_.frameManifold();

            tangentLambda_ = Vec2{};

            if (nPreviousContacts == 0)
                normalLambda_ = Vec2{};
            else if (nPreviousContacts == 1 && frame_.nContacts == 2)
                normalLambda_ = Vec2{normalLambda_[0] / 2, normalLambda_[0] / 2};
            else if (nPreviousContacts == 2 && frame_.nContacts == 1)
            {
                normalLambda_[0] += normalLambda_[1];
                normalLambda_[1] = 0.f;
            }
        }

        for (Uint32 b = 0; b < 2; ++b)
            for (Uint32 c = 0; c < frame_.nContacts; ++c)
                radius_[b][c]
                    = frame_.worldContacts[b][c] - bodies_[b]->centroid();
    }

    void computeKs(bool computeFrictionK)
    {
        const Uint32 ref = manifold_.referenceIndex();
        const Uint32 inc = manifold_.incidentIndex();

        const auto  invMass    = bodies_.inverseMassVec();
        const float refMass    = invMass[3 * ref];
        const float incMass    = invMass[3 * inc];
        const float refInertia = invMass[3 * ref + 2];
        const float incInertia = invMass[3 * inc + 2];

        const float linearMass = refMass + incMass;
        const float normalMass = linearMass * normSquared(frame_.normal);

        if (computeFrictionK)
        {
            for (Uint32 c = 0; c < manifold_.nContacts(); ++c)
            {
                tangentK_[c]
                    = linearMass * normSquared(frame_.tangent)
                      + refInertia
                            * squared(cross(radius_[ref][c], frame_.tangent))
                      + incInertia
                            * squared(cross(radius_[inc][c], frame_.tangent));
            }
        }

        for (Uint32 c = 0; c < manifold_.nContacts(); ++c)
        {
            normalK_(c, c)
                = normalMass
                  + refInertia * squared(cross(radius_[ref][c], frame_.normal))
                  + incInertia * squared(cross(radius_[inc][c], frame_.normal));
        }

        if (manifold_.nContacts() == 2)
        {
            normalK_(0, 1) = normalMass
                             + refInertia * cross(radius_[ref][0], frame_.normal)
                                   * cross(radius_[ref][1], frame_.normal)
                             + incInertia * cross(radius_[inc][0], frame_.normal)
                                   * cross(radius_[inc][1], frame_.normal);

            normalK_(1, 0) = normalK_(0, 1);
        }
    }

    void computeBounce()
    {
        const Uint32 ref = manifold_.referenceIndex();
        const Uint32 inc = manifold_.incidentIndex();

        bounce_ = Vec2{};

        Vec2 linearRelVel = bodies_[inc]->velocity() - bodies_[ref]->velocity();
        for (Uint32 c = 0; c < manifold_.nContacts(); ++c)
        {
            Vec2 relVel
                = linearRelVel
                  + bodies_[inc]->angularVelocity() * perp(radius_[inc][c])
                  - bodies_[ref]->angularVelocity() * perp(radius_[ref][c]);

            bounce_[c] = dot(relVel, frame_.normal);
            if (bounce_[c] > -restitutionTreshold)
                bounce_[c] = 0.f;
        }

        bounce_ = std::min(bounce_, Vec2::filled(0.f));
        bounce_ *= restitutionCoeff_;
    }

    typedef typename Bodies<2>::State   State;
    typedef typename Bodies<2>::Impulse Impulse;

    Impulse normalImpulse(float lambda, Uint32 contact = 0) const
    {
        const Uint32 ref = manifold_.referenceIndex();
        const Uint32 inc = manifold_.incidentIndex();

        Vec2    dir = frame_.normal;
        Impulse J;
        J[3 * inc + 0] = dir[0];
        J[3 * inc + 1] = dir[1];
        J[3 * inc + 2] = cross(radius_[inc][contact], frame_.normal);
        J[3 * ref + 0] = -dir[0];
        J[3 * ref + 1] = -dir[1];
        J[3 * ref + 2] = -cross(radius_[ref][contact], frame_.normal);

        return lambda * J;
    }

    Impulse normalImpulse(Vec2 lambda) const
    {
        return normalImpulse(lambda[0], 0) + normalImpulse(lambda[1], 1);
    }

    Impulse tangentImpulse(float lambda, Uint32 contact = 0) const
    {
        const Uint32 ref = manifold_.referenceIndex();
        const Uint32 inc = manifold_.incidentIndex();

        Vec2    dir = frame_.tangent;
        Impulse J;
        J[3 * inc + 0] = dir[0];
        J[3 * inc + 1] = dir[1];
        J[3 * inc + 2] = cross(radius_[inc][contact], frame_.tangent);
        J[3 * ref + 0] = -dir[0];
        J[3 * ref + 1] = -dir[1];
        J[3 * ref + 2] = -cross(radius_[ref][contact], frame_.tangent);

        return lambda * J;
    }

    Impulse tangentImpulse(Vec2 lambda) const
    {
        return tangentImpulse(lambda[0], 0) + tangentImpulse(lambda[1], 1);
    }

    std::array<Vec2, 2> relativeVelocity() const
    {
        const Uint32 ref = manifold_.referenceIndex();
        const Uint32 inc = manifold_.incidentIndex();

        Vec2 linearRelVel = bodies_[inc]->velocity() - bodies_[ref]->velocity();

        std::array<Vec2, 2> relVel;
        for (Uint32 c = 0; c < frame_.nContacts; ++c)
        {
            relVel[c]
                = linearRelVel
                  + bodies_[inc]->angularVelocity() * perp(radius_[inc][c])
                  - bodies_[ref]->angularVelocity() * perp(radius_[ref][c]);
        }

        return relVel;
    }

    std::array<Vec2, 2> relativePosition() const
    {
        const Uint32 ref = manifold_.referenceIndex();
        const Uint32 inc = manifold_.incidentIndex();

        std::array<Vec2, 2> relPos;
        for (Uint32 c = 0; c < frame_.nContacts; ++c)
        {
            relPos[c]
                = frame_.worldContacts[inc][c] - frame_.worldContacts[ref][c];
        }

        return relPos;
    }

    void solveFrictionVelocity()
    {
        for (Uint32 c = 0; c < frame_.nContacts; ++c)
        {
            std::array<Vec2, 2> relVel = relativeVelocity();

            float tangentVel     = dot(relVel[c], frame_.tangent);
            float dTangentLambda = -tangentVel / tangentK_[c];

            float oldTangentLambda = tangentLambda_[c];
            tangentLambda_[c] += dTangentLambda;
            float bound       = frictionCoeff_ * normalLambda_[c];
            tangentLambda_[c] = clamp(tangentLambda_[c], -bound, bound);
            dTangentLambda    = tangentLambda_[c] - oldTangentLambda;

            bodies_.applyImpulse(tangentImpulse(dTangentLambda, c));
        }
    }

    void solveContactVelocity()
    {
        auto relVel = relativeVelocity();

        if (frame_.nContacts == 1)
        {
            float normalVel     = dot(relVel[0], frame_.normal);
            float dNormalLambda = -(normalVel + bounce_[0]) / normalK_(0, 0);

            float oldNormalLambda = normalLambda_[0];
            normalLambda_[0] += dNormalLambda;
            normalLambda_[0] = std::max(0.f, normalLambda_[0]);
            dNormalLambda    = normalLambda_[0] - oldNormalLambda;

            bodies_.applyImpulse(normalImpulse(dNormalLambda));
        }
        else if (frame_.nContacts == 2)
        {
            Vec2 err{dot(relVel[0], frame_.normal), dot(relVel[1], frame_.normal)};
            Vec2 alreadyComputed = normalK_ * normalLambda_;

            Vec2 oldNormalLambda = normalLambda_;

            normalLambda_ = solveInequalities(
                normalK_,
                -(err - alreadyComputed + bounce_),
                [](const Vec2& lambda, Uint32 i) {
                    return std::max(0.f, lambda[i]);
                },
                normalLambda_
            );

            Vec2 dNormalLambda = normalLambda_ - oldNormalLambda;

            bodies_.applyImpulse(normalImpulse(dNormalLambda));
        }
    }

    Bodies<2>                               bodies_;
    ContactManifold                         manifold_;
    typename ContactManifold::FrameManifold frame_;
    std::array<std::array<Vec2, 2>, 2>      radius_;

    Vec2 tangentK_;
    Mat2 normalK_;

    Vec2 tangentLambda_{};
    Vec2 normalLambda_{};

    Vec2 bounce_{};

    float restitutionCoeff_;
    float frictionCoeff_;

    // TODO: Should be modifiable
    static constexpr float restitutionTreshold = 1.f;

    static constexpr float posCorrectionFactor = 0.2f;
    static constexpr float maxCorrection       = 0.2f;

    // tolerate sinkTolerance * penetration as the acceptable range where
    //  no position correction is needed
    static constexpr float sinkTolerance = 3.f; // [1.f, inf[
};


} // namespace simu

#include "Simu/physics/Constraint.inl.hpp"
