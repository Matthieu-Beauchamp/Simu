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

template <ConstraintFunction F, ConstraintSolver S = EqualitySolver<F>>
class ConstraintImplementation : public Constraint
{
public:

    typedef typename F::Value Value;
    typedef S                 Solver;

    ConstraintImplementation(const Bodies& bodies, const F& f, bool disableContacts)
        : Constraint{bodies},
          f{f},
          solver{bodies, f},
          disableContacts_{disableContacts}
    {
    }

    void onConstruction(World& world) override;
    void onDestruction(World& world) override;

    bool isActive() override
    {
        // TODO: Shouldn't something similar be done for inequality constraints?
        if constexpr (std::is_same_v<S, LimitsSolver<F>>)
            return solver.isActive(bodies(), f);
        else
            return true;
    }

    void initSolve() override { solver.initSolve(bodies(), f); }
    void warmstart(float dt) override { solver.warmstart(bodies(), f, dt); }

    void solveVelocities(float dt) override
    {
        solver.solveVelocity(bodies(), f, dt);
    }

    void solvePositions() override { solver.solvePosition(bodies(), f); }


protected:

    F f;
    S solver;

private:

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

    RotationConstraint(const Bodies& bodies, bool disableContacts = true)
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
        const Bodies& bodies,
        Vec2          worldSpaceSharedPoint,
        bool          disableContacts = true
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


    WeldConstraint(const Bodies& bodies, bool disableContacts = true)
        : Base{bodies, makeWeldFunction(bodies), disableContacts}
    {
    }

private:

    static Vec2 centroidMidPoint(const Bodies& bodies)
    {
        auto b = bodies.bodies();
        return (b[0]->centroid() + b[1]->centroid()) / 2;
    }

    static F makeWeldFunction(const Bodies& bodies)
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

    ContactConstraint(const Bodies& bodies)
        : Constraint{bodies}, 
          manifold_{this->bodies()}, 
          restitutionCoeff_{CombinableProperty{bodies.bodies()[0]->material().bounciness, 
                                               bodies.bodies()[1]->material().bounciness}.value},
          frictionCoeff_{CombinableProperty{bodies.bodies()[0]->material().friction, 
                                            bodies.bodies()[1]->material().friction}.value}
    {
    }

    void preStep() override { updateContacts(); }

    bool isActive() override
    {
        frame_ = manifold_.frameManifold(bodies());
        return frame_.nContacts != 0;
    }

    void initSolve() override
    {
        // TODO: Use baumgarte?
        // if (normSquared(penetration_) >= maxPen_ * maxPen_)
        //     contactConstraint_->setRestitution(0.05f);
        // else
        //     contactConstraint_->setRestitution(0.f);

        computeJacobians(true);
        computeKs(true);
        computeBounce();
    }

    void warmstart(float /* dt */) override
    {
        Impulse P = transpose(Jf) * tangentLambda_;
        P += transpose(Jn) * normalLambda_;
        bodies().applyImpulse(P);
    }

    void solveVelocities(float /* dt */) override
    {
        auto p = bodies().proxies();

        const auto invMassVec = bodies().inverseMassVec();
        auto       velocity   = bodies().velocity();

        for (Uint32 c = 0; c < frame_.nContacts; ++c)
        {
            auto  J              = tangentJacobian(c);
            float tangentVel     = (J * velocity)[0];
            float dTangentLambda = -tangentVel / tangentKs_[c];

            float oldTangentLambda = tangentLambda_[c];
            tangentLambda_[c] += dTangentLambda;
            float bound       = frictionCoeff_ * normalLambda_[c];
            tangentLambda_[c] = clamp(tangentLambda_[c], -bound, bound);
            dTangentLambda    = tangentLambda_[c] - oldTangentLambda;

            velocity
                += elementWiseMul(invMassVec, transpose(J) * dTangentLambda);
        }

        if (frame_.nContacts == 1)
        {
            auto  J             = normalJacobian(0);
            float normalVel     = (J * velocity)[0];
            float dNormalLambda = -(normalVel + bounce_[0]) / normalK_(0, 0);

            float oldNormalLambda = normalLambda_[0];
            normalLambda_[0] += dNormalLambda;
            normalLambda_[0] = std::max(0.f, normalLambda_[0]);
            dNormalLambda    = normalLambda_[0] - oldNormalLambda;

            velocity += elementWiseMul(invMassVec, transpose(J) * dNormalLambda);
        }
        else if (frame_.nContacts == 2)
        {
            Vec2 err             = Jn * velocity;
            Vec2 alreadyComputed = normalK_ * normalLambda_;

            Vec2 oldNormalLambda = normalLambda_;

            // PGS ////////////////////////////
            // normalLambda_ = solveInequalities(
            //     normalK_,
            //     -(err - alreadyComputed + bounce_),
            //     [](const Vec2& lambda, Uint32 i) {
            //         return std::max(0.f, lambda[i]);
            //     },
            //     normalLambda_
            // );
            //////////////////////////////////

            // Box2d's LCP solver ////////////
            auto normalLambda
                = solveLcp(normalK_, -(err - alreadyComputed + bounce_));

            if (!normalLambda.has_value())
                return;
            else
                normalLambda_ = normalLambda.value();
            //////////////////////////////////

            Vec2 dNormalLambda = normalLambda_ - oldNormalLambda;

            auto P = transpose(Jn) * dNormalLambda;
            velocity += elementWiseMul(invMassVec, P);
        }

        p[0]->setVelocity(Vec2{velocity[0], velocity[1]}, velocity[2]);
        p[1]->setVelocity(Vec2{velocity[3], velocity[4]}, velocity[5]);
    }

    void solvePositions() override
    {
        frame_ = manifold_.frameManifold(bodies());
        computeJacobians(false);
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

        // TODO: Sequential solving should be faster

        if (frame_.nContacts == 1)
        {
            float posLambda = -error[0] / normalK_(0, 0);
            posLambda       = std::max(posLambda, 0.f);

            bodies().applyPositionCorrection(elementWiseMul(
                bodies().inverseMassVec(),
                transpose(normalJacobian(0)) * posLambda
            ));
        }
        else if (frame_.nContacts == 2)
        {
            Vec2 posLambda;
            // PGS /////////////////////////////////
            // Vec2 posLambda = solveInequalities(
            //     normalK_,
            //     -error,
            //     [](const Vec2& lambda, Uint32 i) {
            //         return std::max(0.f, lambda[i]);
            //     }
            // );
            ////////////////////////////////////////


            // Box2d's LCP solver /////////////////
            auto res = solveLcp(normalK_, -error);
            if (!res.has_value())
                return;
            else
                posLambda = res.value();
            ///////////////////////////////////////


            bodies().applyPositionCorrection(
                bodies().inverseMass() * transpose(Jn) * posLambda
            );
        }
    }

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
        frame_ = manifold_.frameManifold(bodies());

        // TODO: Use vertex matching in contact manifold
        // TODO: All of this goes into contact manifold's update logic
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
                bodies().bodies()[inc]->collider(),
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

            manifold_.update(bodies());
            frame_ = manifold_.frameManifold(bodies());

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
    }

    void computeKs(bool computeFrictionK)
    {
        const Uint32 ref = manifold_.referenceIndex();
        const Uint32 inc = manifold_.incidentIndex();

        const auto  invMass    = bodies().inverseMassVec();
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
                tangentKs_[c] = linearMass * normSquared(frame_.tangent)
                                + refInertia * squared(Jf(c, 3 * ref + 2))
                                + incInertia * squared(Jf(c, 3 * inc + 2));
            }
        }

        for (Uint32 c = 0; c < manifold_.nContacts(); ++c)
        {
            normalK_(c, c) = normalMass
                             + refInertia * squared(Jn(c, 3 * ref + 2))
                             + incInertia * squared(Jn(c, 3 * inc + 2));
        }

        if (manifold_.nContacts() == 2)
        {
            normalK_(0, 1)
                = normalMass
                  + refInertia * Jn(0, 3 * ref + 2) * Jn(1, 3 * ref + 2)
                  + incInertia * Jn(0, 3 * inc + 2) * Jn(1, 3 * inc + 2);

            normalK_(1, 0) = normalK_(0, 1);
        }
    }

    void computeBounce()
    {
        bounce_ = Vec2{};

        Vec2 relVel = Jn * bodies().velocity();

        for (Uint32 c = 0; c < manifold_.nContacts(); ++c)
        {
            if (bounce_[c] > -restitutionTreshold)
                bounce_[c] = 0.f;
        }

        bounce_ *= restitutionCoeff_;
    }

    typedef typename Bodies::State   State;
    typedef typename Bodies::Impulse Impulse;

    void computeJacobians(bool computeFriction)
    {
        const Uint32 ref = manifold_.referenceIndex();
        const Uint32 inc = manifold_.incidentIndex();

        auto p = bodies().proxies();

        std::array<std::array<Vec2, 2>, 2> r{};
        for (Uint32 b = 0; b < 2; ++b)
        {
            Vec2 centroid = p[b]->centroid();
            for (Uint32 c = 0; c < frame_.nContacts; ++c)
            {
                r[b][c] = frame_.worldContacts[b][c] - centroid;
            }
        }

        Jf = Jacobian{};
        Jn = Jacobian{};

        Vec2 n = frame_.normal;
        Vec2 t = frame_.tangent;

        Uint32 incIndex = 3 * inc;
        Uint32 refIndex = 3 * ref;
        for (Uint32 c = 0; c < frame_.nContacts; ++c)
        {
            if (computeFriction)
            {
                Jf(c, incIndex + 0) = t[0];
                Jf(c, incIndex + 1) = t[1];
                Jf(c, incIndex + 2) = cross(r[inc][c], t);
                Jf(c, refIndex + 0) = -t[0];
                Jf(c, refIndex + 1) = -t[1];
                Jf(c, refIndex + 2) = -cross(r[ref][c], t);
            }

            Jn(c, incIndex + 0) = n[0];
            Jn(c, incIndex + 1) = n[1];
            Jn(c, incIndex + 2) = cross(r[inc][c], n);
            Jn(c, refIndex + 0) = -n[0];
            Jn(c, refIndex + 1) = -n[1];
            Jn(c, refIndex + 2) = -cross(r[ref][c], n);
        }
    }

    Matrix<float, 1, 6> normalJacobian(Uint32 contact) const
    {
        return transpose(Jn.asRows()[contact]);
    }

    Matrix<float, 1, 6> tangentJacobian(Uint32 contact) const
    {
        return transpose(Jf.asRows()[contact]);
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

    ContactManifold manifold_;

    typename ContactManifold::FrameManifold frame_;

    typedef Matrix<float, 2, 6> Jacobian;
    Jacobian                    Jf;
    Jacobian                    Jn;

    Vec2 tangentKs_;
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
