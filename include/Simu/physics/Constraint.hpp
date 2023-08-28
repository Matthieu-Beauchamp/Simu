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

#include "Simu/math/ShapeCollision.hpp"
#include "Simu/math/Polygon.hpp"

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
        : Constraint{bodies, disableContacts}, f{f}, solver{}
    {
    }

    bool isActive(const Proxies& proxies) override
    {
        // TODO: Shouldn't something similar be done for inequality constraints?
        if constexpr (std::is_same_v<S, LimitsSolver<F>>)
            return solver.isActive(proxies, f);
        else
            return true;
    }

    void initSolve(const Proxies& proxies, float dt) override
    {
        solver.initSolve(proxies, f, dt);
    }
    void warmstart(Proxies& proxies) override { solver.warmstart(proxies); }

    void solveVelocities(Proxies& proxies, float dt) override
    {
        solver.solveVelocity(proxies, f, dt);
    }

    void solvePositions(Proxies& proxies) override
    {
        solver.solvePosition(proxies, f);
    }


protected:

    F f;
    S solver;
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

    HingeConstraint(const Bodies& bodies, Vec2 worldSpaceSharedPoint, bool disableContacts = true)
        : Base{
            bodies,
            F{bodies, worldSpaceSharedPoint},
            disableContacts
    }
    {
    }
};


typedef ConstraintFunctions<HingeConstraintFunction, RotationConstraintFunction> WeldConstraintFunction;

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
        return (bodies[0]->centroid() + bodies[1]->centroid()) / 2;
    }

    static F makeWeldFunction(const Bodies& bodies)
    {
        return F{
            HingeF{bodies, centroidMidPoint(bodies)},
            RotF{bodies}
        };
    }
};


class ContactConstraint : public Constraint
{
public:

    // tolerate sinkTolerance * penetration as the acceptable range where
    //  no position correction is needed
    static constexpr float sinkTolerance = 3.f; // [1.f, inf[

    ContactConstraint(Collider& first, Collider& second, CollisionCallback collide)
        : Constraint{Bodies{first.body(), second.body()}, false}, 
          collide_{collide}, A_{&first}, B_{&second},
          restitutionCoeff_{CombinableProperty{first.material().bounciness, 
                                               second.material().bounciness}.value},
          frictionCoeff_{CombinableProperty{first.material().friction, 
                                            second.material().friction}.value},
          minPen_{CombinableProperty{first.material().penetration, 
                                     second.material().penetration}.value}
    {
        updateContacts();
    }

    std::array<const Collider*, 2> colliders() const { return {A_, B_}; }

    const CollisionManifold& contactInfo() const { return worldManifold_; }

    // frame and manifold will be updated after this call,
    //  until the positions of the bodies change.
    void updateContacts()
    {
        bool isPolyCollision = A_->shape().type() == Shape::polygon
                               && B_->shape().type() == Shape::polygon;

        bool needsNewManifold = worldManifold_.nContacts == 0 || !isPolyCollision;

        if (!needsNewManifold)
        {
            // TODO: Specific to Polygon-Polygon...
            auto  relPos        = relativePosition();
            float distTolerance = minPen_;

            bool tangentDistanceExceeded = false;
            bool roseTooHigh             = false;
            for (Uint32 c = 0; c < worldManifold_.nContacts; ++c)
            {
                float tangentDist = dot(relPos[c], worldManifold_.tangent);
                if (squared(tangentDist) > squared(distTolerance))
                    tangentDistanceExceeded = true;

                float separation = dot(relPos[c], worldManifold_.normal);
                if (separation > distTolerance)
                    roseTooHigh = true;
            }

            Vec2 nearestIncident = *furthestVertexInDirection(
                static_cast<const Polygon&>(A_->shape()), -worldManifold_.normal
            );

            bool hasNewCandidate = any(worldManifold_.contactsA[0] != nearestIncident);

            if (worldManifold_.nContacts == 2)
                hasNewCandidate = hasNewCandidate
                                  && any(worldManifold_.contactsA[1] != nearestIncident);

            needsNewManifold = tangentDistanceExceeded || roseTooHigh
                               || hasNewCandidate;
        }

        if (needsNewManifold)
        {
            Uint32 nPreviousContacts = worldManifold_.nContacts;

            worldManifold_ = collide_(A_->shape(), B_->shape());
            localManifold_ = worldManifold_.transformed(
                A_->body()->toLocalSpace(), B_->body()->toLocalSpace()
            );

            tangentLambda_ = Vec2{};

            if (nPreviousContacts == 0)
                normalLambda_ = Vec2{};
            else if (nPreviousContacts == 1 && worldManifold_.nContacts == 2)
                normalLambda_ = Vec2{normalLambda_[0] / 2, normalLambda_[0] / 2};
            else if (nPreviousContacts == 2 && worldManifold_.nContacts == 1)
            {
                normalLambda_[0] += normalLambda_[1];
                normalLambda_[1] = 0.f;
            }
        }
    }

    bool isActive(const Proxies&) final override
    {
        // preStep should not be used, update here instead.
        // But currently Islands may call isActive multiple times, since
        //  updateContacts is expensive, we cheat a bit.
        return localManifold_.nContacts != 0;
    }

    void initSolve(const Proxies& proxies, float) final override
    {
        computeJacobians(proxies, true);
        computeKs(proxies, true);
        computeBounce(proxies);

        previousTangentLambda_ = tangentLambda_;
        previousNormalLambda_  = normalLambda_;
        tangentLambda_         = Vec2{};
        normalLambda_          = Vec2{};
    }

    void warmstart(Proxies& proxies) final override
    {
        Impulse P = transpose(Jf[0]) * previousTangentLambda_[0]
                    + transpose(Jf[1]) * previousTangentLambda_[1];
        P += transpose(Jn) * previousNormalLambda_;
        proxies.applyImpulse(P);

        normalLambda_  = previousNormalLambda_;
        tangentLambda_ = previousTangentLambda_;
    }

    void solveVelocities(Proxies& proxies, float /* dt */) final override
    {
        const auto invMassVec = proxies.invMassVec();
        auto       velocity   = proxies.velocity();

        for (Uint32 c = 0; c < worldManifold_.nContacts; ++c)
        {
            auto  J              = Jf[c];
            float tangentVel     = (J * velocity)[0];
            float dTangentLambda = -tangentVel * invTangentKs_[c];

            float bound         = frictionCoeff_ * normalLambda_[c];
            float tangentLambda = clamp(
                tangentLambda_[c] + dTangentLambda, -bound, bound
            );
            dTangentLambda    = tangentLambda - tangentLambda_[c];
            tangentLambda_[c] = tangentLambda;

            velocity += elementWiseMul(invMassVec, transpose(J) * dTangentLambda);
        }

        if (worldManifold_.nContacts == 1)
        {
            auto  J             = normalJacobian(0);
            float normalVel     = (J * velocity)[0];
            float dNormalLambda = -(normalVel + bounce_[0]) * invNormalK11_;

            float oldNormalLambda = normalLambda_[0];
            normalLambda_[0] += dNormalLambda;
            normalLambda_[0] = std::max(0.f, normalLambda_[0]);
            dNormalLambda    = normalLambda_[0] - oldNormalLambda;

            velocity += elementWiseMul(invMassVec, transpose(J) * dNormalLambda);
        }
        else if (worldManifold_.nContacts == 2)
        {
            Vec2 err             = Jn * velocity;
            Vec2 alreadyComputed = normalKSolver_.original() * normalLambda_;

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
            Vec2 b        = -(err - alreadyComputed + bounce_);
            normalLambda_ = normalKSolver_.solve(b);

            //////////////////////////////////

            Vec2 dNormalLambda = normalLambda_ - oldNormalLambda;

            auto P = transpose(Jn) * dNormalLambda;
            velocity += elementWiseMul(invMassVec, P);
        }

        proxies[0].setVelocity(Vec2{velocity[0], velocity[1]}, velocity[2]);
        proxies[1].setVelocity(Vec2{velocity[3], velocity[4]}, velocity[5]);
    }

    void solvePositions(Proxies& proxies) final override
    {
        updateWorldManifold(proxies);
        computeJacobians(proxies, false);
        computeKs(proxies, false);

        Vec2 C{};
        auto relPos = relativePosition();
        for (Uint32 c = 0; c < worldManifold_.nContacts; ++c)
            C[c] = dot(worldManifold_.normal, relPos[c]);


        Vec2 acceptablePen = -sinkTolerance * Vec2::filled(minPen_);
        if (all(C > acceptablePen))
            return;

        Vec2 error = clamp(
            posCorrectionFactor * (C + Vec2::filled(minPen_)),
            -Vec2::filled(maxCorrection),
            Vec2::filled(0.f)
        );

        // TODO: Sequential solving should be faster

        if (worldManifold_.nContacts == 1)
        {
            float posLambda = -error[0] * invNormalK11_;
            posLambda       = std::max(posLambda, 0.f);

            proxies.applyPositionCorrection(transpose(normalJacobian(0)) * posLambda);
        }
        else if (worldManifold_.nContacts == 2)
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
            posLambda = normalKSolver_.solve(-error);

            ///////////////////////////////////////

            proxies.applyPositionCorrection(transpose(Jn) * posLambda);
        }
    }

private:

    void updateWorldManifold(const Proxies& proxies)
    {
        worldManifold_ = localManifold_.transformed(
            proxies[0].toWorldSpace(), proxies[1].toWorldSpace()
        );
    }

    void computeKs(const Proxies& proxies, bool computeFrictionK)
    {
        const auto  invMass    = proxies.invMassVec();
        const float refMass    = invMass[3 * ref];
        const float incMass    = invMass[3 * inc];
        const float refInertia = invMass[3 * ref + 2];
        const float incInertia = invMass[3 * inc + 2];

        const float linearMass = refMass + incMass;
        const float normalMass = linearMass * normSquared(worldManifold_.normal);

        if (computeFrictionK)
        {
            for (Uint32 c = 0; c < worldManifold_.nContacts; ++c)
            {
                invTangentKs_[c] = linearMass * normSquared(worldManifold_.tangent)
                                   + refInertia * squared(Jf[c][3 * ref + 2])
                                   + incInertia * squared(Jf[c][3 * inc + 2]);

                invTangentKs_[c] = 1.f / invTangentKs_[c];
            }
        }


        Mat2 normalK;
        for (Uint32 c = 0; c < worldManifold_.nContacts; ++c)
        {
            normalK(c, c) = normalMass + refInertia * squared(Jn(c, 3 * ref + 2))
                            + incInertia * squared(Jn(c, 3 * inc + 2));
        }

        if (worldManifold_.nContacts == 1)
        {
            invNormalK11_ = 1.f / normalK(0, 0);
        }
        else if (worldManifold_.nContacts == 2)
        {
            normalK(0, 1) = normalMass
                            + refInertia * Jn(0, 3 * ref + 2) * Jn(1, 3 * ref + 2)
                            + incInertia * Jn(0, 3 * inc + 2) * Jn(1, 3 * inc + 2);

            normalK(1, 0) = normalK(0, 1);

            normalKSolver_ = LcpSolver<float>(normalK);
        }
    }

    void computeBounce(const Proxies& proxies)
    {
        bounce_ = Jn * proxies.velocity();

        for (Uint32 c = 0; c < worldManifold_.nContacts; ++c)
        {
            if (bounce_[c] > -restitutionTreshold)
                bounce_[c] = 0.f;
        }

        bounce_ *= restitutionCoeff_;
    }

    typedef typename Proxies::State   State;
    typedef typename Proxies::Impulse Impulse;

    void computeJacobians(const Proxies& proxies, bool computeFriction)
    {
        std::array<Vec2, 2> rA;
        std::array<Vec2, 2> rB;

        Vec2 cA = proxies[0].centroid();
        Vec2 cB = proxies[1].centroid();
        for (Uint32 c = 0; c < worldManifold_.nContacts; ++c)
        {
            rA[c] = worldManifold_.contactsA[c] - cA;
            rB[c] = worldManifold_.contactsB[c] - cB;
        }


        Jf = JacobianRows{};
        Jn = Jacobian{};

        Vec2 n = worldManifold_.normal;
        Vec2 t = worldManifold_.tangent;

        Uint32 incIndex = 3 * inc;
        Uint32 refIndex = 3 * ref;
        for (Uint32 c = 0; c < worldManifold_.nContacts; ++c)
        {
            if (computeFriction)
            {
                Jf[c][incIndex + 0] = t[0];
                Jf[c][incIndex + 1] = t[1];
                Jf[c][incIndex + 2] = cross(rA[c], t);
                Jf[c][refIndex + 0] = -t[0];
                Jf[c][refIndex + 1] = -t[1];
                Jf[c][refIndex + 2] = -cross(rB[c], t);
            }

            Jn(c, incIndex + 0) = n[0];
            Jn(c, incIndex + 1) = n[1];
            Jn(c, incIndex + 2) = cross(rA[c], n);
            Jn(c, refIndex + 0) = -n[0];
            Jn(c, refIndex + 1) = -n[1];
            Jn(c, refIndex + 2) = -cross(rB[c], n);
        }
    }

    Matrix<float, 1, 6> normalJacobian(Uint32 contact) const
    {
        return transpose(Jn.asRows()[contact]);
    }

    std::array<Vec2, 2> relativePosition() const
    {
        std::array<Vec2, 2> relPos;
        for (Uint32 c = 0; c < worldManifold_.nContacts; ++c)
        {
            relPos[c] = worldManifold_.contactsA[c] - worldManifold_.contactsB[c];
        }

        return relPos;
    }

    typedef Matrix<float, 2, 6>        Jacobian;
    typedef Matrix<float, 1, 6>        JacobianRow;
    typedef std::array<JacobianRow, 2> JacobianRows;

    JacobianRows Jf;
    Jacobian     Jn;

    Vec2 invTangentKs_;

    union
    {
        float            invNormalK11_;
        LcpSolver<float> normalKSolver_;
    };

    Vec2 tangentLambda_{};
    Vec2 previousTangentLambda_{};
    Vec2 normalLambda_{};
    Vec2 previousNormalLambda_{};

    Vec2 bounce_{};

    CollisionManifold worldManifold_{};
    CollisionManifold localManifold_{};

    CollisionCallback collide_;
    const Collider*   A_;
    const Collider*   B_;

    float restitutionCoeff_;
    float frictionCoeff_;
    float minPen_;

    // normal points out of ref.
    static constexpr Uint32 inc = 0;
    static constexpr Uint32 ref = 1;

    // TODO: Should be modifiable
    static constexpr float restitutionTreshold = 1.f;

    static constexpr float posCorrectionFactor = 0.2f;
    static constexpr float maxCorrection       = 0.2f;
};


} // namespace simu

#include "Simu/physics/Constraint.inl.hpp"
