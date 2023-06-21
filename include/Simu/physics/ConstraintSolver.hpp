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

#include "Simu/config.hpp"
#include "Simu/physics/PhysicsBody.hpp"

namespace simu
{

template <Uint32 nConstraints>
using ConstraintValue = Vector<float, nConstraints>;

template <Uint32 nConstraints, Uint32 nBodies>
using Jacobian = Matrix<float, nConstraints, 3 * nBodies>;

template <Uint32 nBodies>
using VelocityVector = Vector<float, 3 * nBodies>;

template <Uint32 nBodies>
using MassMatrix = Matrix<float, 3 * nBodies, 3 * nBodies>;

template <Uint32 nBodies>
using Dominance = Vector<float, nBodies>;

// TODO: Compute position correction impulses separately, 
class ConstraintSolver
{
public:

    template <Uint32 nBodies>
    static VelocityVector<nBodies> velocity(const ConstBodies<nBodies>& bodies)
    {
        VelocityVector<nBodies> v{};
        Uint32                  i = 0;
        for (const PhysicsBody* body : bodies)
        {
            v[i++] = body->velocity()[0];
            v[i++] = body->velocity()[1];
            v[i++] = body->angularVelocity();
        }

        return v;
    }

    template <Uint32 nBodies>
    static MassMatrix<nBodies>
    inverseMass(const ConstBodies<nBodies>& bodies, const Dominance<nBodies>& dominance)
    {
        // TODO: Body dominance ratios
        Vector<float, 3 * nBodies> diagonal;

        Uint32 i       = 0;
        Uint32 nthBody = 0;
        for (const PhysicsBody* body : bodies)
        {
            diagonal[i++] = dominance[nthBody] / body->properties().mass;
            diagonal[i++] = dominance[nthBody] / body->properties().mass;
            diagonal[i++] = dominance[nthBody++] / body->properties().inertia;
        }

        return MassMatrix<nBodies>::diagonal(diagonal);
    }

    template <Uint32 nConstraints, Uint32 nBodies>
    static ConstraintValue<nConstraints> solveLambda(
        const ConstBodies<nBodies>&     bodies,
        const Dominance<nBodies>&       dominance,
        ConstraintValue<nConstraints>   C,
        Jacobian<nConstraints, nBodies> J,
        float                           dt,
        ConstraintValue<nConstraints>   bias,
        ConstraintValue<nConstraints>   beta,
        ConstraintValue<nConstraints>   gamma,
        ConstraintValue<nConstraints>   oldLambda
    )
    {
        typedef Matrix<float, nConstraints, nConstraints> MatType;

        MatType K = J * inverseMass(bodies, dominance) * transpose(J)
                    + MatType::diagonal(gamma);

        ConstraintValue<nConstraints> rhs
            = -(J * velocity(bodies) + bias + MatType::diagonal(beta) * C / dt
                + MatType::diagonal(gamma) * oldLambda);

        return solve(K, rhs);
    }

    template <Uint32 nConstraints, Uint32 nBodies>
    static VelocityVector<nBodies> impulse(
        const Jacobian<nConstraints, nBodies>& J,
        const ConstraintValue<nConstraints>&   lambda
    )
    {
        return transpose(J) * lambda;
    }

    template <Uint32 nBodies>
    static void apply(
        const Bodies<nBodies>&         bodies,
        const Dominance<nBodies>&      dominance,
        const VelocityVector<nBodies>& impulse
    )
    {
        VelocityVector<nBodies> dv
            = inverseMass<nBodies>(bodies, dominance) * impulse;
        Uint32 i = 0;
        for (PhysicsBody* body : bodies)
        {
            body->velocity() += Vec2{dv[i], dv[i + 1]};
            body->angularVelocity() += dv[i + 2];
            i += 3;
        }
    }
};


} // namespace simu
