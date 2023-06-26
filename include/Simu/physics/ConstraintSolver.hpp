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

#include "Simu/utility/PointerArray.hpp"

namespace simu
{

class PhysicsBody;

template <Uint32 nBodies>
using Bodies = PointerArray<PhysicsBody, nBodies, false>;

template <Uint32 nBodies>
using ConstBodies = PointerArray<PhysicsBody, nBodies, true>;


template <class F>
concept ConstraintFunction = requires(
    F                       f,
    typename F::Value       lambda,
    ConstBodies<F::nBodies> bodies,
    float                   dt
) {
    // clang-format off
    typename F::Value;
    std::is_same_v<typename F::Value, Vector<float, F::dimension>>; 
    
    typename F::Jacobian;
    std::is_same_v<typename F::Jacobian, Matrix<float, F::dimension, 3*F::nBodies>>; 

    { f.eval(bodies) } -> std::same_as<typename F::Value>;
    { f.bias(bodies) } -> std::same_as<typename F::Value>;

    { f.jacobian(bodies) } -> std::same_as<typename F::Jacobian>;
    
    { f.clampLambda(lambda, dt) } -> std::same_as<typename F::Value>;
    { f.clampPositionLambda(lambda) } -> std::same_as<typename F::Value>;
    // clang-format on
};


template <ConstraintFunction F>
class ConstraintSolver
{
public:

    static constexpr Uint32 nBodies   = F::nBodies;
    static constexpr Uint32 dimension = F::dimension;

    typedef const ConstBodies<nBodies>& CBodies;
    typedef const Bodies<nBodies>&      Bodies;

    typedef F::Value    Value;
    typedef F::Jacobian Jacobian;

    typedef Vector<float, 3 * nBodies> State;
    typedef State                      Velocity;
    typedef State                      Impulse;

    typedef Matrix<float, 3 * nBodies, 3 * nBodies> MassMatrix;
    typedef Vector<float, nBodies>                  Dominance;

    typedef Matrix<float, dimension, dimension> KMatrix;
    typedef Solver<float, dimension>            KSolver;


    ConstraintSolver(Bodies bodies, const F& f, const Dominance& dominance)
        : invMass_{inverseMass(bodies, dominance)}, solver_{KMatrix{}}
    {
        initSolver(bodies, f);
    }

    void initSolve(Bodies bodies, const F& f, float dt)
    {
        initSolver(bodies, f);

        lambda_ = f.clampLambda(lambda_, dt);
        applyImpulse(bodies, impulse(J_, lambda_));
    }

    void solveVelocity(Bodies bodies, const F& f, float dt)
    {
        Value error = J_ * velocity(bodies);
        Value bias  = f.bias(bodies);
        Value baumgarteStabilization
            = KMatrix::diagonal(restitution()) * f.eval(bodies) / dt;
        Value previousDamping = KMatrix::diagonal(damping()) * lambda_;

        Value rhs = -(error + bias + baumgarteStabilization + previousDamping);

        Value dLambda   = solver_.solve(rhs);
        Value oldLambda = lambda_;
        lambda_ += dLambda;
        lambda_ = f.clampLambda(lambda_, dt);

        applyImpulse(bodies, impulse(J_, lambda_ - oldLambda));
    }

    void applyImpulse(Bodies bodies, const Impulse& impulse) const
    {
        Velocity dv = invMass_ * impulse;
        Uint32   i  = 0;
        for (PhysicsBody* body : bodies)
        {
            body->velocity() += Vec2{dv[i], dv[i + 1]};
            body->angularVelocity() += dv[i + 2];
            i += 3;
        }
    }

    void solvePosition(Bodies bodies, const F& f)
    {
        Value error = f.eval(bodies);

        Jacobian J = f.jacobian(bodies);
        solver_    = KSolver{J * invMass_ * transpose(J)};

        Value posLambda          = f.clampPositionLambda(solver_.solve(-error));
        State positionCorrection = invMass_ * transpose(J) * posLambda;

        applyPositionCorrection(bodies, positionCorrection);
    }

    void applyPositionCorrection(Bodies bodies, const State& correction) const
    {
        Uint32 i = 0;
        for (PhysicsBody* body : bodies)
        {
            body->position_ += Vec2{correction[i], correction[i + 1]};
            body->orientation_ += correction[i + 2];
            body->collider_.update(body->toWorldSpace());

            i += 3;
        }
    }

    Value&       restitution() { return restitution_; }
    const Value& restitution() const { return restitution_; }

    Value&       damping() { return damping_; }
    const Value& damping() const { return damping_; }


    ////////////////////////////////////////////////////////////
    // Static methods
    ////////////////////////////////////////////////////////////

    static Velocity velocity(CBodies bodies)
    {
        Velocity v{};
        Uint32   i = 0;
        for (const PhysicsBody* body : bodies)
        {
            v[i++] = body->velocity()[0];
            v[i++] = body->velocity()[1];
            v[i++] = body->angularVelocity();
        }

        return v;
    }

    static MassMatrix inverseMass(CBodies bodies, const Dominance& dominance)
    {
        Vector<float, 3 * nBodies> diagonal;

        Uint32 i       = 0;
        Uint32 nthBody = 0;
        for (const PhysicsBody* body : bodies)
        {
            diagonal[i++] = dominance[nthBody] / body->properties().mass;
            diagonal[i++] = dominance[nthBody] / body->properties().mass;
            diagonal[i++] = dominance[nthBody++] / body->properties().inertia;
        }

        return MassMatrix::diagonal(diagonal);
    }

    static Impulse impulse(const Jacobian& J, const Value& lambda)
    {
        return transpose(J) * lambda;
    }

    static KSolver makeSolver(const KMatrix& K) { return Solver{K}; }

    ////////////////////////////////////////////////////////////
    // Unusual
    ////////////////////////////////////////////////////////////

    MassMatrix getInverseMass() const { return invMass_; }
    Jacobian   getJacobian() const { return J_; }
    Value      getAccumulatedLambda() const { return lambda_; }
    void       setLambdaHint(Value lambda) { lambda_ = lambda; }

private:

    void initSolver(CBodies bodies, const F& f)
    {
        J_      = f.jacobian(bodies);
        auto K  = J_ * invMass_ * transpose(J_) + KMatrix::diagonal(damping());
        solver_ = Solver{K};
        SIMU_ASSERT(solver_.isValid(), "Constraint cannot be solved");
    }

    MassMatrix invMass_;
    KSolver    solver_;

    Value    lambda_{};
    Jacobian J_{};

    Value restitution_{};
    Value damping_{};
};


} // namespace simu
