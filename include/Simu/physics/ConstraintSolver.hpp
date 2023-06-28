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
#include "Simu/math/Matrix.hpp"
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


template <ConstraintFunction F_>
class ConstraintSolverBase
{
public:

    typedef F_ F;

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


    ConstraintSolverBase(Bodies bodies, const F& f, const Dominance& dominance)
        : invMass_{inverseMass(bodies, dominance)}
    {
    }

    Value&       restitution() { return restitution_; }
    const Value& restitution() const { return restitution_; }

    Value&       damping() { return damping_; }
    const Value& damping() const { return damping_; }

    KMatrix computeEffectiveMass(CBodies bodies, const F& f, bool addDamping)
    {
        J_                    = f.jacobian(bodies);
        KMatrix effectiveMass = J_ * invMass_ * transpose(J_);

        if (addDamping)
            effectiveMass += KMatrix::diagonal(damping());

        return effectiveMass;
    }

    Value computeRhs(CBodies bodies, const F& f, float dt, bool addDamping) const
    {
        Value error = J_ * velocity(bodies);
        Value bias  = f.bias(bodies);
        Value baumgarteStabilization
            = KMatrix::diagonal(restitution()) * f.eval(bodies) / dt;

        Value previousDamping
            = addDamping ? KMatrix::diagonal(damping()) * lambda_ : Value{};

        return -(error + bias + baumgarteStabilization + previousDamping);
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

    void updateLambda(Bodies bodies, const F& f, float dt, Value dLambda)
    {
        Value oldLambda = lambda_;
        lambda_ += dLambda;
        lambda_ = f.clampLambda(lambda_, dt);

        applyImpulse(bodies, impulse(J_, lambda_ - oldLambda));
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

    MassMatrix getInverseMass() const { return invMass_; }
    Jacobian   getJacobian() const { return J_; }
    Value      getAccumulatedLambda() const { return lambda_; }
    void       setLambdaHint(Value lambda) { lambda_ = lambda; }

private:

    MassMatrix invMass_;

    Value    lambda_{};
    Jacobian J_{};

    Value restitution_{};
    Value damping_{};
};

template <class S>
concept ConstraintSolver = requires(
    S                  s,
    typename S::Bodies bodies,
    typename S::F      f,
    float              dt
) {
    // clang-format off
    typename S::F;
    ConstraintFunction<typename S::F>;

    std::derived_from<S, ConstraintSolverBase<typename S::F>>;

    std::constructible_from<S, 
        typename S::Bodies, 
        typename S::F, 
        typename S::Dominance
    >;

    { s.initSolve(bodies, f, dt) };
    { s.solveVelocity(bodies, f, dt) };
    { s.solvePosition(bodies, f) };

    // clang-format on
};


template <ConstraintFunction F>
class EqualitySolver : public ConstraintSolverBase<F>
{
public:

    typedef ConstraintSolverBase<F> Base;

    typedef typename Base::Value    Value;
    typedef typename Base::Jacobian Jacobian;
    typedef typename Base::State    State;

    typedef typename Base::Bodies  Bodies;
    typedef typename Base::CBodies CBodies;

    typedef typename Base::Dominance       Dominance;
    typedef typename Base::KMatrix         KMatrix;
    typedef Solver<float, Base::dimension> KSolver;


    EqualitySolver(Bodies bodies, const F& f, const Dominance& dominance)
        : Base{bodies, f, dominance}, solver_{KMatrix{}}
    {
        initSolver(bodies, f);
    }

    void initSolve(Bodies bodies, const F& f, float dt)
    {
        initSolver(bodies, f);

        Value lambda = this->getAccumulatedLambda();
        this->setLambdaHint(Value::filled(0.f));

        this->updateLambda(bodies, f, dt, lambda);
    }

    void solveVelocity(Bodies bodies, const F& f, float dt)
    {
        this->updateLambda(
            bodies,
            f,
            dt,
            solver_.solve(this->computeRhs(bodies, f, dt, true))
        );
    }

    void solvePosition(Bodies bodies, const F& f)
    {
        Value error = f.eval(bodies);

        Jacobian J = f.jacobian(bodies);
        solver_    = KSolver{this->computeEffectiveMass(bodies, f, false)};

        Value posLambda = f.clampPositionLambda(solver_.solve(-error));
        State positionCorrection
            = this->getInverseMass() * transpose(J) * posLambda;

        this->applyPositionCorrection(bodies, positionCorrection);
    }


private:

    void initSolver(CBodies bodies, const F& f)
    {
        solver_ = Solver{this->computeEffectiveMass(bodies, f, true)};
        SIMU_ASSERT(solver_.isValid(), "Constraint cannot be solved");
    }

    KSolver solver_;
};


template <ConstraintFunction F>
class InequalitySolver : public ConstraintSolverBase<F>
{
public:

    typedef ConstraintSolverBase<F> Base;

    typedef typename Base::Value    Value;
    typedef typename Base::Jacobian Jacobian;
    typedef typename Base::State    State;

    typedef typename Base::Bodies  Bodies;
    typedef typename Base::CBodies CBodies;

    typedef typename Base::Dominance       Dominance;
    typedef typename Base::KMatrix         KMatrix;
    typedef Solver<float, Base::dimension> KSolver;


    InequalitySolver(Bodies bodies, const F& f, const Dominance& dominance)
        : Base{bodies, f, dominance}
    {
    }

    void initSolve(Bodies bodies, const F& f, float dt)
    {
        initSolver(bodies, f);

        Value lambda = this->getAccumulatedLambda();
        this->setLambdaHint(Value::filled(0.f));

        this->updateLambda(bodies, f, dt, lambda);
    }

    void solveVelocity(Bodies bodies, const F& f, float dt)
    {
        Value correctedError = this->computeEffectiveMass(bodies, f, false)
                               * this->getAccumulatedLambda();

        Value lambda = solveInequalities(
            effectiveMass_,
            this->computeRhs(bodies, f, dt, false) + correctedError,
            [=, &f](Value lambda) { return f.clampLambda(lambda, dt); },
            this->getAccumulatedLambda()
        );

        this->updateLambda(bodies, f, dt, lambda - this->getAccumulatedLambda());
    }

    void solvePosition(Bodies bodies, const F& f)
    {
        Value error = f.eval(bodies);

        Jacobian J     = f.jacobian(bodies);
        effectiveMass_ = J * this->getInverseMass() * transpose(J);

        // TODO: Inequality solver should be using:
        // Value posLambda = solveInequalities(
        //     effectiveMass_,
        //     -error,
        //     [&](Value lambda) { return f.clampPositionLambda(lambda); }
        // );
        //
        // but it's too inaccurate for contacts

        Solver<float, F::dimension> s{effectiveMass_};
        if (!s.isValid())
            return; // Jacobians are parallel, TODO:

        Value posLambda = s.solve(-error);
        posLambda       = f.clampPositionLambda(posLambda);

        State positionCorrection
            = this->getInverseMass() * transpose(J) * posLambda;

        this->applyPositionCorrection(bodies, positionCorrection);
    }


private:

    void initSolver(CBodies bodies, const F& f)
    {
        effectiveMass_ = this->computeEffectiveMass(bodies, f, true);
    }

    KMatrix effectiveMass_;
};

} // namespace simu
