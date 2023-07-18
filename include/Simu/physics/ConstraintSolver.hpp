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

#include <iostream>

#include "Simu/config.hpp"
#include "Simu/math/Matrix.hpp"

#include "Simu/physics/ConstraintInterfaces.hpp"

namespace simu
{

class Body;


template <ConstraintFunction F_>
class ConstraintSolverBase
{
public:

    typedef F_ F;

    static constexpr Uint32 nBodies   = F::nBodies;
    static constexpr Uint32 dimension = F::dimension;

    typedef typename F::Value    Value;
    typedef typename F::Jacobian Jacobian;

    typedef typename Bodies<nBodies>::State    State;
    typedef typename Bodies<nBodies>::Velocity Velocity;
    typedef typename Bodies<nBodies>::Impulse  Impulse;

    typedef Matrix<float, dimension, dimension> KMatrix;


    ConstraintSolverBase(Bodies<nBodies> /* bodies */, const F& /* f */) {}

    Value&       restitution() { return restitution_; }
    const Value& restitution() const { return restitution_; }

    Value&       damping() { return damping_; }
    const Value& damping() const { return damping_; }

    KMatrix
    computeEffectiveMass(const Bodies<nBodies>& bodies, const F& f, bool addDamping)
    {
        J_                    = f.jacobian(bodies);
        KMatrix effectiveMass = J_ * bodies.inverseMass() * transpose(J_);

        if (addDamping)
            effectiveMass += KMatrix::diagonal(damping());

        return effectiveMass;
    }

    Value
    computeRhs(const Bodies<nBodies>& bodies, const F& f, float dt, bool addDamping) const
    {
        Value error = J_ * bodies.velocity();
        Value bias  = f.bias(bodies);
        Value baumgarteStabilization
            = KMatrix::diagonal(restitution()) * f.eval(bodies) / dt;

        Value previousDamping
            = addDamping ? KMatrix::diagonal(damping()) * lambda_ : Value{};

        return -(error + bias + baumgarteStabilization + previousDamping);
    }

    void
    updateLambda(Bodies<nBodies>& bodies, const F& f, float dt, Value dLambda)
    {
        Value oldLambda = lambda_;
        lambda_ += dLambda;
        lambda_ = f.clampLambda(lambda_, dt);

        bodies.applyImpulse(impulse(J_, lambda_ - oldLambda));
    }

    static Impulse impulse(const Jacobian& J, const Value& lambda)
    {
        return transpose(J) * lambda;
    }

    Jacobian getJacobian() const { return J_; }
    Value    getAccumulatedLambda() const { return lambda_; }
    void     setLambdaHint(Value lambda) { lambda_ = lambda; }

private:

    Value    lambda_{};
    Jacobian J_{};

    Value restitution_{};
    Value damping_{};
};


template <ConstraintFunction F>
class EqualitySolver : public ConstraintSolverBase<F>
{
public:

    typedef ConstraintSolverBase<F> Base;

    static constexpr Uint32 nBodies   = F::nBodies;
    static constexpr Uint32 dimension = F::dimension;

    typedef typename Base::Value    Value;
    typedef typename Base::Jacobian Jacobian;
    typedef typename Base::State    State;

    typedef typename Base::KMatrix         KMatrix;
    typedef Solver<float, Base::dimension> KSolver;


    EqualitySolver(const Bodies<nBodies>& bodies, const F& f)
        : Base{bodies, f}, solver_{KMatrix{}}
    {
        initSolver(bodies, f);
    }

    void initSolve(Bodies<nBodies>& bodies, const F& f, float dt)
    {
        initSolver(bodies, f);

        Value lambda = this->getAccumulatedLambda();
        this->setLambdaHint(Value::filled(0.f));

        this->updateLambda(bodies, f, dt, lambda);
    }

    void solveVelocity(Bodies<nBodies>& bodies, const F& f, float dt)
    {
        this->updateLambda(
            bodies,
            f,
            dt,
            solver_.solve(this->computeRhs(bodies, f, dt, true))
        );
    }

    void solvePosition(Bodies<nBodies>& bodies, const F& f)
    {
        Value error = f.eval(bodies);

        Jacobian J = f.jacobian(bodies);
        solver_    = KSolver{this->computeEffectiveMass(bodies, f, false)};

        Value posLambda = f.clampPositionLambda(solver_.solve(-error));
        State positionCorrection
            = bodies.inverseMass() * transpose(J) * posLambda;

        bodies.applyPositionCorrection(positionCorrection);
    }


private:

    void initSolver(const Bodies<nBodies>& bodies, const F& f)
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

    static constexpr Uint32 nBodies   = F::nBodies;
    static constexpr Uint32 dimension = F::dimension;

    typedef typename Base::Value    Value;
    typedef typename Base::Jacobian Jacobian;
    typedef typename Base::State    State;

    typedef typename Base::KMatrix         KMatrix;
    typedef Solver<float, Base::dimension> KSolver;


    InequalitySolver(const Bodies<nBodies>& bodies, const F& f)
        : Base{bodies, f}
    {
    }

    void initSolve(Bodies<nBodies>& bodies, const F& f, float dt)
    {
        initSolver(bodies, f);

        Value lambda = this->getAccumulatedLambda();
        this->setLambdaHint(Value::filled(0.f));

        this->updateLambda(bodies, f, dt, lambda);
    }

    void solveVelocity(Bodies<nBodies>& bodies, const F& f, float dt)
    {
        Value correctedError = effectiveMass_ * this->getAccumulatedLambda();

        Value lambda = solveInequalities(
            effectiveMass_,
            this->computeRhs(bodies, f, dt, false) + correctedError,
            [=, &f](Value lambda) { return f.clampLambda(lambda, dt); },
            this->getAccumulatedLambda()
        );

        this->updateLambda(bodies, f, dt, lambda - this->getAccumulatedLambda());
    }

    void solvePosition(Bodies<nBodies>& bodies, const F& f)
    {
        Value error = f.eval(bodies);

        Jacobian J     = f.jacobian(bodies);
        effectiveMass_ = J * bodies.inverseMass() * transpose(J);

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
            = bodies.inverseMass() * transpose(J) * posLambda;

        bodies.applyPositionCorrection(positionCorrection);
    }


private:

    void initSolver(const Bodies<nBodies>& bodies, const F& f)
    {
        effectiveMass_ = this->computeEffectiveMass(bodies, f, true);
    }

    KMatrix effectiveMass_;
};

namespace priv
{

// Uses a special form for velocities:
// K( (lambda + dLambda - (1+e)lambda) / (1+e) ) >= -Jv
template <ConstraintFunction F>
class ContactSolver : public ConstraintSolverBase<F>
{
public:

    static_assert(
        std::is_same_v<F, SingleContactFunction>
            || std::is_same_v<F, DoubleContactFunction>,
        "This is reserved for contact constraints."
    );

    typedef ConstraintSolverBase<F> Base;

    static constexpr Uint32 nBodies   = F::nBodies;
    static constexpr Uint32 dimension = F::dimension;

    typedef typename Base::Value    Value;
    typedef typename Base::Jacobian Jacobian;
    typedef typename Base::State    State;

    typedef typename Base::KMatrix         KMatrix;
    typedef Solver<float, Base::dimension> KSolver;


    ContactSolver(const Bodies<nBodies>& bodies, const F& f)
        : Base{bodies, f},
          restitutionCoefficient_{
            CombinableProperty{bodies[0]->material().bounciness, bodies[1]->material().bounciness}
          .value}
    {
    }

    void initSolve(Bodies<nBodies>& bodies, const F& f, float dt)
    {
        initSolver(bodies, f);

        Value lambda = this->getAccumulatedLambda();
        this->setLambdaHint(Value::filled(0.f));

        this->updateLambda(bodies, f, dt, lambda);
    }

    void solveVelocity(Bodies<nBodies>& bodies, const F& f, float dt)
    {
        // auto error = this->computeRhs(bodies, f, dt, false);
        // auto initialGuess
        //     = -(restitutionCoefficient_ / (1.f + restitutionCoefficient_))
        //       * this->getAccumulatedLambda();

        // Value x = solveInequalities(
        //     effectiveMass_,
        //     error,
        //     [this](Value x) { return this->clampX(x); },
        //     initialGuess
        // );

        // Value dLambda = (1.f + restitutionCoefficient_) * x
        //                 + restitutionCoefficient_ * this->getAccumulatedLambda();

        // this->updateLambda(bodies, f, dt, dLambda);

        // using bounce as the restitution bias when the solver is initialized..

        Jacobian J     = this->getJacobian();
        auto     error = -(J * bodies.velocity())
                     + effectiveMass_ * this->getAccumulatedLambda();
        error -= bounce_;

        Value lambda = solveInequalities(
            effectiveMass_,
            error,
            [=, &f](Value lambda) { return f.clampLambda(lambda, dt); },
            this->getAccumulatedLambda()
        );

        Value dLambda = lambda - this->getAccumulatedLambda();

        this->updateLambda(bodies, f, dt, dLambda);
    }

    void solvePosition(Bodies<nBodies>& bodies, const F& f)
    {
        Value error = f.eval(bodies);

        Jacobian J     = f.jacobian(bodies);
        effectiveMass_ = J * bodies.inverseMass() * transpose(J);

        Value posLambda
            = solveInequalities(effectiveMass_, -error, [&f](Value posLambda) {
                  return f.clampPositionLambda(posLambda);
              });

        // Solver<float, F::dimension> s{effectiveMass_};
        // if (!s.isValid())
        //     return; // Jacobians are parallel, TODO:

        // Value posLambda = s.solve(-error);
        // posLambda       = f.clampPositionLambda(posLambda);

        State positionCorrection
            = bodies.inverseMass() * transpose(J) * posLambda;

        bodies.applyPositionCorrection(positionCorrection);
    }


private:

    void initSolver(const Bodies<nBodies>& bodies, const F& f)
    {
        effectiveMass_ = this->computeEffectiveMass(bodies, f, false);
        bounce_
            = restitutionCoefficient_ * this->getJacobian() * bodies.velocity();
        bounce_ = std::min(Value::filled(0.f), bounce_);
    }

    typename F::Value clampX(typename F::Value x)
    {
        return std::max(x, -this->getAccumulatedLambda());
    }

    float   restitutionCoefficient_;
    Value   bounce_;
    KMatrix effectiveMass_;
};

} // namespace priv

} // namespace simu
